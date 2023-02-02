// SPDX-License-Identifier: MIT

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include "hunk-reloc.h"
#include "piscsi/piscsi-enums.h"
#include "piscsi/piscsi.h"

#ifdef FAKESTORM
#define lseek64 lseek
#endif

#define DEBUG_SPAMMY(...)
//#define DEBUG_SPAMMY printf
#define DEBUG(...)
//#define DEBUG printf

#define BE(val) be32toh(val)
#define BE16(val) be16toh(val)

#define READLW(a, b) fread(&a, 4, 1, b); a = be32toh(a);
#define READW(a, b) fread(&a, 2, 1, b); a = be16toh(a);

uint32_t lw = 0;
static uint32_t file_offset = 0, add_size = 0;

char *hunk_id_name(uint32_t index) {
    switch (index) {
        case HUNKTYPE_HEADER:
            return "HUNK_HEADER";
        case HUNKTYPE_CODE:
            return "HUNK_CODE";
        case HUNKTYPE_HUNK_RELOC32:
            return "HUNK_RELOC32";
        case HUNKTYPE_SYMBOL:
            return "HUNK_SYMBOL";
        case HUNKTYPE_BSS:
            return "HUNK_BSS";
        case HUNKTYPE_DATA:
            return "HUNK_DATA";
        case HUNKTYPE_END:
            return "HUNK_END";
        default:
            return "UNKNOWN HUNK TYPE";
    }
}

int process_hunk(uint32_t index, struct hunk_info *info, FILE *f, struct hunk_reloc *r) {
    if (!f)
        return -1;
    
    uint32_t discard = 0, cur_hunk = 0, offs32 = 0;
    
    switch (index) {
        case HUNKTYPE_HEADER:
            DEBUG("[HUNK_RELOC] Processing hunk HEADER.\n");
            do {
                READLW(discard, f);
                if (discard) {
                    if (info->libnames[info->num_libs]) {
                        free(info->libnames[info->num_libs]);
                        info->libnames[info->num_libs] = NULL;
                    }
                    info->libnames[info->num_libs] = malloc(discard * 4);
                    fread(info->libnames[info->num_libs], discard, 4, f);
                    info->num_libs++;
                }
            } while (discard);
            
            READLW(info->table_size, f);
            DEBUG("[HUNK_RELOC] [HEADER] Table size: %d\n", info->table_size);
            READLW(info->first_hunk, f);
            READLW(info->last_hunk, f);
            info->num_hunks = (info->last_hunk - info->first_hunk) + 1;
            DEBUG("[HUNK_RELOC] [HEADER] First: %d Last: %d Num: %d\n", info->first_hunk, info->last_hunk, info->num_hunks);
            if (info->hunk_sizes) {
                free(info->hunk_sizes);
                info->hunk_sizes = NULL;
            }
            if (info->hunk_offsets) {
                free(info->hunk_offsets);
                info->hunk_offsets = NULL;
            }
            info->hunk_sizes = malloc(info->num_hunks * 4);
            info->hunk_offsets = malloc(info->num_hunks * 4);
            for (uint32_t i = 0; i < info->table_size; i++) {
                READLW(info->hunk_sizes[i], f);
                DEBUG("[HUNK_RELOC] [HEADER] Hunk %d: %d (%.8X)\n", i, info->hunk_sizes[i] * 4, info->hunk_sizes[i] * 4);
            }
            info->header_size = (uint32_t)ftell(f) - file_offset;
            DEBUG("[HUNK_RELOC] [HEADER] ~~~~~~~~~~~ Hunk HEADER size is %d ~~~~~~~~~~~~.\n", info->header_size);
            return 0;
            break;
        case HUNKTYPE_CODE:
            DEBUG("[HUNK_RELOC] Hunk %d: CODE.\n", info->current_hunk);
            READLW(discard, f);
            info->hunk_offsets[info->current_hunk] = ftell(f) - file_offset;
            DEBUG("[HUNK_RELOC] [CODE] Code hunk size: %d (%.8X)\n", discard * 4, discard * 4);
            fseek(f, discard * 4, SEEK_CUR);
            return 0;
            break;
        case HUNKTYPE_HUNK_RELOC32:
            DEBUG("[HUNK_RELOC] Hunk %d: RELOC32.\n", info->current_hunk);
            DEBUG("Processing Reloc32 hunk.\n");
            do {
                READLW(discard, f);
                if (discard && discard != 0xFFFFFFFF) {
                    READLW(cur_hunk, f);
                    DEBUG("[HUNK_RELOC] [RELOC32] Relocating %d offsets pointing to hunk %d.\n", discard, cur_hunk);
                    for(uint32_t i = 0; i < discard; i++) {
                        READLW(offs32, f);
                        DEBUG_SPAMMY("[HUNK_RELOC] [RELOC32] #%d: @%.8X in hunk %d\n", i + 1, offs32, cur_hunk);
                        r[info->reloc_hunks].offset = offs32;
                        r[info->reloc_hunks].src_hunk = info->current_hunk;
                        r[info->reloc_hunks].target_hunk = cur_hunk;
                        info->reloc_hunks++;
                    }
                }
            } while(discard);
            return 0;
            break;
        case HUNKTYPE_SYMBOL:
            DEBUG("[HUNK_RELOC] Hunk %d: SYMBOL.\n", info->current_hunk);
            DEBUG("[HUNK_RELOC] [SYMBOL] Processing Symbol hunk.\n");
            READLW(discard, f);
            do {
                if (discard) {
                    char sstr[256];
                    memset(sstr, 0x00, 256);
                    fread(sstr, discard, 4, f);
                    READLW(discard, f);
                    DEBUG("[HUNK_RELOC] [SYMBOL] Symbol: %s - %.8X\n", sstr, discard);
                }
                READLW(discard, f);
            } while (discard);
            return 0;
            break;
        case HUNKTYPE_BSS:
            DEBUG("[HUNK_RELOC] Hunk %d: BSS.\n", info->current_hunk);
            READLW(discard, f);
            info->hunk_offsets[info->current_hunk] = ftell(f) - file_offset;
            DEBUG("[HUNK_RELOC] [BSS] Skipping BSS hunk. Size: %d\n", discard * 4);
            add_size += (discard * 4);
            return 0;
        case HUNKTYPE_DATA:
            DEBUG("[HUNK_RELOC] Hunk %d: DATA.\n", info->current_hunk);
            READLW(discard, f);
            info->hunk_offsets[info->current_hunk] = ftell(f) - file_offset;
            DEBUG("[HUNK_RELOC] [DATA] Skipping data hunk. Size: %d.\n", discard * 4);
            fseek(f, discard * 4, SEEK_CUR);
            return 0;
            break;
        case HUNKTYPE_END:
            DEBUG("[HUNK_RELOC] END: Ending hunk %d.\n", info->current_hunk);
            info->current_hunk++;
            return 0;
            break;
        default:
            DEBUG("[!!!HUNK_RELOC] Unknown hunk type %.8X! Can't process!\n", index);
            break;
    }

    return -1;
}

void reloc_hunk(struct hunk_reloc *h, uint8_t *buf, struct hunk_info *i) {
    uint32_t rel = i->hunk_offsets[h->target_hunk];
    uint32_t *src_ptr = (uint32_t *)(&buf[i->hunk_offsets[h->src_hunk] + h->offset]);

    uint32_t src = be32toh(*src_ptr);
    uint32_t dst = src + i->base_offset + rel;
    DEBUG_SPAMMY("[HUNK-RELOC] %.8X -> %.8X\n", src, dst);
    *src_ptr = htobe32(dst);
}

void process_hunks(FILE *in, struct hunk_info *h_info, struct hunk_reloc *r, uint32_t offset) {
    READLW(lw, in);
    DEBUG_SPAMMY("Hunk ID: %.8X (%s)\n", lw, hunk_id_name(lw));

    file_offset = offset;
    add_size = 0;

    while (!feof(in) && process_hunk(lw, h_info, in, r) != -1) {
        READLW(lw, in);
        if (feof(in)) goto end_parse;
        DEBUG("Hunk ID: %.8X (%s)\n", lw, hunk_id_name(lw));
        DEBUG("File pos: %.8lX\n", ftell(in) - file_offset);
    }
    end_parse:;
    DEBUG("Done processing hunks.\n");
}

void reloc_hunks(struct hunk_reloc *r, uint8_t *buf, struct hunk_info *h_info) {
    DEBUG("[HUNK-RELOC] Relocating %d offsets.\n", h_info->reloc_hunks);
    for (uint32_t i = 0; i < h_info->reloc_hunks; i++) {
        DEBUG_SPAMMY("[HUNK-RELOC] Relocating offset %d.\n", i);
        reloc_hunk(&r[i], buf, h_info);
    }
    DEBUG("[HUNK-RELOC] Done relocating offsets.\n");
}

struct LoadSegBlock {
    uint32_t   lsb_ID;
    uint32_t   lsb_SummedLongs;
    int32_t    lsb_ChkSum;
    uint32_t   lsb_HostID;
    uint32_t   lsb_Next;
    uint32_t   lsb_LoadData[PISCSI_MAX_BLOCK_SIZE / 4];
};
#define	LOADSEG_IDENTIFIER 0x4C534547

int load_lseg(int fd, uint8_t **buf_p, struct hunk_info *i, struct hunk_reloc *relocs, uint32_t block_size) {
    if (fd == -1)
        return -1;
    
    if (block_size == 0)
        block_size = 512;

    uint8_t *block = malloc(block_size);
    uint32_t next_blk = 0;
    struct LoadSegBlock *lsb = (struct LoadSegBlock *)block;

    read(fd, block, block_size);
    if (BE(lsb->lsb_ID) != LOADSEG_IDENTIFIER) {
        DEBUG("[LOAD_LSEG] Attempted to load a non LSEG-block: %.8X", BE(lsb->lsb_ID));
        goto fail;
    }

    char *filename = "data/lsegout.bin";
    FILE *out = fopen(filename, "wb+");

    DEBUG("[LOAD_LSEG] LSEG data:\n");
    DEBUG("[LOAD_LSEG] Longs: %d HostID: %d\n", BE(lsb->lsb_SummedLongs), BE(lsb->lsb_HostID));
    DEBUG("[LOAD_LSEG] Next: %d LoadData: %p\n", BE(lsb->lsb_Next), (void *)lsb->lsb_LoadData);
    next_blk = BE(lsb->lsb_Next);
    do {
        next_blk = BE(lsb->lsb_Next);
        fwrite(lsb->lsb_LoadData, 1, block_size - 20, out);
        lseek64(fd, next_blk * block_size, SEEK_SET);
        read(fd, block, block_size);
    } while (next_blk != 0xFFFFFFFF);
    
    uint32_t file_size = ftell(out);
    fseek(out, 0, SEEK_SET);
    uint8_t *buf = malloc(file_size + 1024);
    fread(buf, file_size, 1, out);
    fseek(out, 0, SEEK_SET);
    process_hunks(out, i, relocs, 0x0);

    fclose(out);
    *buf_p = buf;
    i->byte_size = file_size;
    i->alloc_size = file_size + add_size;

    return 0;

fail:;
    if (block)
        free(block);

    return -1;
}

int load_fs(struct piscsi_fs *fs, char *dosID) {
    char filename[256];
    memset(filename, 0x00, 256);
    sprintf(filename, "./data/fs/%c%c%c.%d", dosID[0], dosID[1], dosID[2], dosID[3]);

    FILE *in = fopen(filename, "rb");
    if (in == NULL)
        return -1;

    fseek(in, 0, SEEK_END);
    uint32_t file_size = ftell(in);
    fseek(in, 0, SEEK_SET);

    fs->binary_data = malloc(file_size);
    fread(fs->binary_data, file_size, 1, in);
    fseek(in, 0, SEEK_SET);
    process_hunks(in, &fs->h_info, fs->relocs, 0x0);
    fs->h_info.byte_size = file_size;
    fs->h_info.alloc_size = file_size + add_size;

    fclose(in);

    return 0;
}

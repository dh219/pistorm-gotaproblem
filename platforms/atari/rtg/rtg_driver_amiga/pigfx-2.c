// SPDX-License-Identifier: MIT

// PiStorm RTG driver, VBCC edition.
// Based in part on the ZZ9000 RTG driver.

#include <proto/exec.h>
#include <proto/expansion.h>
#include <proto/dos.h>
#include <proto/intuition.h>
#include <proto/input.h>
#include <exec/types.h>
#include <exec/memory.h>
#include <exec/libraries.h>
#include <exec/execbase.h>
#include <exec/resident.h>
#include <exec/initializers.h>
#include <clib/debug_protos.h>
#include <devices/inputevent.h>
#include <string.h>
#include <stdint.h>
#include "boardinfo.h"
#include "../rtg_enums.h"

#define STR(s) #s
#define XSTR(s) STR(s)


#define CHECKRTG *((unsigned short *)(CARD_OFFSET))

#define CARD_OFFSET   0x70000000
#define IRTGCMD_OFFSET   0x70000060
#define CARD_REGSIZE  0x00010000
#define CARD_MEMSIZE  0x02000000 // 32MB "VRAM"
#define CARD_SCRATCH  0x72010000

#define WRITESHORT(cmd, val) *(unsigned short *)((unsigned long)(CARD_OFFSET+cmd)) = val;
#define WRITELONG(cmd, val) *(unsigned long *)((unsigned long)(CARD_OFFSET+cmd)) = val;
#define WRITEBYTE(cmd, val) *(unsigned char *)((unsigned long)(CARD_OFFSET+cmd)) = val;

#define READSHORT(cmd, var) var = *(volatile unsigned short *)(CARD_OFFSET + cmd);
#define READLONG(cmd, var) var = *(volatile unsigned long *)(CARD_OFFSET + cmd);

#define RTG_DEBUGME(val) *(volatile unsigned long *)((unsigned long)(CARD_OFFSET+RTG_DEBUGME)) = val;
//#define RTG_DEBUGME(...)
#define IWRITECMD(val) *(volatile unsigned short *)(IRTGCMD_OFFSET) = val;

#define CHIP_RAM_SIZE 0x00200000 // Chip RAM offset, 2MB

struct GFXBase {
    struct Library libNode;
    BPTR segList;
    struct ExecBase* sysBase;
    struct ExpansionBase* expansionBase;
};

#define __saveds__
#define kprintf(...)

struct ExecBase *SysBase;

int FindCard(__REGA0(struct BoardInfo* b));
int InitCard(__REGA0(struct BoardInfo* b));

void SetDAC (__REGA0(struct BoardInfo *b), __REGD7(RGBFTYPE format));
void SetGC (__REGA0(struct BoardInfo *b), __REGA1(struct ModeInfo *mode_info), __REGD0(BOOL border));
void SetColorArray (__REGA0(struct BoardInfo *b), __REGD0(UWORD start), __REGD1(UWORD num));
void SetPanning (__REGA0(struct BoardInfo *b), __REGA1(UBYTE *addr), __REGD0(UWORD width), __REGD1(WORD x_offset), __REGD2(WORD y_offset), __REGD7(RGBFTYPE format));
UWORD SetSwitch (__REGA0(struct BoardInfo *b), __REGD0(UWORD enabled));
UWORD SetDisplay (__REGA0(struct BoardInfo *b), __REGD0(UWORD enabled));

UWORD CalculateBytesPerRow (__REGA0(struct BoardInfo *b), __REGD0(UWORD width), __REGD7(RGBFTYPE format));
APTR CalculateMemory (__REGA0(struct BoardInfo *b), __REGA1(unsigned long addr), __REGD7(RGBFTYPE format));
ULONG GetCompatibleFormats (__REGA0(struct BoardInfo *b), __REGD7(RGBFTYPE format));

LONG ResolvePixelClock (__REGA0(struct BoardInfo *b), __REGA1(struct ModeInfo *mode_info), __REGD0(ULONG pixel_clock), __REGD7(RGBFTYPE format));
ULONG GetPixelClock (__REGA0(struct BoardInfo *b), __REGA1(struct ModeInfo *mode_info), __REGD0(ULONG index), __REGD7(RGBFTYPE format));
void SetClock (__REGA0(struct BoardInfo *b));

void SetMemoryMode (__REGA0(struct BoardInfo *b), __REGD7(RGBFTYPE format));
void SetWriteMask (__REGA0(struct BoardInfo *b), __REGD0(UBYTE mask));
void SetClearMask (__REGA0(struct BoardInfo *b), __REGD0(UBYTE mask));
void SetReadPlane (__REGA0(struct BoardInfo *b), __REGD0(UBYTE plane));

void WaitVerticalSync (__REGA0(struct BoardInfo *b), __REGD0(BOOL toggle));
BOOL GetVSyncState(__REGA0(struct BoardInfo *b), __REGD0(BOOL toggle));

void FillRect (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD w), __REGD3(WORD h), __REGD4(ULONG color), __REGD5(UBYTE mask), __REGD7(RGBFTYPE format));
void InvertRect (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD w), __REGD3(WORD h), __REGD4(UBYTE mask), __REGD7(RGBFTYPE format));
void BlitRect (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD dx), __REGD3(WORD dy), __REGD4(WORD w), __REGD5(WORD h), __REGD6(UBYTE mask), __REGD7(RGBFTYPE format));
void BlitRectNoMaskComplete (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *rs), __REGA2(struct RenderInfo *rt), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD dx), __REGD3(WORD dy), __REGD4(WORD w), __REGD5(WORD h), __REGD6(UBYTE minterm), __REGD7(RGBFTYPE format));
void BlitTemplate (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGA2(struct Template *t), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD w), __REGD3(WORD h), __REGD4(UBYTE mask), __REGD7(RGBFTYPE format));
void BlitPattern (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGA2(struct Pattern *p), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD w), __REGD3(WORD h), __REGD4(UBYTE mask), __REGD7(RGBFTYPE format));
void DrawLine (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGA2(struct Line *l), __REGD0(UBYTE mask), __REGD7(RGBFTYPE format));

void BlitPlanar2Chunky (__REGA0(struct BoardInfo *b), __REGA1(struct BitMap *bm), __REGA2(struct RenderInfo *r), __REGD0(SHORT x), __REGD1(SHORT y), __REGD2(SHORT dx), __REGD3(SHORT dy), __REGD4(SHORT w), __REGD5(SHORT h), __REGD6(UBYTE minterm), __REGD7(UBYTE mask));
void BlitPlanar2Direct (__REGA0(struct BoardInfo *b), __REGA1(struct BitMap *bmp), __REGA2(struct RenderInfo *r), __REGA3(struct ColorIndexMapping *clut), __REGD0(SHORT x), __REGD1(SHORT y), __REGD2(SHORT dx), __REGD3(SHORT dy), __REGD4(SHORT w), __REGD5(SHORT h), __REGD6(UBYTE minterm), __REGD7(UBYTE mask));

void SetSprite (__REGA0(struct BoardInfo *b), __REGD0(BOOL what), __REGD7(RGBFTYPE format));
void SetSpritePosition (__REGA0(struct BoardInfo *b), __REGD0(WORD x), __REGD1(WORD y), __REGD7(RGBFTYPE format));
void SetSpriteImage (__REGA0(struct BoardInfo *b), __REGD7(RGBFTYPE format));
void SetSpriteColor (__REGA0(struct BoardInfo *b), __REGD0(UBYTE idx), __REGD1(UBYTE R), __REGD2(UBYTE G), __REGD3(UBYTE B), __REGD7(RGBFTYPE format));

#define DEVICE_VERSION 43
#define DEVICE_REVISION 20
#define DEVICE_PRIORITY 0
#define DEVICE_ID_STRING "PiGFX " XSTR(DEVICE_VERSION) "." XSTR(DEVICE_REVISION) " " DEVICE_DATE
#define DEVICE_NAME "pigfx.card"
#define DEVICE_DATE "(29 May 2021)"


int __attribute__((no_reorder)) _start()
{
        return -1;
}

asm("romtag:                                    \n"
        "       dc.w    "XSTR(RTC_MATCHWORD)"   \n"
        "       dc.l    romtag                  \n"
        "       dc.l    endcode                 \n"
        "       dc.b    "XSTR(RTF_AUTOINIT)"    \n"
        "       dc.b    "XSTR(DEVICE_VERSION)"  \n"
        "       dc.b    "XSTR(NT_LIBRARY)"      \n"
        "       dc.b    "XSTR(DEVICE_PRIORITY)" \n"
        "       dc.l    _device_name            \n"
        "       dc.l    _device_id_string       \n"
        "       dc.l    _auto_init_tables       \n"
        "endcode:                               \n");

char device_name[] = DEVICE_NAME;
char device_id_string[] = DEVICE_ID_STRING;

__saveds struct GFXBase* OpenLib(__REGA6(struct GFXBase *gfxbase));
BPTR __saveds CloseLib(__REGA6(struct GFXBase *gfxbase));
BPTR __saveds ExpungeLib(__REGA6(struct GFXBase *exb));
ULONG ExtFuncLib(void);
__saveds struct GFXBase* InitLib(__REGA6(struct ExecBase *sysbase),
                                                                 __REGA0(BPTR seglist),
                                                                 __REGD0(struct GFXBase *exb));

#define CLOCK_HZ 100000000

static struct GFXBase *_gfxbase;
const char *gfxname = "PiStorm RTG";
char dummies[128];

__saveds struct GFXBase* __attribute__((used)) InitLib(__REGA6(struct ExecBase *sysbase),
                                                       __REGA0(BPTR seglist),
                                                       __REGD0(struct GFXBase *exb))
{
    _gfxbase = exb;
    SysBase = *(struct ExecBase **)4L;
    return _gfxbase;
}

__saveds struct GFXBase* __attribute__((used)) OpenLib(__REGA6(struct GFXBase *gfxbase))
{
    gfxbase->libNode.lib_OpenCnt++;
    gfxbase->libNode.lib_Flags &= ~LIBF_DELEXP;

    return gfxbase;
}

BPTR __saveds __attribute__((used)) CloseLib(__REGA6(struct GFXBase *gfxbase))
{
    gfxbase->libNode.lib_OpenCnt--;

    if (!gfxbase->libNode.lib_OpenCnt) {
        if (gfxbase->libNode.lib_Flags & LIBF_DELEXP) {
            return (ExpungeLib(gfxbase));
        }
    }
    return 0;
}

BPTR __saveds __attribute__((used)) ExpungeLib(__REGA6(struct GFXBase *exb))
{
    BPTR seglist;
    struct ExecBase *SysBase = *(struct ExecBase **)4L;

    if(!exb->libNode.lib_OpenCnt) {
        ULONG negsize, possize, fullsize;
        UBYTE *negptr = (UBYTE *)exb;

        seglist = exb->segList;

        Remove((struct Node *)exb);

        negsize  = exb->libNode.lib_NegSize;
        possize  = exb->libNode.lib_PosSize;
        fullsize = negsize + possize;
        negptr  -= negsize;

        FreeMem(negptr, fullsize);
        return(seglist);
    }

    exb->libNode.lib_Flags |= LIBF_DELEXP;
    return 0;
}

ULONG ExtFuncLib(void)
{
    return 0;
}

#define LOADLIB(a, b) if ((a = (struct a*)OpenLibrary((STRPTR)b,0L))==NULL) { \
        kprintf((STRPTR)"Failed to load %s.\n", b); \
        return 0; \
    } \

int __attribute__((used)) FindCard(__REGA0(struct BoardInfo* b)) {
    uint16_t card_check = CHECKRTG;
    if (card_check != 0xFFCF) {
        // RTG not enabled
        return 0;
    }

    struct IORequest io;
    if (OpenDevice((STRPTR)"input.device", 0, &io, 0) == 0)
    {
        struct Library *InputBase = (struct Library *)io.io_Device;
        UWORD qual = PeekQualifier();
        CloseDevice(&io);

        if (qual & (IEQUALIFIER_LSHIFT | IEQUALIFIER_RSHIFT))
            return(FALSE);
    }

    struct ExpansionBase *ExpansionBase = NULL;
    struct DOSBase *DOSBase = NULL;
    struct IntuitionBase *IntuitionBase = NULL;

    LOADLIB(ExpansionBase, "expansion.library");
    LOADLIB(DOSBase, "dos.library");
    LOADLIB(IntuitionBase, "intuition.library");

    b->MemorySize = CARD_MEMSIZE;
    b->RegisterBase = (void *)CARD_OFFSET;
    b->MemoryBase = (void *)(CARD_OFFSET + CARD_REGSIZE);

    return 1;
}

int __attribute__((used)) InitCard(__REGA0(struct BoardInfo* b)) {
    int i;

    b->CardBase = (struct CardBase *)_gfxbase;
    b->ExecBase = SysBase;
    b->BoardName = "PiStorm RTG";
    b->BoardType = 14;
    b->PaletteChipType = PCT_S3ViRGE;
    b->GraphicsControllerType = GCT_S3ViRGE;

    b->Flags |= BIF_GRANTDIRECTACCESS | BIF_HARDWARESPRITE | BIF_FLICKERFIXER;// | BIF_BLITTER;
    b->RGBFormats = RGBFF_HICOLOR | RGBFF_TRUECOLOR | RGBFF_TRUEALPHA | RGBFF_CLUT | RGBFF_NONE;
    b->SoftSpriteFlags = 0;
    b->BitsPerCannon = 8;

    for(i = 0; i < MAXMODES; i++) {
        b->MaxHorValue[i] = 8192;
        b->MaxVerValue[i] = 8192;
        b->MaxHorResolution[i] = 8192;
        b->MaxVerResolution[i] = 8192;
        b->PixelClockCount[i] = 1;
    }

    b->MemoryClock = CLOCK_HZ;

    //b->AllocCardMem = (void *)NULL;
    //b->FreeCardMem = (void *)NULL;
    b->SetSwitch = (void *)SetSwitch;
    b->SetColorArray = (void *)SetColorArray;
    b->SetDAC = (void *)SetDAC;
    b->SetGC = (void *)SetGC;
    b->SetPanning = (void *)SetPanning;
    b->CalculateBytesPerRow = (void *)CalculateBytesPerRow;
    b->CalculateMemory = (void *)CalculateMemory;
    b->GetCompatibleFormats = (void *)GetCompatibleFormats;
    b->SetDisplay = (void *)SetDisplay;

    b->ResolvePixelClock = (void *)ResolvePixelClock;
    b->GetPixelClock = (void *)GetPixelClock;
    b->SetClock = (void *)SetClock;

    b->SetMemoryMode = (void *)SetMemoryMode;
    b->SetWriteMask = (void *)SetWriteMask;
    b->SetClearMask = (void *)SetClearMask;
    b->SetReadPlane = (void *)SetReadPlane;

    b->WaitVerticalSync = (void *)WaitVerticalSync;
    //b->SetInterrupt = (void *)NULL;

    //b->WaitBlitter = (void *)NULL;

    //b->ScrollPlanar = (void *)NULL;
    //b->UpdatePlanar = (void *)NULL;

    b->BlitPlanar2Chunky = (void *)BlitPlanar2Chunky;
    b->BlitPlanar2Direct = (void *)BlitPlanar2Direct;

    b->FillRect = (void *)FillRect;
    b->InvertRect = (void *)InvertRect;
    b->BlitRect = (void *)BlitRect;
    b->BlitTemplate = (void *)BlitTemplate;
    b->BlitPattern = (void *)BlitPattern;
    b->DrawLine = (void *)DrawLine;
    b->BlitRectNoMaskComplete = (void *)BlitRectNoMaskComplete;
    //b->EnableSoftSprite = (void *)NULL;

    //b->AllocCardMemAbs = (void *)NULL;
    //b->SetSplitPosition = (void *)NULL;
    //b->ReInitMemory = (void *)NULL;
    //b->WriteYUVRect = (void *)NULL;
    b->GetVSyncState = (void *)GetVSyncState;
    //b->GetVBeamPos = (void *)NULL;
    //b->SetDPMSLevel = (void *)NULL;
    //b->ResetChip = (void *)NULL;
    //b->GetFeatureAttrs = (void *)NULL;
    //b->AllocBitMap = (void *)NULL;
    //b->FreeBitMap = (void *)NULL;
    //b->GetBitMapAttr = (void *)NULL;

    b->SetSprite = (void *)SetSprite;
    b->SetSpritePosition = (void *)SetSpritePosition;
    b->SetSpriteImage = (void *)SetSpriteImage;
    b->SetSpriteColor = (void *)SetSpriteColor;

    //b->CreateFeature = (void *)NULL;
    //b->SetFeatureAttrs = (void *)NULL;
    //b->DeleteFeature = (void *)NULL;

    return 1;
}

void SetDAC (__REGA0(struct BoardInfo *b), __REGD7(RGBFTYPE format)) {
    // Used to set the color format of the video card's RAMDAC.
    // This needs no handling, since the PiStorm doesn't really have a RAMDAC or a video card chipset.
}

void SetGC (__REGA0(struct BoardInfo *b), __REGA1(struct ModeInfo *mode_info), __REGD0(BOOL border)) {
    b->ModeInfo = mode_info;
    // Send width, height and format to the RaspberryPi Targetable Graphics.
    WRITESHORT(RTG_X1, mode_info->Width);
    WRITESHORT(RTG_Y1, mode_info->Height);
    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[b->RGBFormat]);
    WRITESHORT(RTG_COMMAND, RTGCMD_SETGC);
}

int setswitch = -1;
UWORD SetSwitch (__REGA0(struct BoardInfo *b), __REGD0(UWORD enabled)) {
    if (setswitch != enabled) {
        setswitch = enabled;
    }
    
    WRITEBYTE(RTG_U81, setswitch);
    WRITESHORT(RTG_X1, setswitch);
    WRITESHORT(RTG_COMMAND, RTGCMD_SETSWITCH);

    return 1 - enabled;
}

void SetPanning (__REGA0(struct BoardInfo *b), __REGA1(UBYTE *addr), __REGD0(UWORD width), __REGD1(WORD x_offset), __REGD2(WORD y_offset), __REGD7(RGBFTYPE format)) {
    // Set the panning offset, or the offset used for the current display area on the Pi.
    // The address needs to have CARD_BASE subtracted from it to be used as an offset on the Pi side.
#ifndef IRTG
    if (!b)
        return;

    b->XOffset = x_offset;
    b->YOffset = y_offset;

    WRITELONG(RTG_ADDR1, (unsigned long)addr);
    WRITESHORT(RTG_X1, width);
    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[format]);
    WRITESHORT(RTG_X2, b->XOffset);
    WRITESHORT(RTG_Y2, b->YOffset);
    WRITESHORT(RTG_COMMAND, RTGCMD_SETPAN);
#else
    IWRITECMD(RTGCMD_SETPAN);
#endif
}

void SetColorArray (__REGA0(struct BoardInfo *b), __REGD0(UWORD start), __REGD1(UWORD num)) {
    // Sets the color components of X color components for 8-bit paletted display modes.
    if (!b->CLUT)
        return;
    
    int j = start + num;
    
    for(int i = start; i < j; i++) {
        //WRITEBYTE(RTG_U82, (unsigned char)b->CLUT[i].Red);
        //WRITEBYTE(RTG_U83, (unsigned char)b->CLUT[i].Green);
        //WRITEBYTE(RTG_U84, (unsigned char)b->CLUT[i].Blue);
        unsigned long xrgb = 0 | (b->CLUT[i].Red << 16) | (b->CLUT[i].Green << 8) | (b->CLUT[i].Blue);
        WRITEBYTE(RTG_U81, (unsigned char)i);
        WRITELONG(RTG_RGB1, xrgb);
        WRITESHORT(RTG_COMMAND, RTGCMD_SETCLUT);
    }
}

UWORD CalculateBytesPerRow (__REGA0(struct BoardInfo *b), __REGD0(UWORD width), __REGD7(RGBFTYPE format)) {
    if (!b)
        return 0;

    UWORD pitch = width;

    switch(format) {
        case RGBFB_CLUT:
            return pitch;
        default:
            return 128;
        case RGBFB_R5G6B5PC: case RGBFB_R5G5B5PC:
        case RGBFB_R5G6B5: case RGBFB_R5G5B5:
        case RGBFB_B5G6R5PC: case RGBFB_B5G5R5PC:
            return (width * 2);
        case RGBFB_R8G8B8: case RGBFB_B8G8R8:
            // Should actually return width * 3, but I'm not sure if
            // the Pi VC supports 24-bit color formats.
            //return (width * 3);
        case RGBFB_B8G8R8A8: case RGBFB_R8G8B8A8:
        case RGBFB_A8B8G8R8: case RGBFB_A8R8G8B8:
            return (width * 4);
    }
}

APTR CalculateMemory (__REGA0(struct BoardInfo *b), __REGA1(unsigned long addr), __REGD7(RGBFTYPE format)) {
    /*if (!b)
        return (APTR)addr;

    if (addr > (unsigned int)b->MemoryBase && addr < (((unsigned int)b->MemoryBase) + b->MemorySize)) {
        addr = ((addr + 0x1000) & 0xFFFFF000);
    }*/

    return (APTR)addr;
}

ULONG GetCompatibleFormats (__REGA0(struct BoardInfo *b), __REGD7(RGBFTYPE format)) {
    return 0xFFFFFFFF;
}

//static int display_enabled = 0;
UWORD SetDisplay (__REGA0(struct BoardInfo *b), __REGD0(UWORD enabled)) {
    // Enables or disables the display.
    WRITEBYTE(RTG_U82, (unsigned char)enabled);
    WRITESHORT(RTG_COMMAND, RTGCMD_SETDISPLAY);

    return 1;
}

LONG ResolvePixelClock (__REGA0(struct BoardInfo *b), __REGA1(struct ModeInfo *mode_info), __REGD0(ULONG pixel_clock), __REGD7(RGBFTYPE format)) {
    mode_info->PixelClock = CLOCK_HZ;
    mode_info->pll1.Clock = 0;
    mode_info->pll2.ClockDivide = 1;

    return 0;
}

ULONG GetPixelClock (__REGA0(struct BoardInfo *b), __REGA1(struct ModeInfo *mode_info), __REGD0(ULONG index), __REGD7(RGBFTYPE format)) {
    // Just return 100MHz.
    return CLOCK_HZ;
}

// None of these five really have to do anything.
void SetClock (__REGA0(struct BoardInfo *b)) {
}

void SetMemoryMode (__REGA0(struct BoardInfo *b), __REGD7(RGBFTYPE format)) {
}

void SetWriteMask (__REGA0(struct BoardInfo *b), __REGD0(UBYTE mask)) {
}

void SetClearMask (__REGA0(struct BoardInfo *b), __REGD0(UBYTE mask)) {
}

void SetReadPlane (__REGA0(struct BoardInfo *b), __REGD0(UBYTE plane)) {
}

static uint16_t vblank;

void WaitVerticalSync (__REGA0(struct BoardInfo *b), __REGD0(BOOL toggle)) {
    vblank = 0;
    do {
        READSHORT(RTG_WAITVSYNC, vblank);
    } while (!vblank);
}

BOOL GetVSyncState(__REGA0(struct BoardInfo *b), __REGD0(BOOL toggle)) {
    READSHORT(RTG_INVBLANK, vblank);
    return vblank;
}

void FillRect (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD w), __REGD3(WORD h), __REGD4(ULONG color), __REGD5(UBYTE mask), __REGD7(RGBFTYPE format)) {
#ifndef IRTG
    if (!r)
        return;

    WRITELONG(RTG_ADDR1, (unsigned long)r->Memory);
    
    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[format]);
    WRITESHORT(RTG_X1, x);
    WRITESHORT(RTG_X2, w);
    WRITESHORT(RTG_Y1, y);
    WRITESHORT(RTG_Y2, h);
    WRITELONG(RTG_RGB1, color);
    WRITESHORT(RTG_X3, r->BytesPerRow);
    WRITEBYTE(RTG_U81, mask);
    WRITESHORT(RTG_COMMAND, RTGCMD_FILLRECT);
#else
    IWRITECMD(RTGCMD_FILLRECT);
#endif
}

void InvertRect (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD w), __REGD3(WORD h), __REGD4(UBYTE mask), __REGD7(RGBFTYPE format)) {
#ifndef IRTG
    if (!r)
        return;
    
    WRITELONG(RTG_ADDR1, (unsigned long)r->Memory);
    
    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[format]);
    WRITESHORT(RTG_X1, x);
    WRITESHORT(RTG_X2, w);
    WRITESHORT(RTG_Y1, y);
    WRITESHORT(RTG_Y2, h);
    WRITESHORT(RTG_X3, r->BytesPerRow);
    WRITEBYTE(RTG_U81, mask);
    WRITESHORT(RTG_COMMAND, RTGCMD_INVERTRECT);
#else
    IWRITECMD(RTGCMD_INVERTRECT);
#endif
}

void BlitRect (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD dx), __REGD3(WORD dy), __REGD4(WORD w), __REGD5(WORD h), __REGD6(UBYTE mask), __REGD7(RGBFTYPE format)) {
#ifndef IRTG    
    if (!r)
        return;

    WRITELONG(RTG_ADDR1, (unsigned long)r->Memory);

    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[format]);
    WRITESHORT(RTG_X1, x);
    WRITESHORT(RTG_X2, dx);
    WRITESHORT(RTG_X3, w);
    WRITESHORT(RTG_Y1, y);
    WRITESHORT(RTG_Y2, dy);
    WRITESHORT(RTG_Y3, h);
    WRITESHORT(RTG_X4, r->BytesPerRow);
    WRITEBYTE(RTG_U81, mask);
    WRITESHORT(RTG_COMMAND, RTGCMD_BLITRECT);
#else
    IWRITECMD(RTGCMD_BLITRECT);
#endif
}

void BlitRectNoMaskComplete (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *rs), __REGA2(struct RenderInfo *rt), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD dx), __REGD3(WORD dy), __REGD4(WORD w), __REGD5(WORD h), __REGD6(UBYTE minterm), __REGD7(RGBFTYPE format)) {
#ifndef IRTG
    if (!rs || !rt)
        return;

    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[format]);
    WRITELONG(RTG_ADDR1, (unsigned long)rs->Memory);
    WRITELONG(RTG_ADDR2, (unsigned long)rt->Memory);
    WRITESHORT(RTG_X1, x);
    WRITESHORT(RTG_X2, dx);
    WRITESHORT(RTG_X3, w);
    WRITESHORT(RTG_Y1, y);
    WRITESHORT(RTG_Y2, dy);
    WRITESHORT(RTG_Y3, h);
    WRITESHORT(RTG_X4, rs->BytesPerRow);
    WRITESHORT(RTG_X5, rt->BytesPerRow);
    WRITEBYTE(RTG_U81, minterm);
    WRITESHORT(RTG_COMMAND, RTGCMD_BLITRECT_NOMASK_COMPLETE);
#else
    IWRITECMD(RTGCMD_BLITRECT_NOMASK_COMPLETE);
#endif
}

void BlitTemplate (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGA2(struct Template *t), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD w), __REGD3(WORD h), __REGD4(UBYTE mask), __REGD7(RGBFTYPE format)) {
#ifndef IRTG
    if (!r || !t) return;
    if (w < 1 || h < 1) return;

    WRITELONG(RTG_ADDR2, (unsigned long)r->Memory);

    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[format]);
    WRITESHORT(RTG_X1, x);
    WRITESHORT(RTG_X2, w);
    WRITESHORT(RTG_X3, t->XOffset);
    WRITESHORT(RTG_Y1, y);
    WRITESHORT(RTG_Y2, h);
    WRITESHORT(RTG_Y3, 0);

    if ((unsigned long)t->Memory > CHIP_RAM_SIZE) {
        WRITELONG(RTG_ADDR1, (unsigned long)t->Memory);
    }
    else {
        unsigned long dest = CARD_SCRATCH;
        memcpy((unsigned char *)dest, t->Memory, (t->BytesPerRow * h));
        WRITELONG(RTG_ADDR1, (unsigned long)dest);
        WRITELONG(RTG_ADDR3, (unsigned long)t->Memory);
    }

    WRITELONG(RTG_RGB1, t->FgPen);
    WRITELONG(RTG_RGB2, t->BgPen);

    WRITESHORT(RTG_X4, r->BytesPerRow);
    WRITESHORT(RTG_X5, t->BytesPerRow);

    WRITEBYTE(RTG_U81, mask);
    WRITEBYTE(RTG_U82, t->DrawMode);
    WRITESHORT(RTG_COMMAND, RTGCMD_BLITTEMPLATE);
#else
    IWRITECMD(RTGCMD_BLITTEMPLATE);
#endif
}

void BlitPattern (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGA2(struct Pattern *p), __REGD0(WORD x), __REGD1(WORD y), __REGD2(WORD w), __REGD3(WORD h), __REGD4(UBYTE mask), __REGD7(RGBFTYPE format)) {
#ifndef IRTG
    if (!r || !p) return;
    if (w < 1 || h < 1) return;

    WRITELONG(RTG_ADDR2, (unsigned long)r->Memory);

    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[format]);
    WRITESHORT(RTG_X1, x);
    WRITESHORT(RTG_X2, w);
    WRITESHORT(RTG_X3, p->XOffset);
    WRITESHORT(RTG_Y1, y);
    WRITESHORT(RTG_Y2, h);
    WRITESHORT(RTG_Y3, p->YOffset);

    if ((unsigned long)p->Memory > CHIP_RAM_SIZE) {
        WRITELONG(RTG_ADDR1, (unsigned long)p->Memory);
    }
    else {
        unsigned long dest = CARD_SCRATCH;
        memcpy((unsigned char *)dest, p->Memory, (2 * (1 << p->Size)));
        WRITELONG(RTG_ADDR1, (unsigned long)dest);
    }

    WRITELONG(RTG_RGB1, p->FgPen);
    WRITELONG(RTG_RGB2, p->BgPen);

    WRITESHORT(RTG_X4, r->BytesPerRow);
    WRITESHORT(RTG_X5, (1 << p->Size));

    WRITEBYTE(RTG_U81, mask);
    WRITEBYTE(RTG_U82, p->DrawMode);
    WRITEBYTE(RTG_U83, (1 << p->Size));
    WRITESHORT(RTG_COMMAND, RTGCMD_BLITPATTERN);
#else
    IWRITECMD(RTGCMD_BLITPATTERN);
#endif
}

void DrawLine (__REGA0(struct BoardInfo *b), __REGA1(struct RenderInfo *r), __REGA2(struct Line *l), __REGD0(UBYTE mask), __REGD7(RGBFTYPE format)) {
#ifndef IRTG
    if (!r || !b) return;

    WRITELONG(RTG_ADDR1, (unsigned long)r->Memory);

    WRITELONG(RTG_RGB1, l->FgPen);
    WRITELONG(RTG_RGB2, l->BgPen);

    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[format]);

    WRITESHORT(RTG_X1, l->X);
    WRITESHORT(RTG_X2, l->dX);
    WRITESHORT(RTG_Y1, l->Y);
    WRITESHORT(RTG_Y2, l->dY);

    WRITESHORT(RTG_X3, l->Length);
    WRITESHORT(RTG_Y3, l->LinePtrn);

    WRITESHORT(RTG_X4, r->BytesPerRow);
    WRITESHORT(RTG_X5, l->PatternShift);

    WRITEBYTE(RTG_U81, mask);
    WRITEBYTE(RTG_U82, l->DrawMode);
    WRITEBYTE(RTG_U83, l->pad);
    WRITESHORT(RTG_COMMAND, RTGCMD_DRAWLINE);
#else
    IWRITECMD(RTGCMD_DRAWLINE);
#endif
}

void BlitPlanar2Chunky (__REGA0(struct BoardInfo *b), __REGA1(struct BitMap *bm), __REGA2(struct RenderInfo *r), __REGD0(SHORT x), __REGD1(SHORT y), __REGD2(SHORT dx), __REGD3(SHORT dy), __REGD4(SHORT w), __REGD5(SHORT h), __REGD6(UBYTE minterm), __REGD7(UBYTE mask)) {
// iRTG path disabled for now, since it's really slow, see note in rtg-gfx.c.
//#ifndef IRTG    
    if (!b || !r)
        return;

    //uint32_t plane_size = bm->BytesPerRow * bm->Rows;

    uint32_t template_addr = CARD_SCRATCH;

    uint16_t plane_mask = mask;
    uint8_t ff_mask = 0x00;
    uint8_t cur_plane = 0x01;

    uint16_t line_size = (w >> 3) + 2;
    uint32_t output_plane_size = line_size * h;
    //uint16_t x_offset = (x >> 3);

    WRITELONG(RTG_ADDR1, (unsigned long)r->Memory);
    WRITELONG(RTG_ADDR2, template_addr);
    WRITESHORT(RTG_X4, r->BytesPerRow);
    WRITESHORT(RTG_X5, line_size);
    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[r->RGBFormat]);

    WRITEBYTE(RTG_U81, mask);
    WRITEBYTE(RTG_U82, minterm);

    for (int16_t i = 0; i < bm->Depth; i++) {
        uint16_t x_offset = (x >> 3);
        if ((uint32_t)bm->Planes[i] == 0xFFFFFFFF) {
            uint8_t* dest = (uint8_t*)((uint32_t)template_addr);
            memset(dest, 0xFF, output_plane_size);
        }
        else if (bm->Planes[i] != NULL) {
            uint8_t* bmp_mem = (uint8_t*)bm->Planes[i] + (y * bm->BytesPerRow) + x_offset;
            uint8_t* dest = (uint8_t*)((uint32_t)template_addr);
            for (int16_t y_line = 0; y_line < h; y_line++) {
                memcpy(dest, bmp_mem, line_size);
                dest += line_size;
                bmp_mem += bm->BytesPerRow;
            }
        }
        else {
            plane_mask &= (cur_plane ^ 0xFF);
        }
        cur_plane <<= 1;
        template_addr += output_plane_size;
    }

    WRITESHORT(RTG_X1, (x & 0x07));
    WRITESHORT(RTG_X2, dx);
    WRITESHORT(RTG_X3, w);
    WRITESHORT(RTG_Y1, 0);
    WRITESHORT(RTG_Y2, dy);
    WRITESHORT(RTG_Y3, h);

    WRITESHORT(RTG_U1, (plane_mask << 8 | ff_mask));
    WRITEBYTE(RTG_U83, bm->Depth);

    WRITESHORT(RTG_COMMAND, RTGCMD_P2C);
//#else
//    IWRITECMD(RTGCMD_P2C);
//#endif
}

void BlitPlanar2Direct (__REGA0(struct BoardInfo *b), __REGA1(struct BitMap *bm), __REGA2(struct RenderInfo *r), __REGA3(struct ColorIndexMapping *clut), __REGD0(SHORT x), __REGD1(SHORT y), __REGD2(SHORT dx), __REGD3(SHORT dy), __REGD4(SHORT w), __REGD5(SHORT h), __REGD6(UBYTE minterm), __REGD7(UBYTE mask)) {
    if (!b || !r)
        return;

    //uint32_t plane_size = bm->BytesPerRow * bm->Rows;

    uint32_t template_addr = CARD_SCRATCH;

    uint16_t plane_mask = mask;
    uint8_t ff_mask = 0x00;
    uint8_t cur_plane = 0x01;

    uint16_t line_size = (w >> 3) + 2;
    uint32_t output_plane_size = line_size * h;
    //uint16_t x_offset = (x >> 3);

    WRITELONG(RTG_ADDR1, (unsigned long)r->Memory);
    WRITELONG(RTG_ADDR2, template_addr);
    WRITESHORT(RTG_X4, r->BytesPerRow);
    WRITESHORT(RTG_X5, line_size);
    WRITESHORT(RTG_FORMAT, rgbf_to_rtg[r->RGBFormat]);

    WRITEBYTE(RTG_U81, mask);
    WRITEBYTE(RTG_U82, minterm);

    memcpy((uint8_t*)((uint32_t)template_addr), clut->Colors, (256 << 2));
    template_addr += (256 << 2);

    for (int16_t i = 0; i < bm->Depth; i++) {
        uint16_t x_offset = (x >> 3);
        if ((uint32_t)bm->Planes[i] == 0xFFFFFFFF) {
            uint8_t* dest = (uint8_t*)((uint32_t)template_addr);
            memset(dest, 0xFF, output_plane_size);
        }
        else if (bm->Planes[i] != NULL) {
            uint8_t* bmp_mem = (uint8_t*)bm->Planes[i] + (y * bm->BytesPerRow) + x_offset;
            uint8_t* dest = (uint8_t*)((uint32_t)template_addr);
            for (int16_t y_line = 0; y_line < h; y_line++) {
                memcpy(dest, bmp_mem, line_size);
                dest += line_size;
                bmp_mem += bm->BytesPerRow;
            }
        }
        else {
            plane_mask &= (cur_plane ^ 0xFF);
        }
        cur_plane <<= 1;
        template_addr += output_plane_size;
    }

    WRITESHORT(RTG_X1, (x & 0x07));
    WRITESHORT(RTG_X2, dx);
    WRITESHORT(RTG_X3, w);
    WRITESHORT(RTG_Y1, 0);
    WRITESHORT(RTG_Y2, dy);
    WRITESHORT(RTG_Y3, h);

    WRITESHORT(RTG_U1, (plane_mask << 8 | ff_mask));
    WRITEBYTE(RTG_U83, bm->Depth);

    WRITESHORT(RTG_COMMAND, RTGCMD_P2D);
}

void SetSprite (__REGA0(struct BoardInfo *b), __REGD0(BOOL enable), __REGD7(RGBFTYPE format)) {
    WRITESHORT(RTG_U1, enable);
    WRITESHORT(RTG_COMMAND, RTGCMD_SETSPRITE);
}

void SetSpritePosition (__REGA0(struct BoardInfo *b), __REGD0(WORD x), __REGD1(WORD y), __REGD7(RGBFTYPE format)) {
    WRITESHORT(RTG_X1, x);
    WRITESHORT(RTG_Y1, y);

    WRITESHORT(RTG_COMMAND, RTGCMD_SETSPRITEPOS);
}

void SetSpriteImage (__REGA0(struct BoardInfo *b), __REGD7(RGBFTYPE format)) {
    WRITESHORT(RTG_X1, b->XOffset);
    WRITESHORT(RTG_Y1, b->YOffset);
    WRITEBYTE(RTG_U81, b->MouseWidth);
    WRITEBYTE(RTG_U82, b->MouseHeight);

    uint8_t* dest = (uint8_t*)((uint32_t)CARD_SCRATCH);
    uint8_t* src = (uint8_t *)b->MouseImage;
    uint16_t data_size = ((b->MouseWidth >> 3) * 2) * (b->MouseHeight);

    if (b->MouseWidth > 16) src += 8;
    else src += 4;

    memcpy(dest, src, data_size);

    WRITELONG(RTG_ADDR2, CARD_SCRATCH);

    WRITESHORT(RTG_COMMAND, RTGCMD_SETSPRITEIMAGE);
}

void SetSpriteColor (__REGA0(struct BoardInfo *b), __REGD0(UBYTE idx), __REGD1(UBYTE R), __REGD2(UBYTE G), __REGD3(UBYTE B), __REGD7(RGBFTYPE format)) {
    WRITEBYTE(RTG_U81, R);
    WRITEBYTE(RTG_U82, G);
    WRITEBYTE(RTG_U83, B);
    WRITEBYTE(RTG_U84, idx);

    WRITESHORT(RTG_COMMAND, RTGCMD_SETSPRITECOLOR);
}

static uint32_t device_vectors[] = {
    (uint32_t)OpenLib,
    (uint32_t)CloseLib,
    (uint32_t)ExpungeLib,
    0,
    (uint32_t)FindCard,
    (uint32_t)InitCard,
    -1
};

struct InitTable
{
    ULONG LibBaseSize;
    APTR  FunctionTable;
    APTR  DataTable;
    APTR  InitLibTable;
};

const uint32_t auto_init_tables[4] = {
    sizeof(struct GFXBase),
    (uint32_t)device_vectors,
    0,
    (uint32_t)InitLib,
};

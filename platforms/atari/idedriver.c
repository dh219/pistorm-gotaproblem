/*
 *	IDE Emulation Layer for retro-style PIO interfaces
 *
 *	(c) Copyright Alan Cox, 2015-2019
 *
 *	IDE-emu is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	IDE-emu is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with IDE-emu.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _LARGEFILE64_SOURCE 

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <arpa/inet.h>
//#include <endian.h>
#include <fcntl.h>

#include "config_file/config_file.h"
#include "idedriver.h"

#define IDE_IDLE	0
#define IDE_CMD		1
#define IDE_DATA_IN	2
#define IDE_DATA_OUT	3

#define DCR_NIEN 	2
#define DCR_SRST 	4

#define DEVH_HEAD	15
#define DEVH_DEV	16
#define DEVH_LBA	64

#define ERR_AMNF	1
#define ERR_TKNONF	2
#define ERR_ABRT	4
#define ERR_MCR		8
#define ERR_IDNF	16
#define	ERR_MC		32
#define ERR_UNC		64

#define ST_ERR		1
#define ST_IDX		2
#define ST_CORR		4
#define ST_DRQ		8
#define ST_DSC		16
#define ST_DF		32
#define ST_DRDY		64
#define ST_BSY		128

#define DCL_SRST	4
#define DCL_NIEN	2

#define IDE_CMD_CALIB		0x10
#define IDE_CMD_READ		0x20
#define IDE_CMD_READ_NR		0x21
#define IDE_CMD_WRITE		0x30
#define IDE_CMD_WRITE_NR	0x31
#define IDE_CMD_VERIFY		0x40
#define IDE_CMD_VERIFY_NR	0x41
#define IDE_CMD_SEEK		0x70
#define IDE_CMD_EDD		0x90
#define IDE_CMD_INTPARAMS	0x91
#define IDE_CMD_IDENTIFY	0xEC
#define IDE_CMD_SETFEATURES	0xEF

const uint8_t ide_magic[9] = {
  '1','D','E','D','1','5','C','0',0x00
};
/*
static char *charmap(uint8_t v)
{
  static char cbuf[3];
  if (v < 32)
    sprintf(cbuf, "^%c", '@'+v);
  else if (v < 127)
    sprintf(cbuf, " %c", v);
  else if (v == 127)
    sprintf(cbuf, "DL");
  else if (v < 160)
    sprintf(cbuf, ":%c", '@' + v - 128);
  else if (v < 255)
    sprintf(cbuf, "~%c", v - 128);
  else
    sprintf(cbuf, "!D");
  return cbuf;
}

static void hexdump(uint8_t *bp)
{
  int i,j;
  for (i = 0; i < 512; i+= 16) {
    for(j = 0; j < 16; j++)
      fprintf(stderr, "%02X ", bp[i+j]);
    fprintf(stderr, "|");
    for(j = 0; j < 16; j++)
      fprintf(stderr, "%2s", charmap(bp[i+j]));
    fprintf(stderr, "\n");
  }
}
*/
/* FIXME: use proper endian convertors! */
static uint16_t le16(uint16_t v)
{
  uint8_t *p = (uint8_t *)&v;
  return p[0] | (p[1] << 8);
}

static void ide_xlate_errno(struct ide_taskfile *t, int len)
{
  t->status |= ST_ERR;
  if (len == -1) {
    if (errno == EIO)
      t->error = ERR_UNC;
    else
      t->error = ERR_AMNF;
  } else
    t->error = ERR_AMNF;
}

static void ide_fault ( struct ide_drive *d, const char *p )
{
  printf ( "ide: %s: %d: %s\n", d->controller->name,
			(int)(d - d->controller->drive), p );
}

/* Disk translation */
static off_t xlate_block(struct ide_taskfile *t)
{
  struct ide_drive *d = t->drive;
  uint16_t cyl;

  if (d->controller->lba4 & DEVH_LBA) {
/*    fprintf(stderr, "XLATE LBA %02X:%02X:%02X:%02X\n",
      t->lba4, t->lba3, t->lba2, t->lba1);*/
    if (d->lba)
      return ((d->header_present) ? 2 : 0) + (((t->drive->controller->lba4 & DEVH_HEAD) << 24) | (t->drive->controller->lba3 << 16) | (t->drive->controller->lba2 << 8) | t->drive->controller->lba1);
    ide_fault(d, "LBA on non LBA drive");
  }

  /* Some well known software asks for 0/0/0 when it means 0/0/1. Drives appear
     to interpret sector 0 as sector 1 */
  if (t->drive->controller->lba1 == 0) {
    //fprintf(stderr, "[Bug: request for sector offset 0].\n");
    t->drive->controller->lba1 = 1;
  }
  cyl = (t->drive->controller->lba3 << 8) | t->drive->controller->lba2;
  /* fprintf(stderr, "(H %d C %d S %d)\n", t->lba4 & DEVH_HEAD, cyl, t->lba1); */
  if (t->drive->controller->lba1 == 0 || t->drive->controller->lba1 > d->sectors || t->drive->controller->lba4 >= d->heads || cyl >= d->cylinders) {
    return -1;
  }
  /* Sector 1 is first */
  /* Images generally go cylinder/head/sector. This also matters if we ever
     implement more advanced geometry setting */
  //off_t ret = ((d->header_present) ? 1 : -1) + ((cyl * d->heads) + (t->drive->controller->lba4 & DEVH_HEAD)) * d->sectors + t->drive->controller->lba1;
  //printf("Non-LBA xlate block %lX.\n", ret);
  //printf("Cyl: %d Heads: %d Sectors: %d\n", cyl, d->heads, d->sectors);
  //printf("LBA1: %.2X LBA2: %.2X LBA3: %.2X LBA4: %.2X\n", t->drive->controller->lba1, t->drive->controller->lba2, t->drive->controller->lba3, t->drive->controller->lba4);

  return ((d->header_present) ? 1 : -1) + ((cyl * d->heads) + (t->drive->controller->lba4 & DEVH_HEAD)) * d->sectors + t->drive->controller->lba1;
}

/* Indicate the drive is ready */
static void ready(struct ide_taskfile *tf)
{
  tf->status &= ~(ST_BSY|ST_DRQ);
  tf->status |= ST_DRDY;
  tf->drive->state = IDE_IDLE;
}

/* Return to idle state, completing a command */
static void completed(struct ide_taskfile *tf)
{
  ready(tf);
  tf->drive->intrq = 1;
}

static void drive_failed(struct ide_taskfile *tf)
{
  tf->status |= ST_ERR;
  tf->error = ERR_IDNF;
  ready(tf);
}

static void data_in_state(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  d->state = IDE_DATA_IN;
  d->dptr = d->data + 512;
  /* We don't clear DRDY here, drives may well accept a command at this
     point and at least one firmware for RC2014 assumes this */
  tf->status &= ~ST_BSY;
  tf->status |= ST_DRQ;
  d->intrq = 1;			/* Double check */
}

static void data_out_state(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  d->state = IDE_DATA_OUT;
  d->dptr = d->data;
  tf->status &= ~ (ST_BSY|ST_DRDY);
  tf->status |= ST_DRQ;
  d->intrq = 1;			/* Double check */
}

static void edd_setup(struct ide_taskfile *tf)
{
  tf->error = 0x01;		/* All good */
  tf->drive->controller->lba1 = 0x01;		/* EDD always updates drive 0 */
  tf->drive->controller->lba2 = 0x00;
  tf->drive->controller->lba3 = 0x00;
  tf->drive->controller->lba4 = 0x00;
  tf->count = 0x01;
  ready(tf);
}

void ide_reset(struct ide_controller *c)
{
  if (c->drive[0].present) {
    edd_setup(&c->drive[0].taskfile);
    /* A drive could clear busy then set DRDY up to 2 minutes later if its
       mindnumbingly slow to start up ! We don't emulate any of that */
    c->drive[0].taskfile.status = ST_DRDY;
    c->drive[0].eightbit = 0;
  }
  if (c->drive[1].present) {
    edd_setup(&c->drive[1].taskfile);
    c->drive[1].taskfile.status = ST_DRDY;
    c->drive[1].eightbit = 0;
  }
  if (c->selected != 0) {
  }
  c->selected = 0;
}

void IDE_reset_begin(struct ide_controller *c)
{
  if (c->drive[0].present)
    c->drive[0].taskfile.status |= ST_BSY;
  if (c->drive[1].present)
    c->drive[1].taskfile.status |= ST_BSY;
  /* Ought to be a time delay relative to reset or power on */
  ide_reset(c);
}

static void ide_srst_begin(struct ide_controller *c)
{
  ide_reset(c);
  if (c->drive[0].present)
    c->drive[0].taskfile.status |= ST_BSY;
  if (c->drive[1].present)
    c->drive[1].taskfile.status |= ST_BSY;
}

static void ide_srst_end(struct ide_controller *c)
{
  /* Could be time delays here */
  ready(&c->drive[0].taskfile);
  ready(&c->drive[1].taskfile);
}

static void cmd_edd_complete(struct ide_taskfile *tf)
{
  struct ide_controller *c = tf->drive->controller;
  if (c->drive[0].present)
    edd_setup(&c->drive[0].taskfile);
  if (c->drive[1].present)
    edd_setup(&c->drive[1].taskfile);
  c->selected = 0;
}

static void cmd_identify_complete(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  memcpy(d->data, d->identify, 512);
  data_in_state(tf);
  /* Arrange to copy just the identify buffer */
  d->dptr = d->data;
  d->length = 1;

  //hexdump(d->data);
}

static void cmd_initparam_complete(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  /* We only support the current mapping */
  if (tf->count != d->sectors || (tf->drive->controller->lba4 & DEVH_HEAD) + 1 != d->heads) {
    tf->status |= ST_ERR;
    tf->error |= ERR_ABRT;
    tf->drive->failed = 1;		/* Report ID NF until fixed */
/*    fprintf(stderr, "geo is %d %d, asked for %d %d\n",
      d->sectors, d->heads, tf->count, (tf->lba4 & DEVH_HEAD) + 1); */
    ide_fault(d, "invalid geometry");
  } else if (tf->drive->failed == 1)
    tf->drive->failed = 0;		/* Valid translation */
  completed(tf);
}

static void cmd_readsectors_complete(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  /* Move to data xfer */
  if (d->failed) {
    drive_failed(tf);
    return;
  }
  d->offset = xlate_block(tf);
  /* DRDY is not guaranteed here but at least one buggy RC2014 firmware
     expects it */
  tf->status |= ST_DRQ | ST_DSC | ST_DRDY;
  tf->status &= ~ST_BSY;
  /* 0 = 256 sectors */
  d->length = tf->count ? tf->count : 256;
  /* fprintf(stderr, "READ %d SECTORS @ %ld\n", d->length, d->offset); */
  if (d->offset == -1 ||  lseek(d->fd, 512 * d->offset, SEEK_SET) == -1) {
    tf->status |= ST_ERR;
    tf->status &= ~ST_DSC;
    tf->error |= ERR_IDNF;
    /* return null data */
    completed(tf);
    return;
  }
  /* do the xfer */
  data_in_state(tf);
}

static void cmd_verifysectors_complete(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  /* Move to data xfer */
  if (d->failed) {
    drive_failed(tf);
    return;
  }
  d->offset = xlate_block(tf);
  /* 0 = 256 sectors */
  d->length = tf->count ? tf->count : 256;
  if (d->offset == -1 || lseek(d->fd, 512 * (d->offset + d->length - 1), SEEK_SET) == -1) {
    tf->status &= ~ST_DSC;
    tf->status |= ST_ERR;
    tf->error |= ERR_IDNF;
  }
  tf->status |= ST_DSC;
  completed(tf);
}

static void cmd_recalibrate_complete(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  if (d->failed)
    drive_failed(tf);
  if (d->offset == -1 || xlate_block(tf) != 0L) {
    tf->status &= ~ST_DSC;
    tf->status |= ST_ERR;
    tf->error |= ERR_ABRT;
  }
  tf->status |= ST_DSC;
  completed(tf);
}

static void cmd_seek_complete(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  if (d->failed)
    drive_failed(tf);
  d->offset = xlate_block(tf);
  if (d->offset == -1 || lseek(d->fd, 512 * d->offset, SEEK_SET) == -1) {
    tf->status &= ~ST_DSC;
    tf->status |= ST_ERR;
    tf->error |= ERR_IDNF;
  }
  tf->status |= ST_DSC;
  completed(tf);
}

static void cmd_setfeatures_complete(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  switch(tf->feature) {
    case 0x01:
      d->eightbit = 1;
      break;
    case 0x03:
      if ((tf->count & 0xF0) >= 0x20) {
        tf->status |= ST_ERR;
        tf->error |= ERR_ABRT;
      }
      /* Silently accept PIO mode settings */
      break;
    case 0x81:
      d->eightbit = 0;
      break;
    default:
      tf->status |= ST_ERR;
      tf->error |= ERR_ABRT;
  }
  completed(tf);
}

static void cmd_writesectors_complete(struct ide_taskfile *tf)
{
  struct ide_drive *d = tf->drive;
  /* Move to data xfer */
  if (d->failed) {
    drive_failed(tf);
    return;
  }
  d->offset = xlate_block(tf);
  tf->status |= ST_DRQ;
  /* 0 = 256 sectors */
  d->length = tf->count ? tf->count : 256;
/*  fprintf(stderr, "WRITE %d SECTORS @ %ld\n", d->length, d->offset); */
  if (d->offset == -1 ||  lseek(d->fd, 512 * d->offset, SEEK_SET) == -1) {
    tf->status |= ST_ERR;
    tf->error |= ERR_IDNF;
    tf->status &= ~ST_DSC;
    /* return null data */
    completed(tf);
    return;
  }
  /* do the xfer */
  data_out_state(tf);
}

static void ide_set_error(struct ide_drive *d)
{
  if ( d->lba )
    d->controller->lba4 &= ~DEVH_HEAD;

  if (d->controller->lba4 & DEVH_LBA) {
    d->controller->lba1 = d->offset & 0xFF;
    d->controller->lba2 = (d->offset >> 8) & 0xFF;
    d->controller->lba3 = (d->offset >> 16) & 0xFF;

    if ( d->lba )
      d->controller->lba4 |= (d->offset >> 24) & DEVH_HEAD;

  } 
  
  else 
  {
    d->controller->lba1 = d->offset % d->sectors + 1;
    d->offset /= d->sectors;

    if ( d->lba )
      d->controller->lba4 |= d->offset / (d->cylinders * d->sectors);

    d->offset %= (d->cylinders * d->sectors);
    d->controller->lba2 = d->offset & 0xFF;
    d->controller->lba3 = (d->offset >> 8) & 0xFF;
  }
  d->taskfile.count = d->length;
  d->taskfile.status |= ST_ERR;
  d->state = IDE_IDLE;
  completed(&d->taskfile);
}

static int ide_read_sector(struct ide_drive *d)
{
  int len;

  d->dptr = d->data;
  if ((len = read(d->fd, d->data, 512)) != 512) {
    //perror("ide_read_sector");
    //printf ("%s: read failed - cnt %d\n", __func__, len );
    d->taskfile.status |= ST_ERR;
    d->taskfile.status &= ~ST_DSC;
    ide_xlate_errno(&d->taskfile, len);
    return -1;
  }
  //hexdump(d->data);
  d->offset += 512;
  return 0;
}

static int ide_write_sector(struct ide_drive *d)
{
  int len;

  d->dptr = d->data;
  if ((len = write(d->fd, d->data, 512)) != 512) {
    d->taskfile.status |= ST_ERR;
    d->taskfile.status &= ~ST_DSC;
    ide_xlate_errno(&d->taskfile, len);
    return -1;
  }
//  hexdump(d->data);
  d->offset += 512;
  return 0;
}


static uint16_t ide_data_in(struct ide_drive *d, int len)
{
  uint16_t v;
  //printf ("%s: d->state = %d, d->data = 0x%x, d->dptr = 0x%x\n", __func__, d->state, d->data, d->dptr );
  if (d->state == IDE_DATA_IN) 
  {
    if (d->dptr == d->data + 512) 
    {
      //printf ("%s: 2\n", __func__ );
      if (ide_read_sector(d) < 0) 
      {
        ide_set_error(d);	/* Set the LBA or CHS etc */
        return 0xFFFF;		/* and error bits set by read_sector */
      }
    }

    //printf ("%s: 3\n", __func__ );

    v = *d->dptr;
    if (!d->eightbit) {
      if (len == 2)
        v |= (d->dptr[1] << 8);
        //v = v << 8 | d->dptr[1]; // cryptodad byte swapped or not?
      d->dptr+=2;
    } else
      d->dptr++;
    d->taskfile.data = v;
    if (d->dptr == d->data + 512) {
      d->length--;
      d->intrq = 1;		/* we don't yet emulate multimode */
      if (d->length == 0) {
        d->state = IDE_IDLE;
        completed(&d->taskfile);
      }
    }
  } else
  {
    //ide_fault(d, "bad data read"); // don't show this - some tests (APT94.PRG) do a 0 byte io
  }

  if (len == 1)
    return d->taskfile.data & 0xFF;
  return d->taskfile.data;
}

static void ide_data_out(struct ide_drive *d, uint16_t v, int len)
{
  (void)len;
  if (d->state != IDE_DATA_OUT) {
    //ide_fault(d, "bad data write");
    d->taskfile.data = v;
  } else {
    if (d->eightbit)
      v &= 0xFF;
    *d->dptr++ = v;
    d->taskfile.data = v;
    if (!d->eightbit) {
      *d->dptr++ = v >> 8;
      d->taskfile.data = v >> 8;
    }
    if (d->dptr == d->data + 512) {
      if (ide_write_sector(d) < 0) {
        ide_set_error(d);
        return;
      }
      d->length--;
      d->intrq = 1;
      if (d->length == 0) {
        d->state = IDE_IDLE;
        d->taskfile.status |= ST_DSC;
        completed(&d->taskfile);
      }
    }
  }
}

static void ide_issue_command(struct ide_taskfile *t)
{
  t->status &= ~(ST_ERR|ST_DRDY);
  t->status |= ST_BSY;
  t->error = 0;
  t->drive->state = IDE_CMD;

  /* We could complete with delays but don't do so yet */
  switch(t->command) {
    case IDE_CMD_EDD:	/* 0x90 */
      cmd_edd_complete(t);
      break;
    case IDE_CMD_IDENTIFY:	/* 0xEC */
      cmd_identify_complete(t);

      
      break;
    case IDE_CMD_INTPARAMS:	/* 0x91 */
      cmd_initparam_complete(t);
      break;
    case IDE_CMD_READ:		/* 0x20 */
    case IDE_CMD_READ_NR:	/* 0x21 */
      cmd_readsectors_complete(t);
      break;
    case IDE_CMD_SETFEATURES:	/* 0xEF */
      cmd_setfeatures_complete(t);
      break;
    case IDE_CMD_VERIFY:	/* 0x40 */
    case IDE_CMD_VERIFY_NR:	/* 0x41 */
      cmd_verifysectors_complete(t);
      break;
    case IDE_CMD_WRITE:		/* 0x30 */
    case IDE_CMD_WRITE_NR:	/* 0x31 */
      cmd_writesectors_complete(t);
      break;
    default:
      if ((t->command & 0xF0) == IDE_CMD_CALIB)	/* 1x */
        cmd_recalibrate_complete(t);
      else if ((t->command & 0xF0) == IDE_CMD_SEEK) /* 7x */
        cmd_seek_complete(t);
      else {
        /* Unknown */
        t->status |= ST_ERR;
        t->error |= ERR_ABRT;
        completed(t);
      }
  }
}


/*
 *	8bit IDE controller emulation
 */

uint8_t IDE_read8 ( struct ide_controller *c, uint8_t r) 
{
  struct ide_drive *d = &c->drive[c->selected];
  struct ide_taskfile *t = &d->taskfile;

  switch ( r ) 
  {
    case IDE_data:
        //printf ( "%s: IDE_data\n", __func__ );
      return ide_data_in ( d, 1 );
    case IDE_error_r:
      return t->error;
    case IDE_sec_count:
      return t->count;
    case IDE_lba_low:
      return c->lba1;
    case IDE_lba_mid:
      return c->lba2;
    case IDE_lba_hi:
      return c->lba3;
    case IDE_lba_top:
      return c->lba4 | ((c->selected) ? 0x10 : 0x00);
    case IDE_status_r:
      d->intrq = 0;		/* Acked */
      /* Fallthrough */
      /* no break */
    case IDE_altst_r:
      return t->status;
    default:
      ide_fault(d, "bogus register");
      return 0xFF;
  }
}


void IDE_write8(struct ide_controller *c, uint8_t r, uint8_t v)
{
  struct ide_drive *d = &c->drive[c->selected];
  struct ide_taskfile *t = &d->taskfile;

  if (r != IDE_devctrl_w) {
    if (t->status & ST_BSY) {
      ide_fault(d, "command written while busy");
      return;
    }
    /* Not clear this is the right emulation */
    if (d->present == 0 && r != IDE_lba_top) {
      //ide_fault(d, "not present");
      return;
    }
  }

//  uint8_t ve;

  switch(r) {
    case IDE_data:
      ide_data_out(d, v, 1);
      break;
    case IDE_feature_w:
      t->feature = v;
      break;
    case IDE_sec_count:
      t->count = v;
      break;
    case IDE_lba_low:
      c->lba1 = v;
      break;
    case IDE_lba_mid:
      c->lba2 = v;
      break;
    case IDE_lba_hi:
      c->lba3 = v;
      break;
    case IDE_lba_top:
      c->selected = (v & DEVH_DEV) ? 1 : 0;

      if ( d->lba ) 
        c->lba4 = v & (DEVH_HEAD|/*DEVH_DEV|*/DEVH_LBA);

      break;
    case IDE_command_w:
      t->command = v;
      ide_issue_command(t);
      break;
    case IDE_devctrl_w:
      /* ATA: "When the Device Control register is written, both devices
         respond to the write regardless of which device is selected" */
      if ((v ^ t->devctrl) & DCL_SRST) {
        if (v & DCL_SRST)
          ide_srst_begin(c);
        else
          ide_srst_end(c);
      }
      c->drive[0].taskfile.devctrl = v;	/* Check versus real h/w does this end up cleared */
      c->drive[1].taskfile.devctrl = v;
      break;
  }
}


/*
 *	16bit IDE controller emulation
 */

uint16_t IDE_read16 ( struct ide_controller *c, uint8_t r )
{
  struct ide_drive *d = &c->drive [c->selected];

  if ( r == IDE_data )
  {
    return htons ( ide_data_in ( d, 2 ) );
  }

  return IDE_read8 ( c, r );
}


void IDE_write16(struct ide_controller *c, uint8_t r, uint16_t v)
{
  struct ide_drive *d = &c->drive[c->selected];
  struct ide_taskfile *t = &d->taskfile;

  if (r != IDE_devctrl_w && (t->status & ST_BSY)) {
    ide_fault(d, "command written while busy");
    return;
  }
  if (r == IDE_data)
    ide_data_out(d, ntohs(v), 2);
  else
    IDE_write8(c, r, v);
}


/*
 *	Allocate a new IDE controller emulation
 */
struct ide_controller *IDE_allocate(const char *name)
{
  struct ide_controller *c = calloc(1, sizeof(*c));
  if (c == NULL)
    return NULL;
  c->name = strdup(name);
  if (c->name == NULL) {
    free(c);
    return NULL;
  }
  c->drive[0].controller = c;
  c->drive[1].controller = c;
  c->drive[0].taskfile.drive = &c->drive[0];
  c->drive[1].taskfile.drive = &c->drive[1];
  return c;
}

#if (0)
/*
 *	Attach a file to a device on the controller
 */
int IDE_attach(struct ide_controller *c, int drive, int fd)
{
  struct ide_drive *d = &c->drive[drive];
  if (d->present) {
    ide_fault(d, "double attach");
    return -1;
  }
  d->fd = fd;
  
  if (read(d->fd, d->data, 512) != 512 ||
      read(d->fd, d->identify, 512) != 512) {
    ide_fault(d, "i/o error on attach");
    return -1;
  }
  //if (memcmp(d->data, ide_magic, 8)) {
  //  ide_fault(d, "bad magic");
  //  return -1;
  //}


  d->fd = fd;
  d->present = 1;
  d->heads = d->identify[3];
  d->sectors = d->identify[6];
  d->cylinders = le16(d->identify[1]);
printf ( "[HDD%d] cyl: %d, hds: %d, sec: %d\n", drive, d->cylinders, d->heads, d->sectors );

  d->header_present = 1;
  if (d->identify[49] & le16(1 << 9))
    d->lba = 1;
  else
    d->lba = 0;
  return 0;
}
#endif

/* cryptodad */
/* attach a floppy disk image .ST */
int ide_attach_st (struct ide_controller *c, int drive, int fd)
{
  struct ide_drive *d = &c->drive [drive & 1];

  if (d->present) 
  {
    //DEBUG_PRINTF ("[IDE/FDD] Drive already attached.\n");
    return -1;
  }

  d->fd = fd;
  d->present = 1;
  d->lba = 0;
  d->header_present = 0;

  d->heads = 1;
  d->sectors = 9;
  d->cylinders = 79;

  d->controller->lba4 = 0;

  uint64_t file_size = lseek(fd, 0, SEEK_END);
  lseek(fd, 0, SEEK_SET);

  if (file_size > 720 * 1024) {
    //printf ( "[IDE/FDD] Floppy image is too big\n" );
    return -1;
  }

  ide_make_ident(drive, d->cylinders, d->heads, d->sectors, "PISTORM IDE FD", d->identify);

  return 0;
}




// Attach a headerless HDD image to the controller
// ATA defines limitations for capacity to 8GB
int ide_attach_hdf ( struct ide_controller *c, int drive, int fd )
{
  struct ide_drive *d = &c->drive [drive & 1];

  if ( d->present ) 
  {
    //DEBUG_PRINTF ("[IDE/HDL] Drive already attached.\n");
    return -1;
  }

  d->fd = fd;
  d->present = 1;
  d->lba = 1;

  d->heads = 16;
  d->sectors = 63;
  d->header_present = 0;

  off64_t file_size = lseek64 ( fd, 0, SEEK_END );
  lseek64 ( fd, 0, SEEK_SET );

  if (file_size < 4 * 1000 * 1000) {
    //printf ( "[IDE/HDL] File size is too small. Image must be > 4 MB\n" );
    return -1;
  }

  /* if drive < 528 MB word[1] (default cylinders) <= 1024 */
  /* word[3] <= 16 */
  /*
  if ( file_size < (528 * 1000 * 1000) ) 
  {
    d->cylinders = (file_size / (d->sectors * d->heads) / 512) + 1;
  }

  else if ( file_size < (1056 * 1000 * 1000) ) 
  {
    d->heads = 32;
  }

  else if ( file_size < (2112 * 1000 * 1000) ) 
  {
    d->heads = 64;
  }

  else if ( file_size < (4224 * 1000 * 1000) ) 
  {
    d->heads = 128;
  }
  */
  d->cylinders = (file_size / 512) / (d->sectors * d->heads);
  

  //printf ("[IDE/HDL] Cylinders: %d Heads: %d Sectors: %d - filesize = %llu\n", d->cylinders, d->heads, d->sectors, file_size );


  ide_make_ident ( drive, d->cylinders, d->heads, d->sectors, "PISTORM IDE DK", d->identify );

  return 0;
}


/*
 *	Detach an IDE device from the interface (not hot pluggable)
 */
void ide_detach(struct ide_drive *d)
{
  close(d->fd);
  d->fd = -1;
  d->present = 0;
}


/*
 *	Free up and release and IDE controller
 */
void ide_free(struct ide_controller *c)
{
  if (c->drive[0].present)
    ide_detach(&c->drive[0]);
  if (c->drive[1].present)
    ide_detach(&c->drive[1]);
  free((void *)c->name);
  free(c);
}


/*
 *	Emulation interface for an 8bit controller using latches on the
 *	data register
 */
uint8_t ide_read_latched(struct ide_controller *c, uint8_t reg)
{
  uint16_t v;
  if (reg == IDE_data_latch)
    return c->data_latch;
  v = IDE_read16(c, reg);
  if (reg == IDE_data) {
    c->data_latch = v >> 8;
    v &= 0xFF;
  }
  return v;
}


void ide_write_latched ( struct ide_controller *c, uint8_t reg, uint8_t v )
{
  uint16_t d = v;

  if (reg == IDE_data_latch) {
    c->data_latch = v;
    return;
  }
  if (reg == IDE_data)
    d |=  (c->data_latch << 8);
  IDE_write16(c, reg, d);
}


static void make_ascii(uint16_t *p, const char *t, int len)
{
  int i;
  char *d = (char *)p;
//  strncpy(d, t, len);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
  strncpy(d, t, len);
#pragma GCC diagnostic pop

  //for (i = 0; i < len; i += 2) {
  //  char c = *d;
    //*d = d[1];
    //d[1] = c;
    //d[0] = d[0];
    //d[1] = d[1];
  //  d += 2;
  //}
}


static void make_serial(uint16_t *p)
{
  char buf[21];
  srand(getpid()^time(NULL));
  snprintf(buf, 21, "%08d%08d%04d", rand(), rand(), rand());
  make_ascii(p, buf, 20);
}






#define FW_VERSION "1.0"

static void put_le16(uint16_t *p, unsigned int v)
{
    *p = v;
}

static void padstr(char *str, const char *src, int len)
{
	int i, v;
	for (i = 0; i < len; i++)
	{
		if (*src)
			v = *src++;
		else
			v = ' ';
		str[i^1] = v;
	}
}


int ide_make_ident ( int drive, uint16_t c, uint8_t h, uint8_t s, char *name, uint16_t *target )
{
	uint32_t oldsize;
	char buf[40];
  uint16_t *p = target;
  uint32_t sectors;

  memset ( p, 0, 512 );
  oldsize = c * h * s;

	
	put_le16(p + 0, (1 << 15) | (1 << 6));//0x0040);
	put_le16(p + 1, c);
	put_le16(p + 3, h);
	
	put_le16(p + 6, s);
	snprintf(buf, sizeof(buf), "QM%05d", 12345 );
	
  strncpy((char*)(p + 10), buf, 20);

	put_le16(p + 21, 512); /* cache size in sectors */
	put_le16(p + 22, 4); /* ecc bytes */
	
  strncpy((char*)(p + 23), FW_VERSION, 8);
	/* Use the same convention for the name as SCSI disks are using: The
	 * first 8 characters should be the vendor, i.e. use 2 spaces here */
	snprintf(buf, sizeof(buf), "%s%d %liM", name, drive,
            (long)( oldsize / (1024 * 1024 / 512 )));
	
  strncpy((char*)(p + 27), buf, 40);

#if MAX_MULT_SECTORS > 1
	put_le16(p + 47, 0x8000 | 1);//MAX_MULT_SECTORS);
#endif

	put_le16(p + 48, 1); /* dword I/O */
	put_le16(p + 49, (1 << 9)); //(1 << 11) | (1 << 9) | (1 << 8)); /* DMA and LBA supported */
	put_le16(p + 51, (240 << 8)); //0x200); /* PIO transfer cycle */
	put_le16(p + 52, 0x200); /* DMA transfer cycle */
	put_le16(p + 53, 1);// | (1 << 1) | (1 << 2)); /* words 54-58,64-70,88 are valid */
  
	put_le16(p + 54, c);
	put_le16(p + 55, h);
	put_le16(p + 56, s);

  sectors = c * h * s;

	put_le16(p + 57, sectors & 0xffff);
	put_le16(p + 58, sectors >> 16);

	*(p + 60) = *(p + 57);
	*(p + 61) = *(p + 58);


  return 0;
}


int IDE_make_drive(uint8_t type, int fd)
{
  uint8_t s, h;
  uint16_t c;
  uint32_t sectors;
  uint16_t ident[256];

  if (type < 1 || type > MAX_DRIVE_TYPE)
    return -2;

  memset(ident, 0, 512);
  memcpy(ident, ide_magic, 8);
  if (write(fd, ident, 512) != 512)
    return -1;

  memset(ident, 0, 8);
  ident[0] = le16((1 << 15) | (1 << 6));	/* Non removable */
  make_serial(ident + 10);
  ident[47] = 0; /* no read multi for now */
  ident[51] = le16(240 /* PIO2 */ << 8);	/* PIO cycle time */
  ident[53] = le16(1);		/* Geometry words are valid */

  switch(type) {
    case ACME_ROADRUNNER:
      /* 504MB drive with LBA support */
      c = 1024;
      h = 16;
      s = 63;
      make_ascii(ident + 23, "A001.001", 8);
      make_ascii(ident + 27, "ACME ROADRUNNER v0.1", 40);
      ident[49] = le16(1 << 9); /* LBA */
      break;
    case ACME_ULTRASONICUS:
      /* 40MB drive with LBA support */
      c = 977;
      h = 5;
      s = 16;
      ident[49] = le16(1 << 9); /* LBA */
      make_ascii(ident + 23, "A001.001", 8);
      make_ascii(ident + 27, "ACME ULTRASONICUS AD INFINITUM v0.1", 40);
      break;
    case ACME_NEMESIS:
      /* 20MB drive with LBA support */
      c = 615;
      h = 4;
      s = 16;
      ident[49] = le16(1 << 9); /* LBA */
      make_ascii(ident + 23, "A001.001", 8);
      make_ascii(ident + 27, "ACME NEMESIS RIDICULII v0.1", 40);
      break;
    case ACME_COYOTE:
      /* 20MB drive without LBA support */
      c = 615;
      h = 4;
      s = 16;
      make_ascii(ident + 23, "A001.001", 8);
      make_ascii(ident + 27, "ACME COYOTE v0.1", 40);
      break;
    case ACME_ACCELLERATTI:
      c = 1024;
      h = 16;
      s = 16;
      ident[49] = le16(1 << 9); /* LBA */
      make_ascii(ident + 23, "A001.001", 8);
      make_ascii(ident + 27, "ACME ACCELLERATTI INCREDIBILUS v0.1", 40);
      break;
    case ACME_ZIPPIBUS:
      c = 1024;
      h = 16;
      s = 32;
      ident[49] = le16(1 << 9); /* LBA */
      make_ascii(ident + 23, "A001.001", 8);
      make_ascii(ident + 27, "ACME ZIPPIBUS v0.1", 40);
      break;
  }
  ident[1] = le16(c);
  ident[3] = le16(h);
  ident[6] = le16(s);
  ident[54] = ident[1];
  ident[55] = ident[3];
  ident[56] = ident[6];
  sectors = c * h * s;
  ident[57] = le16(sectors & 0xFFFF);
  ident[58] = le16(sectors >> 16);
  ident[60] = ident[57];
  ident[61] = ident[58];
  if (write(fd, ident, 512) != 512)
    return -1;

  memset(ident, 0xE5, 512);
  while(sectors--)
    if (write(fd, ident, 512) != 512)
      return -1;
  return 0;
}
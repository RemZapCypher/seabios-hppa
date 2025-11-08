// NCR 53c710 SCSI definitions
//
// Copyright (C) 2025 Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
//
// This driver was developed as part of the Google Summer of Code 2025
// program for the SeaBIOS-hppa firmware of the QEMU project.
// Under mentorship of Helge Deller <deller@gmx.de>.
//
// Based on the lsi-scsi.c, but hacked! Not to support PCI device.
// This is the bios side for the LASI's NCR53C710 SCSI Controller for QEMU.
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_GLOBALFLAT
#include "block.h" // struct drive_s
#include "blockcmd.h" // scsi_drive_setup
#include "config.h" // CONFIG_*
#include "fw/paravirt.h" // runningOnQEMU
#include "malloc.h" // free
#include "output.h" // dprintf
#include "parisc/hppa_hardware.h" // LASI_SCSI_HPA
#include "stacks.h" // run_thread
#include "std/disk.h" // DISK_RET_SUCCESS
#include "string.h" // memset
#include "util.h" // usleep

extern void flush_data_cache(char *start, size_t length);

#define ENABLE_DEBUG 0
#if ENABLE_DEBUG
#define DBG(x)          x
#else
#define DBG(x)          do { } while (0)
#endif

// PA-RISC is big-endian, but NCR710 registers are little-endian.
// We follow the same  endianness conversion that is done in the Linux kernel.
// For byte accesses on big-endian systems, XOR the register address with 3.
// This converts: BE -> LE NCR710 registers
#define bE 3

#define NCR_REG_SCNTL0    0x00
#define NCR_REG_SCNTL1    0x01
#define NCR_REG_SCID      0x04
#define NCR_REG_SXFER     0x05
#define NCR_REG_DSTAT     0x0C
#define NCR_REG_SSTAT0    0x0D
#define NCR_REG_SSTAT1    0x0E
#define NCR_REG_ISTAT     0x21
#define NCR_REG_CTEST8    0x22
#define NCR_REG_DSP0      0x2C
#define NCR_REG_DSP1      0x2D
#define NCR_REG_DSP2      0x2E
#define NCR_REG_DSP3      0x2F
#define NCR_REG_DSPS      0x30
#define NCR_REG_DCNTL     0x3B

// Helper macros for register access with endianness conversion
#define NCR_READ_REG(iobase, reg)  inb((iobase) + ((reg) ^ bE))
#define NCR_WRITE_REG(iobase, reg, val)  outb((val), (iobase) + ((reg) ^ bE))

#define NCR_DSTAT_SIR     0x04
#define NCR_ISTAT_RST     0x40

#define LASI_SCSI_CORE_OFFSET 0x100

struct ncr_lun_s {
    struct drive_s drive;
    u32 iobase;
    u8 target;
    u8 lun;
};

static void
ncr710_reset(u32 iobase)
{
    NCR_WRITE_REG(iobase, NCR_REG_ISTAT, NCR_ISTAT_RST);
    usleep(25000);
    NCR_WRITE_REG(iobase, NCR_REG_ISTAT, 0);
    usleep(5000);

    NCR_WRITE_REG(iobase, NCR_REG_SCID, 0x07);
    NCR_WRITE_REG(iobase, NCR_REG_SXFER, 0x00);
    NCR_WRITE_REG(iobase, NCR_REG_DCNTL, 0x40);
}

int
ncr710_scsi_process_op(struct disk_op_s *op)
{
    if (!CONFIG_NCR710_SCSI) {
        return DISK_RET_EBADTRACK;
    }
    struct ncr_lun_s *llun_gf =
        container_of(op->drive_fl, struct ncr_lun_s, drive);
    u16 target = GET_GLOBALFLAT(llun_gf->target);
    u8 cdbcmd[16];
    int blocksize = scsi_fill_cmd(op, cdbcmd, sizeof(cdbcmd));
    if (blocksize < 0) {
        return default_process_op(op);
    }
    u32 iobase = GET_GLOBALFLAT(llun_gf->iobase);
    u32 dma = ((scsi_is_read(op) ? 0x01000000 : 0x00000000) |
               (op->count * blocksize));
    u8 status = 0xff;
    u8 msgin = 0xff;

    u32 script[12] __attribute__((aligned(4))) = {
        0x40000000 | (1 << target) << 16, /* SELECT target */
        0x00000000,
        0x02000010,                        /* Send CDB (16 bytes) */
        (u32)MAKE_FLATPTR(GET_SEG(SS), cdbcmd),
        dma,                               /* DATA IN/OUT transfer */
        (u32)op->buf_fl,
        0x03000001,                        /* Receive STATUS (1 byte) */
        (u32)MAKE_FLATPTR(GET_SEG(SS), &status),
        0x07000001,                        /* Receive MESSAGE IN (1 byte) */
        (u32)MAKE_FLATPTR(GET_SEG(SS), &msgin),
        0x98080000,                        /* INT with success code */
        0x00000401,
    };
    u32 dsp = (u32)MAKE_FLATPTR(GET_SEG(SS), &script);
    flush_data_cache((char *)&script, sizeof(script));
    flush_data_cache((char *)cdbcmd, sizeof(cdbcmd));
    flush_data_cache((char *)&status, sizeof(status));
    flush_data_cache((char *)&msgin, sizeof(msgin));
    if (op->buf_fl && op->count * blocksize > 0) {
        /* For write operations, flush the data buffer */
        if (!scsi_is_read(op)) {
            flush_data_cache((char *)op->buf_fl, op->count * blocksize);
        }
    }
    NCR_WRITE_REG(iobase, NCR_REG_DSP0, dsp & 0xff);
    NCR_WRITE_REG(iobase, NCR_REG_DSP1, (dsp >> 8) & 0xff);
    NCR_WRITE_REG(iobase, NCR_REG_DSP2, (dsp >> 16) & 0xff);
    NCR_WRITE_REG(iobase, NCR_REG_DSP3, (dsp >> 24) & 0xff);

    int poll_count = 0;
    for (;;) {
        poll_count++;
        u8 istat = NCR_READ_REG(iobase, NCR_REG_ISTAT);
        u8 dstat = NCR_READ_REG(iobase, NCR_REG_DSTAT);
        u8 sstat0 = NCR_READ_REG(iobase, NCR_REG_SSTAT0);
        u8 sstat1 = NCR_READ_REG(iobase, NCR_REG_SSTAT1);
        if (dstat & NCR_DSTAT_SIR) {
            u8 dsps_bytes[4];
            dsps_bytes[0] = NCR_READ_REG(iobase, NCR_REG_DSPS + 0);
            dsps_bytes[1] = NCR_READ_REG(iobase, NCR_REG_DSPS + 1);
            dsps_bytes[2] = NCR_READ_REG(iobase, NCR_REG_DSPS + 2);
            dsps_bytes[3] = NCR_READ_REG(iobase, NCR_REG_DSPS + 3);

            u32 dsps = (dsps_bytes[3] << 24) | (dsps_bytes[2] << 16) |
                       (dsps_bytes[1] << 8) | dsps_bytes[0];

            if (dsps == 0x00000401) {
                flush_data_cache((char *)&status, sizeof(status));
                flush_data_cache((char *)&msgin, sizeof(msgin));
                if (scsi_is_read(op) && op->buf_fl && op->count * blocksize > 0) {
                    flush_data_cache((char *)op->buf_fl, op->count * blocksize);
                }
                return DISK_RET_SUCCESS;
            } else {
                goto fail;
            }
        }

        if (istat & 0x02) {  /* SIP bit - SCSI interrupt pending */
            if (sstat0 & 0x80) {  /* MA - Message Acknowledge / Phase Mismatch */
                goto fail;
            }

            if (sstat0 & 0x04) {  /* UDC - Unexpected Disconnect */
                return DISK_RET_SUCCESS;
            }
        }

        if (sstat0 & 0x20) {
            goto fail;
        }
        if (sstat0 & ~0xA0) {
            goto fail;
        }
        if (sstat1 & 0x1B) {
            goto fail;
        }

        if (dstat & 0x70) {
            goto fail;
        }

        usleep(5);
    }

fail:
    return DISK_RET_EBADTRACK;
}

static int
ncr710_detect_controller(u32 iobase)
{
    // TEMP register is at 0x1C - using direct byte access with XOR
    u32 temp_reg_base = 0x1C;

    u8 original_temp[4];
    original_temp[0] = NCR_READ_REG(iobase, temp_reg_base + 0);
    original_temp[1] = NCR_READ_REG(iobase, temp_reg_base + 1);
    original_temp[2] = NCR_READ_REG(iobase, temp_reg_base + 2);
    original_temp[3] = NCR_READ_REG(iobase, temp_reg_base + 3);

    NCR_WRITE_REG(iobase, temp_reg_base + 0, 0x12);
    NCR_WRITE_REG(iobase, temp_reg_base + 1, 0x34);
    NCR_WRITE_REG(iobase, temp_reg_base + 2, 0x56);
    NCR_WRITE_REG(iobase, temp_reg_base + 3, 0x78);

    u8 read_back[4];
    read_back[0] = NCR_READ_REG(iobase, temp_reg_base + 0);
    read_back[1] = NCR_READ_REG(iobase, temp_reg_base + 1);
    read_back[2] = NCR_READ_REG(iobase, temp_reg_base + 2);
    read_back[3] = NCR_READ_REG(iobase, temp_reg_base + 3);

    NCR_WRITE_REG(iobase, temp_reg_base + 0, original_temp[0]);
    NCR_WRITE_REG(iobase, temp_reg_base + 1, original_temp[1]);
    NCR_WRITE_REG(iobase, temp_reg_base + 2, original_temp[2]);
    NCR_WRITE_REG(iobase, temp_reg_base + 3, original_temp[3]);

    if (read_back[0] == 0x12 && read_back[1] == 0x34 &&
        read_back[2] == 0x56 && read_back[3] == 0x78) {
        return 0;
    }
    return -1;
}

static void
ncr710_scsi_init_lun(struct ncr_lun_s *nlun, u32 iobase, u8 target, u8 lun)
{
    memset(nlun, 0, sizeof(*nlun));
    nlun->drive.type = DTYPE_NCR710_SCSI;
    nlun->drive.cntl_id = 0;
    nlun->target = target;
    nlun->lun = lun;
    nlun->iobase = iobase;
}

static int
ncr710_scsi_add_lun(u32 lun, struct drive_s *tmpl_drv)
{
    struct ncr_lun_s *tmpl_nlun = container_of(tmpl_drv, struct ncr_lun_s, drive);
    struct ncr_lun_s *nlun = malloc_fseg(sizeof(*nlun));
    if (!nlun) {
        warn_noalloc();
        return -1;
    }

    ncr710_scsi_init_lun(nlun, tmpl_nlun->iobase, tmpl_nlun->target, lun);

    char *name = znprintf(MAXDESCSIZE, "ncr710 %d:%d", nlun->target, nlun->lun);
    int prio = bootprio_find_scsi_device(NULL, nlun->target, nlun->lun);
    int ret = scsi_drive_setup(&nlun->drive, name, prio, nlun->target, nlun->lun);
    free(name);

    if (ret) {
        free(nlun);
        return -1;
    }
    return 0;
}

static void
ncr710_scsi_scan_target(u32 iobase, u8 target)
{
    struct ncr_lun_s nlun0;
    ncr710_scsi_init_lun(&nlun0, iobase, target, 0);

    if (scsi_rep_luns_scan(&nlun0.drive, ncr710_scsi_add_lun) < 0) {
        scsi_sequential_scan(&nlun0.drive, 8, ncr710_scsi_add_lun);
    }
}

static void
init_ncr710_scsi(u32 base_addr)
{
    u32 iobase = base_addr + LASI_SCSI_CORE_OFFSET;
    ncr710_reset(iobase);

    if (ncr710_detect_controller(iobase) < 0) {
        return;
    }


    int i;
    for (i = 0; i < 7; i++) {
        ncr710_scsi_scan_target(iobase, i);
    }
}

void
ncr710_scsi_setup(void)
{
    ASSERT32FLAT();
    if (!CONFIG_NCR710_SCSI) {
        return;
    }

    if (!runningOnQEMU()) {
        return;
    }

    if (!CONFIG_PARISC || sizeof(long) != 4 || !lasi_hpa) {
        return;
    }

    u32 device_id = inl(lasi_hpa + 0x6000);

    if (device_id != 0x5000082) {
        return;
    }

    init_ncr710_scsi(lasi_hpa + 0x6000);
}

/* SPDX-License-Identifier: MIT */

#ifndef HV_XFER_H
#define HV_XFER_H

#ifdef HV_XFER_HOST_TEST
/* Host build: the register/ABI definitions below and the device logic in
 * hv_xfer.c are exercised natively by tests/hv_xfer/. Same shim style as
 * src/wireless_handoff_abi.h. */
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
/* `unsigned long`, not uint64_t: on the aarch64-linux-gnu target uint64_t IS
 * unsigned long, and the device's printf() format strings say %lx. Using
 * unsigned long long on a macOS host would make -Wformat reject the very code
 * that is correct for the target. */
typedef unsigned long u64;
typedef int32_t s32;
#ifndef BIT
#define BIT(_bit) (UINT32_C(1) << (_bit))
#endif
#ifndef PACKED
#define PACKED __attribute__((packed))
#endif
#else
#include "types.h"
#endif

struct exc_info;

/*
 * hv_xfer: a bulk host<->guest data channel that costs ONE VM exit per buffer
 * instead of one per byte.
 *
 * The vUART (src/hv_vuart.c) is a trapping stage-2 hook, so every single byte
 * the guest reads or writes is a VM exit serviced in EL2 -- measured at ~94 us
 * per byte on J414s, i.e. 2.6-5.0 KB/s, and independent of the host baud rate.
 * Pushing a 164 KB driver package therefore takes about a minute. This device
 * removes the per-byte trap from the data path entirely:
 *
 *   * A "window" of ordinary DRAM is mapped into the guest as a NORMAL stage-2
 *     mapping (hv_map_hw), so guest loads and stores to it are full-speed
 *     native memory accesses that never trap.
 *   * One 16 KiB doorbell page IS hooked. The guest writes a command word to
 *     it; that single trap runs hv_exc_proxy(), the tethered host moves the
 *     whole window in one REQ_MEMWRITE/REQ_MEMREAD over the existing proxy
 *     pipe, and the guest resumes.
 *
 * So the transfer cost becomes one VM exit per window, not per byte, and the
 * bytes themselves ride the proxy's bulk path -- the same path that already
 * uploads the whole kernel image at boot.
 *
 * This is strictly additive. The vUART is untouched: if this device is never
 * mapped, or the host end is absent, or any command fails, the console and the
 * PowerShell agent keep working exactly as before. Nothing here is on the
 * critical path of a boot.
 *
 * Deliberately NOT reimplemented here: a second bulk protocol on the CDC pipe
 * the vUART uses (IODEV_USB_VUART, CDC_ACM_PIPE_1). That pipe is the guest's
 * console lifeline, has no framing, and a blocking bulk read on it from EL2
 * has no timeout and no watchdog (hv_exc_proxy suspends it). The proxy pipe
 * already has framing, checksums, exception guards and a host implementation.
 */

/* Doorbell register file. All registers are 32-bit; 64-bit accesses to an
 * aligned pair read/write both halves. Offsets are stable ABI -- mirrored by
 * proxyclient/m1n1/hv/xfer.py and by the guest driver. */
#define HV_XFER_REG_ID        0x000 /* RO 'HVXF' */
#define HV_XFER_REG_VERSION   0x004 /* RO protocol version */
#define HV_XFER_REG_FEATURES  0x008 /* RO see HV_XFER_FEAT_* */
#define HV_XFER_REG_PAGESIZE  0x00c /* RO window alignment granule */
#define HV_XFER_REG_WIN_LO    0x010 /* RO active window guest-phys base, low */
#define HV_XFER_REG_WIN_HI    0x014 /* RO active window guest-phys base, high */
#define HV_XFER_REG_WIN_SIZE  0x018 /* RO active window size in bytes */
#define HV_XFER_REG_MAX_XFER  0x01c /* RO largest LEN a single command accepts */
#define HV_XFER_REG_CMD       0x020 /* WO write executes; read returns last cmd */
#define HV_XFER_REG_OFF       0x024 /* RW byte offset into the active window */
#define HV_XFER_REG_LEN       0x028 /* RW byte count */
#define HV_XFER_REG_TAG       0x02c /* RW opaque stream id handed to the host */
#define HV_XFER_REG_STATUS    0x030 /* RO s32 result of the last command */
#define HV_XFER_REG_RESULT    0x034 /* RO bytes actually moved by the last cmd */
#define HV_XFER_REG_SEQ       0x038 /* RO completed-command counter */
#define HV_XFER_REG_ERRORS    0x03c /* RO failed-command counter */
#define HV_XFER_REG_UWIN_LO   0x040 /* RW guest-supplied window base, low */
#define HV_XFER_REG_UWIN_HI   0x044 /* RW guest-supplied window base, high */
#define HV_XFER_REG_UWIN_SIZE 0x048 /* RW guest-supplied window size */
#define HV_XFER_REG_NAME      0x080 /* RW 64-byte NUL-padded ASCII stream name */

#define HV_XFER_ID      0x46585648 /* 'HVXF' little-endian */
#define HV_XFER_VERSION 1

#define HV_XFER_NAME_SIZE 64
#define HV_XFER_PAGE_SIZE 0x4000
/* Register window; one full 16 KiB stage-2 page so no L4 subpage table is
 * needed and nothing else can share the page. */
#define HV_XFER_REGS_SIZE 0x4000

/* FEATURES bits */
#define HV_XFER_FEAT_HOST       BIT(0) /* a host engine answered at least once */
#define HV_XFER_FEAT_SET_WINDOW BIT(1) /* SET_WINDOW/RESET_WINDOW implemented */

/* Commands written to HV_XFER_REG_CMD. */
#define HV_XFER_CMD_NOP          0
#define HV_XFER_CMD_PING         1 /* round-trip to the host; moves no data */
#define HV_XFER_CMD_OPEN         2 /* open stream NAME, mode in TAG's high bit */
#define HV_XFER_CMD_GET          3 /* host -> window[OFF..OFF+LEN) */
#define HV_XFER_CMD_PUT          4 /* window[OFF..OFF+LEN) -> host */
#define HV_XFER_CMD_CLOSE        5 /* finish the stream (host commits the file) */
#define HV_XFER_CMD_SET_WINDOW   6 /* bind the window to UWIN_* (EL2 only) */
#define HV_XFER_CMD_RESET_WINDOW 7 /* rebind to the m1n1-owned window (EL2 only) */

/* STATUS values (s32). Negative is always a failure. */
#define HV_XFER_ST_OK      0
#define HV_XFER_ST_INVAL   -1 /* malformed command */
#define HV_XFER_ST_NODEV   -2 /* no host engine attached */
#define HV_XFER_ST_IO      -3 /* the host declined the command */
#define HV_XFER_ST_RANGE   -4 /* OFF/LEN outside the active window */
#define HV_XFER_ST_FAULT   -5 /* SET_WINDOW target is not usable guest DRAM */
#define HV_XFER_ST_BADCMD  -6 /* unknown command */

/*
 * Handed to the host with each HV_XFER proxy event. Mirrored by XferExcInfo in
 * proxyclient/m1n1/hv/xfer.py -- keep the two in step.
 *
 * win_phys is a REAL physical address (the window is identity-mapped into the
 * guest, or was validated to be so), so the host can aim REQ_MEMWRITE and
 * REQ_MEMREAD straight at it. name_phys points at this device's own 64-byte
 * name field, which lives in m1n1's heap and is likewise host-readable.
 */
struct hv_xfer_exc_info {
    u64 devbase;   /* doorbell base of the device raising the event */
    u64 win_phys;  /* physical base of the active window */
    u64 win_size;  /* size of the active window */
    u64 name_phys; /* physical address of the 64-byte NAME field */
    u32 cmd;
    u32 tag;
    u32 off;
    u32 len;
    u32 result; /* host fills: bytes actually moved */
    s32 status; /* host fills: 0 = ok, anything else = declined */
} PACKED;

/*
 * Map a bulk channel with its doorbell at `base` (HV_XFER_REGS_SIZE bytes) and
 * its default window at `win` (`win_size` bytes).
 *
 * `win` must be 16 KiB aligned, `win_size` a nonzero multiple of 16 KiB, and
 * the whole range must lie inside physical DRAM and OUTSIDE anything the guest
 * already has mapped -- in practice the caller allocates it from the proxy
 * heap, which sits below the guest's boot_args phys_base and is therefore
 * invisible to the guest's memory map. The window is then mapped into the
 * guest identity (IPA == PA) as a plain HW mapping.
 *
 * Returns 0, or negative on refusal. Refusing is always safe: the guest simply
 * has no bulk channel and the vUART is unaffected.
 */
int hv_map_xfer(u64 base, u64 win, u64 win_size);

#endif

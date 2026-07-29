/* SPDX-License-Identifier: MIT */

#include "hv_tpm.h"
#include "hv.h"
#include "malloc.h"
#include "string.h"
#include "utils.h"

/*
 * Register offsets within a locality page.
 * TCG PC Client Platform TPM Profile v1.05 r14, 6.3.2 Table 12 / 6.5.3.1
 * Table 25, cross-checked against edk2 MdePkg/Include/IndustryStandard/
 * TpmPtp.h:249-323 -- the two agree on every entry.
 */
#define TPM_LOC_STATE     0x000
#define TPM_LOC_CTRL      0x008
#define TPM_LOC_STS       0x00c
#define TPM_INTF_ID       0x030
#define TPM_CTRL_EXT      0x038
#define TPM_CTRL_REQ      0x040 /* the ACPI table points HERE, not at base */
#define TPM_CTRL_STS      0x044
#define TPM_CTRL_CANCEL   0x048
#define TPM_CTRL_START    0x04c
#define TPM_INT_ENABLE    0x050
#define TPM_INT_STS       0x054
#define TPM_CTRL_CMD_SIZE 0x058
#define TPM_CMD_LADDR     0x05c
#define TPM_CMD_HADDR     0x060
#define TPM_CTRL_RSP_SIZE 0x064
#define TPM_CTRL_RSP_ADDR 0x068
#define TPM_DATA_BUFFER   0x080

#define TPM_LOCALITY_SIZE 0x1000
#define TPM_DATA_SIZE     (TPM_LOCALITY_SIZE - TPM_DATA_BUFFER)

#define REQ_CMD_READY BIT(0)
#define REQ_GO_IDLE   BIT(1)
#define STS_TPM_STS   BIT(0)
#define STS_TPM_IDLE  BIT(1)
#define START_START   BIT(0)
#define CANCEL_CANCEL BIT(0)

#define LOC_CTRL_REQUEST_ACCESS BIT(0)
#define LOC_CTRL_RELINQUISH     BIT(1)
#define LOC_STS_GRANTED         BIT(0)

#define LOC_STATE_LOC_ASSIGNED BIT(1)
#define LOC_STATE_VALID_STS    BIT(7)

/*
 * TPM_CRB_INTF_ID. edk2's Tpm2GetPtpInterface() (Tpm2Ptp.c:421-427) accepts a
 * device as CRB only if InterfaceType == 1, InterfaceVersion is 1 or 2, and
 * CapCRB is set. Anything else is rejected by Mu before Windows is ever told a
 * TPM exists, so this constant is the single most load-bearing value here.
 *
 * CapLocality is deliberately clear: only locality 0 is implemented, and
 * advertising more would invite a request that never gets granted.
 * CapCRBIdleBypass is clear so the guest walks the full goIdle/cmdReady
 * handshake -- the path edk2 actually exercises.
 */
#define INTF_ID_VALUE                                                                              \
    ((1 << 0) |  /* InterfaceType   = CRB */                                                       \
     (1 << 4) |  /* InterfaceVersion = 1  */                                                       \
     (1 << 14) | /* CapCRB               */                                                        \
     (1 << 16))  /* InterfaceSelector = CRB */

/* TPM 2.0 response header: tag(2) size(4) responseCode(4), big-endian. */
#define TPM_ST_NO_SESSIONS 0x8001
#define TPM_RC_FAILURE     0x00000101
#define TPM_RSP_HEADER_LEN 10

struct tpm_dev {
    u64 base;
    u32 loc_state;
    u32 loc_sts;
    u32 ctrl_req;
    u32 ctrl_sts;
    u32 ctrl_start;
    u32 ctrl_cancel;
    u32 cmd_size;
    u32 rsp_size;
    u64 cmd_addr;
    u64 rsp_addr;
    bool ready;   /* cmdReady acknowledged: the buffer may be written */
    bool running; /* START asserted, command not yet answered */
    hv_tpm_backend_t *backend;
    void *cookie;
    u8 data[TPM_DATA_SIZE];
};

static struct tpm_dev *tpm_device = NULL;

static void tpm_put_be32(u8 *p, u32 v)
{
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

static void tpm_fail_response(struct tpm_dev *dev)
{
    dev->data[0] = TPM_ST_NO_SESSIONS >> 8;
    dev->data[1] = TPM_ST_NO_SESSIONS & 0xff;
    tpm_put_be32(&dev->data[2], TPM_RSP_HEADER_LEN);
    tpm_put_be32(&dev->data[6], TPM_RC_FAILURE);
    dev->rsp_size = TPM_RSP_HEADER_LEN;
}

static void tpm_execute(struct tpm_dev *dev)
{
    size_t len = 0;

    /*
     * edk2 points both the command and the response at the data buffer
     * itself (Tpm2Ptp.c:253-259), so the backend reads and writes the same
     * region. cmd_size is what the guest declared; clamp it rather than
     * trusting it, because it is guest-controlled and indexes our buffer.
     */
    u32 cmd_len = dev->cmd_size;
    if (cmd_len > TPM_DATA_SIZE)
        cmd_len = TPM_DATA_SIZE;

    if (dev->backend)
        len = dev->backend(dev->cookie, dev->data, cmd_len, dev->data, TPM_DATA_SIZE);

    if (!len || len > TPM_DATA_SIZE) {
        /*
         * No engine, or a backend that misbehaved. Answer with a well-formed
         * TPM_RC_FAILURE rather than leaving stale bytes in the buffer: the
         * guest would otherwise parse whatever was there as a response.
         */
        tpm_fail_response(dev);
    } else {
        dev->rsp_size = len;
    }

    dev->ctrl_start = 0; /* the guest polls for this to clear */
    dev->ctrl_cancel = 0;
    dev->running = false;
}

static bool handle_tpm(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    struct tpm_dev *dev = tpm_device;
    UNUSED(ctx);

    if (!dev || (addr & ~0xfffUL) != dev->base)
        return false;

    addr &= 0xfff;

    /*
     * The data buffer is byte-addressed. edk2 copies the command in and reads
     * the response out ONE BYTE AT A TIME (Tpm2Ptp.c:249-251, :304-336), so
     * byte accesses here are the normal case, not an edge case.
     */
    if (addr >= TPM_DATA_BUFFER) {
        u64 off = addr - TPM_DATA_BUFFER;
        int n = width;

        if (n < 1 || n > 8 || off + (u64)n > TPM_DATA_SIZE)
            return false;

        if (write) {
            /* Only writable once the guest has completed the handshake. */
            if (!dev->ready && !dev->running)
                return true;
            for (int i = 0; i < n; i++)
                dev->data[off + i] = (*val >> (8 * i)) & 0xff;
        } else {
            u64 v = 0;
            for (int i = 0; i < n; i++)
                v |= ((u64)dev->data[off + i]) << (8 * i);
            *val = v;
        }
        return true;
    }

    if (write) {
        switch (addr) {
            case TPM_LOC_CTRL:
                if (*val & LOC_CTRL_REQUEST_ACCESS) {
                    dev->loc_sts |= LOC_STS_GRANTED;
                    dev->loc_state |= LOC_STATE_LOC_ASSIGNED | LOC_STATE_VALID_STS;
                }
                if (*val & LOC_CTRL_RELINQUISH) {
                    dev->loc_sts &= ~LOC_STS_GRANTED;
                    dev->loc_state &= ~LOC_STATE_LOC_ASSIGNED;
                    dev->ready = false;
                }
                break;

            case TPM_CTRL_REQ:
                /*
                 * edk2 sets cmdReady then waits for the bit to clear AND for
                 * tpmIdle to clear. Acknowledging synchronously is correct for
                 * an emulation and avoids a poll loop that could time out.
                 */
                if (*val & REQ_CMD_READY) {
                    dev->ctrl_req &= ~REQ_CMD_READY;
                    dev->ctrl_sts &= ~STS_TPM_IDLE;
                    dev->ready = true;
                }
                if (*val & REQ_GO_IDLE) {
                    dev->ctrl_req &= ~REQ_GO_IDLE;
                    dev->ctrl_sts |= STS_TPM_IDLE;
                    dev->ready = false;
                }
                break;

            case TPM_CTRL_START:
                if (!(*val & START_START))
                    break;
                /*
                 * Fail closed. Starting without the handshake would execute
                 * whatever stale bytes were in the buffer -- for a TPM that is
                 * security-relevant, not merely a bug.
                 */
                if (!dev->ready || dev->running)
                    break;
                dev->ctrl_start = START_START;
                dev->running = true;
                tpm_execute(dev);
                break;

            case TPM_CTRL_CANCEL:
                dev->ctrl_cancel = *val;
                break;
            case TPM_CTRL_CMD_SIZE:
                dev->cmd_size = *val;
                break;
            case TPM_CMD_LADDR:
                dev->cmd_addr = (dev->cmd_addr & 0xffffffff00000000UL) | (u32)*val;
                break;
            case TPM_CMD_HADDR:
                dev->cmd_addr = (dev->cmd_addr & 0xffffffffUL) | ((u64)(u32)*val << 32);
                break;
            case TPM_CTRL_RSP_SIZE:
                dev->rsp_size = *val;
                break;
            case TPM_CTRL_RSP_ADDR:
                dev->rsp_addr = *val;
                break;
            default:
                /*
                 * Reserved, read-only or unimplemented. Absorbed rather than
                 * refused: an MMIO abort inside tpm.sys is far worse than a
                 * dropped write to a register nobody implements.
                 */
                break;
        }
        return true;
    }

    switch (addr) {
        case TPM_LOC_STATE:
            *val = dev->loc_state;
            break;
        case TPM_LOC_STS:
            *val = dev->loc_sts;
            break;
        case TPM_INTF_ID:
            *val = INTF_ID_VALUE;
            break;
        case TPM_CTRL_REQ:
            *val = dev->ctrl_req;
            break;
        case TPM_CTRL_STS:
            *val = dev->ctrl_sts;
            break;
        case TPM_CTRL_START:
            *val = dev->ctrl_start;
            break;
        case TPM_CTRL_CANCEL:
            *val = dev->ctrl_cancel;
            break;
        case TPM_CTRL_CMD_SIZE:
            *val = dev->cmd_size;
            break;
        case TPM_CMD_LADDR:
            *val = (u32)dev->cmd_addr;
            break;
        case TPM_CMD_HADDR:
            *val = dev->cmd_addr >> 32;
            break;
        case TPM_CTRL_RSP_SIZE:
            *val = dev->rsp_size;
            break;
        case TPM_CTRL_RSP_ADDR:
            *val = dev->rsp_addr;
            break;
        default:
            /*
             * Zero, never all-ones: an all-ones read is how a MISSING device
             * presents, and edk2's probe would then misclassify the interface.
             */
            *val = 0;
            break;
    }

    return true;
}

int hv_map_tpm(u64 base, hv_tpm_backend_t *backend, void *cookie)
{
    struct tpm_dev *dev;

    if (tpm_device) {
        printf("tpm: a CRB is already mapped at 0x%lx\n", tpm_device->base);
        return -1;
    }

    if (base & (TPM_LOCALITY_SIZE - 1)) {
        printf("tpm: base 0x%lx is not locality-aligned\n", base);
        return -1;
    }

    dev = calloc(1, sizeof(*dev));
    if (!dev)
        return -1;

    dev->base = base;
    dev->backend = backend;
    dev->cookie = cookie;
    /* A freshly reset device is idle, and its status register is valid. */
    dev->ctrl_sts = STS_TPM_IDLE;
    dev->loc_state = LOC_STATE_VALID_STS;

    tpm_device = dev;

    printf("tpm: CRB at 0x%lx, control area 0x%lx%s\n", base, base + TPM_CTRL_REQ,
           backend ? "" : " (no engine: commands answer TPM_RC_FAILURE)");

    return hv_map_hook(base, handle_tpm, TPM_LOCALITY_SIZE);
}

void hv_tpm_prepare_for_guest(void)
{
    struct tpm_dev *dev = tpm_device;

    if (!dev)
        return;

    /* TEE ACPI Profile 4.6.3: Error, Cancel and Start clear at handoff. */
    dev->ctrl_sts = STS_TPM_IDLE;
    dev->ctrl_cancel = 0;
    dev->ctrl_start = 0;
    dev->running = false;
}

/* SPDX-License-Identifier: MIT */

#ifndef HV_TPM_H
#define HV_TPM_H

#include "types.h"

struct exc_info;

/*
 * Emulated TPM 2.0 CRB (Command Response Buffer) device.
 *
 * J414s has no TPM. Windows will not accept a TIS/FIFO device on ARM64 --
 * tpm.sys only works with CRB (TCG start method 7) -- so this synthesises a
 * CRB register file over one 4 KiB locality page and lets Project Mu publish
 * an ACPI TPM2 table and an MSFT0101 device pointing at it.
 *
 * EL2 is the only layer that can host this: UEFI DXE code stops running at
 * ExitBootServices, so a Mu-only implementation would leave Windows' MMIO
 * accesses landing on nothing.
 *
 * The register layout, the INTF_ID probe value and the ACPI table encoding are
 * stated once, host-tested, in the companion driver repository at
 * drivers/AppleTpmCrb/AppleTpmCrbCore.{h,c}. Its tests assert that the address
 * published in the TPM2 table equals the address this code decodes CTRL_REQ
 * at. Do not change one side without the other.
 */

/* A command is pending; the backend must fill the response and return its
 * length in bytes. Returning 0 makes the device answer TPM_RC_FAILURE.
 * `ctx` is the trapping vCPU's exception context, so a backend may bounce
 * the command to the host over the proxy (hv_exc_proxy needs it). */
typedef size_t(hv_tpm_backend_t)(struct exc_info *ctx, void *cookie, const u8 *cmd,
                                 size_t cmd_len, u8 *rsp, size_t rsp_max);

/*
 * Map an emulated CRB at `base` (one locality page). `backend` may be NULL, in
 * which case every command is answered with a well-formed TPM_RC_FAILURE --
 * enough for Mu's probe to classify the interface and for Windows to enumerate
 * the device, but not a working TPM.
 */
int hv_map_tpm(u64 base, hv_tpm_backend_t *backend, void *cookie);

/*
 * The built-in host-side backend: one HV_TPM proxy event per command, exactly
 * the virtio notify_avail() pattern. The tethered Mac reads the command out
 * of the CRB data buffer, runs it through a real TPM 2.0 engine (swtpm or
 * ms-tpm-20-ref's simulator -- see the driver repo's
 * drivers/AppleTpmCrb/host/tpm_host.py), writes the response back and fills
 * in hv_tpm_exc_info. If the host declines or the cable is gone, the guest
 * sees TPM_RC_FAILURE -- a TPM that failed, never a TPM that lied.
 */
int hv_map_tpm_proxy(u64 base);

/* The struct handed to the host with each HV_TPM event. Mirrored by
 * TpmExcInfo in proxyclient/m1n1/hv/tpm.py -- keep the two in step. */
struct hv_tpm_exc_info {
    u64 devbase;  /* CRB locality 0 base */
    u64 buf;      /* address of the shared command/response buffer */
    u32 cmd_len;  /* exact command length (wire-header validated) */
    u32 rsp_max;  /* capacity of buf for the response */
    u32 rsp_len;  /* host fills: response length */
    u32 status;   /* host fills: 0 = ok; anything else = declined */
} PACKED;

/*
 * TEE ACPI Profile 4.6.3 requires Error, Cancel and Start to be clear when
 * firmware hands off after ExitBootServices. Call before entering the guest.
 */
/*
 * Hardware entropy from the SEP boot ROM.
 *
 * This is the ONLY use of the Secure Enclave sanctioned anywhere in this
 * design, and the boundary is deliberate. `sep_get_random()` talks to the SEP
 * *boot ROM* endpoint (SEP_EP_ROM 0xff, SEP_MSG_GETRAND 16) and needs no SEPOS
 * boot, no SBIO, no Catacomb and no xART -- m1n1 already uses it to seed
 * Linux's KASLR. Everything else about the SEP is unreachable from a non-macOS
 * OS: SEPOS has never been booted on any T602x, xART's anti-replay epoch chain
 * is anchored in a macOS filesystem store, and SKS unlock tokens are minted
 * from a macOS credential.
 *
 * So: entropy, yes. Key wrapping, NV storage, attestation -- no. Do not extend
 * this to reach for them; the TPM's keys and NV live elsewhere by necessity,
 * and pretending otherwise would misrepresent the security model.
 *
 * Returns the number of bytes actually obtained, which may be short (0 if the
 * SEP does not answer). Callers must treat a short read as failure rather than
 * padding it out with anything weaker.
 */
size_t hv_tpm_get_entropy(void *buffer, size_t len);

void hv_tpm_prepare_for_guest(void);

#endif

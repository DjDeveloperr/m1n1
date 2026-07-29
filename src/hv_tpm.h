/* SPDX-License-Identifier: MIT */

#ifndef HV_TPM_H
#define HV_TPM_H

#include "types.h"

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
 * length in bytes. Returning 0 makes the device answer TPM_RC_FAILURE. */
typedef size_t(hv_tpm_backend_t)(void *cookie, const u8 *cmd, size_t cmd_len, u8 *rsp,
                                 size_t rsp_max);

/*
 * Map an emulated CRB at `base` (one locality page). `backend` may be NULL, in
 * which case every command is answered with a well-formed TPM_RC_FAILURE --
 * enough for Mu's probe to classify the interface and for Windows to enumerate
 * the device, but not a working TPM.
 */
int hv_map_tpm(u64 base, hv_tpm_backend_t *backend, void *cookie);

/*
 * TEE ACPI Profile 4.6.3 requires Error, Cancel and Start to be clear when
 * firmware hands off after ExitBootServices. Call before entering the guest.
 */
void hv_tpm_prepare_for_guest(void);

#endif

# SPDX-License-Identifier: MIT
"""Host side of the emulated TPM 2.0 CRB: one proxy event per command.

The EL2 device (src/hv_tpm.c) validates each guest command's wire header,
then raises one HV_TPM proxy event carrying hv_tpm_exc_info. This module
reads the command out of the CRB data buffer, runs it through a real TPM
2.0 engine on the Mac, writes the response back, and declares the outcome
in the info struct. If anything goes wrong the event is answered with a
nonzero status and the guest sees a well-formed TPM_RC_FAILURE -- a TPM
that failed, never a TPM that lied or a guest that hangs.

The engine transports and the NV store live in the Windows-driver repo
(drivers/AppleTpmCrb/host/tpm_host.py, host-tested there, golden-vector
cross-checked against the C core). This module only carries bytes; it
deliberately contains no TPM knowledge beyond the info struct layout.

Set NTASI_DRIVERS_ROOT to override the default drivers-repo location.
"""

import importlib.util
import os
import sys
from pathlib import Path

from construct import Struct, Int64ul, Int32ul

__all__ = ["TpmExcInfo", "TpmHostDevice", "load_tpm_host"]

# Mirrors struct hv_tpm_exc_info in src/hv_tpm.h -- keep the two in step.
TpmExcInfo = Struct(
    "devbase" / Int64ul,
    "buf" / Int64ul,
    "cmd_len" / Int32ul,
    "rsp_max" / Int32ul,
    "rsp_len" / Int32ul,
    "status" / Int32ul,
)

TPM_STATUS_OK = 0
TPM_STATUS_DECLINED = 1

_DEFAULT_DRIVERS_ROOT = Path.home() / "Developer" / "apple_silicon_nt_drivers"


def load_tpm_host():
    """Import drivers/AppleTpmCrb/host/tpm_host.py from the drivers repo.

    Loud on failure: attaching a TPM without the transport module must be
    an error naming the missing path, not a mysterious AttributeError three
    layers down.
    """
    root = Path(os.environ.get("NTASI_DRIVERS_ROOT", _DEFAULT_DRIVERS_ROOT))
    path = root / "drivers" / "AppleTpmCrb" / "host" / "tpm_host.py"
    if not path.is_file():
        raise ImportError(
            f"TPM host transport not found at {path}; set NTASI_DRIVERS_ROOT "
            "to your apple_silicon_nt_drivers checkout")
    spec = importlib.util.spec_from_file_location("ntasi_tpm_host", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["ntasi_tpm_host"] = module
    spec.loader.exec_module(module)
    return module


class TpmHostDevice:
    """Binds a started, probed engine to the HV_TPM event stream."""

    def __init__(self, engine, tpm_host, verbose=True):
        self.engine = engine
        self.tpm_host = tpm_host
        self.verbose = verbose
        self.commands = 0
        self.failures = 0

    def execute(self, cmd):
        """Run one raw TPM command; returns the raw response or None.

        Returning None makes the EL2 device answer TPM_RC_FAILURE. Every
        None is counted and logged: a silent failure stream here would look
        exactly like a broken TPM to Windows, and the operator must be able
        to tell the difference from the console.
        """
        self.commands += 1
        try:
            rsp = self.engine.send(bytes(cmd))
            self.tpm_host.parse_response(rsp)  # exact-size + tag validation
            if self.verbose:
                code = int.from_bytes(cmd[6:10], "big")
                rc = int.from_bytes(rsp[6:10], "big")
                print(f"tpm: cc=0x{code:03x} len={len(cmd)} -> "
                      f"rc=0x{rc:03x} len={len(rsp)}")
            return rsp
        except Exception as exc:
            self.failures += 1
            print(f"tpm: host engine error on command {self.commands}: {exc}")
            return None

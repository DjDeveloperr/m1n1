/* SPDX-License-Identifier: MIT */

#ifndef TPS6598X_H
#define TPS6598X_H

#include "i2c.h"
#include "types.h"

typedef struct tps6598x_dev tps6598x_dev_t;

tps6598x_dev_t *tps6598x_init(const char *adt_path, i2c_dev_t *i2c);
void tps6598x_shutdown(tps6598x_dev_t *dev);

int tps6598x_command(tps6598x_dev_t *dev, const char *cmd, const u8 *data_in, size_t len_in,
                     u8 *data_out, size_t len_out);
int tps6598x_powerup(tps6598x_dev_t *dev);
int tps6598x_prepare_host(tps6598x_dev_t *dev, u32 hpm_index, u32 controller_count,
                          s32 preserved_index);

#define CD3218B12_IRQ_WIDTH 9

typedef struct tps6598x_irq_state {
    u8 int_mask1[CD3218B12_IRQ_WIDTH];
    bool valid;
} tps6598x_irq_state_t;

int tps6598x_disable_irqs(tps6598x_dev_t *dev, tps6598x_irq_state_t *state);
int tps6598x_restore_irqs(tps6598x_dev_t *dev, tps6598x_irq_state_t *state);

/*
 * TPS6598x / CD3217 STATUS register (register number 0x1a).
 *
 * Register number and bit positions are transcribed from Linux's tipd
 * driver, which drives the same silicon: `#define TPS_REG_STATUS 0x1a`
 * (drivers/usb/typec/tipd/core.c:41), read as a 32-bit register by
 * tps6598x_read_status() (tipd/core.c:540-554), with the bit layout at
 * drivers/usb/typec/tipd/tps6598x.h:17-26.
 *
 * PLUG_UPSIDE_DOWN is the *only* source of cable orientation on this
 * platform.  tipd turns it into TYPEC_ORIENTATION_REVERSE/NORMAL
 * (tipd/core.c:768-772) and hands it to the ATC PHY through
 * typec_set_orientation -> typec_switch_set -> atcphy_sw_set
 * (class.c:2390-2405, atc.c:2046-2064), which latches it as
 * `swap_lanes` for the next atcphy_configure().
 *
 * Byte 0's layout was additionally confirmed live on this machine's hpm2
 * on 2026-07-29 (see proxyclient/m1n1/atcphy.py:320-338).  The register
 * *number* is the part that rests on tipd convention alone, so if a
 * decoded STATUS looks insane, suspect 0x1a first.
 */
#define TPS6598X_REG_STATUS              0x1a
#define TPS6598X_STATUS_PLUG_PRESENT     BIT(0)
#define TPS6598X_STATUS_PLUG_UPSIDE_DOWN BIT(4)
#define TPS6598X_STATUS_PORTROLE         BIT(5)
#define TPS6598X_STATUS_DATAROLE         BIT(6)
#define TPS6598X_STATUS_VCONN            BIT(7)

/*
 * Read STATUS.  This is a pure SMBus *read* of one register: it issues no
 * command, changes no configuration, and must never be extended to write.
 * The CD3217 on this machine is known to reject System Configuration
 * writes outright, and the bus it lives on also carries the PD controller
 * behind the m1n1 proxy's own console port.
 *
 * `status` receives the low 32 bits, matching tipd's tps6598x_read32() view
 * of the same register. Returns 0 on success, -1 on failure (a diagnostic
 * is printed); on failure `status` is left untouched.
 */
int tps6598x_read_status(tps6598x_dev_t *dev, u32 *status);

/* The I2C bus address this device was resolved to, for logging and for the
 * caller's own corroboration checks. Returns 0 for a NULL device. */
u8 tps6598x_i2c_addr(const tps6598x_dev_t *dev);

#endif

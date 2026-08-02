/* SPDX-License-Identifier: MIT */

#ifndef USB_H
#define USB_H

#include "iodev.h"
#include "types.h"
#include "usb_dwc3.h"

dwc3_dev_t *usb_bringup(u32 idx);

void usb_init(void);
void usb_hpm_restore_irqs(bool force);
void usb_hpm_handoff_host(iodev_id_t keep);

/*
 * Cable orientation for one Type-C port, as reported by that port's CD3217
 * PD controller. See usb_hpm_read_orientation() in usb.c for the full why,
 * the source citations, and the read-only guarantee.
 *
 * UNREADABLE is deliberately distinct from NO_PLUG: "we could not find out"
 * and "there is nothing plugged in, so there is no orientation to find out"
 * are different facts, and a caller that collapses them is back to guessing.
 */
typedef enum {
    USB_HPM_ORIENTATION_UNREADABLE = -1,
    USB_HPM_ORIENTATION_NO_PLUG = 0,
    USB_HPM_ORIENTATION_NORMAL = 1,
    USB_HPM_ORIENTATION_FLIPPED = 2,
} usb_hpm_orientation_t;

/*
 * Returns a usb_hpm_orientation_t. `status_out` may be NULL; when non-NULL
 * and the read succeeded it receives the raw STATUS dword, so callers can
 * log the evidence rather than just the verdict.
 */
int usb_hpm_read_orientation(u32 idx, u32 *status_out);
void usb_iodev_init(void);
void usb_iodev_shutdown_except(iodev_id_t keep);
void usb_iodev_shutdown(void);
void usb_iodev_vuart_setup(iodev_id_t iodev);
size_t usb_iodev_vuart_write_space(void);

#endif

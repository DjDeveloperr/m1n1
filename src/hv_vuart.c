/* SPDX-License-Identifier: MIT */

#include "hv.h"
#include "aic.h"
#include "iodev.h"
#include "uart.h"
#include "uart_regs.h"
#include "usb.h"

bool active = false;

u32 ucon = 0;
u32 utrstat = 0;
u32 ufstat = 0;

int vuart_irq = 0;

/*
 * Guest TX bytes discarded because the host end of the CDC pipe was not
 * draining. Non-zero means "currently dropping"; it is reset to 0 and reported
 * when the host catches up. See the UTXH case in handle_vuart().
 */
static u32 vuart_tx_dropped = 0;

static void update_irq(void)
{
    ssize_t rx_queued;

    iodev_handle_events(IODEV_USB_VUART);

    utrstat |= UTRSTAT_TXBE | UTRSTAT_TXE;
    utrstat &= ~UTRSTAT_RXD;

    ufstat = 0;
    if ((rx_queued = iodev_can_read(IODEV_USB_VUART))) {
        utrstat |= UTRSTAT_RXD;
        if (rx_queued > 15)
            ufstat = FIELD_PREP(UFSTAT_RXCNT, 15) | UFSTAT_RXFULL;
        else
            ufstat = FIELD_PREP(UFSTAT_RXCNT, rx_queued);

        if (FIELD_GET(UCON_RXMODE, ucon) == UCON_MODE_IRQ && ucon & UCON_RXTO_ENA) {
            utrstat |= UTRSTAT_RXTO;
        }
    }

    if (FIELD_GET(UCON_TXMODE, ucon) == UCON_MODE_IRQ && ucon & UCON_TXTHRESH_ENA) {
        utrstat |= UTRSTAT_TXTHRESH;
    }

    if (vuart_irq) {
        uart_clear_irqs();
        if (utrstat & (UTRSTAT_TXTHRESH | UTRSTAT_RXTHRESH | UTRSTAT_RXTO)) {
            aic_set_sw(vuart_irq, true);
        } else {
            aic_set_sw(vuart_irq, false);
        }
    }

    //     printf("HV: vuart UTRSTAT=0x%x UFSTAT=0x%x UCON=0x%x\n", utrstat, ufstat, ucon);
}

static void handle_vuart_passthrough(uint8_t b)
{
    const char PREFIX[] = "HVLOG: ";
    static int state = 0;

    if (!PREFIX[state]) {
        if (b == '\r' || b == '\n') {
            printf("\n");
            state = 0;
            return;
        }
        printf("%c", b);
        return;
    }

    if (b == PREFIX[state])
        state++;
    else
        state = 0;

    if (!PREFIX[state])
        printf("%s", PREFIX);
}

static bool handle_vuart(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    UNUSED(ctx);
    UNUSED(width);

    addr &= 0xfff;

    update_irq();

    if (write) {
        //         printf("HV: vuart W 0x%lx <- 0x%lx (%d)\n", addr, *val, width);
        switch (addr) {
            case UCON:
                ucon = *val;
                break;
            case UTXH: {
                uint8_t b = *val;
                /*
                 * NEVER let a guest TX byte block here.
                 *
                 * This runs in EL2 exception context, under the big hypervisor
                 * lock, with the 1 s HV watchdog armed. iodev_write() ->
                 * usb_dwc3_queue() spins until the TX ring accepts the byte, and
                 * iodev_can_write() only reports "the host configured the CDC
                 * pipe", not "there is room". So the previous guard was not a
                 * guard at all: any host that stops draining the secondary TTY
                 * (terminal closed, socat SIGKILLed, client crashed, or simply a
                 * guest that outruns the reader) would wedge EL2 and trip the
                 * watchdog -- stranding the single physical serial link and
                 * forcing a physical reboot.
                 *
                 * Policy is therefore drop-on-full, which is the standard
                 * behaviour of a real UART whose FIFO has overrun: the guest is
                 * told the byte was accepted (UTRSTAT already reports TXBE/TXE
                 * unconditionally in update_irq()), and the byte is discarded.
                 * Losing console output is recoverable; losing the link is not.
                 */
                if (usb_iodev_vuart_write_space() > 0) {
                    if (vuart_tx_dropped) {
                        printf("HV: vuart: host drained, resuming TX (dropped %u byte%s)\n",
                               vuart_tx_dropped, vuart_tx_dropped == 1 ? "" : "s");
                        vuart_tx_dropped = 0;
                    }
                    iodev_write(IODEV_USB_VUART, &b, 1);
                } else {
                    /*
                     * Report only the transition into the dropping state, once.
                     * The recovery message above reports the total. Printing per
                     * dropped byte would flood m1n1's own console -- the very
                     * link we are protecting.
                     */
                    if (!vuart_tx_dropped)
                        printf("HV: vuart: TX buffer full, host is not draining; dropping bytes\n");
                    if (vuart_tx_dropped != UINT32_MAX)
                        vuart_tx_dropped++;
                }
                handle_vuart_passthrough(b);
                break;
            }
            case UTRSTAT:
                utrstat &= ~(*val & (UTRSTAT_TXTHRESH | UTRSTAT_RXTHRESH | UTRSTAT_RXTO));
                break;
        }
    } else {
        switch (addr) {
            case UCON:
                *val = ucon;
                break;
            case URXH:
                if (iodev_can_read(IODEV_USB_VUART)) {
                    uint8_t c;
                    iodev_read(IODEV_USB_VUART, &c, 1);
                    *val = c;
                } else {
                    *val = 0;
                }
                break;
            case UTRSTAT:
                *val = utrstat;
                break;
            case UFSTAT:
                //
                // HACK HACK: the below code needs to account for whether we require SAM5250 semantics for the Windows
                // UART driver or if we can get away with using the 8900 UART raw (they're basically compatible but not fully.)
                //
                *val = utrstat & UTRSTAT_TXBE ? ufstat : ufstat | BIT(24);
                break;
            case UERSTAT:
                *val = 0;
                break;
            default:
                *val = 0;
                break;
        }
        //         printf("HV: vuart R 0x%lx -> 0x%lx (%d)\n", addr, *val, width);
    }

    return true;
}

void hv_vuart_poll(void)
{
    if (!active)
        return;

    update_irq();
}

void hv_map_vuart(u64 base, int irq, iodev_id_t iodev)
{
    hv_map_hook(base, handle_vuart, 0x1000);
    usb_iodev_vuart_setup(iodev);
    vuart_irq = irq;
    active = true;
}

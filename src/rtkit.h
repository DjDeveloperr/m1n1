/* SPDX-License-Identifier: MIT */

#ifndef RTKIT_H
#define RTKIT_H

#include "asc.h"
#include "dart.h"
#include "iova.h"
#include "sart.h"
#include "types.h"

typedef struct rtkit_dev rtkit_dev_t;

struct rtkit_message {
    u8 ep;
    u64 msg;
};

struct rtkit_buffer {
    void *bfr;
    u64 dva;
    size_t sz;
};

rtkit_dev_t *rtkit_init(const char *name, asc_dev_t *asc, dart_dev_t *dart,
                        iova_domain_t *dart_iovad, sart_dev_t *sart, bool sram);
/*
 * Permit IOP-owned preallocated buffers only inside one explicit physical
 * aperture while retaining the normal DART/SART mapping path for AP-owned
 * buffers. This is intended for coprocessors such as MTP that combine a DART
 * with a dedicated SRAM region.
 */
bool rtkit_set_phys_window(rtkit_dev_t *rtk, u64 base, size_t size);
bool rtkit_quiesce(rtkit_dev_t *rtk);
bool rtkit_sleep(rtkit_dev_t *rtk);
void rtkit_free(rtkit_dev_t *rtk);

bool rtkit_start_ep(rtkit_dev_t *rtk, u8 ep);
bool rtkit_boot(rtkit_dev_t *rtk);
/*
 * Bounded boot variant. In addition to timing out while waiting for IOP=ON,
 * this waits for the AP=ON acknowledgement before returning.
 */
bool rtkit_boot_timed(rtkit_dev_t *rtk, u32 timeout_usec);

bool rtkit_can_recv(rtkit_dev_t *rtk);

int rtkit_recv(rtkit_dev_t *rtk, struct rtkit_message *msg);
bool rtkit_send(rtkit_dev_t *rtk, const struct rtkit_message *msg);

bool rtkit_map(rtkit_dev_t *rtk, void *phys, size_t sz, u64 *dva);
bool rtkit_unmap(rtkit_dev_t *rtk, u64 dva, size_t sz);

bool rtkit_alloc_buffer(rtkit_dev_t *rtk, struct rtkit_buffer *bfr, size_t sz);
bool rtkit_free_buffer(rtkit_dev_t *rtk, struct rtkit_buffer *bfr);

#endif

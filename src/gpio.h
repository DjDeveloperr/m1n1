/* SPDX-License-Identifier: MIT */

#ifndef GPIO_H
#define GPIO_H

#ifdef APPLE_GPIO_HOST_TEST
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#define BIT(x)                 (1UL << (x))
#define GENMASK(msb, lsb)      ((BIT((msb + 1) - (lsb)) - 1) << (lsb))
#define _FIELD_LSB(field)      ((field) & ~((field) - 1))
#define FIELD_PREP(field, val) (((val) * (_FIELD_LSB(field))) & (field))
#define FIELD_GET(field, val)  (((val) & (field)) / _FIELD_LSB(field))
#else
#include "types.h"
#endif

/*
 * Apple AP GPIO ("pinctrl") register layout.
 *
 * Source of every definition below: Linux
 * drivers/pinctrl/pinctrl-apple-gpio.c (AsahiLinux/linux, asahi branch),
 * which names them REG_GPIO(x)/REG_GPIOx_*.  Do not "improve" these from
 * guesswork -- they are the authoritative bit assignments.
 *
 * One 32-bit register per pin, at controller_base + 4 * pin.
 */
#define APPLE_GPIO_REG(pin) (4 * (u64)(pin))

#define APPLE_GPIO_DATA BIT(0)

#define APPLE_GPIO_MODE            GENMASK(3, 1)
#define APPLE_GPIO_MODE_OUT        1
#define APPLE_GPIO_MODE_IN_IRQ_HI  2
#define APPLE_GPIO_MODE_IN_IRQ_LO  3
#define APPLE_GPIO_MODE_IN_IRQ_UP  4
#define APPLE_GPIO_MODE_IN_IRQ_DN  5
#define APPLE_GPIO_MODE_IN_IRQ_ANY 6
#define APPLE_GPIO_MODE_IN_IRQ_OFF 7

#define APPLE_GPIO_PERIPH GENMASK(6, 5)

#define APPLE_GPIO_PULL           GENMASK(8, 7)
#define APPLE_GPIO_PULL_OFF       0
#define APPLE_GPIO_PULL_DOWN      1
#define APPLE_GPIO_PULL_UP_STRONG 2
#define APPLE_GPIO_PULL_UP        3

#define APPLE_GPIO_INPUT_ENABLE    BIT(9)
#define APPLE_GPIO_DRIVE_STRENGTH0 GENMASK(11, 10)
#define APPLE_GPIO_SCHMITT         BIT(15)
#define APPLE_GPIO_GRP             GENMASK(18, 16)
#define APPLE_GPIO_LOCK            BIT(21)
#define APPLE_GPIO_DRIVE_STRENGTH1 GENMASK(23, 22)

/*
 * An ADT `function-<name>` property is a packed record:
 *
 *   u32 phandle;   target controller node ("AAPL,phandle")
 *   u32 fourcc;    little-endian u32 whose big-endian rendering is the name
 *   u32 args[];    controller specific; args[0] is the pin for a GPIO target
 *
 * m1n1's proxyclient renders the FourCC big-endian (see
 * proxyclient/m1n1/utils.py: FourCC = ExprAdapter(Int32ul, ...to_bytes(4,
 * "big")...)), so a property displayed as name='GPIO' is stored in memory as
 * the bytes 'O','I','P','G' and reads back as the u32 below.  A property
 * whose target is an SMC-backed GPIO carries a different FourCC and must not
 * be poked as MMIO; callers get rejected on this check.
 */
#define APPLE_GPIO_FUNCTION_FOURCC UINT32_C(0x4750494f) /* renders as "GPIO" */

#define APPLE_GPIO_FUNCTION_MIN_LEN 12

struct apple_gpio_function {
    u32 phandle;
    u32 fourcc;
    u32 pin;
    u32 arg1;
    bool have_arg1;
};

/* A resolved output-capable pin on a memory-mapped AP GPIO controller. */
struct apple_gpio_pin {
    u64 base;
    u32 pin;
    bool valid;
};

/*
 * Pure helpers.  These have no MMIO or ADT dependency and are covered by
 * tests/gpio/test_apple_gpio.c.
 */
int apple_gpio_parse_function(const void *value, u32 len, struct apple_gpio_function *out);
bool apple_gpio_function_is_mmio_gpio(const struct apple_gpio_function *fn);
u32 apple_gpio_reg_mode(u32 reg);
bool apple_gpio_reg_is_locked(u32 reg);
bool apple_gpio_reg_level(u32 reg);
/* Value to write to drive `pin` as an output at `level`, preserving pad config. */
u32 apple_gpio_reg_output(u32 reg, bool level);
/* True when `pin` is addressable inside a controller reg window of `size`. */
bool apple_gpio_pin_in_window(u32 pin, u64 size);

#ifndef APPLE_GPIO_HOST_TEST

/*
 * Resolve `function-<name>` on ADT node `node` to a memory-mapped AP GPIO
 * pin.  Returns 0 on success.  A missing property, a malformed record, a
 * non-"GPIO" FourCC (e.g. an SMC-backed rail), an unresolvable phandle or an
 * out-of-window pin all return < 0 without touching any register, so callers
 * can treat "no PERST# GPIO here" as a normal, non-fatal outcome.
 */
int apple_gpio_resolve_function(int node, const char *name, struct apple_gpio_pin *out);

/*
 * Drive a resolved pin as an output at `level`, preserving every other field
 * in the pad register.  Fails closed: a locked pad, or a read-back that does
 * not match, returns < 0.
 */
int apple_gpio_set_output(const struct apple_gpio_pin *pin, bool level);

#endif /* !APPLE_GPIO_HOST_TEST */

#endif

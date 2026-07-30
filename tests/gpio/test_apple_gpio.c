/* SPDX-License-Identifier: MIT */

/*
 * Host tests for the Apple AP GPIO pad decode and the ADT `function-*` record
 * parser.  Every expected value here is anchored to a named source:
 *
 *  - Register layout: Linux drivers/pinctrl/pinctrl-apple-gpio.c (REG_GPIOx_*).
 *  - FourCC byte order: proxyclient/m1n1/utils.py, FourCC = ExprAdapter(Int32ul,
 *    lambda d: d.to_bytes(4, "big")), i.e. a property rendered as 'GPIO' is
 *    stored little-endian and therefore reads as the bytes 'O','I','P','G'.
 *  - The pristine J414s pad value 0x00076a02 was captured from live hardware on
 *    /arm-io/gpio0 pins 4 and 5 (both ports read identically).
 */

#include "gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;

static void check(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "apple-gpio: FAIL: %s\n", message);
        failures++;
    }
}

static void check_u32(u32 got, u32 want, const char *message)
{
    if (got != want) {
        fprintf(stderr, "apple-gpio: FAIL: %s (got %#x, want %#x)\n", message, got, want);
        failures++;
    }
}

/* The live J414s PERST# pad, before m1n1 touches it. */
#define J414S_PERST_PAD_PRISTINE UINT32_C(0x00076a02)

static void test_register_layout(void)
{
    check_u32((u32)APPLE_GPIO_DATA, 0x1u, "DATA is BIT(0)");
    check_u32((u32)APPLE_GPIO_MODE, 0xeu, "MODE is GENMASK(3, 1)");
    check_u32((u32)APPLE_GPIO_PERIPH, 0x60u, "PERIPH is GENMASK(6, 5)");
    check_u32((u32)APPLE_GPIO_PULL, 0x180u, "PULL is GENMASK(8, 7)");
    check_u32((u32)APPLE_GPIO_INPUT_ENABLE, 0x200u, "INPUT_ENABLE is BIT(9)");
    check_u32((u32)APPLE_GPIO_DRIVE_STRENGTH0, 0xc00u, "DRIVE_STRENGTH0 is GENMASK(11, 10)");
    check_u32((u32)APPLE_GPIO_SCHMITT, 0x8000u, "SCHMITT is BIT(15)");
    check_u32((u32)APPLE_GPIO_GRP, 0x70000u, "GRP is GENMASK(18, 16)");
    check_u32((u32)APPLE_GPIO_LOCK, 0x200000u, "LOCK is BIT(21)");
    check_u32((u32)APPLE_GPIO_DRIVE_STRENGTH1, 0xc00000u, "DRIVE_STRENGTH1 is GENMASK(23, 22)");

    check_u32(APPLE_GPIO_MODE_OUT, 1u, "MODE value 1 is output");

    /* One 32-bit register per pin. */
    check_u32((u32)APPLE_GPIO_REG(0), 0u, "pin 0 is at offset 0");
    check_u32((u32)APPLE_GPIO_REG(4), 16u, "pin 4 is at offset 0x10");
    check_u32((u32)APPLE_GPIO_REG(5), 20u, "pin 5 is at offset 0x14");
}

static void test_pristine_j414s_pad(void)
{
    u32 reg = J414S_PERST_PAD_PRISTINE;

    /*
     * The decode that matters: the pad is already an output, it is not locked,
     * and PERST# is asserted (DATA == 0, pad is active low).  If any of these
     * were false, driving the pin would need more than a DATA write.
     */
    check_u32(apple_gpio_reg_mode(reg), APPLE_GPIO_MODE_OUT,
              "J414s PERST# pad is already MODE=OUT");
    check(!apple_gpio_reg_is_locked(reg), "J414s PERST# pad is not locked");
    check(!apple_gpio_reg_level(reg), "J414s PERST# pad reads low, i.e. PERST# asserted");

    /* Corroborating field decode, so a future layout change is caught here. */
    check_u32((u32)FIELD_GET(APPLE_GPIO_PULL, reg), APPLE_GPIO_PULL_OFF, "pull is off");
    check(reg & APPLE_GPIO_INPUT_ENABLE, "input enable is set");
    check_u32((u32)FIELD_GET(APPLE_GPIO_DRIVE_STRENGTH0, reg), 2u, "drive strength 0 is 2");
    check(!(reg & APPLE_GPIO_SCHMITT), "schmitt is off");
    check_u32((u32)FIELD_GET(APPLE_GPIO_GRP, reg), 7u, "group is 7");
    check_u32((u32)FIELD_GET(APPLE_GPIO_DRIVE_STRENGTH1, reg), 0u, "drive strength 1 is 0");
}

static void test_reg_output(void)
{
    u32 reg = J414S_PERST_PAD_PRISTINE;

    /*
     * Releasing PERST# must produce exactly the value the live probe accepted
     * (0x00076a03): only DATA changes, every pad-config field is preserved.
     */
    check_u32(apple_gpio_reg_output(reg, true), UINT32_C(0x00076a03),
              "releasing PERST# only sets DATA");
    check_u32(apple_gpio_reg_output(reg, false), UINT32_C(0x00076a02),
              "asserting PERST# only clears DATA");

    /* Idempotent. */
    check_u32(apple_gpio_reg_output(apple_gpio_reg_output(reg, true), true),
              UINT32_C(0x00076a03), "setting an already-high pad is idempotent");

    /* A pad left in an input/IRQ mode is switched to output, nothing else. */
    reg = (u32)FIELD_PREP(APPLE_GPIO_MODE, APPLE_GPIO_MODE_IN_IRQ_ANY) | APPLE_GPIO_GRP;
    check_u32(apple_gpio_reg_mode(apple_gpio_reg_output(reg, false)), APPLE_GPIO_MODE_OUT,
              "an input pad is switched to output");
    check_u32((u32)FIELD_GET(APPLE_GPIO_GRP, apple_gpio_reg_output(reg, false)), 7u,
              "switching to output preserves the group field");

    /* Every unrelated bit survives, including the reserved ones. */
    reg = UINT32_C(0xffffffff) & ~(u32)APPLE_GPIO_DATA;
    check_u32(apple_gpio_reg_output(reg, false),
              (UINT32_C(0xffffffff) & ~(u32)APPLE_GPIO_MODE & ~(u32)APPLE_GPIO_DATA) |
                  (u32)FIELD_PREP(APPLE_GPIO_MODE, APPLE_GPIO_MODE_OUT),
              "only MODE and DATA are rewritten");
}

static void test_pin_window(void)
{
    /* /arm-io/gpio0 on J414s is 0x4000 bytes, so pins 0..4095 are addressable. */
    check(apple_gpio_pin_in_window(4, 0x4000), "pin 4 fits the J414s gpio0 window");
    check(apple_gpio_pin_in_window(5, 0x4000), "pin 5 fits the J414s gpio0 window");
    check(apple_gpio_pin_in_window(4095, 0x4000), "the last pin fits");
    check(!apple_gpio_pin_in_window(4096, 0x4000), "one past the last pin is rejected");
    check(!apple_gpio_pin_in_window(0, 0), "an empty window addresses nothing");
    check(!apple_gpio_pin_in_window(1, 4), "a one-register window holds only pin 0");
}

/*
 * Build the on-disk bytes of an ADT `function-*` property.  The FourCC is
 * written in the memory order the ADT actually uses, which is the reverse of
 * the rendered name.
 */
static u32 build_function(u8 *out, u32 phandle, const char *rendered_name, const u32 *args,
                          u32 arg_count)
{
    u32 len = 0;

    out[len++] = (u8)(phandle & 0xff);
    out[len++] = (u8)((phandle >> 8) & 0xff);
    out[len++] = (u8)((phandle >> 16) & 0xff);
    out[len++] = (u8)((phandle >> 24) & 0xff);

    for (int i = 3; i >= 0; i--)
        out[len++] = (u8)rendered_name[i];

    for (u32 i = 0; i < arg_count; i++) {
        out[len++] = (u8)(args[i] & 0xff);
        out[len++] = (u8)((args[i] >> 8) & 0xff);
        out[len++] = (u8)((args[i] >> 16) & 0xff);
        out[len++] = (u8)((args[i] >> 24) & 0xff);
    }

    return len;
}

static void test_function_fourcc_byte_order(void)
{
    u8 raw[32];
    const u32 args[] = {4, 0};
    u32 len = build_function(raw, 163, "GPIO", args, 2);
    struct apple_gpio_function fn;

    /*
     * Pin the byte order explicitly: a property rendered 'GPIO' is stored as
     * 'O','I','P','G'.  If this ever flips, the FourCC guard below would start
     * accepting SMC-backed rails as MMIO pads.
     */
    check_u32(raw[4], (u8)'O', "fourcc byte 0 is 'O'");
    check_u32(raw[5], (u8)'I', "fourcc byte 1 is 'I'");
    check_u32(raw[6], (u8)'P', "fourcc byte 2 is 'P'");
    check_u32(raw[7], (u8)'G', "fourcc byte 3 is 'G'");
    check_u32(APPLE_GPIO_FUNCTION_FOURCC, UINT32_C(0x4750494f), "'GPIO' is 0x4750494f");

    check(apple_gpio_parse_function(raw, len, &fn) == 0, "the J414s record parses");
    check_u32(fn.phandle, 163u, "phandle is 163");
    check_u32(fn.fourcc, APPLE_GPIO_FUNCTION_FOURCC, "fourcc is 'GPIO'");
    check_u32(fn.pin, 4u, "pin is 4");
    check(fn.have_arg1 && fn.arg1 == 0, "the second arg is 0");
    check(apple_gpio_function_is_mmio_gpio(&fn), "'GPIO' is a memory-mapped controller");
}

static void test_function_parse_j414s_ports(void)
{
    u8 raw[32];
    struct apple_gpio_function fn;
    const u32 port0_args[] = {4, 0};
    const u32 port1_args[] = {5, 0};

    /*
     * The two J414s ports, as measured: pci-bridge0 uses pin 4 and pci-bridge1
     * uses pin 5, both on phandle 163 (/arm-io/gpio0).  This is also what
     * Asahi's arch/arm64/boot/dts/apple/t602x-die0.dtsi declares as
     * reset-gpios = <&pinctrl_ap 4 GPIO_ACTIVE_LOW> / <&pinctrl_ap 5 ...>.
     */
    check(apple_gpio_parse_function(raw, build_function(raw, 163, "GPIO", port0_args, 2), &fn) == 0,
          "bridge0 record parses");
    check_u32(fn.pin, 4u, "bridge0 PERST# is pin 4");

    check(apple_gpio_parse_function(raw, build_function(raw, 163, "GPIO", port1_args, 2), &fn) == 0,
          "bridge1 record parses");
    check_u32(fn.pin, 5u, "bridge1 PERST# is pin 5");
}

static void test_function_rejects(void)
{
    u8 raw[32];
    struct apple_gpio_function fn;
    const u32 args[] = {22, 0};
    u32 len;

    /* Too short to hold phandle + fourcc + one arg. */
    len = build_function(raw, 163, "GPIO", args, 0);
    check(apple_gpio_parse_function(raw, len, &fn) < 0, "a record with no args is rejected");
    check(apple_gpio_parse_function(raw, 0, &fn) < 0, "an empty record is rejected");
    check(apple_gpio_parse_function(NULL, 12, &fn) < 0, "a NULL record is rejected");
    check(apple_gpio_parse_function(raw, 12, NULL) < 0, "a NULL output is rejected");

    /* Not a whole number of words. */
    len = build_function(raw, 163, "GPIO", args, 1);
    check(apple_gpio_parse_function(raw, len + 1, &fn) < 0, "a ragged record is rejected");

    /* Phandle 0 never names a node. */
    len = build_function(raw, 0, "GPIO", args, 1);
    check(apple_gpio_parse_function(raw, len, &fn) < 0, "phandle 0 is rejected");

    /*
     * An SMC-backed rail parses as a record but must never be driven as MMIO.
     * On J414s the SD reader's power enable is exactly this shape.
     */
    len = build_function(raw, 200, "gpio", args, 2);
    check(apple_gpio_parse_function(raw, len, &fn) == 0, "a non-GPIO record still parses");
    check(!apple_gpio_function_is_mmio_gpio(&fn),
          "a lowercase 'gpio' FourCC is not a memory-mapped controller");
    check(!apple_gpio_function_is_mmio_gpio(NULL), "a NULL function is not memory-mapped");
}

int main(void)
{
    test_register_layout();
    test_pristine_j414s_pad();
    test_reg_output();
    test_pin_window();
    test_function_fourcc_byte_order();
    test_function_parse_j414s_ports();
    test_function_rejects();

    if (failures) {
        fprintf(stderr, "apple-gpio: %d failure(s)\n", failures);
        return 1;
    }

    printf("apple-gpio: PASS\n");
    return 0;
}

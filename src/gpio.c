/* SPDX-License-Identifier: MIT */

#include "gpio.h"

#ifndef APPLE_GPIO_HOST_TEST
#include "adt.h"
#include "utils.h"
#endif

/*
 * The ADT stores the FourCC as a little-endian u32 (see gpio.h), so decode it
 * with an explicit byte assembly rather than a struct overlay.  Cached RVAs and
 * RE'd struct layouts have been wrong here before; keep the decode anchored to
 * the byte order that is asserted by the host tests.
 */
static u32 apple_gpio_load_u32(const u8 *bytes)
{
    return (u32)bytes[0] | ((u32)bytes[1] << 8) | ((u32)bytes[2] << 16) | ((u32)bytes[3] << 24);
}

int apple_gpio_parse_function(const void *value, u32 len, struct apple_gpio_function *out)
{
    const u8 *bytes = value;

    if (!value || !out)
        return -1;
    if (len < APPLE_GPIO_FUNCTION_MIN_LEN)
        return -1;
    /* Trailing bytes that do not form whole words mean this is not a function. */
    if (len % 4)
        return -1;

    *out = (struct apple_gpio_function){0};
    out->phandle = apple_gpio_load_u32(bytes + 0);
    out->fourcc = apple_gpio_load_u32(bytes + 4);
    out->pin = apple_gpio_load_u32(bytes + 8);
    if (len >= 16) {
        out->arg1 = apple_gpio_load_u32(bytes + 12);
        out->have_arg1 = true;
    }

    /* A phandle of 0 never names a node. */
    if (out->phandle == 0)
        return -1;

    return 0;
}

bool apple_gpio_function_is_mmio_gpio(const struct apple_gpio_function *fn)
{
    return fn && fn->fourcc == APPLE_GPIO_FUNCTION_FOURCC;
}

u32 apple_gpio_reg_mode(u32 reg)
{
    return (u32)FIELD_GET(APPLE_GPIO_MODE, reg);
}

bool apple_gpio_reg_is_locked(u32 reg)
{
    return (reg & APPLE_GPIO_LOCK) != 0;
}

bool apple_gpio_reg_level(u32 reg)
{
    return (reg & APPLE_GPIO_DATA) != 0;
}

u32 apple_gpio_reg_output(u32 reg, bool level)
{
    /*
     * Match Linux's apple_gpio_gpio_direction_output()/set(): only MODE and
     * DATA change.  Pull, drive strength, Schmitt, group and the peripheral
     * mux are pad configuration that iBoot already programmed correctly, and
     * rewriting them from guesswork is how pins get bricked.
     */
    reg &= ~(u32)APPLE_GPIO_MODE;
    reg |= (u32)FIELD_PREP(APPLE_GPIO_MODE, APPLE_GPIO_MODE_OUT);

    if (level)
        reg |= (u32)APPLE_GPIO_DATA;
    else
        reg &= ~(u32)APPLE_GPIO_DATA;

    return reg;
}

bool apple_gpio_pin_in_window(u32 pin, u64 size)
{
    /* One 32-bit register per pin; the pad register must fit entirely. */
    return APPLE_GPIO_REG(pin) + 4 <= size;
}

#ifndef APPLE_GPIO_HOST_TEST

/*
 * adt_get_reg() needs the full ancestry of the target node, not just its
 * offset, because `reg` may have to be translated through each parent's
 * `ranges`.  adt_path_offset_trace() builds that array for a path; there is no
 * equivalent for a phandle, so walk the tree and record the breadcrumbs the
 * same way (children of the root only -- the root's own offset is 0 and is
 * used as the array terminator, exactly as ADTNode::from_path_trace() does in
 * rust/src/adt.rs).
 */
#define APPLE_GPIO_ADT_MAX_DEPTH 8

static int apple_gpio_find_phandle(int node, u32 phandle, int *path, int depth)
{
    /* Leave room for this level plus the terminating zero. */
    if (depth + 1 >= APPLE_GPIO_ADT_MAX_DEPTH)
        return -1;

    ADT_FOREACH_CHILD(adt, node)
    {
        u32 value;

        path[depth] = node;
        path[depth + 1] = 0;

        if (ADT_GETPROP(adt, node, "AAPL,phandle", &value) >= 0 && value == phandle)
            return node;

        int found = apple_gpio_find_phandle(node, phandle, path, depth + 1);
        if (found >= 0)
            return found;
    }

    path[depth] = 0;
    return -1;
}

int apple_gpio_resolve_function(int node, const char *name, struct apple_gpio_pin *out)
{
    char prop[64];
    int path[APPLE_GPIO_ADT_MAX_DEPTH];
    struct apple_gpio_function fn;
    const void *value;
    u32 len = 0;
    u64 base, size;

    if (!out)
        return -1;
    *out = (struct apple_gpio_pin){0};

    snprintf(prop, sizeof(prop), "function-%s", name);

    value = adt_getprop(adt, node, prop, &len);
    if (!value || !len)
        return -1;

    if (apple_gpio_parse_function(value, len, &fn) < 0) {
        printf("gpio: %s is malformed (%u bytes)\n", prop, len);
        return -1;
    }

    if (!apple_gpio_function_is_mmio_gpio(&fn)) {
        /*
         * Some rails are SMC-backed (e.g. the SD reader's power enable).
         * Those are not MMIO pads and must never be written as such.
         */
        printf("gpio: %s targets FourCC %#x, not a memory-mapped GPIO\n", prop, fn.fourcc);
        return -1;
    }

    for (int i = 0; i < APPLE_GPIO_ADT_MAX_DEPTH; i++)
        path[i] = 0;

    if (apple_gpio_find_phandle(0, fn.phandle, path, 0) < 0) {
        printf("gpio: %s phandle %u does not resolve\n", prop, fn.phandle);
        return -1;
    }

    if (adt_get_reg(adt, path, "reg", 0, &base, &size)) {
        printf("gpio: %s controller has no usable reg\n", prop);
        return -1;
    }

    if (!apple_gpio_pin_in_window(fn.pin, size)) {
        printf("gpio: %s pin %u is outside the %#lx byte window at %#lx\n", prop, fn.pin, size,
               base);
        return -1;
    }

    out->base = base;
    out->pin = fn.pin;
    out->valid = true;

    return 0;
}

int apple_gpio_set_output(const struct apple_gpio_pin *pin, bool level)
{
    u64 address;
    u32 reg, want, got;

    if (!pin || !pin->valid)
        return -1;

    address = pin->base + APPLE_GPIO_REG(pin->pin);
    reg = read32(address);

    if (apple_gpio_reg_is_locked(reg)) {
        printf("gpio: pad %u at %#lx is locked (%#x)\n", pin->pin, pin->base, reg);
        return -1;
    }

    want = apple_gpio_reg_output(reg, level);
    write32(address, want);

    got = read32(address);
    if (got != want) {
        printf("gpio: pad %u at %#lx rejected %#x (read %#x)\n", pin->pin, pin->base, want, got);
        return -1;
    }

    return 0;
}

#endif /* !APPLE_GPIO_HOST_TEST */

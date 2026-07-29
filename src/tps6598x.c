/* SPDX-License-Identifier: MIT */

#include "tps6598x.h"
#include "adt.h"
#include "i2c.h"
#include "iodev.h"
#include "malloc.h"
#include "tps6598x_host_policy.h"
#include "types.h"
#include "utils.h"

#define TPS_REG_CMD1          0x08
#define TPS_REG_DATA1         0x09
#define TPS_REG_INT_EVENT1    0x14
#define TPS_REG_INT_MASK1     0x16
#define TPS_REG_INT_CLEAR1    0x18
#define TPS_REG_POWER_STATE   0x20
#define TPS_REG_SYSTEM_CONFIG 0x28
#define TPS_CMD_INVALID       0x21434d44 // !CMD
#define TPS_CMD_TIMEOUT_US    (5 * 1000 * 1000)

struct tps6598x_dev {
    i2c_dev_t *i2c;
    u8 addr;
};

tps6598x_dev_t *tps6598x_init(const char *adt_node, i2c_dev_t *i2c)
{
    int adt_offset;
    adt_offset = adt_path_offset(adt, adt_node);
    if (adt_offset < 0) {
        printf("tps6598x: Error getting %s node\n", adt_node);
        return NULL;
    }

    const u8 *iic_addr = adt_getprop(adt, adt_offset, "hpm-iic-addr", NULL);
    if (iic_addr == NULL) {
        printf("tps6598x: Error getting %s hpm-iic-addr\n.", adt_node);
        return NULL;
    }

    tps6598x_dev_t *dev = calloc(1, sizeof(*dev));
    if (!dev)
        return NULL;

    dev->i2c = i2c;
    dev->addr = *iic_addr;
    return dev;
}

void tps6598x_shutdown(tps6598x_dev_t *dev)
{
    free(dev);
}

int tps6598x_command(tps6598x_dev_t *dev, const char *cmd, const u8 *data_in, size_t len_in,
                     u8 *data_out, size_t len_out)
{
    if (len_in) {
        if (i2c_smbus_write(dev->i2c, dev->addr, TPS_REG_DATA1, data_in, len_in) < 0)
            return -1;
    }

    if (i2c_smbus_write(dev->i2c, dev->addr, TPS_REG_CMD1, (const u8 *)cmd, 4) < 0)
        return -1;

    u32 cmd_status;
    u64 timeout = timeout_calculate(TPS_CMD_TIMEOUT_US);
    do {
        if (i2c_smbus_read32(dev->i2c, dev->addr, TPS_REG_CMD1, &cmd_status))
            return -1;
        if (cmd_status == TPS_CMD_INVALID)
            return -1;
        if (cmd_status == 0)
            break;
        if (timeout_expired(timeout)) {
            printf("tps6598x: command %.4s timed out\n", cmd);
            return -1;
        }
        udelay(100);
    } while (true);

    if (len_out) {
        if (i2c_smbus_read(dev->i2c, dev->addr, TPS_REG_DATA1, data_out, len_out) !=
            (ssize_t)len_out)
            return -1;
    }

    return 0;
}

int tps6598x_disable_irqs(tps6598x_dev_t *dev, tps6598x_irq_state_t *state)
{
    size_t read;
    int written;
    static const u8 zeros[CD3218B12_IRQ_WIDTH] = {0x00};
    static const u8 ones[CD3218B12_IRQ_WIDTH] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                                 0xFF, 0xFF, 0xFF, 0xFF};

    // store IntEvent 1 to restore it later
    read = i2c_smbus_read(dev->i2c, dev->addr, TPS_REG_INT_MASK1, state->int_mask1,
                          sizeof(state->int_mask1));
    if (read != CD3218B12_IRQ_WIDTH) {
        printf("tps6598x: reading TPS_REG_INT_MASK1 failed\n");
        return -1;
    }
    state->valid = 1;

    // mask interrupts and ack all interrupt flags
    written = i2c_smbus_write(dev->i2c, dev->addr, TPS_REG_INT_CLEAR1, ones, sizeof(ones));
    if (written != sizeof(zeros)) {
        printf("tps6598x: writing TPS_REG_INT_CLEAR1 failed, written: %d\n", written);
        return -1;
    }
    written = i2c_smbus_write(dev->i2c, dev->addr, TPS_REG_INT_MASK1, zeros, sizeof(zeros));
    if (written != sizeof(ones)) {
        printf("tps6598x: writing TPS_REG_INT_MASK1 failed, written: %d\n", written);
        return -1;
    }

#ifdef DEBUG
    u8 tmp[CD3218B12_IRQ_WIDTH] = {0x00};
    read = i2c_smbus_read(dev->i2c, dev->addr, TPS_REG_INT_MASK1, tmp, CD3218B12_IRQ_WIDTH);
    if (read != CD3218B12_IRQ_WIDTH)
        printf("tps6598x: failed verification, can't read TPS_REG_INT_MASK1\n");
    else {
        printf("tps6598x: verify: TPS_REG_INT_MASK1 vs. saved IntMask1\n");
        hexdump(tmp, sizeof(tmp));
        hexdump(state->int_mask1, sizeof(state->int_mask1));
    }
#endif
    return 0;
}

int tps6598x_restore_irqs(tps6598x_dev_t *dev, tps6598x_irq_state_t *state)
{
    int written;

    written = i2c_smbus_write(dev->i2c, dev->addr, TPS_REG_INT_MASK1, state->int_mask1,
                              sizeof(state->int_mask1));
    if (written != sizeof(state->int_mask1)) {
        printf("tps6598x: restoring TPS_REG_INT_MASK1 failed\n");
        return -1;
    }

#ifdef DEBUG
    int read;
    u8 tmp[CD3218B12_IRQ_WIDTH];
    read = i2c_smbus_read(dev->i2c, dev->addr, TPS_REG_INT_MASK1, tmp, sizeof(tmp));
    if (read != sizeof(tmp))
        printf("tps6598x: failed verification, can't read TPS_REG_INT_MASK1\n");
    else {
        printf("tps6598x: verify saved IntMask1 vs. TPS_REG_INT_MASK1:\n");
        hexdump(state->int_mask1, sizeof(state->int_mask1));
        hexdump(tmp, sizeof(tmp));
    }
#endif

    return 0;
}

int tps6598x_powerup(tps6598x_dev_t *dev)
{
    u8 power_state;

    if (i2c_smbus_read8(dev->i2c, dev->addr, TPS_REG_POWER_STATE, &power_state))
        return -1;

    if (power_state == 0)
        return 0;

    const u8 data = 0;
    tps6598x_command(dev, "SSPS", &data, 1, NULL, 0);

    if (i2c_smbus_read8(dev->i2c, dev->addr, TPS_REG_POWER_STATE, &power_state))
        return -1;

    if (power_state != 0)
        return -1;

    return 0;
}

int tps6598x_prepare_host(tps6598x_dev_t *dev, u32 hpm_index, u32 controller_count,
                          s32 preserved_index)
{
    u8 current[TPS6598X_SYSTEM_CONFIG_LEN];
    u8 desired[TPS6598X_SYSTEM_CONFIG_LEN];
    u8 readback[TPS6598X_SYSTEM_CONFIG_LEN];

    int read = i2c_smbus_read(dev->i2c, dev->addr, TPS_REG_SYSTEM_CONFIG, current, sizeof(current));
    if (read != sizeof(current)) {
        printf("tps6598x: hpm%u failed to read System Configuration\n", hpm_index);
        return -1;
    }

    int ret = tps6598x_host_policy_prepare(hpm_index, controller_count, preserved_index, current,
                                           desired);
    if (ret == TPS6598X_HOST_POLICY_SKIP)
        return 0;
    if (ret < 0) {
        printf("tps6598x: hpm%u refused host policy (PortInfo=%u, error=%d)\n", hpm_index,
               current[0] & 7, ret);
        return -1;
    }
    if (ret == TPS6598X_HOST_POLICY_READY) {
        printf("tps6598x: hpm%u already Source/DFP (PortInfo=%u)\n", hpm_index, current[0] & 7);
        return 0;
    }

    int written =
        i2c_smbus_write(dev->i2c, dev->addr, TPS_REG_SYSTEM_CONFIG, desired, sizeof(desired));
    if (written != sizeof(desired)) {
        printf("tps6598x: hpm%u failed to write System Configuration\n", hpm_index);
        return -1;
    }

    read = i2c_smbus_read(dev->i2c, dev->addr, TPS_REG_SYSTEM_CONFIG, readback, sizeof(readback));
    if (read != sizeof(readback) || tps6598x_host_policy_verify(desired, readback) < 0) {
        printf("tps6598x: hpm%u System Configuration readback mismatch\n", hpm_index);
        return -1;
    }

    printf("tps6598x: hpm%u Source/DFP policy applied (PortInfo %u -> %u)\n", hpm_index,
           current[0] & 7, desired[0] & 7);
    return 0;
}

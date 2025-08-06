#include "driver_icm20x48.h"
#include "icm20x48_internal.h"
#include <common/bswap.h>
#include <modules/timing/timing.h>
#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#include <modules/uavcan_debug/uavcan_debug.h>
#define ICM_DEBUG(...) uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "ICM", __VA_ARGS__)
#else
#define ICM_DEBUG(...) {}
#endif

static uint8_t icm20x48_get_whoami(enum icm20x48_imu_type_t imu_type);
static void icm20x48_select_bank(struct icm20x48_instance_s* instance, uint8_t bank);

bool icm20x48_init(struct icm20x48_instance_s* instance, uint8_t spi_idx, uint32_t select_line, enum icm20x48_imu_type_t imu_type) {
    // Ensure sufficient power-up time has elapsed
    chThdSleep(LL_MS2ST(100));

    instance->curr_bank = 99;

    spi_device_init(&instance->spi_dev, spi_idx, select_line, 7000000, 16, SPI_DEVICE_FLAG_CPHA|SPI_DEVICE_FLAG_CPOL);

    if (icm20x48_read_reg(instance, ICM20948_REG_WHO_AM_I) != icm20x48_get_whoami(imu_type)) {
        return false;
    }

    // Read USER_CTRL, disable MST_I2C, write USER_CTRL, and wait long enough for any active I2C transaction to complete
    icm20x48_write_reg(instance, ICM20948_REG_USER_CTRL,  icm20x48_read_reg(instance, ICM20948_REG_USER_CTRL) & ~(1<<5));
    chThdSleep(LL_MS2ST(10));
    // Add an interrupt on sensor data ready
    icm20x48_write_reg(instance, ICM20948_REG_INT_ENABLE_1, 1<<0);
    chThdSleep(LL_MS2ST(10));

    // // set configs for the accelerometer and gyro

    // // select user bank 2
    // icm20x48_select_bank(instance, REG_BANK2);
    // chThdSleep(LL_MS2ST(10));
    // // set accel config
    // icm20x48_write_reg(instance, ICM20948_REG_ACCEL_CONFIG, (0<<2)|(1<<1)|(1<<0)); // set bits 1 and 0 to 1
    // chThdSleep(LL_MS2ST(10));
    // // set gyro config
    // icm20x48_write_reg(instance, ICM20948_REG_GYRO_CONFIG_1, (0<<2)|(1<<1)|(1<<0)); //500dps full scale
    // chThdSleep(LL_MS2ST(10));
    // // select user bank 0
    // icm20x48_select_bank(instance, REG_BANK0);
    // chThdSleep(LL_MS2ST(10));

    // Perform a device reset, wait for completion, then wake the device
    // Datasheet is unclear on time required for wait time after reset, but mentions 100ms under "start-up time for register read/write from power-up"
    icm20x48_write_reg(instance, ICM20948_REG_PWR_MGMT_1, 1<<7);
    chThdSleep(LL_MS2ST(100));

    icm20x48_write_reg(instance, ICM20948_REG_PWR_MGMT_1, 1);
    // Wait for reset to complete
    {
        uint32_t tbegin = chVTGetSystemTimeX();
        while (icm20x48_read_reg(instance, ICM20948_REG_PWR_MGMT_1) & 1<<7) {
            uint32_t tnow = chVTGetSystemTimeX();
            if (tnow-tbegin > LL_MS2ST(100)) {
                return false;
            }
        }
    }

    chThdSleep(LL_MS2ST(10));

    return true;
}

void icm20x48_i2c_slv_read_enable(struct icm20x48_instance_s* instance)
{
    // Disable the I2C slave interface (SPI-only) and enable the I2C master interface to the AK09916
    icm20x48_write_reg(instance, ICM20948_REG_USER_CTRL, (1<<4)|(1<<5));

    // Set up the I2C master clock as recommended in the datasheet
    icm20x48_write_reg(instance, ICM20948_REG_I2C_MST_CTRL, 7);

    // Configure the I2C master to enable access at sample rate specified in SLV4
    icm20x48_write_reg(instance, ICM20948_REG_I2C_MST_DELAY_CTRL, 1<<7);
}

bool icm20x48_i2c_slv_set_passthrough(struct icm20x48_instance_s* instance, uint8_t slv_id, uint8_t addr, uint8_t reg, uint8_t size)
{
    if (size > 15) {
        return false;
    }
    //icm20x48_write_reg(instance, ICM20948_REG_I2C_MST_STATUS, 0);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV0_ADDR + 4*slv_id, addr);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV0_REG  + 4*slv_id, reg);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV0_CTRL + 4*slv_id, (1<<7)|size);
    return true;
}

static void icm20x48_select_bank(struct icm20x48_instance_s* instance, uint8_t bank) {
    uint16_t data = (ICM20948_REG_BANK_SEL << 8) | (bank<<4);
    instance->curr_bank = bank;
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, 1, &data);
    spi_device_end(&instance->spi_dev);
}

uint8_t icm20x48_read_reg(struct icm20x48_instance_s* instance, uint16_t reg){

    uint16_t _reg = (((uint16_t)(GET_REG(reg) | 0x80)) << 8);
    uint16_t ret = 0;
    uint8_t _bank = GET_BANK(reg);
    if (_bank != instance->curr_bank) {
        icm20x48_select_bank(instance, _bank);
    }
    spi_device_begin(&instance->spi_dev);
    spi_device_exchange(&instance->spi_dev, 1, &_reg, &ret);
    chThdSleepMicroseconds(2);
    spi_device_end(&instance->spi_dev);
    return ret;
}

void icm20x48_write_reg(struct icm20x48_instance_s* instance, uint16_t reg, uint8_t value) {
    uint8_t _bank = GET_BANK(reg);
    uint16_t data = (((uint16_t)GET_REG(reg)) << 8) | value;
    if (_bank != instance->curr_bank) {
        icm20x48_select_bank(instance, _bank);
    }
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, 1 , &data);
    chThdSleepMicroseconds(2);
    spi_device_end(&instance->spi_dev);
}

static uint8_t icm20x48_get_whoami(enum icm20x48_imu_type_t imu_type) {
    switch(imu_type) {
        case ICM20x48_IMU_TYPE_ICM20948:
            return 0xEA;
    }
    return 0;
}

static bool icm20x48_i2c_slv_wait_for_completion(struct icm20x48_instance_s* instance, uint32_t timeout_us) {
    uint32_t tbegin_us = micros();
    while(!(icm20x48_read_reg(instance, ICM20948_REG_I2C_MST_STATUS) & (1<<6))) {
        if (micros() - tbegin_us > timeout_us) {
            return false;
        }
    }
    return true;
}

bool icm20x48_i2c_slv_read(struct icm20x48_instance_s* instance, uint8_t address, uint8_t reg, uint8_t* ret) {
    icm20x48_write_reg(instance, ICM20948_REG_I2C_MST_STATUS, 0);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV4_ADDR, address|0x80);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV4_REG, reg);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV4_CTRL, (1<<7)|(1<<6));
    if (!icm20x48_i2c_slv_wait_for_completion(instance, 10000)) {
        return false;
    }
    *ret = icm20x48_read_reg(instance, ICM20948_REG_I2C_SLV4_DI);
    return true;
}

bool icm20x48_i2c_slv_write(struct icm20x48_instance_s* instance, uint8_t address, uint8_t reg, uint8_t value) {
    icm20x48_write_reg(instance, ICM20948_REG_I2C_MST_STATUS, 0);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV4_ADDR, address);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV4_REG, reg);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV4_DO, value);
    icm20x48_write_reg(instance, ICM20948_REG_I2C_SLV4_CTRL, (1<<7)|(1<<6));
    if (!icm20x48_i2c_slv_wait_for_completion(instance, 10000)) {
        return false;
    }
    return true;
}


bool icm20x48_update(struct icm20x48_instance_s* instance) {

    if ((icm20x48_read_reg(instance, ICM20948_REG_INT_STATUS_1)) & (1<<0) == 0) {
        return false;
    }

    uint8_t accel_bytes[6];
    uint8_t gyro_bytes[6];

    for (uint8_t i = 0; i < 6; i++) {
        accel_bytes[i] = icm20x48_read_reg(instance, ICM20948_REG_ACCEL_XOUT_H + i);
        gyro_bytes[i] = icm20x48_read_reg(instance, ICM20948_REG_GYRO_XOUT_H + i);
    }

    int16_t accel_data[3];
    int16_t gyro_data[3];

    // combine high and low bytes into 16-bit values
    for (uint8_t i = 0; i < 3; i++) {
        accel_data[i] = (int16_t)(((uint16_t)accel_bytes[i * 2] << 8) | accel_bytes[i * 2 + 1]);
        gyro_data[i] = (int16_t)(((uint16_t)gyro_bytes[i * 2] << 8) | gyro_bytes[i * 2 + 1]);
    }

    instance->meas.ax = (float)accel_data[0] * 9.81f / 16384.0f;
    instance->meas.ay = (float)accel_data[1] * 9.81f / 16384.0f;
    instance->meas.az = (float)accel_data[2] * 9.81f / 16384.0f;
    instance->meas.gx = (float)gyro_data[0] * 250.0f / 32768.0f;
    instance->meas.gy = (float)gyro_data[1] * 250.0f / 32768.0f;
    instance->meas.gz = (float)gyro_data[2] * 250.0f / 32768.0f;

    return true;
}

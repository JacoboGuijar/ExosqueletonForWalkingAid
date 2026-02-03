#define _DEFAULT_SOURCE
#include "bno055.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>

static uint8_t bno055_i2c_address = BNO055_ADDRESS_A;

/**
 * Write a byte to a BNO055 register
 */
static int bno055_write_register(int fd, uint8_t reg, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;

    if (write(fd, buffer, 2) != 2) {
        perror("Failed to write to I2C device");
        return -1;
    }
    return 0;
}

/**
 * Read a byte from a BNO055 register
 */
static int bno055_read_register(int fd, uint8_t reg, uint8_t *value) {
    if (write(fd, &reg, 1) != 1) {
        perror("Failed to write register address to I2C device");
        return -1;
    }

    if (read(fd, value, 1) != 1) {
        perror("Failed to read from I2C device");
        return -1;
    }
    return 0;
}

/**
 * Read multiple bytes from BNO055 registers
 */
static int bno055_read_registers(int fd, uint8_t reg, uint8_t *buffer, uint8_t len) {
    if (write(fd, &reg, 1) != 1) {
        perror("Failed to write register address to I2C device");
        return -1;
    }

    if (read(fd, buffer, len) != len) {
        perror("Failed to read from I2C device");
        return -1;
    }
    return 0;
}

int bno055_init(const char *i2c_bus, uint8_t address) {
    int fd;
    uint8_t chip_id;

    // Store the I2C address
    bno055_i2c_address = address;

    // Open I2C bus
    fd = open(i2c_bus, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open I2C bus %s: %s\n", i2c_bus, strerror(errno));
        return -1;
    }

    // Set I2C slave address
    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "Failed to set I2C address 0x%02X: %s\n", address, strerror(errno));
        close(fd);
        return -1;
    }

    // Give the sensor some time to start up
    usleep(100000);  // 100ms delay

    // Verify chip ID
    if (bno055_read_register(fd, BNO055_CHIP_ID_ADDR, &chip_id) < 0) {
        fprintf(stderr, "Failed to read chip ID\n");
        close(fd);
        return -1;
    }

    if (chip_id != BNO055_CHIP_ID_VALUE) {
        fprintf(stderr, "Invalid chip ID: 0x%02X (expected 0x%02X)\n",
                chip_id, BNO055_CHIP_ID_VALUE);
        close(fd);
        return -1;
    }

    printf("BNO055 detected (Chip ID: 0x%02X)\n", chip_id);

    // Reset the sensor
    if (bno055_write_register(fd, BNO055_SYS_TRIGGER_ADDR, 0x20) < 0) {
        fprintf(stderr, "Failed to reset sensor\n");
        close(fd);
        return -1;
    }

    // Wait for reset to complete
    usleep(650000);  // 650ms delay (as per datasheet)

    // Verify chip ID again after reset
    uint8_t retries = 0;
    while (retries < 10) {
        if (bno055_read_register(fd, BNO055_CHIP_ID_ADDR, &chip_id) == 0) {
            if (chip_id == BNO055_CHIP_ID_VALUE) {
                break;
            }
        }
        usleep(100000);  // 100ms delay
        retries++;
    }

    if (retries >= 10) {
        fprintf(stderr, "Sensor did not respond after reset\n");
        close(fd);
        return -1;
    }

    // Set to normal power mode
    if (bno055_write_register(fd, BNO055_PWR_MODE_ADDR, BNO055_PWR_MODE_NORMAL) < 0) {
        fprintf(stderr, "Failed to set power mode\n");
        close(fd);
        return -1;
    }
    usleep(10000);  // 10ms delay

    // Set page ID to 0
    if (bno055_write_register(fd, BNO055_PAGE_ID_ADDR, 0x00) < 0) {
        fprintf(stderr, "Failed to set page ID\n");
        close(fd);
        return -1;
    }

    // Set to NDOF (Nine Degrees of Freedom) mode for fusion quaternion output
    if (bno055_write_register(fd, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF) < 0) {
        fprintf(stderr, "Failed to set operation mode\n");
        close(fd);
        return -1;
    }
    usleep(20000);  // 20ms delay to enter NDOF mode

    printf("BNO055 initialized in NDOF mode\n");

    return fd;
}

void bno055_close(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

int bno055_read_quaternion(int fd, bno055_quaternion_t *quat) {
    uint8_t buffer[8];
    int16_t w_raw, x_raw, y_raw, z_raw;

    if (quat == NULL) {
        fprintf(stderr, "Invalid quaternion pointer\n");
        return -1;
    }

    // Read 8 bytes starting from quaternion W LSB register
    if (bno055_read_registers(fd, BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8) < 0) {
        return -1;
    }

    // Combine LSB and MSB into 16-bit signed integers
    w_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    x_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    y_raw = (int16_t)((buffer[5] << 8) | buffer[4]);
    z_raw = (int16_t)((buffer[7] << 8) | buffer[6]);

    // Scale to unit quaternion (divide by 2^14)
    quat->w = w_raw * BNO055_QUATERNION_SCALE;
    quat->x = x_raw * BNO055_QUATERNION_SCALE;
    quat->y = y_raw * BNO055_QUATERNION_SCALE;
    quat->z = z_raw * BNO055_QUATERNION_SCALE;

    return 0;
}

int bno055_read_chip_id(int fd) {
    uint8_t chip_id;

    if (bno055_read_register(fd, BNO055_CHIP_ID_ADDR, &chip_id) < 0) {
        return -1;
    }

    return chip_id;
}

int bno055_get_calibration_status(int fd) {
    uint8_t calib_stat;

    if (bno055_read_register(fd, BNO055_CALIB_STAT_ADDR, &calib_stat) < 0) {
        return -1;
    }

    return calib_stat;
}

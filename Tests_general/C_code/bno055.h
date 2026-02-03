#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>

// BNO055 I2C Addresses
#define BNO055_ADDRESS_A    0x28  // Default address (COM3 LOW)
#define BNO055_ADDRESS_B    0x29  // Alternative address (COM3 HIGH)

// BNO055 Register Addresses
#define BNO055_CHIP_ID_ADDR         0x00
#define BNO055_PAGE_ID_ADDR         0x07
#define BNO055_ACCEL_REV_ID_ADDR    0x01
#define BNO055_MAG_REV_ID_ADDR      0x02
#define BNO055_GYRO_REV_ID_ADDR     0x03
#define BNO055_SW_REV_ID_LSB_ADDR   0x04
#define BNO055_SW_REV_ID_MSB_ADDR   0x05
#define BNO055_BL_REV_ID_ADDR       0x06

// Power mode register
#define BNO055_PWR_MODE_ADDR        0x3E
#define BNO055_PWR_MODE_NORMAL      0x00

// Operation mode register
#define BNO055_OPR_MODE_ADDR        0x3D
#define BNO055_OPERATION_MODE_CONFIG    0x00
#define BNO055_OPERATION_MODE_NDOF      0x0C  // Nine degrees of freedom (fusion mode)

// System status and error registers
#define BNO055_SYS_TRIGGER_ADDR     0x3F
#define BNO055_SYS_STATUS_ADDR      0x39
#define BNO055_SYS_ERR_ADDR         0x3A

// Unit selection register
#define BNO055_UNIT_SEL_ADDR        0x3B

// Calibration status register
#define BNO055_CALIB_STAT_ADDR      0x35

// Quaternion data registers (LSB first)
#define BNO055_QUATERNION_DATA_W_LSB_ADDR   0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR   0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR   0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR   0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR   0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR   0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR   0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR   0x27

// Expected chip ID
#define BNO055_CHIP_ID_VALUE        0xA0

// Quaternion scale factor (LSB per unit)
// Quaternion data is in 1 unit = 2^14 LSB
#define BNO055_QUATERNION_SCALE     (1.0 / 16384.0)  // 2^14 = 16384

// Quaternion data structure
typedef struct {
    double w;  // Real component
    double x;  // i component
    double y;  // j component
    double z;  // k component
} bno055_quaternion_t;

// Function declarations

/**
 * Initialize the BNO055 sensor on the specified I2C bus and address
 *
 * @param i2c_bus I2C bus device path (e.g., "/dev/i2c-1")
 * @param address BNO055 I2C address (BNO055_ADDRESS_A or BNO055_ADDRESS_B)
 * @return File descriptor on success, -1 on failure
 */
int bno055_init(const char *i2c_bus, uint8_t address);

/**
 * Close the I2C connection to the BNO055 sensor
 *
 * @param fd File descriptor returned by bno055_init()
 */
void bno055_close(int fd);

/**
 * Read quaternion data from the BNO055 sensor
 *
 * @param fd File descriptor returned by bno055_init()
 * @param quat Pointer to quaternion structure to store the data
 * @return 0 on success, -1 on failure
 */
int bno055_read_quaternion(int fd, bno055_quaternion_t *quat);

/**
 * Read the chip ID to verify sensor communication
 *
 * @param fd File descriptor returned by bno055_init()
 * @return Chip ID on success, -1 on failure
 */
int bno055_read_chip_id(int fd);

/**
 * Get the current calibration status of the sensor
 *
 * @param fd File descriptor returned by bno055_init()
 * @return Calibration status byte (bits 7-6: SYS, 5-4: GYR, 3-2: ACC, 1-0: MAG), -1 on failure
 */
int bno055_get_calibration_status(int fd);

#endif // BNO055_H

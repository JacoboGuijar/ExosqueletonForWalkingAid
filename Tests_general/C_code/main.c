#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>
#include "bno055.h"

// Global flag for clean exit on Ctrl+C
static volatile bool keep_running = true;

/**
 * Signal handler for graceful shutdown
 */
void signal_handler(int signum) {
    (void)signum;  // Unused parameter
    keep_running = false;
    printf("\nShutting down...\n");
}

/**
 * Print calibration status in human-readable format
 */
void print_calibration_status(int calib_stat) {
    if (calib_stat < 0) {
        printf("Calibration status: N/A\n");
        return;
    }

    uint8_t sys_calib = (calib_stat >> 6) & 0x03;
    uint8_t gyr_calib = (calib_stat >> 4) & 0x03;
    uint8_t acc_calib = (calib_stat >> 2) & 0x03;
    uint8_t mag_calib = calib_stat & 0x03;

    printf("Calibration - SYS: %d, GYR: %d, ACC: %d, MAG: %d (0=uncalibrated, 3=fully calibrated)\n",
           sys_calib, gyr_calib, acc_calib, mag_calib);
}

/**
 * Calculate the angle between two quaternions in degrees
 *
 * @param q1 First quaternion (should be unit quaternion)
 * @param q2 Second quaternion (should be unit quaternion)
 * @return Angle in degrees (0-180°)
 */
double quaternion_angle_degrees(const bno055_quaternion_t *q1,
                                const bno055_quaternion_t *q2) {
    // Calculate dot product
    double dot = q1->w * q2->w +
                 q1->x * q2->x +
                 q1->y * q2->y +
                 q1->z * q2->z;

    // Clamp to avoid numerical errors in acos
    // (dot product should be in [-1, 1] for unit quaternions)
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;

    // Use absolute value to handle quaternion double-cover
    // (q and -q represent the same rotation)
    double angle_rad = 2.0 * acos(fabs(dot));

    // Convert to degrees
    double angle_deg = angle_rad * (180.0 / M_PI);

    return angle_deg;
}

/**
 * Calculate the conjugate of a quaternion
 * Conjugate: q* = (w, -x, -y, -z)
 *
 * @param q Input quaternion
 * @return Conjugate quaternion
 */
bno055_quaternion_t quaternion_conjugate(const bno055_quaternion_t *q) {
    bno055_quaternion_t result;
    result.w = q->w;
    result.x = -q->x;
    result.y = -q->y;
    result.z = -q->z;
    return result;
}

/**
 * Multiply two quaternions using Hamilton product
 * q_result = q1 × q2
 *
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Product quaternion
 */
bno055_quaternion_t quaternion_multiply(const bno055_quaternion_t *q1,
                                        const bno055_quaternion_t *q2) {
    bno055_quaternion_t result;

    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

    return result;
}

/**
 * Extract yaw angle (rotation around Y axis) from a quaternion
 * Assumes Y-up coordinate system (vertical axis)
 *
 * @param q Quaternion to extract yaw from
 * @return Yaw angle in degrees (-180° to +180°)
 */
double extract_yaw_angle(const bno055_quaternion_t *q) {
    // Formula for yaw (rotation around Y axis) with Y-up convention
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
    double yaw_rad = atan2(2.0 * (q->w * q->z + q->x * q->y),
                           1.0 - 2.0 * (q->y * q->y + q->z * q->z));

    // Convert to degrees
    double yaw_deg = yaw_rad * (180.0 / M_PI);

    return yaw_deg;
}

int main(int argc, char *argv[]) {
    int sensor1_fd, sensor2_fd, sensor3_fd;
    bno055_quaternion_t quat1, quat2, quat3;
    const char *i2c_bus1 = "/dev/i2c-1";  // Sensor 1 on i2c-1
    const char *i2c_bus2 = "/dev/i2c-1";  // Sensor 2 on i2c-1
    const char *i2c_bus3 = "/dev/i2c-3";  // Sensor 3 on i2c-3
    uint8_t i2c_address1 = BNO055_ADDRESS_A;  // Sensor 1 address (0x28)
    uint8_t i2c_address2 = BNO055_ADDRESS_B;  // Sensor 2 address (0x29)
    uint8_t i2c_address3 = BNO055_ADDRESS_A;  // Sensor 3 address (0x28)
    int read_rate_hz = 50;  // Default read rate (50 Hz)
    bool show_calib_once = true;

    // Parse command line arguments
    if (argc > 1) {
        read_rate_hz = atoi(argv[1]);
        if (read_rate_hz <= 0 || read_rate_hz > 100) {
            fprintf(stderr, "Invalid read rate. Using default: 50 Hz\n");
            read_rate_hz = 50;
        }
    }

    printf("BNO055 Triple Quaternion Reader\n");
    printf("================================\n");
    printf("Sensor 1 - I2C Bus: %s, Address: 0x%02X\n", i2c_bus1, i2c_address1);
    printf("Sensor 2 - I2C Bus: %s, Address: 0x%02X\n", i2c_bus2, i2c_address2);
    printf("Sensor 3 - I2C Bus: %s, Address: 0x%02X\n", i2c_bus3, i2c_address3);
    printf("Read Rate: %d Hz\n", read_rate_hz);
    printf("Press Ctrl+C to exit\n\n");

    // Set up signal handler for Ctrl+C
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize Sensor 1 (i2c-1)
    printf("Initializing Sensor 1 on %s...\n", i2c_bus1);
    sensor1_fd = bno055_init(i2c_bus1, i2c_address1);
    if (sensor1_fd < 0) {
        fprintf(stderr, "Failed to initialize Sensor 1 on %s\n", i2c_bus1);
        fprintf(stderr, "\nTroubleshooting:\n");
        fprintf(stderr, "1. Check if the sensor is properly connected to I2C pins\n");
        fprintf(stderr, "2. Run 'i2cdetect -y 1' to verify the sensor is detected\n");
        fprintf(stderr, "3. Ensure you have permissions to access I2C (add user to 'i2c' group)\n");
        return EXIT_FAILURE;
    }

    // Initialize Sensor 2 (i2c-1)
    printf("Initializing Sensor 2 on %s...\n", i2c_bus2);
    sensor2_fd = bno055_init(i2c_bus2, i2c_address2);
    if (sensor2_fd < 0) {
        fprintf(stderr, "Failed to initialize Sensor 2 on %s\n", i2c_bus2);
        fprintf(stderr, "\nTroubleshooting:\n");
        fprintf(stderr, "1. Check if the sensor is properly connected to I2C pins\n");
        fprintf(stderr, "2. Run 'i2cdetect -y 1' to verify the sensor is detected at address 0x29\n");
        fprintf(stderr, "3. Ensure you have permissions to access I2C (add user to 'i2c' group)\n");
        bno055_close(sensor1_fd);
        return EXIT_FAILURE;
    }

    // Initialize Sensor 3 (i2c-3)
    printf("Initializing Sensor 3 on %s...\n", i2c_bus3);
    sensor3_fd = bno055_init(i2c_bus3, i2c_address3);
    if (sensor3_fd < 0) {
        fprintf(stderr, "Failed to initialize Sensor 3 on %s\n", i2c_bus3);
        fprintf(stderr, "\nTroubleshooting:\n");
        fprintf(stderr, "1. Check if the sensor is properly connected to I2C pins\n");
        fprintf(stderr, "2. Run 'i2cdetect -y 3' to verify the sensor is detected\n");
        fprintf(stderr, "3. Ensure you have permissions to access I2C (add user to 'i2c' group)\n");
        bno055_close(sensor1_fd);
        bno055_close(sensor2_fd);
        return EXIT_FAILURE;
    }

    printf("\nAll three sensors initialized successfully!\n");

    // Show calibration status for all three sensors
    if (show_calib_once) {
        printf("\nInitial Calibration Status:\n");
        printf("---------------------------\n");
        printf("Sensor 1 (i2c-1, 0x28): ");
        int calib_stat1 = bno055_get_calibration_status(sensor1_fd);
        print_calibration_status(calib_stat1);

        printf("Sensor 2 (i2c-1, 0x29): ");
        int calib_stat2 = bno055_get_calibration_status(sensor2_fd);
        print_calibration_status(calib_stat2);

        printf("Sensor 3 (i2c-3, 0x28): ");
        int calib_stat3 = bno055_get_calibration_status(sensor3_fd);
        print_calibration_status(calib_stat3);
        printf("\n");
        show_calib_once = false;
    }

    // Calculate delay between reads (in microseconds)
    int delay_us = 1000000 / read_rate_hz;

    // Wait a moment before starting continuous reading
    usleep(1000000);  // 1 second delay

    // Main reading loop
    while (keep_running) {
        // Remove everything on screen
        printf("\033[2J");

        // Move cursor to home position (no clear to avoid flicker)
        printf("\033[H");

        // Read quaternion data from all three sensors
        bool sensor1_ok = (bno055_read_quaternion(sensor1_fd, &quat1) == 0);
        bool sensor2_ok = (bno055_read_quaternion(sensor2_fd, &quat2) == 0);
        bool sensor3_ok = (bno055_read_quaternion(sensor3_fd, &quat3) == 0);

        // Display header
        printf("BNO055 Y-Axis Angle Reader (Rate: %d Hz)\033[K\n", read_rate_hz);
        printf("=========================================\033[K\n\n");

        // Calculate and display Y-axis angles relative to Sensor 3
        if (sensor1_ok && sensor3_ok) {
            // Calculate relative rotation: q_rel = q3 × q1*
            bno055_quaternion_t q1_conj = quaternion_conjugate(&quat1);
            bno055_quaternion_t q_rel_1_3 = quaternion_multiply(&quat3, &q1_conj);

            // Extract Y-axis angle
            double yaw_1_3 = extract_yaw_angle(&q_rel_1_3);

            printf("Y-axis angle (Sensor 1 → Sensor 3): %7.2f°\033[K\n", yaw_1_3);
        } else {
            printf("Y-axis angle (Sensor 1 → Sensor 3): ERROR - ");
            if (!sensor1_ok) printf("Sensor 1 failed");
            if (!sensor3_ok) printf("Sensor 3 failed");
            printf("\033[K\n");
        }

        if (sensor2_ok && sensor3_ok) {
            // Calculate relative rotation: q_rel = q3 × q2*
            bno055_quaternion_t q2_conj = quaternion_conjugate(&quat2);
            bno055_quaternion_t q_rel_2_3 = quaternion_multiply(&quat3, &q2_conj);

            // Extract Y-axis angle
            double yaw_2_3 = extract_yaw_angle(&q_rel_2_3);

            printf("Y-axis angle (Sensor 2 → Sensor 3): %7.2f°\033[K\n", yaw_2_3);
        } else {
            printf("Y-axis angle (Sensor 2 → Sensor 3): ERROR - ");
            if (!sensor2_ok) printf("Sensor 2 failed");
            if (!sensor3_ok) printf("Sensor 3 failed");
            printf("\033[K\n");
        }

        printf("\033[K\n");
        printf("Press Ctrl+C to exit\033[K\n");

        // Force immediate output
        fflush(stdout);

        // Delay based on configured read rate
        usleep(delay_us);
    }

    // Clean up
    bno055_close(sensor1_fd);
    bno055_close(sensor2_fd);
    bno055_close(sensor3_fd);
    printf("\nAll three BNO055 sensors closed successfully\n");

    return EXIT_SUCCESS;
}

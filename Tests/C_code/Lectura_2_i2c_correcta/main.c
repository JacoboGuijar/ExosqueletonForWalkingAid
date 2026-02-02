#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
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

int main(int argc, char *argv[]) {
    int sensor1_fd, sensor2_fd;
    bno055_quaternion_t quat1, quat2;
    const char *i2c_bus1 = "/dev/i2c-1";  // Sensor 1 on i2c-1
    const char *i2c_bus2 = "/dev/i2c-3";  // Sensor 2 on i2c-3
    uint8_t i2c_address1 = BNO055_ADDRESS_A;  // Sensor 1 address (0x28)
    uint8_t i2c_address2 = BNO055_ADDRESS_B;  // Sensor 2 address (0x29)
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

    printf("BNO055 Dual Quaternion Reader\n");
    printf("==============================\n");
    printf("Sensor 1 - I2C Bus: %s, Address: 0x%02X\n", i2c_bus1, i2c_address1);
    printf("Sensor 2 - I2C Bus: %s, Address: 0x%02X\n", i2c_bus2, i2c_address2);
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

    // Initialize Sensor 2 (i2c-3)
    printf("Initializing Sensor 2 on %s...\n", i2c_bus2);
    sensor2_fd = bno055_init(i2c_bus2, i2c_address2);
    if (sensor2_fd < 0) {
        fprintf(stderr, "Failed to initialize Sensor 2 on %s\n", i2c_bus2);
        fprintf(stderr, "\nTroubleshooting:\n");
        fprintf(stderr, "1. Check if the sensor is properly connected to I2C pins\n");
        fprintf(stderr, "2. Run 'i2cdetect -y 3' to verify the sensor is detected\n");
        fprintf(stderr, "3. Ensure you have permissions to access I2C (add user to 'i2c' group)\n");
        bno055_close(sensor1_fd);
        return EXIT_FAILURE;
    }

    printf("\nBoth sensors initialized successfully!\n");

    // Show calibration status for both sensors
    if (show_calib_once) {
        printf("\nInitial Calibration Status:\n");
        printf("---------------------------\n");
        printf("Sensor i2c-1: ");
        int calib_stat1 = bno055_get_calibration_status(sensor1_fd);
        print_calibration_status(calib_stat1);

        printf("Sensor i2c-3: ");
        int calib_stat2 = bno055_get_calibration_status(sensor2_fd);
        print_calibration_status(calib_stat2);
        printf("\n");
        show_calib_once = false;
    }

    // Calculate delay between reads (in microseconds)
    int delay_us = 1000000 / read_rate_hz;

    // Wait a moment before starting continuous reading
    usleep(1000000);  // 1 second delay

    // Main reading loop
    while (keep_running) {
        // Clear screen (ANSI escape codes)
        printf("\033[2J\033[H");

        // Read quaternion data from both sensors
        bool sensor1_ok = (bno055_read_quaternion(sensor1_fd, &quat1) == 0);
        bool sensor2_ok = (bno055_read_quaternion(sensor2_fd, &quat2) == 0);

        // Display header
        printf("BNO055 Dual Sensor Quaternion Data (Rate: %d Hz)\n", read_rate_hz);
        printf("=================================================\n\n");

        // Display Sensor 1 data
        if (sensor1_ok) {
            printf("i2c-1: w: %7.4f, x: %7.4f, y: %7.4f, z: %7.4f\n",
                   quat1.w, quat1.x, quat1.y, quat1.z);
        } else {
            printf("i2c-1: ERROR - Failed to read quaternion data\n");
        }

        // Display Sensor 2 data
        if (sensor2_ok) {
            printf("i2c-3: w: %7.4f, x: %7.4f, y: %7.4f, z: %7.4f\n",
                   quat2.w, quat2.x, quat2.y, quat2.z);
        } else {
            printf("i2c-3: ERROR - Failed to read quaternion data\n");
        }

        printf("\nPress Ctrl+C to exit\n");

        // Delay based on configured read rate
        usleep(delay_us);
    }

    // Clean up
    bno055_close(sensor1_fd);
    bno055_close(sensor2_fd);
    printf("\nBoth BNO055 sensors closed successfully\n");

    return EXIT_SUCCESS;
}

# BNO055 Dual Sensor Relative Angle Measurement System

## Overview

This project implements a real-time relative orientation measurement system using two Bosch BNO055 9-DOF absolute orientation sensors connected to a Raspberry Pi 5. The system reads quaternion data from both sensors and calculates the angular difference between their orientations.

### Key Features

- **Dual sensor support**: Simultaneous reading from two BNO055 sensors on different I2C buses
- **Real-time angle calculation**: Computes relative orientation angle using quaternion mathematics
- **High update rate**: Configurable sampling rate up to 100 Hz
- **Flicker-free display**: Optimized terminal output using ANSI cursor positioning
- **Robust error handling**: Independent sensor failure detection and recovery
- **Modular architecture**: Reusable BNO055 library for future projects

### System Requirements

- **Hardware**: Raspberry Pi 5 (compatible with RPi 4, 3, Zero)
- **Sensors**: Two BNO055 absolute orientation sensors
- **OS**: Raspberry Pi OS (tested on Debian-based distributions)
- **Compiler**: GCC 14.2.0 or later with C11 support
- **Libraries**: Standard C library, Linux I2C subsystem

---

## Hardware Setup

### BNO055 Sensor Overview

The BNO055 is a 9-DOF (Degrees of Freedom) absolute orientation sensor combining:
- 3-axis accelerometer
- 3-axis gyroscope
- 3-axis magnetometer
- ARM Cortex-M0 processor with sensor fusion algorithm

In NDOF (Nine Degrees of Freedom) mode, the sensor outputs quaternions representing absolute 3D orientation.

### Wiring Diagram

#### Sensor 1 (on I2C bus 1)

```
BNO055 Pin    →    Raspberry Pi 5
─────────────────────────────────
VIN           →    3.3V (Pin 1)
GND           →    GND (Pin 6)
SDA           →    GPIO 2 (SDA1, Pin 3)
SCL           →    GPIO 3 (SCL1, Pin 5)
COM3          →    GND (for address 0x28)
```

#### Sensor 2 (on I2C bus 3)

```
BNO055 Pin    →    Raspberry Pi 5
─────────────────────────────────
VIN           →    3.3V (Pin 17)
GND           →    GND (Pin 14)
SDA           →    GPIO 4 (SDA3)
SCL           →    GPIO 5 (SCL3)
COM3          →    3.3V (for address 0x29)
```

### I2C Address Configuration

The BNO055 has two possible I2C addresses:
- **0x28** (40 decimal): COM3 pin connected to GND (default)
- **0x29** (41 decimal): COM3 pin connected to VCC

This allows two sensors on the same I2C bus, or simplifies connection on separate buses.

### Hardware Verification

After connecting both sensors, verify I2C communication:

```bash
# Check sensor on I2C bus 1 (should show device at 0x28)
i2cdetect -y 1

# Check sensor on I2C bus 3 (should show device at 0x29)
i2cdetect -y 3
```

Expected output for bus 1:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

### I2C Permissions

Ensure your user has permission to access I2C devices:

```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER

# Log out and back in for changes to take effect
```

---

## Software Architecture

### File Structure

```
C_code/
├── bno055.h          # BNO055 library header (register defs, API declarations)
├── bno055.c          # BNO055 library implementation (I2C communication)
├── main.c            # Application entry point and angle calculation
├── Makefile          # Build system configuration
└── README.md         # This documentation
```

### Module Descriptions

#### bno055.h (Header File)

Defines the public API for BNO055 sensor interaction:
- **Register definitions**: I2C addresses, register addresses, operation modes
- **Data structures**: `bno055_quaternion_t` for quaternion representation
- **Function prototypes**: Initialization, reading, and control functions
- **Constants**: Quaternion scale factor, chip ID, calibration registers

#### bno055.c (Library Implementation)

Implements low-level I2C communication with BNO055:
- **I2C operations**: Read/write register functions using Linux I2C subsystem
- **Sensor initialization**: Reset, mode configuration, power management
- **Data acquisition**: Quaternion reading with proper byte ordering (LSB first)
- **Error handling**: I2C communication failure detection

Reference: [Linux I2C Subsystem Documentation](https://www.kernel.org/doc/Documentation/i2c/dev-interface)

#### main.c (Application)

Application logic and angle calculation:
- **Dual sensor management**: Independent initialization and reading
- **Quaternion angle calculation**: Dot product method implementation
- **Display management**: Flicker-free terminal output with ANSI codes
- **Signal handling**: Graceful shutdown on Ctrl+C

#### Makefile

Build system with standard targets:
- `make` / `make all`: Compile the project
- `make clean`: Remove build artifacts
- `make rebuild`: Clean and build from scratch
- `make run`: Build and execute
- `make install`: Install to `/usr/local/bin` (requires sudo)

### Data Flow

```
┌─────────────┐       ┌─────────────┐
│  BNO055     │       │  BNO055     │
│  Sensor 1   │       │  Sensor 2   │
│  (i2c-1)    │       │  (i2c-3)    │
└──────┬──────┘       └──────┬──────┘
       │                     │
       │ I2C Read            │ I2C Read
       │ Quaternion          │ Quaternion
       ▼                     ▼
┌──────────────────────────────────┐
│      bno055_read_quaternion()    │
│      (from bno055.c library)     │
└──────────┬───────────────────────┘
           │
           │ quat1 (w,x,y,z)     quat2 (w,x,y,z)
           ▼
┌──────────────────────────────────┐
│  quaternion_angle_degrees()      │
│  Calculate: 2·acos(|q1·q2|)      │
└──────────┬───────────────────────┘
           │
           │ angle (degrees)
           ▼
┌──────────────────────────────────┐
│  Terminal Display (ANSI output)  │
│  "Relative angle: XX.XX°"        │
└──────────────────────────────────┘
```

---

## Mathematical Explanation

### Quaternion Representation

Quaternions provide a singularity-free representation of 3D rotations, avoiding gimbal lock inherent in Euler angles.

A unit quaternion **q** is a 4-dimensional complex number:

```
q = w + xi + yj + zk
```

Where:
- **w**: Real (scalar) component
- **x, y, z**: Imaginary (vector) components
- **i, j, k**: Imaginary units with properties: i²=j²=k²=ijk=-1

For a rotation of angle θ around unit axis **n** = (n_x, n_y, n_z):

```
q = (cos(θ/2), sin(θ/2)·n_x, sin(θ/2)·n_y, sin(θ/2)·n_z)
```

Unit quaternions satisfy: w² + x² + y² + z² = 1

### Angle Between Two Quaternions

Given two orientation quaternions **q₁** and **q₂**, we need to find the angle of rotation between them.

#### Method 1: Relative Quaternion (Not Used)

The relative rotation quaternion is:

```
q_rel = q₂ · q₁*
```

Where q₁* is the conjugate of q₁: (w, -x, -y, -z)

Extract angle: θ = 2·arccos(|q_rel.w|)

#### Method 2: Dot Product (Implemented)

A more efficient method uses the quaternion dot product:

```
q₁ · q₂ = w₁·w₂ + x₁·x₂ + y₁·y₂ + z₁·z₂
```

**Key insight**: For unit quaternions, the dot product equals the w-component of the relative quaternion.

Therefore:

```
θ = 2·arccos(|q₁ · q₂|)
```

#### Why Absolute Value?

Quaternions exhibit **double cover**: both **q** and **-q** represent the same physical rotation. Taking the absolute value ensures we always measure the shortest angular path, yielding angles in [0°, 180°].

### Formula Derivation

1. Start with relative quaternion: q_rel = q₂ · q₁*
2. For unit quaternions: q₁* = conjugate(q₁)
3. The scalar part of q_rel: w_rel = q₁ · q₂
4. From quaternion-to-angle formula: θ = 2·arccos(w_rel)
5. Apply absolute value for shortest path: θ = 2·arccos(|q₁ · q₂|)

### Implementation in C

```c
double quaternion_angle_degrees(const bno055_quaternion_t *q1,
                                const bno055_quaternion_t *q2) {
    // Calculate dot product
    double dot = q1->w * q2->w +
                 q1->x * q2->x +
                 q1->y * q2->y +
                 q1->z * q2->z;

    // Clamp to avoid numerical errors in acos
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;

    // Calculate angle: 2·arccos(|dot|)
    double angle_rad = 2.0 * acos(fabs(dot));

    // Convert to degrees
    return angle_rad * (180.0 / M_PI);
}
```

### Numerical Stability Considerations

1. **Domain clamping**: `acos(x)` requires x ∈ [-1, 1]. Floating-point errors can produce values like 1.0000001, causing `acos()` to return NaN. Always clamp before calling `acos()`.

2. **Double precision**: Use `double` rather than `float` for accumulating dot products to minimize rounding errors.

3. **Quaternion normalization**: The BNO055 outputs normalized quaternions. Verify: ||q|| = √(w² + x² + y² + z²) ≈ 1.0

4. **Small angle stability**: For very small angles (dot ≈ 1), `acos()` is well-behaved. No special-case approximation needed.

### Output Range

**Result**: 0° to 180°

- **0°**: Sensors have identical orientation
- **90°**: Sensors are perpendicular
- **180°**: Sensors have maximally different orientations

This range represents the shortest rotation path between two orientations in 3D space.

---

## API Reference

### Data Structures

#### bno055_quaternion_t

```c
typedef struct {
    double w;  // Real component
    double x;  // i component
    double y;  // j component
    double z;  // k component
} bno055_quaternion_t;
```

Unit quaternion representing 3D orientation. Components normalized such that w² + x² + y² + z² = 1.

### Constants

#### I2C Addresses

```c
#define BNO055_ADDRESS_A    0x28  // Default address (COM3 LOW)
#define BNO055_ADDRESS_B    0x29  // Alternative address (COM3 HIGH)
```

#### Register Addresses

```c
#define BNO055_CHIP_ID_ADDR         0x00  // Chip ID (should be 0xA0)
#define BNO055_OPR_MODE_ADDR        0x3D  // Operation mode register
#define BNO055_PWR_MODE_ADDR        0x3E  // Power mode register
#define BNO055_QUATERNION_DATA_W_LSB_ADDR   0x20  // Quaternion W LSB
// ... (see bno055.h for complete list)
```

#### Operation Modes

```c
#define BNO055_OPERATION_MODE_CONFIG    0x00  // Configuration mode
#define BNO055_OPERATION_MODE_NDOF      0x0C  // Nine DOF fusion mode
```

#### Scaling Factors

```c
#define BNO055_QUATERNION_SCALE     (1.0 / 16384.0)  // 2^14 = 16384
```

Quaternion data is in 16-bit fixed-point format with scale factor 2^14 LSB per unit.

### Core Functions

#### bno055_init()

```c
int bno055_init(const char *i2c_bus, uint8_t address);
```

Initialize BNO055 sensor on specified I2C bus and address.

**Parameters**:
- `i2c_bus`: I2C device path (e.g., "/dev/i2c-1")
- `address`: I2C address (BNO055_ADDRESS_A or BNO055_ADDRESS_B)

**Returns**:
- File descriptor (>= 0) on success
- -1 on failure

**Behavior**:
1. Opens I2C bus device file
2. Sets I2C slave address using `ioctl(I2C_SLAVE)`
3. Verifies chip ID (expects 0xA0)
4. Performs software reset
5. Waits for sensor to restart
6. Configures normal power mode
7. Sets operation mode to NDOF

**Example**:
```c
int fd = bno055_init("/dev/i2c-1", BNO055_ADDRESS_A);
if (fd < 0) {
    fprintf(stderr, "Initialization failed\n");
    return EXIT_FAILURE;
}
```

#### bno055_close()

```c
void bno055_close(int fd);
```

Close I2C connection to sensor.

**Parameters**:
- `fd`: File descriptor returned by `bno055_init()`

**Example**:
```c
bno055_close(fd);
```

#### bno055_read_quaternion()

```c
int bno055_read_quaternion(int fd, bno055_quaternion_t *quat);
```

Read quaternion orientation data from sensor.

**Parameters**:
- `fd`: File descriptor from `bno055_init()`
- `quat`: Pointer to quaternion structure to populate

**Returns**:
- 0 on success
- -1 on I2C read failure

**Behavior**:
1. Reads 8 bytes starting from register 0x20 (quaternion W LSB)
2. Combines LSB and MSB into 16-bit signed integers
3. Scales by 2^14 to convert to unit quaternion
4. Populates `quat` structure

**Register Layout**:
```
0x20: W_LSB  0x21: W_MSB
0x22: X_LSB  0x23: X_MSB
0x24: Y_LSB  0x25: Y_MSB
0x26: Z_LSB  0x27: Z_MSB
```

**Example**:
```c
bno055_quaternion_t quat;
if (bno055_read_quaternion(fd, &quat) == 0) {
    printf("w:%.4f x:%.4f y:%.4f z:%.4f\n",
           quat.w, quat.x, quat.y, quat.z);
}
```

#### bno055_read_chip_id()

```c
int bno055_read_chip_id(int fd);
```

Read and return chip ID for verification.

**Returns**:
- Chip ID (should be 0xA0) on success
- -1 on read failure

#### bno055_get_calibration_status()

```c
int bno055_get_calibration_status(int fd);
```

Get current calibration status byte.

**Returns**:
- Calibration status byte (bits 7-6: SYS, 5-4: GYR, 3-2: ACC, 1-0: MAG)
- -1 on read failure

**Status Interpretation**:
- Each 2-bit field ranges from 0 (uncalibrated) to 3 (fully calibrated)
- Extract: `sys = (status >> 6) & 0x03`
- SYS calibration depends on all subsystems being calibrated

### Angle Calculation Function

#### quaternion_angle_degrees()

```c
double quaternion_angle_degrees(const bno055_quaternion_t *q1,
                                const bno055_quaternion_t *q2);
```

Calculate angle between two quaternion orientations.

**Parameters**:
- `q1`: Pointer to first quaternion (unit quaternion)
- `q2`: Pointer to second quaternion (unit quaternion)

**Returns**:
- Angle in degrees [0°, 180°]

**Algorithm**:
1. Compute dot product: q1·q2
2. Clamp to [-1, 1] for numerical stability
3. Calculate: θ = 2·arccos(|dot|)
4. Convert radians to degrees

**Example**:
```c
bno055_quaternion_t q1, q2;
// ... read quaternions from sensors ...
double angle = quaternion_angle_degrees(&q1, &q2);
printf("Relative angle: %.2f°\n", angle);
```

---

## Build and Usage

### Compilation

Build the project using the provided Makefile:

```bash
# Clean build
make clean
make

# Or in one command
make rebuild
```

The Makefile compiles with:
- **Compiler**: GCC with C11 standard
- **Flags**: `-Wall -Wextra -O2` (all warnings, optimization level 2)
- **Linking**: `-lm` (math library for acos, fabs, M_PI)

### Command-Line Usage

```bash
# Run with default 50 Hz update rate
./bno055_reader

# Run with custom update rate (e.g., 20 Hz)
./bno055_reader 20
```

**Arguments**:
- `[rate]`: Optional read rate in Hz (1-100), defaults to 50 Hz

### Output Format

```
BNO055 Dual Sensor Relative Angle (Rate: 50 Hz)
================================================

Relative angle between sensors: 45.32°

Press Ctrl+C to exit
```

The display updates continuously at the specified rate. Use Ctrl+C for graceful shutdown.

### Installation (Optional)

Install to system path for global access:

```bash
sudo make install
```

This copies the executable to `/usr/local/bin/bno055_reader`.

Uninstall:
```bash
sudo make uninstall
```

---

## Troubleshooting

### Sensor Not Detected

**Symptom**: "Failed to initialize BNO055 sensor" or invalid chip ID

**Solutions**:
1. Verify physical connections (VIN, GND, SDA, SCL)
2. Check I2C bus is enabled: `ls /dev/i2c*`
3. Run `i2cdetect -y 1` (or 3) to verify sensor appears at correct address
4. Ensure 3.3V power supply is adequate (BNO055 requires ~12mA)
5. Check for I2C address conflicts

**Enable I2C on Raspberry Pi**:
```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
```

### I2C Permission Denied

**Symptom**: "Failed to open I2C bus: Permission denied"

**Solution**:
```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER

# Verify group membership
groups

# Log out and back in for changes to take effect
```

Temporary workaround (not recommended for production):
```bash
sudo ./bno055_reader
```

### Calibration Issues

**Symptom**: Erratic angles or poor sensor performance

**Solution**:
- BNO055 requires calibration for optimal performance
- System calibration (SYS) depends on gyro, accelerometer, and magnetometer calibration
- Perform figure-8 motion for magnetometer calibration
- Move sensor through various orientations for accelerometer/gyro calibration
- Check calibration status in startup output

**Calibration Status**:
```
Calibration - SYS: 3, GYR: 3, ACC: 3, MAG: 3
              ^^^^  ^^^^  ^^^^  ^^^^
              │     │     │     └─ Magnetometer (0-3)
              │     │     └─────── Accelerometer (0-3)
              │     └───────────── Gyroscope (0-3)
              └─────────────────── System (0-3)
```

### NaN or Invalid Angles

**Symptom**: "Relative angle: nan" or extremely large values

**Causes and Solutions**:
1. **Numerical overflow in acos()**: Clamping is implemented, but verify quaternions are normalized
   - Check: w² + x² + y² + z² ≈ 1.0
2. **Sensor not responding**: Verify both sensors are readable
3. **I2C communication errors**: Check for loose connections

### Screen Flickering

**Symptom**: Display flashes or flickers during updates

**Current Implementation**: Uses cursor positioning (`\033[H`) instead of full clear (`\033[2J`) to minimize flicker.

**Further Reduction**:
- Lower update rate: `./bno055_reader 20`
- Use ncurses library for double-buffered display (requires code modification)

### Compilation Errors

**Missing math.h functions**:
```
undefined reference to `acos'
```

**Solution**: Ensure `-lm` flag is in Makefile (already included)

**I2C headers not found**:
```
fatal error: linux/i2c-dev.h: No such file or directory
```

**Solution**:
```bash
sudo apt-get install libi2c-dev
```

### Reading Errors During Operation

**Symptom**: Intermittent "ERROR - Failed to read sensor data"

**Causes**:
1. **Loose connections**: Check wiring
2. **I2C bus noise**: Add pull-up resistors (typically 2.2kΩ to 4.7kΩ)
3. **Cable too long**: Keep I2C wires under 30cm for reliable operation
4. **Sensor power brownout**: Ensure adequate power supply

---

## Technical Details

### I2C Communication Protocol

The BNO055 communicates via I2C (Inter-Integrated Circuit) bus at 400 kHz (fast mode).

**Write Operation**:
1. Start condition
2. Send slave address (0x28 or 0x29) + write bit
3. Send register address
4. Send data byte(s)
5. Stop condition

**Read Operation**:
1. Start condition
2. Send slave address + write bit
3. Send register address
4. Repeated start condition
5. Send slave address + read bit
6. Read data byte(s)
7. Stop condition

**Linux I2C Subsystem**:
- Access via `/dev/i2c-X` device files
- Use `ioctl(fd, I2C_SLAVE, address)` to select device
- Standard `read()`/`write()` system calls for data transfer

Reference: https://www.kernel.org/doc/Documentation/i2c/dev-interface

### BNO055 Initialization Sequence

1. **Open I2C bus** → System call to `/dev/i2c-X`
2. **Set slave address** → `ioctl(I2C_SLAVE, 0x28/0x29)`
3. **Read chip ID** (register 0x00) → Verify response is 0xA0
4. **Software reset** → Write 0x20 to register 0x3F
5. **Wait 650ms** → Allow reset to complete
6. **Poll chip ID** → Retry up to 10 times until sensor responds
7. **Set power mode** → Write 0x00 (normal) to register 0x3E
8. **Set page ID** → Write 0x00 to register 0x07
9. **Set operation mode** → Write 0x0C (NDOF) to register 0x3D
10. **Wait 20ms** → Allow mode transition

### NDOF Fusion Mode

**NDOF (Nine Degrees of Freedom)** is a fusion mode that combines:
- Absolute orientation from magnetometer (heading)
- Relative orientation from gyroscope (fast, no drift)
- Gravity vector from accelerometer (tilt)

The onboard Cortex-M0 processor runs a proprietary sensor fusion algorithm, outputting:
- Quaternions (4 values, no gimbal lock)
- Euler angles (roll, pitch, yaw)
- Linear acceleration (gravity compensated)
- Gravity vector

**Update Rate**: 100 Hz (quaternion output rate)

**Advantages**:
- No gimbal lock
- Smooth output
- Automatic drift compensation
- Lower CPU load (fusion on sensor, not host)

### Anti-Flicker Display Implementation

**Problem**: Clearing entire screen with `\033[2J` causes visible flicker at high refresh rates.

**Solution**: Cursor positioning with line clearing
1. Move cursor to home: `\033[H`
2. Overwrite existing text in place
3. Clear to end of each line: `\033[K`
4. Flush output buffer: `fflush(stdout)`

**ANSI Escape Codes Used**:
- `\033[H` → Move cursor to position (0,0)
- `\033[K` → Clear from cursor to end of line
- `\033[2J` → Clear entire screen (NOT used in current implementation)

This approach reduces display latency and eliminates flicker even at 50+ Hz update rates.

### Quaternion Data Format

**BNO055 Output Format**:
- 16-bit signed integers (two's complement)
- Little-endian byte order (LSB first)
- Scale factor: 1 unit = 2^14 LSB = 16384 LSB

**Conversion**:
```c
int16_t w_raw = (buffer[1] << 8) | buffer[0];  // MSB, LSB
double w = w_raw / 16384.0;
```

**Raw Value Range**: -32768 to +32767 (16-bit signed)
**Scaled Range**: -2.0 to +2.0 (but ||q|| = 1, so typically -1.0 to +1.0)

---

## Performance Characteristics

### Timing Analysis

- **I2C Read Latency**: ~1ms per sensor (8 bytes at 400 kHz + overhead)
- **Angle Calculation**: ~5 μs (floating-point operations)
- **Display Update**: ~2ms (terminal I/O)
- **Total Loop Time**: ~5ms for 50 Hz rate (actual: 20ms with sleep)

### Achievable Update Rates

- **Theoretical Maximum**: ~200 Hz (limited by I2C speed and dual reads)
- **Sensor Limit**: 100 Hz (BNO055 quaternion update rate)
- **Practical Maximum**: 100 Hz (set via command-line argument)
- **Recommended**: 20-50 Hz (good balance of responsiveness and CPU usage)

### Accuracy

- **Quaternion Precision**: 16-bit → ~0.006% resolution
- **Angle Resolution**: Limited by `acos()` floating-point precision → ~0.01°
- **Practical Accuracy**: ±2-5° depending on sensor calibration

---

## License and Attribution

This project uses the BNO055 absolute orientation sensor from Bosch Sensortec.

**References**:
- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- [Linux I2C Subsystem](https://www.kernel.org/doc/Documentation/i2c/dev-interface)
- Quaternion mathematics: Kuipers, "Quaternions and Rotation Sequences"

**Author**: Generated for TFM/Tests project
**Date**: 2025
**Platform**: Raspberry Pi 5, Raspberry Pi OS

---

## Future Enhancements

Possible improvements for future versions:

1. **Data Logging**: CSV export of angle measurements over time
2. **Calibration Persistence**: Save/load calibration data to avoid recalibration
3. **Multiple Sensors**: Extend to 3+ sensors with angle matrix output
4. **Euler Angles**: Add roll/pitch/yaw output alongside quaternions
5. **GUI Interface**: GTK or Qt graphical display with real-time 3D visualization
6. **Network Streaming**: Broadcast sensor data via UDP for remote monitoring
7. **Python Bindings**: Wrapper library for Python integration
8. **Rotation Axis**: Output axis of rotation in addition to angle

---

## Contact and Support

For issues, questions, or contributions related to this project, please refer to your project documentation or contact the maintainer.

**Hardware Support**: Refer to [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)

**BNO055 Support**: Refer to [Bosch Sensortec](https://www.bosch-sensortec.com/)

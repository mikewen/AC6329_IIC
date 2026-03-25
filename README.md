# AC6329\_IIC

Firmware for **AC6329** that reads IMU and magnetometer data over I²C and streams it via **BLE notifications**.

\---

## Overview

This project runs on the AC6329 MCU and performs:

* I²C communication with:

  * **MMC5603** (magnetometer)
  * **QMI8658C** (accelerometer + gyroscope)
* Sensor data acquisition (Accel, Gyro, Mag)
* Packaging into a fixed 20-byte binary format
* Transmission over BLE using notifications

Designed for **low-power, real-time, deterministic sensor streaming**.

\---

## Hardware

* MCU: AC6329
* Sensors:

  * MMC5603 (Magnetometer)
  * QMI8658C (6-axis IMU)
* Interface: I²C
* Wireless: BLE

\---

## Compile-Time Configuration

Sensors can be enabled or disabled at compile time:

```c
#define QMI8658  1   // IMU (accelerometer + gyroscope)
#define MMC5603  1   // Magnetometer
```

### Behavior

|Condition|Behavior|
|-|-|
|Sensor enabled|Data read over I²C|
|Sensor disabled|Values set to **0**, no I²C access|
|Read failure|Values remain **0**, no frame drop|

### Design Implications

* No wasted power on unused sensors
* No I²C contention when disabled
* **BLE packet format remains fixed** (no version mismatch)
* Stable data rate (no skipped packets)

\---

## BLE Interface

### Custom Service

* **UUID:** `0xAE30`

### Characteristics

|UUID|Properties|Description|
|-|-|-|
|AE02|Notify|Sensor data stream|
|AE10|Read / Write|Control / configuration|

\---

## BLE Packet Format

Each notification packet is **20 bytes**:

```
Byte 0   : Header (0xA1)
Byte 1   : Sequence number

Byte 2-7   : Accelerometer (ax, ay, az)   int16 (LSB first)
Byte 8-13  : Gyroscope     (gx, gy, gz)   int16 (LSB first)
Byte 14-19 : Magnetometer  (mx, my, mz)   int16 (LSB first)
```

### Key Properties

* **Fixed size (20 bytes)** → fits single BLE notification
* **Little-endian encoding**
* **Zero-filled fields** when sensor disabled or read fails
* **No optional fields** → simple and robust parsing

\---

## Packing Implementation

```c
static void pack\_ble\_packet(int16\_t ax, int16\_t ay, int16\_t az,
                            int16\_t gx, int16\_t gy, int16\_t gz)
{
    ble\_pkt\[0] = 0xA1;
    ble\_pkt\[1] = ble\_seq++;

    /\* Accel \*/
    ble\_pkt\[2] = ax \& 0xFF;
    ble\_pkt\[3] = ax >> 8;
    ble\_pkt\[4] = ay \& 0xFF;
    ble\_pkt\[5] = ay >> 8;
    ble\_pkt\[6] = az \& 0xFF;
    ble\_pkt\[7] = az >> 8;

    /\* Gyro \*/
    ble\_pkt\[8]  = gx \& 0xFF;
    ble\_pkt\[9]  = gx >> 8;
    ble\_pkt\[10] = gy \& 0xFF;
    ble\_pkt\[11] = gy >> 8;
    ble\_pkt\[12] = gz \& 0xFF;
    ble\_pkt\[13] = gz >> 8;

    /\* Mag \*/
    ble\_pkt\[14] = mx\_raw \& 0xFF;
    ble\_pkt\[15] = mx\_raw >> 8;
    ble\_pkt\[16] = my\_raw \& 0xFF;
    ble\_pkt\[17] = my\_raw >> 8;
    ble\_pkt\[18] = mz\_raw \& 0xFF;
    ble\_pkt\[19] = mz\_raw >> 8;
}
```

\---

## BLE GATT Profile

### Service: `AE30`

#### Notify Characteristic (AE02)

* Handle: `0x000C`
* CCC: `0x000D`
* Used for continuous sensor streaming

#### Control Characteristic (AE10)

* Handle: `0x0017`
* Supports read/write
* Reserved for runtime configuration

\---

## Handle Map

```c
#define ATT\_CHARACTERISTIC\_ae02\_01\_VALUE\_HANDLE                 0x000c
#define ATT\_CHARACTERISTIC\_ae02\_01\_CLIENT\_CONFIGURATION\_HANDLE  0x000d
#define ATT\_CHARACTERISTIC\_ae10\_01\_VALUE\_HANDLE                 0x0017
```

\---

## Data Interpretation

All values:

* Type: `int16\_t`
* Endianness: **Little-endian**

### Units (application-level)

* Accelerometer → m/s² (after scaling)
* Gyroscope → deg/s or rad/s
* Magnetometer → raw field strength

\---

## Data Flow

```
Timer ISR
   ↓
Read MMC5603 (slow, measurement trigger)
   ↓
Read QMI8658 (12-byte burst)
   ↓
Pack BLE packet
   ↓
Send notification (AE02)
```

\---

## Client (Android / Receiver) Notes

* Always expect **20-byte packets**
* Use sequence number to detect packet loss
* Interpret **zero values** as:

  * Sensor disabled, or
  * Temporary read failure

\---

## Use Cases

* AHRS / orientation estimation
* BLE IMU streaming
* Marine autopilot input
* Robotics and motion tracking

\---

## Notes

* Optimized for **low RAM and low power**
* Deterministic timing (ISR-safe design)
* No timestamps → should be handled on receiver side

\---

## Future Improvements

* Add sensor validity bitmask
* Add timestamp / delta time
* Configurable output rate (AE10)
* Calibration (bias / scale)
* Sensor fusion (quaternion output)

\---

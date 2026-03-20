# AC6329_IIC

Firmware for **AC6329** that reads IMU and magnetometer data over I²C and streams it via **BLE notifications**.

## Overview

This project runs on the AC6329 MCU and performs:

* I²C communication with:

  * **MMC5603** (magnetometer)
  * **QMI8658C** (accelerometer + gyroscope)
* Sensor data acquisition (Accel, Gyro, Mag)
* Packaging into a compact binary format
* Transmission over BLE using **notifications**

Designed for low-power, real-time motion sensing applications.

---

## Hardware

* MCU: AC6329
* Sensors:

  * MMC5603 (Magnetometer)
  * QMI8658C (6-axis IMU)
* Interface: I²C
* Wireless: BLE

---

## BLE Interface

### Custom Service

* **UUID:** `0xAE30`

### Characteristics

| UUID | Properties   | Description             |
| ---- | ------------ | ----------------------- |
| AE02 | Notify       | Sensor data stream      |
| AE10 | Read / Write | Control / configuration |

---

## BLE Packet Format

Each notification packet is **20 bytes**:

```
Byte 0   : Header (0xA1)
Byte 1   : Sequence number

Byte 2-7 : Accelerometer (ax, ay, az)   int16 (LSB first)
Byte 8-13: Gyroscope     (gx, gy, gz)   int16 (LSB first)
Byte 14-19: Magnetometer (mx, my, mz)   int16 (LSB first)
```

### Packing Implementation

```c
static void pack_ble_packet(int16_t ax, int16_t ay, int16_t az,
                            int16_t gx, int16_t gy, int16_t gz)
{
    ble_pkt[0] = 0xA1;
    ble_pkt[1] = ble_seq++;

    /* Accel */
    ble_pkt[2] = ax & 0xFF;
    ble_pkt[3] = ax >> 8;
    ble_pkt[4] = ay & 0xFF;
    ble_pkt[5] = ay >> 8;
    ble_pkt[6] = az & 0xFF;
    ble_pkt[7] = az >> 8;

    /* Gyro */
    ble_pkt[8]  = gx & 0xFF;
    ble_pkt[9]  = gx >> 8;
    ble_pkt[10] = gy & 0xFF;
    ble_pkt[11] = gy >> 8;
    ble_pkt[12] = gz & 0xFF;
    ble_pkt[13] = gz >> 8;

    /* Mag */
    ble_pkt[14] = mx_raw & 0xFF;
    ble_pkt[15] = mx_raw >> 8;
    ble_pkt[16] = my_raw & 0xFF;
    ble_pkt[17] = my_raw >> 8;
    ble_pkt[18] = mz_raw & 0xFF;
    ble_pkt[19] = mz_raw >> 8;
}
```

---

## BLE GATT Profile

### Service: `AE30`

#### Notify Characteristic (AE02)

* Handle: `0x000C`
* CCC: `0x000D`
* Used for streaming sensor data

#### Control Characteristic (AE10)

* Handle: `0x0017`
* Supports read/write
* Can be used for runtime configuration

---

## Handle Map

```c
#define ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE                 0x000c
#define ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE  0x000d
#define ATT_CHARACTERISTIC_ae10_01_VALUE_HANDLE                 0x0017
```

---

## Data Interpretation

All sensor values are:

* **Signed 16-bit integers (int16_t)**
* **Little-endian**

### Typical Usage

* Accelerometer → m/s² (scaled in application)
* Gyroscope → deg/s or rad/s
* Magnetometer → raw field strength (for heading)

---

## Workflow

1. Initialize I²C
2. Configure MMC5603 and QMI8658C
3. Periodically read:

   * Accel
   * Gyro
   * Mag
4. Pack data into BLE packet
5. Send via **AE02 notification**

---

## Use Cases

* AHRS / orientation estimation
* BLE IMU streaming
* Robotics / autopilot input
* Motion tracking systems

---

## Notes

* Packet size is optimized to fit within a single BLE notification (20 bytes)
* Sequence number allows packet loss detection
* No timestamp included → handled on receiver side if needed

---

## Future Improvements

* Add timestamp or delta time
* Add configurable output rate via AE10
* Add calibration (bias / scale)
* Add quaternion or fused orientation output

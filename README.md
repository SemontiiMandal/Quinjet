# Quinjet
A custom mini quadcopter and remote controller. Built entirely from scratch, covering the PCB layout, Zephyr firmware, and flight hardware.

## Hardware Design & Schematics
Custome PCB designed in Altium Designer.

### Four Layer Flight Controller Board

**Layout and 3D View**

<img width="208" height="320" alt="image" src="https://github.com/user-attachments/assets/233277cd-628c-4c13-bc4a-4fa8b1c2f2c3" />
<img width="209" height="317" alt="image" src="https://github.com/user-attachments/assets/f9604bc2-ef96-401c-bba7-bffee0c3b0f9" />

<img width="203" height="304" alt="image" src="https://github.com/user-attachments/assets/80425b7c-5c54-4b40-9792-e77d1a1b7b54" />

<img width="220" height="323" alt="image" src="https://github.com/user-attachments/assets/795a7350-67ce-4f17-937b-cf581d9c7a31" />


**Schematic**

<img width="612" height="399" alt="image" src="https://github.com/user-attachments/assets/60333128-02d6-4497-9b37-2ff077dc3d59" />


### Two Layer Remote control Board

**Layout and 3D View**

<img width="401" height="238" alt="image" src="https://github.com/user-attachments/assets/1a04b550-618b-487b-a58e-6793f5aee49b" />
<img width="427" height="251" alt="image" src="https://github.com/user-attachments/assets/b67331ba-49f9-4812-a4f9-c8ba9dedc432" />
<img width="392" height="235" alt="image" src="https://github.com/user-attachments/assets/4bf446c2-cf03-4cfb-8820-360f66ba5855" />
<img width="392" height="238" alt="image" src="https://github.com/user-attachments/assets/7459c6a0-8f2e-445b-81c9-21f48d0c49fa" />

**Schematic**

<img width="545" height="358" alt="image" src="https://github.com/user-attachments/assets/c9dc4d27-9403-4da2-b354-0ec91f743330" />

---

## System Components and Interconnections

The Quinjet hardware ecosystem consists of two distinct nodes. Each utilizes the nRF52840's peripheral set to handle real-time sensor fusion, radio communication, and motor control.

### Flight Controller (FC) Hardware

* **BMI270 IMU (SPI):** Connects to the nRF52840 via the SPI bus. It uses a dedicated GPIO interrupt pin to wake the flight controller thread exactly when new sensor data is ready, ensuring 1000Hz synchronization.
* **Coreless Motors (Direct-Drive):** Four MOSFETs are driven directly by the nRF52840’s hardware PWM peripherals, operating at 32kHz for silent operation.
* **ESB Radio:** The 2.4GHz radio handles packets asynchronously via the Nordic ESB stack.

### Remote Controller (RC) Hardware

* **Joysticks (SAADC):** Two dual-axis analog joysticks connect to the nRF52840’s SAADC channels, sampled at 100Hz and normalized for transmission.
* **OLED Display (I2C):** Provides the pilot with real-time telemetry and system status.
* **Vibration Motor:** Driven via PWM to provide physical feedback to the pilot.

### Components Summary

| Peripheral | Board | Interface | Purpose |
| --- | --- | --- | --- |
| **BMI270** | FC | SPI | 6-Axis Motion Tracking |
| **Motors (x4)** | FC | PWM | Coreless DC Motor Control |
| **Joysticks** | RC | SAADC | Pilot Input |
| **OLED** | RC | I2C | Telemetry Display |
| **Radio Link** | Both | ESB (2.4GHz) | Low-latency control |

---

## Firmware Architecture

### Flight Controller Board

The Flight Controller (FC) is a high-performance, real-time system engineered to maintain a strict, 1000Hz control loop, ensuring the drone responds to flight dynamics with millisecond precision.

* **Physics-First Timing:** The firmware is driven by a hardware interrupt from the BMI270 IMU. This ensures the flight loop runs at a deterministic 1000Hz, perfectly synchronized to the physical reality of the drone's orientation.
* **Lock-Free Concurrency:** The system uses `k_spinlock` for memory protection, allowing the radio radio to asynchronously dump pilot commands into shared memory without ever blocking the critical PID loop or halting the Zephyr scheduler.
* **Safety & Control:** The system features a custom PID implementation with Anti-Windup logic and a Motor Mixer that acts as a hardware kill-switch, ensuring motors only spin when explicitly armed and commanded.

**For a deep dive into flight dynamics, spinlock logic, and PID tuning, see the [Flight Controller Architecture Documentation](https://github.com/SemontiiMandal/Quinjet/blob/main/firmware/flight_controller/Architecture.md).**

---
### Remote Controller Board

The Quinjet system utilizes a custom, low-latency firmware stack built on **Zephyr RTOS**. The remote controller and flight controller communicate via a proprietary Enhanced ShockBurst (ESB) radio link, sharing a strictly packed data structure for real-time control input.

* **Producer-Consumer Design:** The system uses a lock-free approach where the ADC thread produces joystick state, and the radio thread consumes it via semaphore-driven wakeups.
* **Hardware Safety:** The Remote Controller features a custom state machine that enforces a Zero-Throttle Gate, preventing the drone from arming unless the throttle stick is physically at the zero position.
* **Resource Safety:** Shared memory between threads is protected by mutexes, ensuring thread-safe operations during high-frequency telemetry updates.

**For a deep dive into the firmware design, concurrency models, and interrupt-driven logic, see the [Remote Controller Architecture Documentation](https://github.com/SemontiiMandal/Quinjet/blob/Separate-controller/firmware/remote_controller/Architecture.md).**

---


## Pics of Assembly in progress
<img width="1673" height="747" alt="WhatsApp Image 2026-06-20 at 6 49 02 PM" src="https://github.com/user-attachments/assets/09175b22-ffd7-4547-8345-a01407d9d10e" />

## Testing IMU SPI init and value reading (stationary board)
<img width="491" height="172" alt="image" src="https://github.com/user-attachments/assets/34b10cb0-b8a3-4e95-9363-3732c4dfbc26" />

## Testing Remote Controller (RC) Transmission (no RX connected yet)
<img width="374" height="99" alt="image" src="https://github.com/user-attachments/assets/52db7c10-d8ef-413e-8a44-60d6728cf272" />

## Testing Flight Controller
<img width="417" height="100" alt="image" src="https://github.com/user-attachments/assets/4fb4f298-1acd-43d1-ad62-25b044804eaf" />



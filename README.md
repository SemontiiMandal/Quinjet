# Quinjet
A custom mini quadcopter and remote controller. Built entirely from scratch, covering the PCB layout, Zephyr firmware, and flight hardware.

## Four Layer Flight Controller Board

**Altium View**

<img width="208" height="320" alt="image" src="https://github.com/user-attachments/assets/233277cd-628c-4c13-bc4a-4fa8b1c2f2c3" />
<img width="209" height="317" alt="image" src="https://github.com/user-attachments/assets/f9604bc2-ef96-401c-bba7-bffee0c3b0f9" />
<img width="203" height="304" alt="image" src="https://github.com/user-attachments/assets/80425b7c-5c54-4b40-9792-e77d1a1b7b54" />
<img width="220" height="323" alt="image" src="https://github.com/user-attachments/assets/795a7350-67ce-4f17-937b-cf581d9c7a31" />

## Two Layer Remote control Board

**Altium View**

<img width="401" height="238" alt="image" src="https://github.com/user-attachments/assets/1a04b550-618b-487b-a58e-6793f5aee49b" />
<img width="427" height="251" alt="image" src="https://github.com/user-attachments/assets/b67331ba-49f9-4812-a4f9-c8ba9dedc432" />
<img width="392" height="235" alt="image" src="https://github.com/user-attachments/assets/4bf446c2-cf03-4cfb-8820-360f66ba5855" />
<img width="392" height="238" alt="image" src="https://github.com/user-attachments/assets/7459c6a0-8f2e-445b-81c9-21f48d0c49fa" />

## Pics of Assembly in progress
<img width="1673" height="747" alt="WhatsApp Image 2026-06-20 at 6 49 02 PM" src="https://github.com/user-attachments/assets/09175b22-ffd7-4547-8345-a01407d9d10e" />

---

## Firmware Architecture

## Remote Controller Board

The Quinjet system utilizes a custom, low-latency firmware stack built on **Zephyr RTOS**. The remote controller and flight controller communicate via a proprietary Enhanced ShockBurst (ESB) radio link, sharing a strictly packed data structure for real-time control input.

* **Producer-Consumer Design:** The system uses a lock-free approach where the ADC thread produces joystick state, and the radio thread consumes it via semaphore-driven wakeups.
* **Hardware Safety:** The Remote Controller features a custom state machine that enforces a Zero-Throttle Gate, preventing the drone from arming unless the throttle stick is physically at the zero position.
* **Resource Safety:** Shared memory between threads is protected by mutexes, ensuring thread-safe operations during high-frequency telemetry updates.

**For a deep dive into the firmware design, concurrency models, and interrupt-driven logic, see the [Remote Controller Architecture Documentation](https://github.com/SemontiiMandal/Quinjet/blob/Separate-controller/firmware/remote_controller/Architecture.md).**

---

## Testing IMU SPI init and value reading (stationary board)
<img width="491" height="172" alt="image" src="https://github.com/user-attachments/assets/34b10cb0-b8a3-4e95-9363-3732c4dfbc26" />

## Testing Remote Controller (RC) Transmission (no RX connected yet)
<img width="374" height="99" alt="image" src="https://github.com/user-attachments/assets/52db7c10-d8ef-413e-8a44-60d6728cf272" />


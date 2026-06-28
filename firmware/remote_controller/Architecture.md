# Quinjet RC Controller Architecture

The Quinjet Remote Controller firmware is a real-time system built on the **nRF52840** and **Zephyr RTOS**. It is engineered to maintain a strict, low-latency hot path from the user’s fingertips to the radio transmitter, ensuring the drone responds instantly to pilot commands.

## 1. Core Workflow

The system operates as a producer-consumer engine where data flows from hardware sensors to the radio link through protected memory.

* **Joystick Sampling (Producer):** The `adc_thread` runs a deterministic 100Hz loop (every 10ms). It queries four ADC channels, converts the raw 12-bit hardware values into normalized floating-point coordinates (`-1.0` to `1.0`), and applies dead-zone and jitter compensation logic.
* **Resource Protection:** To prevent "data tearing" (where the radio sends a partially updated packet), the system **protects shared resources with a mutex** (`joystick_data`). The ADC thread locks this mutex before updating the shared `data_packet`, and the radio thread locks it before reading the payload.
* **Thread Synchronization:** Efficiency is achieved by letting the radio thread sleep when no new data is present. The ADC thread **wakes the radio thread by giving a semaphore** (`wake_esb`) immediately after it has finished refreshing the shared memory, effectively triggering the transmission only when absolutely necessary.
* **Air Link (Consumer):** The `esb_tx_thread` remains blocked, consuming zero CPU cycles until the semaphore wakes it. Once active, it creates a local snapshot of the control state, releases the mutex, and blasts the packet over the 2.4GHz Enhanced ShockBurst (ESB) link.

## 2. Safety and State Management

Flying hardware requires strict safety gates. We move away from polling in favor of hardware-level event handling.

* **Interrupt-Driven Logic:** The `rc_state_machine` uses physical GPIO interrupts on the joystick buttons. When a button is pressed, the ISR fires instantly to flip a `volatile bool` flag. This avoids the latency and CPU waste of checking pin states in a loop.
* **The Zero-Throttle Gate:** We implement a software-defined safety gate. The transition from `IDLE` to `ARMED` is only permitted if the `throttle` coordinate is below `0.05f` (near zero). This prevents the drone from jumping into the air if the stick is accidentally left pushed forward during the arming sequence.

## 3. Architectural Design Concepts

| Concept | Implementation Strategy |
| --- | --- |
| **Data Packing** | We use `__attribute__((packed))` on the shared `data_packet` struct. This forces the compiler to ignore padding bytes, ensuring the RC transmitter and FC receiver interpret the memory layout identically over the air. |
| **Modularity** | The system is split into `rc/`, `fc/`, and `shared/` directories. Common structs (like the 20-byte `data_packet`) reside in `shared/`, ensuring that if you update your packet format, both boards stay in sync. |
| **Jitter Mitigation** | The `decode_joystick` module performs software-based noise filtering on ADC samples. By mapping raw values to a consistent coordinate space, the flight controller remains agnostic to the specific potentiometer hardware used. |
| **Deterministic Timing** | By using `k_msleep(10)` for our control loop, we provide the Flight Controller with a perfectly steady stream of data, which is essential for stable PID loop performance. |

## 4. Logical Flow Summary

The system behaves as a synchronous link between the pilot and the flight hardware:

1. **Input:** Pilot moves the joysticks or toggles the arm button.
2. **Processing:** The ADC thread performs normalization and enforces the "Zero-Throttle" safety gate.
3. **Synchronization:** The shared memory is updated, and the radio thread is woken via semaphore.
4. **Transmission:** The radio thread copies the struct, handles ESB auto-ACKs, and pushes the data to the air interface.

This architecture ensures that the hot path is never blocked by slow radio hardware or erratic button presses, providing the literal locked-in feel required for reliable flight control!
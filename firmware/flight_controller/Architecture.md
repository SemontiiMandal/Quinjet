# Quinjet Flight Controller Architecture

The Quinjet Flight Controller (FC) firmware runs on an nRF52840 using the Zephyr RTOS. While the Remote Controller operates based on user input, the FC operates on a strict, deterministic 1000Hz loop to keep the quadcopter stable in real-time.

## 1. Core Workflow

The system separates the asynchronous arrival of radio commands from the synchronous execution of the flight math.

* **IMU Interrupts (The Clock):** The flight loop is driven by the BMI270 IMU. It triggers a hardware interrupt 1,000 times a second when new accelerometer and gyroscope data is ready. This ISR uses a semaphore (`imu_data_ready_sem`) to wake the flight control thread.
* **Radio Reception (Asynchronous):** The ESB radio listens in the background. When a packet from the RC arrives, it triggers the `esb_callback` ISR.
* **Spinlock Protection:** Because the radio data arrives via an interrupt, we can't use a standard Mutex (which would cause a kernel panic). Instead, we protect the shared `data_packet` using a `k_spinlock`. This briefly disables local CPU interrupts to safely copy the pilot's command into memory.
* **Flight Control Thread:** The main thread is statically allocated (`K_THREAD_DEFINE`) to avoid runtime memory fragmentation and context-switching delays. It sleeps until the IMU wakes it, grabs the latest data, runs the PID math, and goes right back to sleep.

## 2. Flight Dynamics and Safety

Controlling physical hardware requires built-in safety limits.

* **Custom PID with Anti-Windup:** The PID loops are written from scratch in C. The Integral (I) term includes strict anti-windup clamping so the motors don't spool up indefinitely if the drone gets stuck or blocked.
* **Software Kill-Switch:** The `mix_motors` function serves as the primary safety gate. If the pilot disarms the drone or drops the throttle to zero, the mixer ignores the PID outputs and forces a 0% duty cycle. This prevents sensor noise from spinning the props while on the ground.
* **Sensor Fusion:** A Complementary Filter merges the raw BMI270 data. It relies on the gyroscope for short-term rotation tracking and uses the accelerometer as a long-term reference to correct for gyro drift.

## 3. Architectural Design Concepts

| Concept | Implementation Strategy |
| --- | --- |
| **Interrupt-Safe Memory** | Using `k_spinlock` instead of mutexes ensures the radio can dump data into shared memory without halting the Zephyr scheduler or delaying the 1000Hz flight loop. |
| **No Stale Data** | We don't queue or buffer incoming radio packets. The PID loop only ever looks at the single newest command. Queuing would force the drone to execute outdated maneuvers. |
| **High-Frequency PWM** | The DeviceTree overrides standard PWM defaults to drive the coreless motors at **32 kHz**. This is above human hearing, which stops the motors from screeching and prevents electrical switching noise from messing with the IMU. |
| **Hardware Initialization** | The code enforces a strict boot sequence. This ensures the physical decoupling capacitors have time to stabilize the VDD/VDDIO rails before the SPI driver configures the IMU's interrupt registers, preventing brownouts. |

## 4. Logical Flow Summary

The system runs as a continuous, low-latency loop:

1. **Input:** The Radio ISR silently updates the shared memory buffer with pilot commands in the background.
2. **Trigger:** The BMI270 finishes a reading and fires an interrupt, waking the main thread.
3. **Processing:** The thread fetches the IMU data, runs Sensor Fusion, and calculates the Pitch, Roll, and Yaw PID corrections.
4. **Mixing:** The PID outputs are mapped to the physical Quad X motor layout.
5. **Output:** The final duty cycles (0–10000) are sent to the hardware PWM registers to update the motor speeds.
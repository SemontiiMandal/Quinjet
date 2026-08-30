# Quinjet Flight Controller Architecture

The Quinjet Flight Controller (FC) runs on an nRF52840 using Zephyr RTOS. The flight control loop executes at 1kHz, driven directly by a hardware interrupt from the BMI270 IMU.

## 1. Core Workflow

The system handles asynchronous radio commands while maintaining a synchronous 1kHz flight loop.

* **IMU Interrupt (1kHz Base):** The BMI270 triggers a hardware interrupt every 1ms when new data is ready. This ISR uses a semaphore (`imu_data_ready_sem`) to wake the flight control thread.
* **Radio Reception:** The ESB radio receives pilot commands asynchronously via the `esb_callback` ISR.
* **Spinlock Protection:** Radio data is protected using `k_spinlock`. This briefly disables local CPU interrupts to safely copy data into shared memory without halting the Zephyr scheduler or delaying the 1kHz loop.
* **Radio Failsafe:** The thread checks Zephyr's `k_uptime_get()`. If no valid radio packet is received for 500ms, the system triggers a hardware failsafe, dropping motor duty cycles to 0.
* **Flight Control Thread:** Statically allocated (`K_THREAD_DEFINE`) as a cooperative thread (Priority -1). It wakes, executes DSP and PID calculations uninterrupted, and immediately yields back to the scheduler.

## 2. Flight Dynamics

* **Back-Calculation Anti-Windup:** The PID loops track hardware saturation. If outputs demand more than 100% motor power, the system calculates the excess "phantom power" and subtracts it from the integral memory to prevent overshoot.
* **Measurement Derivative:** The D-term is calculated on the rate of change of the physical measurement rather than the error, preventing derivative kick during rapid joystick movements.
* **Dynamic Mixer Scaling & AirMode:** The motor mixer evaluates all four commanded outputs. If a motor exceeds 100% duty cycle, the mixer mathematically scales all four motors down equally to preserve differential thrust without an artificial throttle cap. It also scales motors up (AirMode) if they drop below minimum idle speed to maintain stabilization during free-fall.

## 3. Digital Signal Processing (DSP) Pipeline

The DSP pipeline cleans the raw sensor data before feeding it to the PID controller.

* **Data Ingestion (1kHz):** Driven by the 1ms IMU interrupt, raw accelerometer and gyroscope data is read and cast directly to 32-bit `float` variables. This locks the pipeline inside the nRF52840's hardware Floating-Point Unit (FPU), bypassing software math emulation.
* **Sliding Window FFT:** The system uses `memmove` to shift an overlapping 64-sample buffer every 1ms. Every 8ms (an 8-sample hop size), a pre-calculated Hann window is applied, and a CMSIS-DSP fast Fourier transform (`arm_rfft_fast_f32`) executes to identify the dominant motor resonance in the 100Hz–400Hz band.
* **Dynamic Notch Filter:** A biquad notch filter updates its center frequency based on the FFT output to mathematically remove the specific motor vibration peak from the gyroscope data.
* **Sensor Fusion:** A complementary filter (`Angle = 0.98 * (Angle + Gyro * dt) + 0.02 * Accel_Angle`) calculates orientation. It uses the gyroscope for short-term tracking and the accelerometer as a long-term reference to correct drift.
* **D-Term Smoothing:** The D-term passes through an independent 30Hz CMSIS biquad low-pass filter to prevent high-frequency noise from reaching the motors.

## 4. Architectural Design Concepts

| Concept | Implementation Strategy |
| --- | --- |
| **Interrupt-Safe Memory** | Using `k_spinlock` instead of mutexes ensures the radio can dump data into shared memory without halting the Zephyr scheduler or delaying the 1000Hz flight loop. |
| **No Stale Data** | We do not queue or buffer incoming radio packets. The PID loop only reads the single newest command. Queuing would force the drone to execute outdated maneuvers. |
| **High-Frequency PWM** | The DeviceTree overrides standard PWM defaults to drive the coreless motors at **32 kHz**. This is above human hearing and prevents electrical switching noise from interfering with the IMU. |

## 5. Logical Flow Summary

The system runs a continuous, low-latency loop, triggered by the IMU data-ready interrupts:

1. **Input:** The Radio ISR updates the shared memory buffer with pilot commands in the background.
2. **Trigger:** The BMI270 finishes a reading and fires an interrupt, waking the main thread.
3. **Failsafe Check:** The thread verifies the radio connection timestamp via OS uptime.
4. **Processing:** The thread fetches the IMU data, runs the DSP pipeline, executes Sensor Fusion, and calculates Pitch, Roll, and Yaw PID corrections.
5. **Mixing:** The PID outputs are mapped to the physical Quad-X motor layout using dynamic overflow scaling.
6. **Output:** The final duty cycles (0–10000) are sent to the hardware PWM registers to update the motor speeds.
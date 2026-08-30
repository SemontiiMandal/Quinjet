# Quinjet Digital Signal Processing (DSP) Pipeline

## 1. Data Ingestion & Formatting

The BMI270 IMU triggers a hardware interrupt every 1 millisecond. Raw 16-bit integer readings are fetched via SPI and immediately converted into standard floating-point metrics (m/s² for acceleration, rad/s for angular velocity). This strictly enforces FPU hardware execution for all subsequent stages.

## 2. Resonance Detection (Sliding Window FFT)

Motors and propellers generate high-frequency vibrations that travel through the frame into the gyroscope. To find this noise peak without delaying the flight loop, we use an overlapping sliding window FFT.

* **The Buffer Shift:** Every 1ms, a 64-sample buffer shifts left by one, dropping the oldest reading and appending the newest.
* **The Hop Size:** Every 8 milliseconds, the FFT executes on the 64-sample snapshot.
* **The Hann Window:** Before the FFT runs, the raw data is multiplied by a Hann window to taper the edges to zero. This prevents the FFT from seeing the chunk's boundaries as sudden, high-frequency "cliffs" (spectral leakage). The window is pre-calculated on boot:

$$w[n] = 0.5 \left( 1 - \cos\left(\frac{2\pi n}{N-1}\right) \right)$$


* **Magnitude Calculation:** The CMSIS-DSP `arm_rfft_fast_f32` function returns interleaved real and imaginary numbers. The true amplitude of each frequency bin $k$ is calculated as:

$$M[k] = \sqrt{\text{Re}(X[k])^2 + \text{Im}(X[k])^2}$$


The algorithm scans the 100Hz–400Hz range to pinpoint the exact frequency $f_c$ with the highest magnitude.

## 3. Dynamic Notch Filtering

Once the FFT identifies the peak motor resonance, passing it through a Biquad Notch Filter eliminates specific frequency from the gyroscope data.

As the pilot adjusts the throttle and the motor RPM shifts, the FFT detects the new vibration frequency and dynamically recalculates the biquad filter coefficients ($a_n$, $b_n$). The filter operates using the standard discrete-time transfer function:


$$H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{a_0 + a_1 z^{-1} + a_2 z^{-2}}$$


This basically neutralizes motor noise while passing the lower-frequency physical drone rotations through.

## 4. Sensor Fusion (Complementary Filter)

Now the system determines the drone's true orientation by fusing the gyroscope and accelerometer.

* **Accelerometer (Absolute but Noisy):** Trigonometry determines the angle relative to gravity.

$$\phi_{acc} = \arctan\left(\frac{a_y}{\sqrt{a_x^2 + a_z^2}}\right)$$


* **Gyroscope (Fast but Drifts):** The angular velocity is multiplied by the loop time ($dt = 0.001$) to find the rotation amount.
* **The Fusion:** A complementary filter merges them. It trusts the gyroscope for 98% of the split-second maneuver and mixes in 2% of the accelerometer to permanently anchor the horizon and prevent gyro drift:

$$\theta = \alpha (\theta + \omega dt) + (1 - \alpha) \theta_{acc}$$



*(where $\alpha = 0.98$)*

## 5. PID Control & D-Term Smoothing

Next, the PID controller calculates the motor adjustments needed to match the pilot's requested angle.

* **Measurement Derivative:** Derivative kick is a sudden, massive spike in the control output of a PID controller caused by an abrupt change in the setpoint. To prevent derivative kick if the pilot rapidly changes the joystick to a new setpoint, the derivative (braking) force is calculated purely on the rate of change of the drone's physical movement, ignoring the mathematical error:

$$D_{raw} = -\left(\frac{\text{Measured}_t - \text{Measured}_{t-1}}{dt}\right)$$


* **D-Term Low-Pass Filter:** Because taking a derivative mathematically amplifies any microscopic jitter left over from the notch filter, the raw D-term is passed through a dedicated 30Hz Biquad Low-Pass filter before it can multiply with the $K_d$ gain.
* **Back-Calculation Anti-Windup:** If the final PID output demands a duty cycle greater than 100%, the difference ($E_{excess}$) is instantly subtracted from the integral sum ($I_{sum}$) scaled by a back-tracking gain ($K_b$). This ensures the integral memory never winds up beyond the physical limits of the motors:

$$I_{sum} = I_{sum} - (E_{excess} \cdot K_b \cdot dt)$$
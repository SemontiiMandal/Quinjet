import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.signal import iirnotch, butter, lfilter, lfilter_zi

fs = 1000.0
N = 1000
t = np.arange(N) / fs
true_motion = np.sin(2 * np.pi * 2.0 * t) 
broadband_noise = np.random.normal(0, 0.2, N)

# Pre-calculate Hann Window (matches dynamic_notch_init)
fft_size = 64
hann_window = 0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(fft_size) / (fft_size - 1)))

def run_flight_controller_sim(motor_end_hz, lpf_fc, kd):
    # 1. Simulate a Throttle Punch (Motor frequency sweeps from 150Hz to slider value)
    f_start = 150.0
    f_t = np.linspace(f_start, motor_end_hz, N)
    phase = 2 * np.pi * np.cumsum(f_t) / fs
    motor_noise = 1.5 * np.sin(phase)
    raw_gyro = true_motion + motor_noise + broadband_noise

    # Output arrays
    notched_gyro = np.zeros(N)
    raw_d_term = np.zeros(N)
    clean_d_term = np.zeros(N)
    tracked_freq = np.zeros(N)

    # C-Code State Variables
    buffer = np.zeros(fft_size)
    prev_measurement = 0.0
    current_notch_hz = 150.0
    
    # Initialize Biquad States (zi)
    bn, an = iirnotch(current_notch_hz, 3.0, fs)
    zi_notch = np.zeros(max(len(bn), len(an)) - 1)
    
    bl, al = butter(2, lpf_fc / (fs / 2), btype='low')
    zi_lpf = np.zeros(max(len(bl), len(al)) - 1)

    # --- THE 1kHz FLIGHT LOOP ---
    for i in range(N):
        in_sample = raw_gyro[i]

        # 1. Sliding Buffer (memmove equivalent)
        buffer[:-1] = buffer[1:]
        buffer[-1] = in_sample

        # 2. FFT Update (Hop Size = 8)
        if i >= fft_size and i % 8 == 0:
            windowed = buffer * hann_window
            spectrum = np.abs(np.fft.rfft(windowed))
            freqs = np.fft.rfftfreq(fft_size, 1/fs)

            # Search 100Hz - 400Hz band
            valid_idx = np.where((freqs >= 100) & (freqs <= 400))[0]
            if len(valid_idx) > 0:
                peak_idx = valid_idx[np.argmax(spectrum[valid_idx])]
                detected_hz = freqs[peak_idx]

                # Update coefficients if frequency shifted significantly
                if abs(detected_hz - current_notch_hz) > 10.0:
                    current_notch_hz = detected_hz
                    bn, an = iirnotch(current_notch_hz, 3.0, fs)

        tracked_freq[i] = current_notch_hz

        # 3. Dynamic Notch Filter (Sample-by-Sample)
        out_notch, zi_notch = lfilter(bn, an, [in_sample], zi=zi_notch)
        notched_gyro[i] = out_notch[0]

        # 4. Raw Derivative (Finite Difference)
        raw_derivative = -(notched_gyro[i] - prev_measurement) * fs
        prev_measurement = notched_gyro[i]
        raw_d_term[i] = raw_derivative * kd

        # 5. D-Term LPF (Sample-by-Sample)
        out_lpf, zi_lpf = lfilter(bl, al, [raw_d_term[i]], zi=zi_lpf)
        clean_d_term[i] = out_lpf[0]

    return raw_gyro, notched_gyro, raw_d_term, clean_d_term, f_t, tracked_freq

# --- UI and Plotting ---
fig_time, axs_time = plt.subplots(4, 1, figsize=(10, 9))
fig_time.canvas.manager.set_window_title('True 1kHz Pipeline Simulator')
plt.subplots_adjust(hspace=0.6, top=0.92, bottom=0.05)

titles = [
    '1. Raw Gyro (Simulating Throttle Punch: Sweeping Motor Noise)', 
    '2. Dynamic Notch Output (Real-Time Suppression)', 
    '3. Raw D-Term (The Danger of Unfiltered Derivatives)', 
    '4. Filtered D-Term (Motors are Safe)'
]
colors = ['#d62728', '#ff7f0e', '#1f77b4', '#9467bd']
lines_time = []

for i in range(4):
    line, = axs_time[i].plot(t, np.zeros_like(t), color=colors[i])
    axs_time[i].set_title(titles[i], fontweight='bold')
    axs_time[i].set_ylim(-4, 4) if i < 2 else axs_time[i].set_ylim(-10, 10)
    axs_time[i].grid(True, alpha=0.3)
    lines_time.append(line)

# Second Window: FFT Tracker Proof
fig_track, ax_track = plt.subplots(figsize=(8, 4))
fig_track.canvas.manager.set_window_title('FFT Tracking Performance')
plt.subplots_adjust(bottom=0.4, top=0.85)

line_actual, = ax_track.plot(t, np.zeros_like(t), label='Actual Motor Hz (Sweeping)', color='gray', linestyle='--')
line_tracked, = ax_track.plot(t, np.zeros_like(t), label='FFT Tracked Hz (64-Sample Res)', color='green', lw=2)
ax_track.set_title('Sliding Window FFT Tracking the Motor Noise', fontweight='bold')
ax_track.set_ylabel('Frequency (Hz)')
ax_track.set_xlabel('Time (s)')
ax_track.set_ylim(100, 420)
ax_track.legend(loc='upper left')
ax_track.grid(True, alpha=0.3)

# Sliders (Safely spaced to prevent overlap)
ax_motor = plt.axes([0.15, 0.20, 0.75, 0.04])
ax_lpf   = plt.axes([0.15, 0.12, 0.75, 0.04])
ax_kd    = plt.axes([0.15, 0.04, 0.75, 0.04])

s_motor = Slider(ax_motor, 'Punch End Hz', 150.0, 400.0, valinit=350.0, color='gray')
s_lpf   = Slider(ax_lpf, 'LPF Cutoff Hz', 10.0, 100.0, valinit=30.0, color='#1f77b4')
s_kd    = Slider(ax_kd, 'Kd Gain', 0.0, 0.1, valinit=0.02, color='#9467bd')

def update(val):
    raw, notched, raw_d, clean_d, f_actual, f_tracked = run_flight_controller_sim(
        s_motor.val, s_lpf.val, s_kd.val
    )
    
    lines_time[0].set_ydata(raw)
    lines_time[1].set_ydata(notched)
    lines_time[2].set_ydata(raw_d)
    lines_time[3].set_ydata(clean_d)
    
    line_actual.set_ydata(f_actual)
    line_tracked.set_ydata(f_tracked)
    
    fig_time.canvas.draw_idle()
    fig_track.canvas.draw_idle()

s_motor.on_changed(update)
s_lpf.on_changed(update)
s_kd.on_changed(update)

update(None)
plt.show()
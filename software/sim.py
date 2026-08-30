import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.signal import iirnotch, butter, lfilter, freqz

fs = 1000.0
t = np.arange(0, 1.0, 1/fs)
true_motion = np.sin(2 * np.pi * 2.0 * t) 
broadband_noise = np.random.normal(0, 0.3, len(t))

fig_time, axs_time = plt.subplots(4, 1, figsize=(10, 8))
fig_time.canvas.manager.set_window_title('Time Domain: Signal Processing Pipeline')
plt.subplots_adjust(hspace=0.6, top=0.9, bottom=0.08)

lines = []
titles = [
    '1. Raw Sensor Data (True Motion + Motor Noise + Broadband)', 
    '2. After Dynamic Notch Filter (Remove Motor Noise)', 
    '3. After D-Term Biquad LPF (Smoothing Broadband Jitter)', 
    '4. Final PID D-Term Output (Derivative Braking Force)'
]
colors = ['#d62728', '#ff7f0e', '#1f77b4', '#9467bd']

for i in range(4):
    line, = axs_time[i].plot(t, np.zeros_like(t), color=colors[i])
    axs_time[i].set_title(titles[i], fontweight='bold')
    axs_time[i].set_ylim(-4, 4)
    axs_time[i].grid(True, alpha=0.3)
    lines.append(line)

# --- Window 2: Live Bode Plot & Interactive Controls ---
fig_ctrl, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(8, 8))
fig_ctrl.canvas.manager.set_window_title('Frequency Domain: Live Bode Plot & Controls')
plt.subplots_adjust(bottom=0.4, hspace=0.3, top=0.9)

line_mag, = ax_mag.semilogx([], [], color='black', lw=2)
ax_mag.set_title('Live Combined Bode Plot (Notch + LPF)', fontweight='bold')
ax_mag.set_ylabel('Magnitude (dB)')
ax_mag.set_xlim(10, 500)
ax_mag.set_ylim(-80, 5)
ax_mag.grid(True, which='both', alpha=0.4)

line_phase, = ax_phase.semilogx([], [], color='black', lw=2)
ax_phase.set_ylabel('Phase (degrees)')
ax_phase.set_xlabel('Frequency (Hz)')
ax_phase.set_xlim(10, 500)
ax_phase.set_ylim(-360, 10)
ax_phase.grid(True, which='both', alpha=0.4)

# Dedicated Slider Axes
ax_motor = plt.axes([0.15, 0.25, 0.75, 0.03])
ax_notch = plt.axes([0.15, 0.19, 0.75, 0.03])
ax_lpf   = plt.axes([0.15, 0.13, 0.75, 0.03])
ax_kd    = plt.axes([0.15, 0.07, 0.75, 0.03])

# Interactive Sliders
s_motor = Slider(ax_motor, 'Actual Motor Hz', 100.0, 400.0, valinit=215.0, color='#d62728')
s_notch = Slider(ax_notch, 'Notch Target Hz', 100.0, 400.0, valinit=215.0, color='#ff7f0e')
s_lpf   = Slider(ax_lpf, 'LPF Cutoff Hz', 10.0, 100.0, valinit=30.0, color='#1f77b4')
s_kd    = Slider(ax_kd, 'Kd (D-Gain)', 0.0, 0.1, valinit=0.02, color='#9467bd')

def update(val):
    # 1. Simulate Actual Motor Drift
    motor_noise = 1.5 * np.sin(2 * np.pi * s_motor.val * t)
    raw_gyro = true_motion + motor_noise + broadband_noise
    lines[0].set_ydata(raw_gyro)
    
    # 2. Notch Filter
    b_n, a_n = iirnotch(s_notch.val, 3.0, fs)
    notched = lfilter(b_n, a_n, raw_gyro)
    lines[1].set_ydata(notched)
    
    # 3. Low Pass Filter (Butterworth Biquad)
    b_l, a_l = butter(2, s_lpf.val / (fs / 2), btype='low')
    clean = lfilter(b_l, a_l, notched)
    lines[2].set_ydata(clean)
    
    # 4. Derivative (D-Term)
    derivative = np.gradient(clean, 1/fs)
    d_term = -s_kd.val * derivative
    lines[3].set_ydata(d_term)
    
    # 5. Live Combined Bode Plot (Notch Cascade * LPF Cascade)
    w, h_n = freqz(b_n, a_n, worN=2000, fs=fs)
    w, h_l = freqz(b_l, a_l, worN=2000, fs=fs)
    h_total = h_n * h_l
    
    mag = 20 * np.log10(np.maximum(np.abs(h_total), 1e-10))
    phase = np.unwrap(np.angle(h_total)) * 180 / np.pi
    
    line_mag.set_data(w, mag)
    line_phase.set_data(w, phase)
    
    fig_time.canvas.draw_idle()
    fig_ctrl.canvas.draw_idle()

s_motor.on_changed(update)
s_notch.on_changed(update)
s_lpf.on_changed(update)
s_kd.on_changed(update)

update(None)
plt.show()
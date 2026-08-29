import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.signal import iirnotch, butter, lfilter

fs = 1000.0
t = np.arange(0, 1.0, 1/fs)

# 1. Synthesize realistic quadcopter data
true_motion = np.sin(2 * np.pi * 2.0 * t) 
motor_noise = 1.5 * np.sin(2 * np.pi * 215.0 * t) 
broadband_noise = np.random.normal(0, 0.3, len(t))
raw_gyro = true_motion + motor_noise + broadband_noise

# 2. Setup Plotting (4 distinct stages)
fig, axs = plt.subplots(4, 1, figsize=(10, 10))
plt.subplots_adjust(bottom=0.25, hspace=0.5)

lines = []
titles = ['1. Raw Sensor Data', '2. After Notch Filter (215Hz)', '3. After PT1 LPF (30Hz)', '4. PID D-Term Output']
colors = ['red', 'orange', 'blue', 'purple']

for i in range(4):
    line, = axs[i].plot(t, np.zeros_like(t), color=colors[i])
    axs[i].set_title(titles[i])
    axs[i].set_ylim(-3, 3)
    lines.append(line)

# Sliders
ax_notch = plt.axes([0.15, 0.12, 0.65, 0.02])
ax_lpf = plt.axes([0.15, 0.08, 0.65, 0.02])
ax_kd = plt.axes([0.15, 0.04, 0.65, 0.02])

s_notch = Slider(ax_notch, 'Notch Freq', 100.0, 400.0, valinit=215.0)
s_lpf = Slider(ax_lpf, 'LPF Cutoff', 10.0, 100.0, valinit=30.0)
s_kd = Slider(ax_kd, 'Kd (D-Gain)', 0.0, 0.1, valinit=0.02)

def update(val):
    # Stage 1: Raw Data
    lines[0].set_ydata(raw_gyro)
    
    # Stage 2: Notch Filter
    b_n, a_n = iirnotch(s_notch.val, 3.0, fs)
    notched = lfilter(b_n, a_n, raw_gyro)
    lines[1].set_ydata(notched)
    
    # Stage 3: Low Pass Filter (Biquad)
    b_l, a_l = butter(2, s_lpf.val / (fs / 2), btype='low')
    clean = lfilter(b_l, a_l, notched)
    lines[2].set_ydata(clean)
    
    # Stage 4: D-Term Calculation (-Kd * d(Measurement)/dt)
    derivative = np.gradient(clean, 1/fs)
    d_term = -s_kd.val * derivative
    lines[3].set_ydata(d_term)
    
    fig.canvas.draw_idle()

s_notch.on_changed(update)
s_lpf.on_changed(update)
s_kd.on_changed(update)

update(None)
plt.show()
# 8520 Coreless Motor PWM Control Profile

This document details the relationship between the applied **32 kHz PWM signal** and the resulting **RPM** for the Powerday 8520 coreless brushed motor powered by a **4.2V lipo battery source**.

---

## 1. Motor & Signal Parameters

### Baseline Specifications
* **Max Speed (No-Load):** 53,000 RPM @ 5.0V
* **Target Source Voltage ($V_{\text{source}}$):** 4.2V (1S LiPo max)
* **PWM Carrier Frequency ($f$):** 32,000 Hz (32 kHz)

### Derived Constants
* **Motor Velocity Constant ($K_v$):**
  $$K_v = \frac{53,000 \text{ RPM}}{5.0\text{V}} = 10,600 \text{ RPM/V}$$
* **PWM Period ($T$):** The exact timeframe for one complete ON/OFF cycle.
  $$T = \frac{1}{f} = \frac{1}{32,000 \text{ Hz}} = 31.25\ \mu\text{s}$$

---

## 2. Core Mathematical Formulas

The system relies on three interconnected equations to translate software values to mechanical motion:

1. **Pulse ON Time ($T_{\text{on}}$):**
   $$T_{\text{on}} = T \times \text{Duty Cycle } (D)$$
2. **Average Output Voltage ($V_{\text{avg}}$):**
   $$V_{\text{avg}} = V_{\text{source}} \times \text{Duty Cycle } (D)$$
3. **Theoretical Motor Speed ($\text{RPM}_{\text{ideal}}$):**
   $$\text{RPM}_{\text{ideal}} = V_{\text{avg}} \times K_v$$

---

## 3. Detailed Duty Cycle Calculations (4.2V Source)

Below are three operational reference scenarios calculated across the voltage profile.

### Scenario A: 100% Duty Cycle (Full Throttle)
* **Duty Cycle ($D$):** $1.0$
* **ON Time ($T_{\text{on}}$):** $31.25\ \mu\text{s} \times 1.0 = 31.25\ \mu\text{s}$ (Continuous high state)
* **Average Voltage ($V_{\text{avg}}$):** $4.2\text{V} \times 1.0 = 4.2\text{V}$
* **Calculation:**
  $$\text{RPM} = 4.2\text{V} \times 10,600 \text{ RPM/V} = \mathbf{44,520 \text{ RPM}}$$

### Scenario B: 75% Duty Cycle (Cruising Speed)
* **Duty Cycle ($D$):** $0.75$
* **ON Time ($T_{\text{on}}$):** $31.25\ \mu\text{s} \times 0.75 = 23.4375\ \mu\text{s}$
* **Average Voltage ($V_{\text{avg}}$):** $4.2\text{V} \times 0.75 = 3.15\text{V}$
* **Calculation:**
  $$\text{RPM} = 3.15\text{V} \times 10,600 \text{ RPM/V} = \mathbf{33,390 \text{ RPM}}$$

### Scenario C: 25% Duty Cycle (Low Speed/Idle)
* **Duty Cycle ($D$):** $0.25$
* **ON Time ($T_{\text{on}}$):** $31.25\ \mu\text{s} \times 0.25 = 7.8125\ \mu\text{s}$
* **Average Voltage ($V_{\text{avg}}$):** $4.2\text{V} \times 0.25 = 1.05\text{V}$
* **Calculation:**
  $$\text{RPM} = 1.05\text{V} \times 10,600 \text{ RPM/V} = \mathbf{11,130 \text{ RPM}}$$

---

## 4. Summary Duty Cycle to RPM Lookup Table

The following theoretical lookup table assumes an 8-bit resolution microcontroller layout (0 - 255 steps) mapping to the 4.2V ceiling.

| 8-Bit Step | Duty Cycle (%) | Average Voltage ($V_{\text{avg}}$) | Theoretical No-Load RPM |
| :--- | :--- | :--- | :--- |
| **255** | 100% | 4.20V | 44,520 RPM |
| **204** | 80% | 3.36V | 35,616 RPM |
| **153** | 60% | 2.52V | 26,712 RPM |
| **102** | 40% | 1.68V | 17,808 RPM |
| **51** | 20% | 0.84V | 8,904 RPM |
| **25** | ~10% | 0.42V | *Friction Threshold Boundary* |

---

## 5. Physical Implementation Constraints

1. **Aerodynamic Loading (75mm Propeller):** 
   The calculation yields *no-load* RPM metrics. Attaching the 75mm propeller introduces aerodynamic drag torque. Because propeller power demands scale cubically with velocity ($P \propto \text{RPM}^3$), the linear voltage-to-RPM relationship will **flatten out aggressively at high duty cycles**. Actual operational RPM will sit lower than theoretical models.
2. **High Carrier Frequency Advantage:** 
   Utilizing a **32 kHz** carrier provides an ultrashort period ($31.25\ \mu\text{s}$). Coreless motors have ultra low core inductance, so standard low-frequency PWM signals cause current ripples, causing heating up and motor ripple. A 32 kHz frequency not only ensures clean and steady DC equivalence, but also dampens audible switching out of human hearing range!

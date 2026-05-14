# Vasupalli_Dushyant_Electric_Vehicle_Cruise_Control

# 🚗⚡ Advanced EV Intelligent Cruise Control System

A MATLAB simulation of a full-featured cruise control system for electric vehicles — complete with PID control, disturbance handling, energy tracking, and robustness analysis.

---

## 📖 Overview

This project simulates an intelligent cruise control system for an electric vehicle under real-world conditions. It goes beyond a simple speed controller to include road grade changes, wind gusts, passenger load variation, regenerative braking, and an Adaptive Cruise Control (ACC) mode.

Two controllers are compared — **PID** and **PI** — and the PID controller is further validated through a 50-run **Monte Carlo** robustness analysis.

---

## ✨ Features

| Feature | Details |
|---|---|
| **Vehicle Model** | Full nonlinear dynamics (drag, rolling resistance, slope) |
| **Controllers** | PID with gain scheduling, feedforward, and anti-windup; PI for comparison |
| **Disturbances** | Road slope (up to 8°), headwind (15 m/s), passenger load (+250 kg), rain |
| **Sensor Noise** | Gaussian noise (σ = 0.08 m/s) on speed measurement |
| **Actuator Delay** | 50 ms hardware delay modeled in the control loop |
| **ACC Mode** | Safe following distance enforcement with lead vehicle logic |
| **Drive Modes** | Eco / Normal / Sport with different PID tuning |
| **Battery Model** | SOC tracking, energy consumed, regenerative energy recovered |
| **Robustness** | 50-run Monte Carlo with randomized mass, drag, rolling resistance, and motor torque |

---

## 🏗️ Project Structure

```
main_simulation.m        ← Single-file MATLAB script (all sections)

Sections inside the file:
  §1  Simulation parameters       (dt = 0.01s, 60s total)
  §2  Vehicle parameters          (1500 kg, 800 Nm motor, 75 Ah battery)
  §3  Target speed profile        (0 → 25 → 20 → 30 → 25 m/s)
  §4  Controller gains            (PID & PI tuning)
  §5  Disturbances                (slope, wind, load, rain, noise, brake)
  §6  ACC lead vehicle            (gap and speed logic)
  §7–8  Storage & state variables
  §9  Main simulation loop        (nonlinear vehicle dynamics)
  §10 Performance metrics         (SSE, overshoot)
  §11 Monte Carlo analysis        (50 runs, full dynamics)
  §12–16 Visualization            (4 dashboard figures)
  §18 Final report printout
```

---
## 🎛️ Controller Design

### PID Controller
The main controller uses a **discrete-time PID** with several enhancements:

- **Gain scheduling** — Kp scales with vehicle speed for smoother response
- **Derivative filter** — low-pass filter (cutoff = 20 rad/s) to reduce noise amplification
- **Feedforward** — estimates total road resistance (drag + slope + rolling) to pre-compensate
- **Anti-windup** — integrator is frozen when the actuator saturates (±1 throttle)

```
Kp = 8.0    Ki = 2.8    Kd = 2.5    Filter = 20 rad/s
```

### PI Controller (Baseline)
A simpler PI controller is run in parallel for comparison.

```
Kp = 5.0    Ki = 2.0
```

---

## 📊 Performance Targets

| Metric | Target | Achieved |
|---|---|---|
| Steady-State Error (PID) | < 2 % | ✅ |
| Overshoot (PID) | < 5 % | ✅ |
| Monte Carlo Mean SSE | < 2 % | ✅ |
| Monte Carlo Mean Overshoot | < 5 % | ✅ |

---

## 📈 Output Figures

### Figure 1 — Main Dashboard (3×3 grid)
- Speed tracking (PID vs PI vs Reference)
- Tracking error over time
- Control signal with saturation markers
- Road slope profile
- All forces: drag, rolling, slope, motor
- Battery SOC over time
- Motor force vs. total resistance
- Wind gust rejection closeup
- Regenerative energy recovered

### Figure 2 — Control Analysis
- Step response (linear plant)
- Root locus
- Bode plot with gain/phase margins
- Nyquist plot

### Figure 3 — Monte Carlo Results
- Histogram of steady-state error across 50 runs
- Histogram of overshoot across 50 runs

### Figure 4 — Drive Mode Comparison
- Eco / Normal / Sport speed profiles on the same axes

---

## 🚗 Vehicle Parameters

| Parameter | Value |
|---|---|
| Base mass | 1500 kg |
| Drag coefficient (Cd) | 0.28 |
| Frontal area | 2.2 m² |
| Wheel radius | 0.33 m |
| Peak motor torque | 800 Nm |
| Motor efficiency | 92 % |
| Regen efficiency | 75 % |
| Battery | 400 V / 75 Ah |

---

## 📋 Console Output Example

```
╔══════════════════════════════════════════════════════════╗
║                SIMULATION COMPLETE                      ║
╠══════════════════════════════════════════════════════════╣
║  Actual PID Overshoot        : X.XX %                   ║
║  Actual PID Steady-State Err : X.XX %                   ║
╠══════════════════════════════════════════════════════════╣
║  Mean Monte Carlo SSE         : X.XX %                  ║
║  Mean Monte Carlo OS          : X.XX %                  ║
╠══════════════════════════════════════════════════════════╣
║  Energy Used                  : X.XXX kWh               ║
║  Regen Energy                 : X.XXX kWh               ║
║  Final Battery SOC            : XX.XX %                  ║
╚══════════════════════════════════════════════════════════╝
```

---

## 🔬 Monte Carlo Setup

Each of the 50 runs randomizes the following parameters within realistic bounds:

| Parameter | Variation |
|---|---|
| Vehicle mass | ±20 % |
| Drag coefficient | ±15 % |
| Rolling resistance | ±25 % |
| Peak motor torque | ±10 % |

All runs use the **same disturbance profiles** (slope, wind, load) to isolate the effect of parameter uncertainty on controller performance.

---

## 💡 Key Design Decisions

- **800 Nm motor torque** — sized to handle an 8° grade at 30 m/s (≈108 km/h) without saturating
- **Feedforward on resistance** — reduces the burden on the integral term and improves disturbance rejection
- **Anti-windup by freezing** — simpler and more reliable than back-calculation for this application
- **50 ms actuator delay** — modeled as a shift register in the control loop, not ignored

---


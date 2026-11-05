# Peltier Surface Temperature Control (PID)

Closed-loop PID control of a Peltier-cooled surface. The project covers plant modelling, PID tuning, a Simulink model, Arduino firmware and Python/MATLAB analysis scripts.

*Group project for IE3034 Control Systems Engineering, SLIIT.*

## What it does
- Models the Peltier surface as a first-order-plus-dead-time plant and designs P / PI / PID controllers
- Compares PI vs PID in simulation: gain sweeps, setpoint tracking, disturbance rejection, robustness
- Arduino firmware: 1 s PID loop, 25 kHz PWM drive, safety trips, serial commands and CSV logging
- Python script that cross-checks the MATLAB simulation numbers

## Tech
MATLAB, Simulink, Arduino (C++), Python

## Status
Model parameters (K, tau, dead time) are provisional values used for design and simulation. Replace them with measured values before presenting results as experimental.

## Authors
Vishmini Michel and group members.

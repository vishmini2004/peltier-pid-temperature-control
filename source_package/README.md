# Peltier surface temperature control — IE3034 PID project

Closed-loop PID control of a Peltier-cooled surface: model, simulation, tuning,
Simulink model, Arduino firmware and the analysis scripts for the lab tests.

## Layout

| Folder | Contents |
| --- | --- |
| `matlab/` | Model, simulation, tuning, identification and comparison scripts (run `run_all`) |
| `firmware/peltier_pid/` | Arduino Uno/Nano sketch: PID at 1 s, 25 kHz PWM, safety trips, serial commands, CSV log |
| `verification/` | `verify_sim.py`, a Python copy of `simulate_loop.m`; `results.json` holds every number quoted in the report |
| `data/` | Created by the scripts. Put lab logs here as `openloop_step.csv` and `closedloop_run.csv` |
| `figures/` | Created by the scripts (PNG for the report, FIG for editing) |

## Quick start

1. MATLAB (Control System Toolbox; Simulink optional): `cd matlab`, then `run_all`.
2. Arduino IDE: install the **OneWire** and **DallasTemperature** libraries, then open `firmware/peltier_pid/peltier_pid.ino`.
3. Lab: follow report section 10 (tests E0–E10). Log with `log_serial("COMx", "../data/<name>.csv", seconds)`.
4. After test E2, run `s08_identify_plant`, copy K, tau and L into `peltier_params.m`, re-run `s03_pid_design`, and load the new gains with `KP`/`KI`/`KD`.
5. After the closed-loop tests, run `s09_sim_vs_experiment`.

## Status — read before submitting

- Plant parameters K = 15 degC, tau = 180 s and L = 5 s are **provisional**. K and tau come from the supplied illustrative script; L is assumed. Replace all three with measured values.
- `verify_sim.py` was run and produced the report's numbers. The MATLAB/Simulink files were written without MATLAB available and have **not** been run. The firmware has **not** been compiled. Run `run_all` and compile the sketch first, then check the printed tables against `verification/results.json`.
- Files named `SYNTHETIC_*` are generated test data for exercising s08/s09. They are not measurements.

## Python cross-check

```
cd verification
python verify_sim.py
```

# Adaptive Cruise Control · AutoTech 6DV3

MATLAB/Simulink reproduction of the hierarchical adaptive cruise controller in Fei and Wang (2021). The project includes the four scenarios in the paper, a stop-and-go test, and a steady-cruise test. The controller uses bumper gap and relative speed to switch between cruise and following.

[中文构建与原理指南](README_复现指南.md) · [Reference paper](https://doi.org/10.1109/CAC53003.2021.9727589)

## Run the model

Clone this repository and open its root folder in MATLAB. The baseline was tested with MATLAB and Simulink R2024a Update 3.

```matlab
open_system('acc_paper_baseline')   % Default scenario: lead vehicle deceleration
M4 = run_baseline(4);              % Run Figure 14 and export its data and plot
M = run_baseline;                  % Run all six scenarios and update results/
verify_carsim_interface            % Replay the Figure 14 signals through the adapter
```

The checked-in results come from the MATLAB sedan model. In the Figure 14 scenario, the ego vehicle switches to following at 8.48 s and settles within 1 km/h of 30 km/h at 19.36 s. The CarSim adapter has been checked by signal replay. A closed-loop run with a CarSim 2024.1 vehicle dataset remains pending. [Figure 14 plot](results/fig14_deceleration.png) · [Metrics](results/metrics.csv) · [Acceptance checks](results/checks.csv)

## Where to make changes

| If you are working on | Start here |
| --- | --- |
| Lead trajectory, initial gap, or cruise setpoint | [baseline_case.m](baseline_case.m) |
| Range measurement, mode switching, or desired following distance | [range_sensor.m](range_sensor.m) and [follow_reference.m](follow_reference.m) |
| Speed response, throttle, or brake pressure | [speed_control.m](speed_control.m) and [inverse_actuators.m](inverse_actuators.m) |
| MATLAB vehicle or transmission | [sedan_step.m](sedan_step.m), [baseline_forces.m](baseline_forces.m), and [baseline_gear.m](baseline_gear.m) |
| CarSim signal order, units, and connections | [carsim_feedback.m](carsim_feedback.m) and [build_carsim_interface.m](build_carsim_interface.m) |

For controller or diagram changes, edit the corresponding MATLAB source and run `build_baseline` and `build_carsim_interface`. The builders overwrite the generated models, including manual changes made only in Simulink. Run the affected scenario while editing. Before sharing a controller change, run all six scenarios and the interface replay. Commit the source and rebuilt models together on a feature branch, then open a pull request for the team.

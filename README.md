# A.R.I.A. — Autonomous Rescue & Intelligence Agent

Team Kavosh Senior's controller for **RoboCupJunior Rescue Simulation**, written in Python for the Webots simulator.

A.R.I.A. explores an unknown maze, builds a map as it goes, identifies victims and hazards from camera and lidar data, and reports what it finds — all autonomously, scored against the competition rules.

**Results:** 1st place, Rescue Simulation SuperTeam, RoboCupJunior World Cup 2026 · 2nd place, Rescue Simulation, RoboCup Americas / US Open 2025.

## How it works

| Subsystem | Detail |
| --- | --- |
| Vision | Cameras classify victims as `Harmed`, `Unconscious`, `Stable`, `Chemical`, `Paramedic`, `Fire` or `Other` |
| Ground sensing | Colour sensor reads tile type (checkpoints, holes, swamps) |
| Localisation | GPS and IMU give position and heading; yaw is integrated to keep orientation consistent through turns |
| Ranging | Lidar measures distance to walls and obstacles, feeding both navigation and victim confirmation |
| Mapping | Position is tracked in `x_map` / `y_map`; walls and obstacles accumulate into `mapData` arrays for the final map submission |
| Reporting | Findings go out over the emitter in the format the scoring system expects |

The central routine is `p_find()`, which cross-checks camera imagery against lidar returns before committing to a victim call — a wrong classification costs more than a missed one under the competition's scoring, so detection is deliberately conservative.

## Repository contents

| File | What it is |
| --- | --- |
| `Final_Rescue Line code2025.py` | Rescue Line controller |
| `the changed best code.py` | Current best-performing maze controller |
| `Code explaination.txt` | Written walkthrough of the controller architecture |
| `Design.json`, `Final final file.json` | Robot and world configuration |

## Running it

1. Install [Webots](https://cyberbotics.com/) and open a RoboCupJunior Rescue Simulation world.
2. Set the robot's controller to the Python file you want to run.
3. Run the simulation. The controller drives, maps and reports on its own — no operator input.

## Team

Built with Team Kavosh Senior (KAVOSH AI & Robotics Academy). Maintained by [Shayan Doroudiani](https://github.com/shayan2008).

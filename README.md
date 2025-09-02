# ROS2_Brake_Validation_Sim - ROS 2 closed-loop 1D braking sim (Stage 3)

## Project Context

This repo is **Stage 3** of a project during my [ADAS Learning Sprint](https://github.com/IvanMcCauley/Adas_Learning_Sprint):

1. **[Braking decision library (C++17)](https://github.com/IvanMcCauley/braking_decision_lib)** - standalone math + unit tests  
2. **[ROS 2 integration](https://github.com/IvanMcCauley/ros2_brake_decider/blob/main/README.md)** - parameterized node with pub/sub  
3. **1D longitudinal simulation (this repo)** - validate decisions in a simulation 

**Goal:** sanity-check that my ROS 2 brake decider’s `/brake_cmd` actually stops the car before an obstacle under simple 1D dynamics.

---

## What this is
- Tiny **Python / rclpy** sim that:
  - **Subscribes**: `/brake_cmd` (`std_msgs/Bool`)
  - **Publishes**: `/ego_speed` (m/s), `/obstacle_distance` (m)
  - Integrates 1D motion at **dt = 0.05 s** and logs CSV: `t,x,v,d,brake_cmd,u`
- **Matplotlib** script that replays the CSV to a split-screen **MP4** (car vs wall + `v(t)` / `d(t)` + HUD)
- Keeps it honest: simple physics, expected ON/OFF chatter (no hysteresis yet)

---

## TL;DR - run it

**Terminal A - decider**
```bash
# working dir:
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros2_brake_decider brake_decider.launch.py
```
**Terminal B - sim**
```bash
# working dir:
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run longitudinal_1d_sim sim_node
```
Wait for a line like:
<code>SUCCESS - stopped before obstacle at t=…; d=…; v=0.00 m/s. CSV: ~/ros2_ws/logs/longitudinal_sim_*.csv</code>

**Terminal C - render the video**
```bash
# deps (once):
sudo apt update && sudo apt install -y ffmpeg
python3 -m pip install --user matplotlib numpy

# working dir:
cd ~/ros2_ws/src/longitudinal_1d_sim
python3 scripts/make_video_anim.py
```
MP4 goes to: <code>~/ros2_ws/results/videos/.</code>

---
## Demo
**REMINDER FOR ME: ADD IN GIF HERE**

---

## ROS 2 I/O

**Subscribe**
- `/brake_cmd` - `std_msgs/Bool` (from Stage 2)

**Publish**
- `/ego_speed` - `std_msgs/Float64` (m/s)
- `/obstacle_distance` - `std_msgs/Float64` (m)

**Rate**
- 20 Hz (dt = 0.05 s)

---

## Topics & params (cheat-sheet)

|                | name             | default  |
|----------------|------------------|----------|
| decider param  | `reaction_time`  | 1.0 s    |
| decider param  | `decel`          | 9.0 m/s^2|
| decider param  | `safety_margin`  | 0.10     |
| sim internal   | `dt`             | 0.05 s   |

## Run without Stage 2 (manual toggle)

If you just want to see the sim move, you can publish `/brake_cmd` by hand:

```bash
# brake ON for 1 s, then OFF
ros2 topic pub /brake_cmd std_msgs/Bool '{data: true}' --once
sleep 1
ros2 topic pub /brake_cmd std_msgs/Bool '{data: false}' --once
```
(With Stage 2 running, **don’t** do this—let the decider drive it.)

---

## Dynamics (discrete)


# ROS2_Brake_Validation_Sim - ROS 2 closed-loop 1D braking sim (Stage 3)
---

## Project context
<img src="demo_gif.gif" align="right" width="400">
<br>  <!-- small spacer so it doesn’t collide with the heading -->

This is **Stage 3** of my ADAS Learning Sprint:

1. **[Braking decision library (C++17)](https://github.com/IvanMcCauley/braking_decision_lib)** - math + tests  
2. **[ROS 2 brake decider](https://github.com/IvanMcCauley/ros2_brake_decider)** - node with params/pub/sub  
3. **1D sim (this repo)** - verify the node’s `/brake_cmd` actually stops before an obstacle

**Goal:** sanity-check that `/brake_cmd` prevents collision under a simple 1D model.

<br clear="right"/>

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

## Why this matters
This is a **closed-loop** check: the sim replays the real /brake_cmd from Stage 2, integrates simple dynamics, and verifies we actually stop before the wall.

---

## ROS 2 I/O

**Subscribe**
- `/brake_cmd` - `std_msgs/Bool` (from Stage 2)

**Publish**
- `/ego_speed` - `std_msgs/Float64` (m/s)
- `/obstacle_distance` - `std_msgs/Float64` (m)

**Rate**
- 20 Hz (dt = 0.05 s)


### Topics & params (cheat-sheet)

|                | name             | default  |
|----------------|------------------|----------|
| decider param  | `reaction_time`  | 1.0 s    |
| decider param  | `decel`          | 9.0 m/s^2|
| decider param  | `safety_margin`  | 0.10     |
| sim internal   | `dt`             | 0.05 s   |

### (Optional) Run without Stage 2 
```bash
# toggle brake by hand (demo only)
ros2 topic pub /brake_cmd std_msgs/Bool '{data: true}' --once
sleep 1
ros2 topic pub /brake_cmd std_msgs/Bool '{data: false}' --once
```

---

## Dynamics (discrete)
State: position **x [m]**, velocity **v [m/s]**. Obstacle fixed at `x_obs; distance d = x_obs − x`.

Control **u [m/s^2]** from /brake_cmd:
- `u = 0` if brake = false
- `u = -decel` if brake = true

Update each tick:
```lua
v_{k+1} = max(0, v_k + u_k * dt)
x_{k+1} = x_k + v_k * dt
```
CSV columns: `t, x, v, d, brake_cmd, u`.
Expect some ON/OFF chatter near the threshold (single-threshold rule).

---
## Visualization (CSV → MP4)
- Left: car vs wall; red brake light when braking.
- Right: `v(t)` and `d(t)` with a time cursor (replay of the CSV).
- If MP4 fails, script falls back to GIF.

---

## Repo layout (what matters)
```
longitudinal_1d_sim/
├─ longitudinal_1d_sim/
│  └─ sim_node.py            # ROS 2 sim node (rclpy)
├─ scripts/
│  ├─ make_video.py          # quick static preview (PNG)
│  └─ make_video_anim.py     # CSV → split-screen MP4 replay
├─ package.xml
├─ setup.py / setup.cfg
└─ resource/…                 # standard ament_python bits
```
Runtime folders (created automatically):
```
~/ros2_ws/logs/            # CSV logs (t,x,v,d,brake_cmd,u)
~/ros2_ws/results/videos/  # MP4/GIF renders
```

---

## Install & Build
```bash
# place the package in ~/ros2_ws/src/ and build only this pkg
cd ~/ros2_ws
colcon build --packages-select longitudinal_1d_sim --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# python libs
python3 -m pip install --user matplotlib numpy   # or: sudo apt install python3-matplotlib

```

---

## “Why did it brake?” - decision rule (for context)
```ìni
d_react = v * reaction_time
d_brake = v^2 / (2 * decel)
d_stop  = d_react + d_brake
d_avail = obstacle_distance * (1 - safety_margin)
brake   = (d_stop >= d_avail)
```
The sim does **not** re-implement that rule; it only **consumes** `/brake_cmd` and applies physics to verify the stop.

---

## Debug / sanity checks
```bash
# working dir:
cd ~
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# discovery
ros2 node list
ros2 node info /brake_decider
ros2 topic list | egrep 'ego_speed|obstacle_distance|brake_cmd'

# build only this package
cd ~/ros2_ws
colcon build --packages-select longitudinal_1d_sim --symlink-install

# newest CSV exists?
ls -t ~/ros2_ws/logs/longitudinal_sim_*.csv | head -n 1

# mp4 writer missing?
sudo apt install -y ffmpeg
```

Common gotchas:
- **No CSV:** the sim didn’t reach success or logs dir missing (script creates it).
- **First v ~ 19.6 m/s:** decider braked on the first 50 ms tick - expected.
- **Chatter in video:** single threshold = ON/OFF near the boundary by design for v0.1.

---

## What I learned (short)
- rclpy basics (subs, timers, logging), simple QoS.
- Matplotlib layout (GridSpec, HUD via transforms), saving MP4 with ffmpeg.
- Why threshold logic chatters in discrete time (and fixes like hysteresis).

--- 

## Provenance

Built incrementally from ROS 2 docs/tutorials. I used assistant-generated boilerplate for some scaffolding; design, wiring, and validation are mine.

## Known limitations

1D model, constant decel, no hysteresis/latch, ideal measurements.

## License
MIT (see [License](https://github.com/IvanMcCauley/ros2_brake_validation_sim/blob/main/LICENSE)).

# Real Robot: Setup & Commissioning Guide
## Table of Contents

1. [SSH Connection, Repo Clone & Dependency Install](#1-ssh-connection-repo-clone--dependency-install)
2. [udev Fixed Device Names for RPLidar & ESP32](#2-udev-fixed-device-names-for-rplidar--esp32)
3. [PID Tuning & Physical Calibration](#3-pid-tuning--physical-calibration)
4. [SLAM Toolbox — Building the Map](#4-slam-toolbox--building-the-map)
5. [Nav2 — Autonomous Navigation](#5-nav2--autonomous-navigation)
---

## 1. SSH Connection, Repo Clone & Dependency Install

### 1.1 Find the Pi 5's IP address

Run this on the Pi (connected to a monitor temporarily, or check the router's DHCP table):

```bash
hostname -I
# returns something like 192.168.1.42 — note this down
```
---

### 1.2 SSH from the Windows laptop into Pi 5
Both devices must be on the same network (same Wi-Fi or router).

```powershell
# In Windows PowerShell
ssh hania@192.168.1.42
# Replace the IP with the one from step 1.1
# Default Pi username is 'hania' — change if different
# Accept the fingerprint prompt , enter ur Pi password
```

>  **Tip:** To avoid typing the password every time, set up SSH key auth: run `ssh-keygen` on Windows, then `ssh-copy-id hania@<IP>`.

---

### 1.3 Create workspace and clone the repo

All commands run on Pi 5 over SSH.

```bash
# Create workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone the repo
git clone https://github.com/hania222/real_robot.git

# Verify it cloned
ls real_robot/
```

---

### 1.4 Run install_deps.sh to install all dependencies

Installs ROS 2 packages, Nav2, slam_toolbox, robot_localization, and more.

```bash
cd ~/robot_ws/src/real_robot
chmod +x install_deps.sh
./install_deps.sh
# This will sudo apt install everything — takes a few minutes
```

> **micro-ROS agent is NOT available via apt, it must be built from source.** 

micro-ROS agent must be built from source, it's not available via apt. Build it once here, then run it manually in its own terminal to verify the ESP32 is connected
and publishing before launching bringup. It's commented out in bringup.launch.py intentionally, uncomment it only after confirming everything works end-to-end.:

```bash
cd ~
git clone https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
cd ~/robot_ws
colcon build --packages-select micro_ros_setup
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash
```

---

### 1.5 Build the workspace

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# Add to .bashrc so it auto-sources on every SSH login
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
>note: we will use our built .bashrc directly 
---

## 2. udev Fixed Device Names for RPLidar & ESP32

> **Why this matters:** Linux assigns `/dev/ttyUSB0` and `/dev/ttyUSB1` randomly on boot. without udev rules the launch files will connect to the wrong device. These rules pin names by USB vendor/product ID so the lidar is always `/dev/rplidar` and the ESP32 is always `/dev/esp32`.

### 2.1 Find the USB vendor and product IDs

```bash
# Plug only the RPLidar in first
lsusb
# Look for "Silicon Labs CP210x" or "Prolific" — note idVendor:idProduct
# Example: Bus 001 Device 003: ID 10c4:ea60 Silicon Labs CP210x

# Now also plug in the ESP32
lsusb
# ESP32 usually shows up as Silicon Labs CP2102 or CH340
# Note the different idVendor:idProduct for each

# Also check the serial number to distinguish two Silicon Labs devices
udevadm info /dev/ttyUSB0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT"
udevadm info /dev/ttyUSB1 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT"
```

---

### 2.2 Create the udev rules file

```bash
sudo nano /etc/udev/rules.d/99-robot.rules
```

Paste this: replace values with what we got:

```
# RPLidar A1 (Silicon Labs CP2102 — adjust ATTRS if needed)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="YOUR_LIDAR_SERIAL", \
  SYMLINK+="rplidar", MODE="0666"

# ESP32 (CP2102 or CH340 — adjust idVendor/idProduct)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="YOUR_ESP32_SERIAL", \
  SYMLINK+="esp32", MODE="0666"
```

>  If both devices share the same VID:PID (both are CP2102), use the serial number field to tell them apart — that's why u ran the `ID_SERIAL_SHORT` check above.

---

### 2.3 Reload udev and verify symlinks

Unplug and replug both USB devices after reloading.

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# Unplug both, replug both, then verify:
ls -la /dev/rplidar /dev/esp32
# Both should appear as symlinks → /dev/ttyUSBx
```

> Update `bringup.launch.py`'s `serial_port` to `/dev/rplidar` now that the symlink is stable.

---

## 3. PID Tuning & Physical Calibration

> **Before starting:** launch bringup first (`ros2 launch real_bringup bringup.launch.py`), then the joystick or step_tester in a separate terminal. PlotJuggler subscribes to `/pid_debug` to visualise target vs actual speed in real time.

---

### Step 1 — PID Tuning (KP first, then KI, then KD)

Use `step_tester.py` in linear mode + PlotJuggler `/pid_debug`.

```bash
# Terminal 1 — bringup
ros2 launch real_bringup bringup.launch.py

# Terminal 2 — step tester (3 s ON / 3 s OFF cycle)
python3 step_tester.py linear

# Terminal 3 — live publish gains without reflashing
ros2 topic pub /pid_gains std_msgs/msg/String \
  "data: 'kp_l:80.0 ki_l:5.0 kd_l:1.0 kp_r:80.0 ki_r:5.0 kd_r:1.0'"
```

**Tuning order:**
1. Set KI=0 KD=0. Raise KP until actual speed tracks target without oscillation in PlotJuggler.
2. Slowly raise KI (try 1 → 2 → 5) until the steady-state offset closes. Halve if oscillation appears.
3. Add KD (try 0.5 → 1 → 2) only if velocity still oscillates with good KP + KI. Usually not needed.

> When satisfied, write the final KP/KI/KD values back into `config.h` and reflash the ESP32 — the `/pid_gains` topic is runtime-only and resets on reboot.

---

### Step 2 — Verify wheel radius (tape measure)

Confirm `WHEEL_RADIUS = 0.065 m` in `config.h` matches the physical wheel.

1. Mark a straight line on the floor. Place the robot with the wheel edge on the line.
2. Command a known distance — e.g. run `step_tester.py linear` for exactly 3 s at 0.3 m/s → expected travel = 0.9 m.
3. Measure the actual distance traveled with a tape measure.
4. If actual ≠ expected: `new_radius = (actual / expected) × 0.065`. Update `WHEEL_RADIUS` in `config.h` and reflash.

```bash
# Check /odom reports expected distance too
ros2 topic echo /odom --once
# pose.pose.position.x should match tape measure
```

---

### Step 3 — Verify wheel base (360° spin test)

Confirm `WHEEL_BASE = 0.642 m` matches physical axle separation.

1. Put a marker (tape) under the robot centre. Run `step_tester.py angular` until the robot completes exactly one full 360° spin.
2. If the robot overshoots: increase `WHEEL_BASE`. If it undershoots: decrease it.
3. Formula: `new_base = (actual_degrees / 360) × 0.642`
4. Update `config.h` and also `ekf.yaml` if derived from the URDF joints (URDF has 0.642 m: left_drive y=0.321 + right_drive y=0.321).

```bash
# Monitor yaw angle from odometry during spin
ros2 topic echo /odom | grep -A2 "orientation"
```

---

### Step 4 — Minimum reliable speed (`MIN_LINEAR`)

Find the slowest speed at which both wheels move without stalling.

1. Command progressively slower speeds via `/pid_gains` and `step_tester.py`.
2. In PlotJuggler watch `actual_v_left` and `actual_v_right` — the minimum speed where both wheels consistently move is your `MIN_LINEAR`.
3. Update `MIN_LINEAR` in `joystick_ws_node.py` and `MIN_WHEEL_SPEED` in `config.h` to match.
4. Also set `min_vel_x` in `nav2_params.yaml` to the same number.

Current default: `MIN_LINEAR = 0.10 m/s`

---

### Step 5 — Acceleration / deceleration profile

Tune `MAX_ACCEL_PER_TICK` so the robot starts smoothly without jerking.

1. Run `step_tester.py linear` and watch the *slewed target* line (`debug_data[0]`) in PlotJuggler — it should ramp smoothly.
2. If the gearbox jolts on start: lower `MAX_ACCEL_PER_TICK` from `0.05` to `0.03` in `config.h`.
3. If the robot reaches cruising speed too slowly: raise to `0.1` or `0.2`.

Current default: `0.05 m/s per 50 ms tick = 1.0 m/s² acceleration`

```
# PlotJuggler topics to watch:
# /pid_debug → [0] slewed_left, [1] actual_left, [3] pwm_left
#               [4] slewed_right, [5] actual_right, [7] pwm_right
```

---

### Step 6 — Verify robot footprint dimensions

Tape measure the physical robot for the Nav2 footprint polygon (outermost points including caster wheels).

```yaml
# In nav2_params.yaml, update the footprint polygon:
footprint: [[-0.15, 0.32], [0.50, 0.32], [0.50, -0.32], [-0.15, -0.32]]
# Adjust X,Y values to match tape measure results (in metres)
# X = front/back from robot centre, Y = left/right from centre
```

>  From the URDF: chassis extends roughly +0.35 m to the front, −0.15 m to the rear, ±0.32 m sideways — verify against the real robot.

---

## 4. SLAM Toolbox — Building the Map

>  **TwistStamped vs Twist:** During SLAM, the joystick node publishes `TwistStamped` on `/cmd_vel_stamped` directly to the ESP32. The `twist_to_twiststamped.py` bridge converts `/cmd_vel → /cmd_vel_stamped` if using tools that publish plain Twist (like `teleop_twist_keyboard`). Make sure the bridge is running during SLAM manual driving.

### 4.1 Start bringup

```bash
# Terminal 1 (always start bringup first)
ros2 launch real_bringup bringup.launch.py

# Wait ~3 seconds, then verify TF tree:
ros2 run tf2_tools view_frames
# Should show: odom → CHASSIS → LIDAR and imu_link
# If odom→CHASSIS is missing, EKF hasn't started yet — wait and retry
```

> Uncomment the `micro_ros_agent` block in `bringup.launch.py` before running on the real robot

---

### 4.2 Launch SLAM toolbox

```bash
# Terminal 2
ros2 launch real_bringup slam.launch.py
# slam_params.yaml must set odom_frame: odom and base_frame: CHASSIS
# use_sim_time: false
```

---

### 4.3 Drive the robot to build the map

```bash
#  WebSocket joystick (Terminal 3 on Pi)
ros2 launch real_bringup joystick.launch.py
# Open the joystick HTML page in browser → connect to ws://<PI_IP>:8765
```

Drive slowly and cover all areas. Watch RViz to confirm the map is building cleanly without jumps.

---

### 4.4 View the map in RViz (Windows laptop)

Both machines must be on the same network and share the same `ROS_DOMAIN_ID`.

```powershell
# On Windows (ROS 2 must be installed) — PowerShell
set ROS_DOMAIN_ID=0
rviz2
# Add displays: Map (/map), LaserScan (/scan), TF, RobotModel
```

> Alternatively, run RViz on the Pi over SSH with X forwarding: `ssh -X hania@<PI_IP>` then `rviz2` — but this is slow over Wi-Fi.

---

### 4.5 Save the map

```bash
# While SLAM is still running — save the map
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot_ws/src/real_robot/real_bringup/maps/real_map
# Creates real_map.pgm (image) and real_map.yaml (metadata)
# navigation.launch.py already points to real_map.yaml by default
```

---

## 5. Nav2 — Autonomous Navigation

>  **Twist vs TwistStamped for Nav2:** Nav2's `controller_server` publishes plain `Twist` on `/cmd_vel` by default. The ESP32 subscribes to `TwistStamped` on `/cmd_vel_stamped`. we must either:
> - **(a)** Run the `twist_to_twiststamped.py` bridge alongside Nav2, or
> - **(b)** Configure Nav2 to publish TwistStamped directly: set `enable_stamped_cmd_vel: true` in `nav2_params.yaml` (available in Nav2 Jazzy/Iron+).
>
> Option (b) is cleaner — no extra node needed.

### 5.1 Configure nav2_params.yaml

```yaml
# Under controller_server:
enable_stamped_cmd_vel: true   # publishes TwistStamped on /cmd_vel_stamped

# Under local_costmap and global_costmap:
robot_base_frame: CHASSIS       # must match EKF base_link_frame

# Under DWB or MPPI controller:
min_vel_x: 0.10                 # match MIN_LINEAR from step 3.4
max_vel_x: 0.70                 # match MAX_LINEAR (70% of rated 1.2 m/s)
max_vel_theta: 1.5              # match MAX_ANGULAR

# Footprint (from step 3.6 tape measure)
footprint: "[[-0.15, 0.32], [0.50, 0.32], [0.50, -0.32], [-0.15, -0.32]]"
```

---

### 5.2 Configure amcl_params.yaml

```yaml
base_frame_id: CHASSIS
odom_frame_id: odom
scan_topic:    /scan
laser_model_type: likelihood_field
use_sim_time: false

# Particle filter sizing (start broad, tighten once map is good)
min_particles: 500
max_particles: 2000
```

---

### 5.3 Launch bringup + navigation

```bash
# Terminal 1
ros2 launch real_bringup bringup.launch.py

# Terminal 2 — waits 5 s automatically via TimerAction
ros2 launch real_bringup navigation.launch.py

# Optionally pass a different map
ros2 launch real_bringup navigation.launch.py \
  map:=/path/to/another_map.yaml
```

> The lifecycle manager is delayed 5 s in `navigation.launch.py` to wait for bringup's TF tree to be available. If Nav2 reports "map→odom TF not available", bringup hasn't finished — increase the delay or wait before launching navigation.

---

### 5.4 Set initial pose in RViz and test navigation

1. In RViz, click **2D Pose Estimate** and click on the map at the robot's approximate location, dragging to set orientation.
2. Watch the particle cloud (AMCL) in RViz converge around the robot as it moves.
3. Click **Nav2 Goal** (or **2D Nav Goal**) and click a destination on the map.
4. Monitor `/cmd_vel_stamped` to confirm Nav2 is sending commands to the ESP32.

```bash
# Confirm Nav2 is sending velocity commands
ros2 topic echo /cmd_vel_stamped

# Check AMCL particle convergence
ros2 topic echo /amcl_pose
```

---

### 5.5 Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot doesn't move at all | micro_ros_agent not running or ESP32 not connected | Uncomment `micro_ros_agent` in `bringup.launch.py`, check `/dev/esp32` symlink |
| TF tree missing odom→CHASSIS | EKF not publishing — no `/odom` or `/imu/data` from ESP32 | Check `ros2 topic hz /odom` and `ros2 topic hz /imu/data` |
| SLAM map has jumps/tears | Driving too fast, or odometry error accumulation | Drive slower, recheck wheel radius and wheel base calibration |
| Nav2 not sending /cmd_vel_stamped | `enable_stamped_cmd_vel` not set; bridge not running | Add `enable_stamped_cmd_vel: true` to `nav2_params.yaml` or run `twist_to_twiststamped.py` bridge |
| AMCL particles scattered all over | Bad initial pose estimate or map doesn't match environment | Re-set 2D Pose Estimate in RViz; remap if environment changed significantly |
| Robot hits obstacles in Nav2 | Footprint too small in `nav2_params.yaml` | Update footprint polygon from tape measure (step 3.6) with extra margin |

---

## Quick Reference

| Item | Value |
|---|---|
| Workspace | `~/robot_ws` |
| Package | `real_bringup` |
| Map save path | `real_bringup/maps/real_map` |
| Lidar device | `/dev/rplidar` |
| ESP32 device | `/dev/esp32` |
| ROS 2 distro | Jazzy |
| URDF base frame | `CHASSIS` |
| Odom frame | `odom` |
| Wheel radius | `0.065 m` |
| Wheel base | `0.642 m` |
| Min linear speed | `0.10 m/s` |

---

**Repo:** [github.com/hania222/real_robot](https://github.com/hania222/real_robot) · Workspace: `~/robot_ws` · ROS 2 Jazzy · micro-ROS on ESP32

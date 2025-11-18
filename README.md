## Teleoperation between Meta Quest 3 ↔ Kinova Gen3

This section describes the end-to-end teleoperation pipeline from the **Kinova Vision camera** and Meta Quest 3 (TeleVuer) to the Kinova Gen3 arm.

https://github.com/user-attachments/assets/39013544-9750-41d9-a20a-ba50cde4bcb7

---

## Overview of the runtime pipeline

We use multiple terminals / processes:

- Terminal 1 – ROS 2 + Kinova Vision
- Terminal 2 – Camera multiplexer → shared memory (`camera_mux_to_shm.py`)
- Terminal 3 – TeleVuer producer (`test2.py`, shared memory + VR)
- Browser + Quest 3 – TeleVuer Web UI
- Terminal 4 – Kinova teleoperation controller (`twist2.py`)

---

## 1. Terminal 1 – ROS 2 / Kinova Vision

Run the Kinova Vision ROS 2 driver:

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

ros2 launch kinova_vision kinova_vision.launch.py
```

This publishes the Kinova Vision camera streams into topics such as:

```
/camera/color/image_raw
/camera/depth/image_raw
```

---

## 2. Terminal 2 – Camera multiplexer → Shared Memory

```bash
cd iCAT_teleop/test
python3 camera_mux_to_shm.py
```

This script:

- Subscribes to Kinova Vision RGB and/or Realsense RGB
- Packs the selected feed into the shared-memory buffer `televuer_img`
- Left controller X button toggles:
  - Realsense RGB ↔ Kinova Vision RGB

---

## 3. Terminal 3 – TeleVuer Producer (VR → Shared Memory)

```bash
cd iCAT_teleop/test
python3 test2.py
```

The script initializes shared memory and waits:

```
Press Enter to start TeleVuer producer...
```

Do **NOT** press Enter yet.

---

## 4. TeleVuer Web UI (Browser + Quest 3)

Open the TeleVuer web UI:

```
http://<PC_IP>:<PORT>/
```

Then:

- Wear Quest 3
- Join TeleVuer room
- Ensure VR pose & button data are arriving

---

## Back to Terminal 3 – Start Producer Loop

Go back to the `test2.py` terminal and press Enter.

`test2.py` now:

- Reads right controller pose
- Computes linear/angular velocities
- Writes commands + buttons + gripper fraction → shared memory `cmd`

---

## 5. Terminal 4 – Kinova Teleoperation Controller

```bash
cd iCAT_teleop/test
python3 twist2.py
```

This script:

- Reads from shared memory (`cmd`)
- Sends TOOL-frame Twist commands to Gen3
- Sends gripper fraction command (0–1)
- Logs TOOL0 desired/actual pose

---

## Teleoperation Logic Summary

### Linear Motion (m/s)

- Derived from VR controller absolute motion  
- Filtered using admittance filter  
- Clamped for safety  

### Angular Motion (deg/s)

VR angular velocity is ignored.  
Orientation is controlled by buttons + thumbstick:

- A > 0.5 → pitch +5 deg/s  
- B > 0.5 → pitch -5 deg/s  
- Thumbstick X ≥ +0.5 → yaw +5 deg/s  
- Thumbstick X ≤ -0.5 → yaw -5 deg/s  
- Thumbstick Y ≥ +0.5 → roll +5 deg/s  
- Thumbstick Y ≤ -0.5 → roll -5 deg/s  

These are:

- Filtered  
- Clamped  
- Sent to robot as **deg/s**  
- Integrated as **rad/s** internally for pose logging  

---

## Gripper Control

`cmd[6]` ∈ **0.0 (open) → 1.0 (closed)**

When the value changes:

- `SendGripperCommand` (GRIPPER_POSITION mode)
- `finger.value = fraction (0~1)`

---

## Camera Switching (Left X Button)

Handled inside `camera_mux_to_shm.py`:

- Toggles `televuer_img` source:
  - Realsense RGB
  - Kinova RGB
- VR view updates accordingly.

---

## Recommended Terminal Layout

| Terminal | Command                           | Purpose                        |
|---------|-----------------------------------|--------------------------------|
| 1       | `ros2 launch kinova_vision ...`    | Publish Kinova Vision camera   |
| 2       | `camera_mux_to_shm.py`             | Camera → shared memory         |
| 3       | `test2.py`                         | VR → shared memory producer    |
| Browser | TeleVuer UI                        | VR session                     |
| 4       | `twist2.py`                        | Kinova teleoperation           |


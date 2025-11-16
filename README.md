## Teleoperation between Meta Quest 3 ↔ Kinova Gen3

This section describes the end-to-end teleoperation pipeline from the **Kinova Vision camera** and **Meta Quest 3 (TeleVuer)** to the **Kinova Gen3** arm.

---

## Overview of the runtime pipeline

We use multiple terminals / processes:

- **Terminal 1** – ROS 2 + Kinova Vision
- **Terminal 2** – Camera multiplexer → shared memory (`camera_mux_to_shm.py`)
- **Terminal 3** – TeleVuer producer (`test2.py`, shared memory + VR)
- **Browser + Quest 3** – TeleVuer Web UI
- **Terminal 4** – Kinova teleoperation controller (`twist2.py`)

---

## 1. Terminal 1 – ROS 2 / Kinova Vision

Run the Kinova Vision ROS 2 driver:

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

ros2 launch kinova_vision kinova_vision.launch.py
This publishes the Kinova camera streams, for example:

/camera/color/image_raw

/camera/depth/image_raw

2. Terminal 2 – Camera multiplexer → Shared Memory
Bridge the camera stream(s) into the shared memory buffer used by TeleVuer:

bash
코드 복사
cd iCAT_teleop/test
python3 camera_mux_to_shm.py
Typical behavior (depending on the script implementation):

Subscribes to the Kinova Vision ROS 2 topics (and optionally Realsense RGB)

Packs the selected camera view into the stereo buffer televuer_img
(shape: (480, 1280, 3), RGB)

The left controller X button is used to toggle between:

Realsense RGB view

Kinova Vision RGB view

So pressing X on the left controller switches the VR camera view.

3. Terminal 3 – TeleVuer producer (shared memory + VR)
Start the TeleVuer shared–memory producer and wait at the prompt:

bash
코드 복사
cd iCAT_teleop/test
python3 test2.py
The script will:

Create the stereo image shared memory televuer_img with shape (480, 1280, 3)

Create the command shared memory cmd (for velocities, buttons, gripper, etc.)

Initialize TeleVuerWrapper and block with:

text
코드 복사
Press Enter to start TeleVuer producer...
👉 Do not press Enter yet.
First, connect TeleVuer from the browser + Quest 3.

4. TeleVuer Web UI (Browser + Quest 3)
On the host PC, open a browser and connect to the TeleVuer web interface:

text
코드 복사
http://<THIS_PC_IP>:<PORT>/
Use the same IP/port configured in your TeleVuer setup.

Then:

Put on the Meta Quest 3.

Connect to the same TeleVuer room / session.

Confirm that:

The stereo video is visible in VR (coming from televuer_img).

Controller pose and button data are reaching the PC (no errors in test2.py).

5. Back to Terminal 3 – start the producer loop
Once the TeleVuer Web UI + Quest 3 are connected and you see motion data arriving:

Go back to Terminal 3 (where test2.py is running).

Press Enter at:

text
코드 복사
Press Enter to start TeleVuer producer...
From this point, test2.py will continuously:

Read the right–hand pose from TeleVuer.

Compute linear / angular velocities.

Write:

Linear velocity,

Angular velocity (if used),

Button and trigger states
into the cmd shared memory.

6. Terminal 4 – Kinova teleoperation controller (twist2.py)
Finally, run the teleoperation controller that reads from shared memory and sends Twist + gripper commands to the Kinova Gen3:

bash
코드 복사
cd iCAT_teleop/test
python3 twist2.py
6.1 Linear motion
Uses VR–derived linear velocities from cmd[0:3].

Passes them through an admittance–like filter and clamps to a safe maximum speed.

Sends the result as TwistCommand.twist.linear_* in the TOOL reference frame.

6.2 Angular motion (deg/s, button + stick based)
Angular velocity from TeleVuer itself is ignored.
Instead, twist2.py generates angular velocity (in degrees per second) using buttons and thumbstick:

cmd[7] (right A button) > 0.5 → pitch +5 deg/s

cmd[8] (right B button) > 0.5 → pitch -5 deg/s

cmd[9] (right thumbstick x) ≥ +0.5 → yaw +5 deg/s

cmd[9] ≤ -0.5 → yaw -5 deg/s

cmd[10] (right thumbstick y) ≥ +0.5 → roll +5 deg/s

cmd[10] ≤ -0.5 → roll -5 deg/s

These angular velocities are:

Filtered in deg/s and clamped to a safe limit.

Integrated internally in radians for desired pose estimation and logging.

Sent to the robot as angular velocity (Twist) according to the Kortex API convention (degrees per second).

6.3 Gripper control
cmd[6] is treated as a gripper position fraction:

0.0 → fully open

1.0 → fully closed

When the value changes by more than a small epsilon, twist2.py calls:

SendGripperCommand() in GRIPPER_POSITION mode

Using the fraction in [0.0, 1.0] as the target gripper position.

7. Typical terminal layout (summary)
Terminal 1 – ros2 launch kinova_vision kinova_vision.launch.py

Terminal 2 – python3 camera_mux_to_shm.py (camera → shared memory, left X to toggle)

Terminal 3 – python3 test2.py (TeleVuer producer, shared memory)

Browser + Quest 3 – TeleVuer Web UI session

Terminal 4 – python3 twist2.py (Kinova teleoperation controller)

Once all four terminals are running and the Quest 3 is connected, you should be able to:

Move the right controller to command end-effector translation.

Use right A/B buttons + right thumbstick for orientation control (roll / pitch / yaw).

Use the trigger / gripper fraction for opening/closing the gripper.

Use left X to switch between camera views in VR (Realsense ↔ Kinova Vision).

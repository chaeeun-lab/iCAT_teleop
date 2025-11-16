## How to Run the Full Teleoperation Pipeline

This section describes the end–to–end flow from the Kinova Vision camera and Meta Quest 3 (TeleVuer) to the Kinova Gen3 arm.

### Overview of the runtime pipeline

Terminals / processes:

1. Terminal 1 (ROS 2)*
   Run the Kinova Vision ROS2 driver:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/colcon_ws/install/setup.bash
   ros2 launch kinova_vision kinova_vision.launch.py
   This publishes the Kinova camera streams (e.g. /camera/color/image_raw, /camera/depth/image_raw).

2. Bridge the camera stream(s) into the shared memory buffer used by TeleVuer:
   ```bash
   cd iCAT_teleop/test
   python3 camera_mux_to_shm.py

   Typical behavior (depending on your script implementation):
   Subscribes to the Kinova Vision ROS2 topics (and optionally Realsense RGB)
   Packs the selected camera view into the stereo buffer televuer_img
   Left controller X button is used to toggle between:
   Realsense RGB view<---->Kinova Vision RGB view

3. Terminal 2 (TeleVuer producer, shared memory + VR)
   Start the TeleVuer shared–memory producer and wait at the prompt:
   ```bash
   cd iCAT_teleop/test
   python3 test2.py

   The script will:
   Create the stereo image shared memory televuer_img with shape (480, 1280, 3)
   Create the command shared memory cmd (for velocities, buttons, gripper, etc.)
   Initialize TeleVuerWrapper and block on:
   Press Enter to start TeleVuer producer...
   Do not press Enter yet. We will start the camera multiplexer first.
   Terminal 3 (Camera multiplexer → shared memory)

4. TeleVuer Web UI (Browser + Quest 3)

   On the host PC, open a browser and connect to the TeleVuer web interface:

   http://<THIS_PC_IP>:<PORT>/ (Use the same IP/port configured in your TeleVuer setup.)

   Put on the Meta Quest 3, connect to that TeleVuer room / session, and confirm that:
   Controller pose and button data are reaching the PC (no errors in test2.py).

5. Back to Terminal 3 – start the producer loop
   nce the TeleVuer web UI + Quest are connected and you see motion data arriving, go back to Terminal 2 and press Enter in test2.py:

   Press Enter to start TeleVuer producer...
   Now test2.py will continuously:
   Read right–hand pose from TeleVuer
   Compute linear / angular velocities
   Write them and the button/trigger states into the cmd shared memory

6. Terminal 4 (Kinova teleoperation controller – twist2.py)

   Finally, run the teleoperation controller that reads from shared memory and sends Twist + gripper commands to the Kinova Gen3:
    ```bash
     cd iCAT_teleop/test 
     python3 twist2.py


At runtime:

Linear motion
Uses VR–derived linear velocities from cmd[0:3]
Passes them through an admittance–like filter and clamps to a safe max speed
Sends them as TwistCommand.twist.linear_* in the TOOL frame
Angular motion (deg/s, button + stick based)
VR angular velocity from TeleVuer is ignored. Instead, the script generates angular velocity in degrees per second using buttons and thumbstick:

cmd[7] (right A button) > 0.5 → pitch +5 deg/s
cmd[8] (right B button) > 0.5 → pitch -5 deg/s
cmd[9] (right thumbstick x) ≥ +0.5 → yaw +5 deg/s
cmd[9] ≤ -0.5 → yaw -5 deg/s
cmd[10] (right thumbstick y) ≥ +0.5 → roll +5 deg/s
cmd[10] ≤ -0.5 → roll -5 deg/s

These are filtered (deg/s) and clamped, then: Integrated in radians internally for the desired pose logging

Sent to the robot as angular velocity (Twist) in degrees per second, per Kortex API’s convention

Gripper control

cmd[6] is treated as a gripper position fraction
0.0 → fully open 1.0 → fully closed
When the value changes by more than a small epsilon, twist2.py calls:
SendGripperCommand() in GRIPPER_POSITION mode with that fraction


Left controller X button (camera switching)

Handled inside camera_mux_to_shm.py
Toggles which camera feed (Realsense vs Kinova Vision) is written into televuer_img
The VR view switches accordingly inside TeleVuer.


Typical terminal layout

Terminal 1 – ROS 2 + Kinova Vision
Terminal 2 – test2.py (TeleVuer producer, shared memory)
Terminal 3 – camera_mux_to_shm.py (camera → shared memory, X button toggle)
Browser + Quest 3 – TeleVuer web UI
Terminal 4 – twist2.py (Kinova teleoperation controller)

Once all four terminals are running and the Quest 3 is connected, you should be able to:
Move the right controller to command the end effector translation
Use A/B + thumbstick for orientation control (roll/pitch/yaw)
Use trigger / gripper fraction for closing/opening the gripper
Use left X to switch between camera views in VR.

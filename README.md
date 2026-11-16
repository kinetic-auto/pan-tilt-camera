# pan_tilt_camera

ros2_control bringup for a Dynamixel 2XL430-W250-T pan/tilt camera (ROS 2 Humble, Ubuntu 22.04 / WSL).

```
pan_tilt_camera/
├── urdf/pan_tilt_limits.xacro        <- EDIT JOINT LIMITS HERE (degrees)
├── urdf/pan_tilt.urdf.xacro          links, joints, camera frames
├── urdf/pan_tilt.ros2_control.xacro  Dynamixel hardware config (IDs, modes, servo limits)
├── config/controllers.yaml           joint_state_broadcaster + joint_trajectory_controller
├── launch/pan_tilt.launch.py         starts everything (+ RViz)
└── scripts/move_camera.py            send a pan/tilt move and wait for it to finish
```

## Build

```bash
cd ~/ros2_ws/src            # next to DynamixelSDK, dynamixel_hardware_interface, dynamixel_interfaces
# copy/unzip pan_tilt_camera here
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# 1. No servo: simulated joints, checks URDF / controllers / launch
ros2 launch pan_tilt_camera pan_tilt.launch.py use_mock_hardware:=true

# 2. Real servo (U2D2 attached as /dev/ttyUSB0, 12 V on)
ros2 launch pan_tilt_camera pan_tilt.launch.py
```

In RViz: Fixed Frame = `base_link`, Add > RobotModel (Description Topic `/pan_tilt_camera/robot_description`), Add > TF.
File > Save Config As... to keep the layout.

## Namespace

Everything runs under `/pan_tilt_camera` so it can sit next to another robot without topic
collisions: `/pan_tilt_camera/joint_states`, `/pan_tilt_camera/robot_description`,
`/pan_tilt_camera/controller_manager`, `/pan_tilt_camera/pan_tilt_controller/...`,
`/pan_tilt_camera/dynamixel_hardware_interface/...`. Change it with `namespace:=<ns>`
(then also update the Description Topic in `rviz/pan_tilt.rviz`, and run the scripts with
`--ros-args -r __ns:=/<ns>`). TF (`/tf`, `/tf_static`) is global by ROS convention.

## Move it

```bash
ros2 run pan_tilt_camera move_camera.py 45 20     # pan 45 deg, tilt 20 deg
ros2 run pan_tilt_camera move_camera.py           # demo sequence
```

Or from the command line (radians, both arrive at time_from_start):

```bash
ros2 action send_goal /pan_tilt_camera/pan_tilt_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: [pan_joint, tilt_joint], points: [{positions: [0.8, 0.3], time_from_start: {sec: 2}}]}}"
```

## Check

```bash
ros2 control list_hardware_interfaces -c /pan_tilt_camera/controller_manager
ros2 control list_controllers -c /pan_tilt_camera/controller_manager   # both should be "active"
ros2 topic echo /pan_tilt_camera/joint_states
```

## Joint limits

Edit only `urdf/pan_tilt_limits.xacro`. The same numbers become:

| Layer | What enforces it |
|---|---|
| URDF `<limit>` (radians) | RViz, MoveIt, `move_camera.py` clamps targets |
| Servo Min/Max Position Limit (ticks, written at startup) | The servo itself refuses to go past it, even if ROS sends a bad command |

The ROS 2 Humble trajectory controller does **not** check position limits on its own, so the
servo limit is the real safety stop. A goal outside the limits stops at the limit and the
action reports a goal-tolerance failure.

Before widening limits: move each axis slowly with the mock off and confirm the positive
direction matches the URDF axis. To flip an axis, set its `Drive Mode` to `5` (reverse + time-based).

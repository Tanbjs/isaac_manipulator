# isaac_manipulator

Conveyor sorting with a Franka Panda in Isaac Sim 6.1: YCB objects ride a conveyor belt under an overhead RGB-D
camera; a ROS 2 (Humble) pipeline detects them (OpenCV / YOLO), predicts where they will be, and has the robot pick
them off the moving belt into one bin per class.

## Run the sim

```bash
(cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build)   # once: franka_description
tools/sim.sh                        # GUI
tools/sim.sh --view headless --seed 1 --speed 0.1
```

Flags: `--view gui|stream|headless`, `--seed N`, `--speed m/s`, `--no-spawn`, `--no-ros`, `--steps N`.
The cell layout, object classes and bins are in [isaac_sim/config/cell.yaml](isaac_sim/config/cell.yaml).

With ROS, `sim.sh` also starts (and stops with the sim):

- `ros2 launch franka_description robot_state_publisher.launch.py`: the panda links on `/tf` from
  `/sim/joint_states` and the static `world -> panda_link0` (z = 0.75 m), URDF on `/robot_description`.
  [franka_description](ros2_ws/src/franka_description) is franka_ros's URDF and meshes (Apache-2.0) as bundled with
  Isaac Sim's URDF importer.
- `foxglove_bridge` (ros-humble-foxglove-bridge) on `ws://localhost:8765`, and Foxglove connected to it.
  `--no-foxglove-app` starts only the bridge, `--no-foxglove` neither (`FOXGLOVE_PORT` changes the port).

ROS nodes in other terminals need the same middleware as the sim; [tools/env.sh](tools/env.sh) sources Humble and
ros2_ws and sets it (`ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, kept if already set). `sim.sh`
uses the same file.

```bash
source tools/env.sh
```

## Scene

| | |
|---|---|
| Robot | Franka Panda on a 0.75 m pedestal, base frame = world origin |
| Conveyor | along +y at x = 0.55 m, surface at z = 0.75 m, objects enter at y = -1.3 m |
| Camera | 640x480 RGB-D at 15 Hz, 0.65 m above the belt at y = -0.6 m, looking down (image right = flow) |
| Objects | `tomato_soup_can`, `gelatin_box`, `foam_brick`, `potted_meat_can` (YCB), all graspable from the top |
| Bins | red / blue / green / yellow, one per class |

Objects that pass the belt end or fall are counted as missed; objects resting in a bin are scored correct or wrong
and recycled.

## ROS 2 interface

| Topic | Type | |
|---|---|---|
| `/sensor/camera/rgb` | sensor_msgs/Image (rgb8) | frame `camera_optical_frame` |
| `/sensor/camera/depth` | sensor_msgs/Image (32FC1, m) | |
| `/sensor/camera/camera_info` | sensor_msgs/CameraInfo | |
| `/sim/joint_states` | sensor_msgs/JointState | 7 arm joints + 2 fingers |
| `/tf` | | world -> camera_link, camera_optical_frame (sim); panda links (robot_state_publisher) |
| `/clock` | | sim time (use `use_sim_time:=true`) |
| `/control/joint_command` | sensor_msgs/JointState | position targets by name (any subset) |
| `/control/conveyor_speed` | std_msgs/Float32 | m/s |
| `/control/spawner_enabled` | std_msgs/Bool | |
| `/sim/ground_truth/objects` | vision_msgs/Detection3DArray | world poses + class, for evaluation only |
| `/sim/ground_truth/conveyor_speed` | std_msgs/Float32 | |

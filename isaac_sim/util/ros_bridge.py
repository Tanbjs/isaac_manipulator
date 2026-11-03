"""ROS 2 bridge for the conveyor sorting cell.

Topics are grouped by namespace: /sim (plant state and ground truth), /sensor (camera), /control (inputs);
/clock and /tf stay global.

OmniGraph publishes the heavy data:
    /clock, /sim/joint_states (Franka, incl. fingers),
    /tf (world -> camera_link, world -> camera_optical_frame; the panda links come from robot_state_publisher),
    /sensor/camera/rgb, /sensor/camera/depth (32FC1, m), /sensor/camera/camera_info
rclpy (Isaac's bundled Humble rclpy, node /sim/isaac_conveyor_cell) handles the inputs and the ground truth:
    sub /control/joint_command   (sensor_msgs/JointState: name + position targets, any subset of the 9 DOFs)
    sub /control/conveyor_speed  (std_msgs/Float32, m/s along +y)
    sub /control/spawner_enabled (std_msgs/Bool)
    pub /sim/ground_truth/objects (vision_msgs/Detection3DArray in world: class_id = YOLO label, pose, size;
                                   for evaluation and debugging, not for the pipeline to use)
    pub /sim/ground_truth/conveyor_speed (std_msgs/Float32)

Must be imported after `SimulationApp` has been created.
"""

from __future__ import annotations

import isaacsim.core.experimental.utils.app as app_utils
import omni.graph.core as og
import usdrt

app_utils.enable_extension("isaacsim.ros2.bridge")

import rclpy  # noqa: E402  (available only once the bridge extension is enabled)
from builtin_interfaces.msg import Time  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool, Float32  # noqa: E402
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose  # noqa: E402

from util.scene.cell import CONFIG, ROBOT_PATH  # noqa: E402
from util.sensor.camera import CAMERA_RESOLUTION, Camera  # noqa: E402

SIM_NS = "sim"
SENSOR_NS = "sensor"
GRAPH_PATH = "/ROS2Graph"
CAMERA_GRAPH_PATH = "/ROS2CameraGraph"
WORLD_FRAME = "world"
CAMERA_OPTICAL_FRAME = "camera_optical_frame"


def _stamp(sim_time: float) -> Time:
    sec = int(sim_time)
    return Time(sec=sec, nanosec=int((sim_time - sec) * 1e9))


class RosBridge:
    def __init__(self, camera: Camera, app_rate_hz: float):
        self._build_graph(camera)
        self._build_camera_graph(camera, frame_skip=max(0, round(app_rate_hz / CONFIG["camera"]["rate_hz"]) - 1))

        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("isaac_conveyor_cell", namespace=SIM_NS)
        self._cmd: JointState | None = None
        self._speed: float | None = None
        self._spawner_enabled: bool | None = None
        self._node.create_subscription(JointState, "/control/joint_command", self._on_joint_command, 10)
        self._node.create_subscription(Float32, "/control/conveyor_speed", self._on_speed, 10)
        self._node.create_subscription(Bool, "/control/spawner_enabled", self._on_spawner_enabled, 10)
        self._objects_pub = self._node.create_publisher(Detection3DArray, "ground_truth/objects", 10)
        self._speed_pub = self._node.create_publisher(Float32, "ground_truth/conveyor_speed", 10)

    def _build_graph(self, camera: Camera) -> None:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ],
                keys.SET_VALUES: [
                    ("PublishClock.inputs:topicName", "clock"),
                    ("PublishJointState.inputs:nodeNamespace", SIM_NS),
                    ("PublishJointState.inputs:topicName", "joint_states"),
                    ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
                    ("PublishTF.inputs:topicName", "tf"),
                    # Only the camera: robot_state_publisher (ros2_ws franka_description) publishes the panda links
                    ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(p) for p in camera.frame_paths]),
                ],
            },
        )

    def _build_camera_graph(self, camera: Camera, frame_skip: int) -> None:
        keys = og.Controller.Keys
        height, width = CAMERA_RESOLUTION
        helpers = ("Rgb", "Depth", "Info")
        graph, _, _, _ = og.Controller.edit(
            {
                "graph_path": CAMERA_GRAPH_PATH,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("Rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    *[("RenderProduct.outputs:execOut", f"{n}.inputs:execIn") for n in helpers],
                    *[("RenderProduct.outputs:renderProductPath", f"{n}.inputs:renderProductPath") for n in helpers],
                ],
                keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera.prim)]),
                    ("RenderProduct.inputs:width", width),
                    ("RenderProduct.inputs:height", height),
                    ("Rgb.inputs:type", "rgb"),
                    ("Rgb.inputs:topicName", "camera/rgb"),
                    ("Depth.inputs:type", "depth"),
                    ("Depth.inputs:topicName", "camera/depth"),
                    ("Info.inputs:topicName", "camera/camera_info"),
                    *[(f"{n}.inputs:nodeNamespace", SENSOR_NS) for n in helpers],
                    *[(f"{n}.inputs:frameId", CAMERA_OPTICAL_FRAME) for n in helpers],
                    *[(f"{n}.inputs:frameSkipCount", frame_skip) for n in helpers],
                ],
            },
        )
        # Evaluate once to create the publishers inside the synthetic data pipeline
        og.Controller.evaluate_sync(graph)

    def _on_joint_command(self, msg: JointState) -> None:
        self._cmd = msg

    def _on_speed(self, msg: Float32) -> None:
        self._speed = msg.data

    def _on_spawner_enabled(self, msg: Bool) -> None:
        self._spawner_enabled = msg.data

    def take_inputs(self) -> tuple[JointState | None, float | None, bool | None]:
        """Latest (joint command, conveyor speed, spawner enabled) received since the last call (None if nothing
        new)."""
        rclpy.spin_once(self._node, timeout_sec=0.0)
        inputs = (self._cmd, self._speed, self._spawner_enabled)
        self._cmd = self._speed = self._spawner_enabled = None
        return inputs

    def warn(self, text: str) -> None:
        self._node.get_logger().warning(text)

    def publish_ground_truth(self, sim_time: float, objects, speed: float) -> None:
        """`objects`: (ObjectSlot, world position, world orientation wxyz), see `Spawner.active_objects`."""
        msg = Detection3DArray()
        msg.header.stamp = _stamp(sim_time)
        msg.header.frame_id = WORLD_FRAME
        for slot, position, orientation in objects:
            det = Detection3D()
            det.header = msg.header
            det.id = slot.path.rsplit("/", 1)[-1]
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = slot.label
            hyp.hypothesis.score = 1.0
            p, q = hyp.pose.pose.position, hyp.pose.pose.orientation
            p.x, p.y, p.z = (float(v) for v in position)
            q.w, q.x, q.y, q.z = (float(v) for v in orientation)
            det.results.append(hyp)
            det.bbox.center = hyp.pose.pose
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = (float(v) for v in slot.size)
            msg.detections.append(det)
        self._objects_pub.publish(msg)
        self._speed_pub.publish(Float32(data=float(speed)))

    def shutdown(self) -> None:
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

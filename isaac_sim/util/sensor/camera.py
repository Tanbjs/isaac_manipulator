"""Overhead RGB-D camera above the belt, upstream of the robot (parameters in the `camera` section of
config/cell.yaml).

Frames, all static in the world:
    camera_link           ROS body convention: x = viewing direction (down), z = image up
    camera_link/camera    the USD camera (looks along -Z, +Y = image up)
    camera_optical_frame  ROS optical convention: z = viewing direction, x = image right, y = image down
The image's right is the flow direction (+y world) and its down is +x world (away from the robot).
"""

from __future__ import annotations

from dataclasses import dataclass

import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.sensors.experimental.rtx import RtxCamera
from pxr import Gf

from util.scene.cell import CONFIG, quat_wxyz

CAMERA_LINK = "/World/camera_link"
OPTICAL_FRAME = f"{CAMERA_LINK}/camera_optical_frame"
CAMERA_RESOLUTION = tuple(CONFIG["camera"]["resolution"])  # (height, width)


@dataclass
class Camera:
    prim: str  # USD camera prim, for the render product
    frame_paths: list[str]  # frames to publish on /tf (child of world)


def _axes(x, y, z) -> list[float]:
    """Orientation (wxyz) of a frame whose x, y, z axes are the given vectors in the parent frame."""
    return quat_wxyz(Gf.Matrix4d(*x, 0, *y, 0, *z, 0, 0, 0, 0, 1).ExtractRotation())


def add_camera() -> Camera:
    """Create the camera prims (call after `build_world` and before playing the timeline)."""
    cfg = CONFIG["camera"]
    stage_utils.define_prim(CAMERA_LINK, "Xform")
    XformPrim(CAMERA_LINK, positions=[cfg["position"]], orientations=[_axes((0, 0, -1), (0, -1, 0), (-1, 0, 0))],
              reset_xform_op_properties=True)
    stage_utils.define_prim(OPTICAL_FRAME, "Xform")
    XformPrim(OPTICAL_FRAME, translations=[[0.0, 0.0, 0.0]], orientations=[_axes((0, -1, 0), (0, 0, -1), (1, 0, 0))],
              reset_xform_op_properties=True)

    camera = RtxCamera(
        f"{CAMERA_LINK}/camera",
        translations=[[0.0, 0.0, 0.0]],
        orientations=[_axes((0, -1, 0), (0, 0, 1), (-1, 0, 0))],
    )
    camera.prims[0].GetAttribute("focalLength").Set(cfg["focal_length"])
    camera.prims[0].GetAttribute("clippingRange").Set(Gf.Vec2f(*cfg["clipping_range"]))
    return Camera(prim=camera.paths[0], frame_paths=[CAMERA_LINK, OPTICAL_FRAME])

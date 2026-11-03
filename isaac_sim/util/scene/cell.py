"""Conveyor sorting cell: ground + light, Franka on a pedestal, conveyor belt with side rails, one open bin per
object class on a stand, and a pool of YCB objects parked off the belt (see lib/scene/spawner.py).

Layout and parameters live in `config/cell.yaml`. Must be imported after `SimulationApp` has been created.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import isaacsim.core.experimental.utils.stage as stage_utils
import yaml
from isaacsim.core.experimental.objects import Cube, DistantLight, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation, GeomPrim, XformPrim
from isaacsim.core.experimental.utils.semantics import add_labels
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

CONFIG = yaml.safe_load((Path(__file__).parents[2] / "config" / "cell.yaml").read_text())

YCB_DIR = "/Isaac/Props/YCB/Axis_Aligned"
ROBOT_PATH = "/World/franka"
CONVEYOR_PATH = "/World/conveyor"
BELT_PATH = f"{CONVEYOR_PATH}/belt"
BIN_ROOT = "/World/bins"
OBJECT_ROOT = "/World/objects"
# Pooled objects wait on the floor here, out of the camera's view and the robot's reach
PARKING_ORIGIN = (3.0, -1.5)
PARKING_SPACING = 0.35


@dataclass
class ObjectSlot:
    path: str
    label: str
    rest_orientation: list[float]  # wxyz
    rest_height: float  # origin height above the surface it rests on
    size: tuple[float, float, float]  # bbox in the object's own frame
    parking: tuple[float, float, float]


@dataclass
class World:
    robot: Articulation
    objects: list[ObjectSlot]
    bins: dict[str, dict]  # bin name -> {"center": (x, y), "size": (sx, sy), "floor": z}


def quat_wxyz(rotation: Gf.Rotation) -> list[float]:
    q = rotation.GetQuat()
    return [q.GetReal(), *q.GetImaginary()]


def rpy_rotation(roll: float, pitch: float, yaw: float) -> Gf.Rotation:
    """Fixed-axis roll about x, then pitch about y, then yaw about z (deg)."""
    rx = Gf.Rotation(Gf.Vec3d(1, 0, 0), roll)
    ry = Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch)
    rz = Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw)
    return rx * ry * rz  # Gf composes left to right: rx is applied first


def _box(path: str, center, size, color, *, collide: bool = True) -> None:
    Cube(path, sizes=1.0, scales=[size], colors=color, positions=[center])
    if collide:
        GeomPrim(path, apply_collision_apis=True)


def _spawn_robot(assets_root: str) -> Articulation:
    cfg = CONFIG["robot"]
    z = cfg["base_height"]
    sx, sy = cfg["pedestal_size"]
    _box("/World/pedestal", [0.0, 0.0, z / 2], [sx, sy, z], [0.25, 0.25, 0.28])
    stage_utils.add_reference_to_stage(assets_root + cfg["usd"], path=ROBOT_PATH)
    XformPrim(ROBOT_PATH, positions=[0.0, 0.0, z], reset_xform_op_properties=True)
    robot = Articulation(ROBOT_PATH)
    robot.set_default_state(dof_positions=[cfg["ready_pose"]])
    return robot


def _spawn_conveyor() -> None:
    """Static frame + rails and a kinematic belt whose surface velocity carries the objects along +y."""
    cfg = CONFIG["conveyor"]
    x, top, width, thickness = cfg["center_x"], cfg["top"], cfg["width"], cfg["thickness"]
    y0, y1 = cfg["start_y"] - 0.1, cfg["end_y"] + 0.1
    length, cy = y1 - y0, (y0 + y1) / 2
    stage_utils.define_prim(CONVEYOR_PATH, "Xform")

    _box(BELT_PATH, [x, cy, top - thickness / 2], [width, length, thickness], [0.12, 0.12, 0.13])
    stage = stage_utils.get_current_stage(backend="usd")
    belt = stage.GetPrimAtPath(BELT_PATH)
    UsdPhysics.RigidBodyAPI.Apply(belt).CreateKinematicEnabledAttr(True)
    surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(belt)
    surface.CreateSurfaceVelocityEnabledAttr(True)
    surface.CreateSurfaceVelocityLocalSpaceAttr(False)
    surface.CreateSurfaceVelocityAttr(Gf.Vec3f(0.0, cfg["speed"], 0.0))
    add_labels(BELT_PATH, labels=["conveyor"])

    rail_t, rail_h = 0.02, cfg["rail_height"]
    for side in (-1, 1):
        rx = x + side * (width / 2 + rail_t / 2)
        _box(f"{CONVEYOR_PATH}/rail_{'l' if side > 0 else 'r'}", [rx, cy, top - thickness + (thickness + rail_h) / 2],
             [rail_t, length, thickness + rail_h], [0.6, 0.6, 0.62])
    leg = top - thickness
    for i, ly in enumerate((y0 + 0.1, cy, y1 - 0.1)):
        for side in (-1, 1):
            _box(f"{CONVEYOR_PATH}/leg_{i}_{side + 1}", [x + side * (width / 2 - 0.03), ly, leg / 2],
                 [0.04, 0.04, leg], [0.6, 0.6, 0.62], collide=False)


def _spawn_bins() -> dict[str, dict]:
    cfg = CONFIG["bins"]
    sx, sy = cfg["size"]
    wall, stand = cfg["wall_height"], cfg["stand_height"]
    t = 0.01  # wall and floor thickness
    stage_utils.define_prim(BIN_ROOT, "Xform")
    bins = {}
    for name in sorted({obj["bin"] for obj in CONFIG["objects"].values()}):
        (x, y), color = cfg[name]["position"], cfg[name]["color"]
        path = f"{BIN_ROOT}/{name}"
        stage_utils.define_prim(path, "Xform")
        _box(f"{path}/stand", [x, y, (stand - t) / 2], [sx - 0.04, sy - 0.04, stand - t], [0.35, 0.35, 0.38])
        _box(f"{path}/floor", [x, y, stand - t / 2], [sx, sy, t], color)
        zc = stand + wall / 2
        _box(f"{path}/wall_px", [x + (sx - t) / 2, y, zc], [t, sy, wall], color)
        _box(f"{path}/wall_nx", [x - (sx - t) / 2, y, zc], [t, sy, wall], color)
        _box(f"{path}/wall_py", [x, y + (sy - t) / 2, zc], [sx - 2 * t, t, wall], color)
        _box(f"{path}/wall_ny", [x, y - (sy - t) / 2, zc], [sx - 2 * t, t, wall], color)
        add_labels(path, labels=["bin", f"bin_{name}"])
        bins[name] = {"center": (x, y), "size": (sx, sy), "floor": stand}
    return bins


def _spawn_objects(assets_root: str) -> list[ObjectSlot]:
    """`copies_per_class` instances of every class, made dynamic and parked on the floor."""
    stage = stage_utils.get_current_stage(backend="usd")
    stage_utils.define_prim(OBJECT_ROOT, "Xform")
    copies = CONFIG["spawner"]["copies_per_class"]
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    slots = []
    for row, (label, obj) in enumerate(CONFIG["objects"].items()):
        orientation = quat_wxyz(rpy_rotation(*obj["rest_rpy"]))
        for i in range(copies):
            path = f"{OBJECT_ROOT}/{label}_{i}"
            stage_utils.add_reference_to_stage(f"{assets_root}{YCB_DIR}/{obj['usd']}", path=path)
            prim = stage.GetPrimAtPath(path)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(obj["mass"])
            for child in Usd.PrimRange(prim):
                if child.IsA(UsdGeom.Mesh):
                    UsdPhysics.CollisionAPI.Apply(child)
                    UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr().Set("convexHull")
            add_labels(path, labels=[label])

            XformPrim(path, positions=[[0.0, 0.0, 0.0]], reset_xform_op_properties=True)
            size = tuple(bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetSize())
            XformPrim(path, orientations=[orientation])
            bbox_cache.Clear()
            rest_height = -bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMin()[2]
            parking = (PARKING_ORIGIN[0] + i * PARKING_SPACING, PARKING_ORIGIN[1] + row * PARKING_SPACING,
                       rest_height + 0.002)
            XformPrim(path, positions=[parking], orientations=[orientation])
            slots.append(ObjectSlot(path, label, orientation, rest_height, size, parking))
    return slots


def build_world() -> World:
    """Create a new stage with the whole cell (call before playing the timeline)."""
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Cannot access Isaac Sim assets")

    stage_utils.create_new_stage()
    stage_utils.define_prim("/World", "Xform")
    GroundPlane("/World/ground")
    DomeLight("/World/dome_light").set_intensities(600)
    sun = DistantLight("/World/sun")
    sun.set_intensities(2500)
    XformPrim("/World/sun", orientations=[quat_wxyz(rpy_rotation(20, 30, 0))], reset_xform_op_properties=True)

    _spawn_conveyor()
    bins = _spawn_bins()
    objects = _spawn_objects(assets_root)
    robot = _spawn_robot(assets_root)
    return World(robot=robot, objects=objects, bins=bins)


def yaw_quat(yaw_deg: float) -> list[float]:
    return quat_wxyz(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg))


def compose_wxyz(a: list[float], b: list[float]) -> list[float]:
    """a * b (apply b, then a)."""
    qa, qb = Gf.Quatd(a[0], *a[1:]), Gf.Quatd(b[0], *b[1:])
    q = qa * qb
    return [q.GetReal(), *q.GetImaginary()]


def set_conveyor_speed(speed: float) -> None:
    stage = stage_utils.get_current_stage(backend="usd")
    PhysxSchema.PhysxSurfaceVelocityAPI(stage.GetPrimAtPath(BELT_PATH)).GetSurfaceVelocityAttr().Set(
        Gf.Vec3f(0.0, float(speed), 0.0)
    )


def bin_of(position, bins: dict[str, dict]) -> str | None:
    """Name of the bin whose footprint contains `position` (below its rim), else None."""
    x, y, z = position[:3]
    rim = CONFIG["bins"]["wall_height"]
    for name, b in bins.items():
        (cx, cy), (sx, sy) = b["center"], b["size"]
        if abs(x - cx) < sx / 2 and abs(y - cy) < sy / 2 and b["floor"] <= z <= b["floor"] + rim:
            return name
    return None


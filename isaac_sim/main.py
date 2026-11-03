"""Run the conveyor sorting cell: Franka + conveyor + overhead RGB-D camera + bins, with the ROS 2 bridge.

Isaac Sim is only the plant: YCB objects ride the belt past the camera towards the robot, and the robot follows
/control/joint_command (position targets). Until a command arrives the arm holds the ready pose from
config/cell.yaml. Objects that leave the belt end, fall, or rest in a bin are recycled and the sort is scored.

    ~/isaacsim/python.sh isaac_sim/main.py  (--view gui | stream | headless, default gui)
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--view", choices=["headless", "stream", "gui"], default="gui")
parser.add_argument("--seed", type=int, default=None, help="Seed for object spawning")
parser.add_argument("--speed", type=float, default=None, help="Initial conveyor speed in m/s (default: config)")
parser.add_argument("--no-spawn", action="store_true", help="Start with the spawner off (/control/spawner_enabled)")
parser.add_argument("--no-ros", action="store_true", help="Run the world only, without the camera and ROS 2")
parser.add_argument("--steps", type=int, default=0, help="Stop after N updates (0 = run until closed)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

# Must be created before importing any other isaacsim.* modules
config = {"headless": args.view != "gui", "renderer": "RaytracedLighting"}
if args.view == "stream":
    config["hide_ui"] = False  # show the full UI in the streaming client
simulation_app = SimulationApp(config)

import isaacsim.core.experimental.utils.app as app_utils
import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager

app_utils.enable_extension("isaacsim.core.rendering_manager")
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager  # noqa: E402

from util.scene.cell import CONFIG, build_world, set_conveyor_speed  # noqa: E402
from util.scene.spawner import Spawner  # noqa: E402

if args.view == "stream":
    app_utils.enable_extension("omni.kit.livestream.app")

world = build_world()
# Before play: creating a physics view after the articulation's invalidates it
spawner = Spawner(world, seed=args.seed)
spawner.enabled = not args.no_spawn
speed = CONFIG["conveyor"]["speed"] if args.speed is None else args.speed
set_conveyor_speed(speed)

bridge = None
if not args.no_ros:
    from util.ros_bridge import RosBridge
    from util.sensor.camera import add_camera

    bridge = RosBridge(add_camera(), app_rate_hz=60.0)
if args.view != "headless":
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.9, -1.6, 1.7], target=[0.3, 0.0, 0.75])

# Physics at 120 Hz, app loop (inputs, spawner, /clock, rendering) at 60 Hz: 2 physics steps per update
PHYSICS_DT = 1.0 / 120.0
APP_DT = 1.0 / 60.0
SimulationManager.setup_simulation(dt=PHYSICS_DT)
RenderingManager.set_dt(APP_DT)
omni.timeline.get_timeline_interface().play()
simulation_app.update()

robot = world.robot
robot.reset_to_default_state()
robot.set_dof_position_targets([CONFIG["robot"]["ready_pose"]])
dof_names = robot.dof_names
print(f"[world] Franka DOFs: {dof_names}")
print(f"[world] classes -> bins: { {k: v['bin'] for k, v in CONFIG['objects'].items()} }, belt {speed:.3f} m/s")

step = 0
while simulation_app.is_running() and (args.steps == 0 or step < args.steps):
    sim_time = SimulationManager.get_simulation_time()
    if bridge is not None:
        cmd, new_speed, spawn = bridge.take_inputs()
        if cmd is not None:
            unknown = [n for n in cmd.name if n not in dof_names]
            if unknown or len(cmd.position) != len(cmd.name):
                bridge.warn(f"Rejected /control/joint_command: unknown joints {unknown} or no position per name")
            else:
                robot.set_dof_position_targets([list(cmd.position)], dof_indices=robot.get_dof_indices(cmd.name))
        if new_speed is not None:
            speed = new_speed
            set_conveyor_speed(speed)
        if spawn is not None:
            spawner.enabled = spawn
    spawner.step(sim_time)
    simulation_app.update()
    step += 1
    if bridge is not None and step % 6 == 0:  # 10 Hz
        bridge.publish_ground_truth(SimulationManager.get_simulation_time(), spawner.active_objects(), speed)

print(f"[spawner] after {SimulationManager.get_simulation_time():.1f} s: {spawner.stats}")
if bridge is not None:
    bridge.shutdown()
simulation_app.close()

"""Feeds the conveyor: teleports parked pool objects onto the belt start at random intervals and recycles them once
they leave the belt end, fall off, or have rested in a bin for `bin_dwell` s (counting whether it was the right bin).

All objects are created up front by `build_world` and wrapped in one RigidPrim view before the timeline plays:
creating prims or physics views while simulating would invalidate the robot's articulation view.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

import numpy as np
from isaacsim.core.experimental.prims import RigidPrim

from util.scene.cell import CONFIG, ObjectSlot, World, bin_of, compose_wxyz, yaw_quat

FALLEN_HEIGHT = 0.3  # m, below this an object has fallen to the floor


@dataclass
class SortStats:
    correct: int = 0
    wrong: int = 0
    missed: int = 0  # left the belt end or fell off
    per_class: dict[str, list[int]] = field(default_factory=dict)  # label -> [correct, wrong, missed]

    def count(self, label: str, outcome: str) -> None:
        setattr(self, outcome, getattr(self, outcome) + 1)
        self.per_class.setdefault(label, [0, 0, 0])[("correct", "wrong", "missed").index(outcome)] += 1

    def __str__(self) -> str:
        return f"correct {self.correct}, wrong {self.wrong}, missed {self.missed}"


@dataclass
class ActiveObject:
    index: int
    in_bin_since: float | None = None


class Spawner:
    def __init__(self, world: World, seed: int | None = None):
        self._slots = world.objects
        self._bins = world.bins
        self._view = RigidPrim([slot.path for slot in self._slots])
        self._rng = random.Random(seed)
        self._idle = list(range(len(self._slots)))
        self._active: dict[int, ActiveObject] = {}
        self._next_spawn: float | None = None
        self.stats = SortStats()
        self.enabled = True

        cfg = CONFIG["spawner"]
        self._interval = cfg["interval"]
        self._jitter = cfg["lateral_jitter"]
        self._dwell = cfg["bin_dwell"]
        conveyor = CONFIG["conveyor"]
        self._belt_x, self._belt_top = conveyor["center_x"], conveyor["top"]
        self._start_y, self._end_y = conveyor["start_y"], conveyor["end_y"]

    def step(self, sim_time: float) -> None:
        if self._next_spawn is None:
            self._next_spawn = sim_time
        if self.enabled and sim_time >= self._next_spawn:
            self._spawn()
            self._next_spawn = sim_time + self._rng.uniform(*self._interval)
        self._recycle(sim_time)

    def active_objects(self) -> list[tuple[ObjectSlot, np.ndarray, np.ndarray]]:
        """(slot, world position, world orientation wxyz) of every object currently out of the pool."""
        if not self._active:
            return []
        indices = sorted(self._active)
        positions, orientations = (a.numpy() for a in self._view.get_world_poses(indices=indices))
        return [(self._slots[i], p, q) for i, p, q in zip(indices, positions, orientations)]

    def _spawn(self) -> None:
        # Pick a class first so classes stay balanced, then a free copy of it
        free_labels = sorted({self._slots[i].label for i in self._idle})
        if not free_labels:
            return
        label = self._rng.choice(free_labels)
        index = self._rng.choice([i for i in self._idle if self._slots[i].label == label])
        self._idle.remove(index)
        slot = self._slots[index]
        x = self._belt_x + self._rng.uniform(-self._jitter, self._jitter)
        position = [x, self._start_y, self._belt_top + slot.rest_height + 0.005]
        orientation = compose_wxyz(yaw_quat(self._rng.uniform(-180.0, 180.0)), slot.rest_orientation)
        self._teleport(index, position, orientation)
        self._active[index] = ActiveObject(index)

    def _recycle(self, sim_time: float) -> None:
        if not self._active:
            return
        indices = sorted(self._active)
        positions = self._view.get_world_poses(indices=indices)[0].numpy()
        for index, position in zip(indices, positions):
            active, label = self._active[index], self._slots[index].label
            outcome = None
            bin_name = bin_of(position, self._bins)
            if bin_name is not None:
                if active.in_bin_since is None:
                    active.in_bin_since = sim_time
                elif sim_time - active.in_bin_since >= self._dwell:
                    outcome = "correct" if CONFIG["objects"][label]["bin"] == bin_name else "wrong"
            else:
                active.in_bin_since = None
                if position[1] > self._end_y or position[2] < FALLEN_HEIGHT:
                    outcome = "missed"
            if outcome is not None:
                self.stats.count(label, outcome)
                print(f"[spawner] {self._slots[index].path}: {outcome} ({self.stats})")
                slot = self._slots[index]
                self._teleport(index, list(slot.parking), slot.rest_orientation)
                del self._active[index]
                self._idle.append(index)

    def _teleport(self, index: int, position, orientation) -> None:
        self._view.set_world_poses(positions=[position], orientations=[orientation], indices=[index])
        self._view.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]],
                                  indices=[index])

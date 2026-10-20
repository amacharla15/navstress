import math
from pathlib import Path

import pytest
import yaml

from navstress_fakesim.world import Thing, World, WorldConfig
from navstress_scenarios.goals import GoalSet

CONFIG = Path(__file__).resolve().parent.parent / "config"


@pytest.fixture
def world():
    return World(WorldConfig.from_dict(yaml.safe_load((CONFIG / "world.yaml").read_text())))


def box(name, x, y, h, sx=0.6, sy=0.6):
    return Thing(name, "box", x, y, 0.0, sx, sy, h)


def test_every_configured_goal_has_a_straight_clear_path(world):
    goals = GoalSet.from_yaml(CONFIG / "goals.yaml")
    for gid in goals.ids():
        gx, gy, _ = goals.get(gid)
        path = world.plan(gx, gy)
        assert path is not None, gid
        assert len(path) == 1, f"{gid}: open floor should give one straight segment"


def test_scan_sees_walls_and_tall_obstacles_but_not_low_ones(world):
    n = world.cfg.lidar_rays
    ahead = n // 2  # ray index pointing along +x (angles start at -pi)
    base = world.scan()[ahead]
    assert base == pytest.approx(13.0, abs=1e-6) or math.isinf(base)  # far wall beyond range
    world.things["low"] = box("low", 3.0, 0.0, h=0.1)
    assert world.scan()[ahead] == base
    world.things["tall"] = box("tall", 3.0, 0.0, h=0.5)
    assert world.scan()[ahead] == pytest.approx(2.7, abs=1e-6)


def test_cylinder_ray_hit(world):
    world.things["c"] = Thing("c", "cylinder", 2.0, 0.0, 0.0, 0.5, 0.5, 1.0)
    assert world.scan()[world.cfg.lidar_rays // 2] == pytest.approx(1.75, abs=1e-6)


def test_planner_detours_around_visible_but_not_low_obstacles(world):
    world.things["tall"] = box("tall", 5.5, 0.0, h=0.6, sx=1.2, sy=1.0)
    detour = world.plan(11.0, 0.0)
    assert detour is not None and len(detour) > 1
    assert min(abs(y) for _, y in detour[:-1]) > 0.5
    del world.things["tall"]
    world.things["low"] = box("low", 5.5, 0.0, h=0.08)
    assert world.plan(11.0, 0.0) == [(11.0, 0.0)]  # straight through: it can't see it


def test_robot_is_blocked_by_any_obstacle_regardless_of_height(world):
    assert not world.blocked(2.0, 0.0, 0.0)
    world.things["low"] = box("low", 2.2, 0.0, h=0.05)
    assert world.blocked(2.0, 0.0, 0.0)
    assert world.blocked(12.9, 0.0, 0.0)  # past the room edge
    assert world.blocked(5.0, 2.5, 0.0)  # overlapping a shelf


def test_reset_clears_obstacles_and_restores_start(world):
    world.things["x"] = box("x", 1.0, 1.0, 1.0)
    world.x, world.y = 5.0, 1.0
    world.reset()
    assert not world.things and (world.x, world.y, world.yaw) == (0.0, 0.0, 0.0)

import math

import numpy as np
import pytest
import yaml
from PIL import Image

from navstress_scenarios.generate import make_baseline
from navstress_scenarios.goals import (
    GoalSet,
    NamedGoal,
    OccupancyGrid,
    ReachabilityMap,
    inflate_blocked,
    sample_goal_set,
)
from navstress_scenarios.schema import config_hash


def _write_map(tmp_path, img: np.ndarray, resolution=0.1, origin=(-2.0, -1.0)):
    """img: 0 = free (white), 1 = occupied (black); row 0 is the top of the map."""
    pixels = np.where(img == 1, 0, 254).astype(np.uint8)
    Image.fromarray(pixels, mode="L").save(tmp_path / "m.png")
    meta = {
        "image": "m.png",
        "resolution": resolution,
        "origin": [origin[0], origin[1], 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    (tmp_path / "m.yaml").write_text(yaml.safe_dump(meta))
    return tmp_path / "m.yaml"


def _two_rooms():
    """40x60 cells: border walls, an open left room, and a sealed right room."""
    img = np.zeros((40, 60), dtype=np.uint8)
    img[0, :] = img[-1, :] = img[:, 0] = img[:, -1] = 1
    img[:, 35] = 1  # solid wall, no door: the right room is free but unreachable
    return img


def test_map_loading_and_coordinates(tmp_path):
    grid = OccupancyGrid.from_map_yaml(_write_map(tmp_path, _two_rooms()))
    assert grid.free.shape == (40, 60)
    assert grid.bounds() == pytest.approx((-2.0, 4.0, -1.0, 3.0))
    # bottom-left cell centre is at origin + half a cell, and it's a wall
    assert grid.world_to_cell(-1.95, -0.95) == (39, 0)
    assert not grid.free[39, 0]
    for r, c in [(5, 7), (20, 40), (33, 2)]:
        assert grid.world_to_cell(*grid.cell_to_world(r, c)) == (r, c)
    assert grid.world_to_cell(10.0, 0.0) is None
    assert len(grid.source_sha256) == 64


def test_inflation_blocks_cells_near_walls():
    free = np.ones((15, 15), dtype=bool)
    free[7, 7] = False
    blocked = inflate_blocked(free, 2)
    assert blocked[7, 9] and blocked[5, 7] and not blocked[7, 10]
    # the map edge counts as an obstacle
    assert blocked[1, 7] and blocked[7, 13] and not blocked[2, 7]


def test_sealed_room_is_free_but_unreachable(tmp_path):
    grid = OccupancyGrid.from_map_yaml(_write_map(tmp_path, _two_rooms()))
    reach = ReachabilityMap(grid, start_xy=(0.0, 1.0), robot_radius_m=0.25)
    right_room = grid.cell_to_world(20, 47)
    assert grid.free[20, 47]
    assert not reach.is_reachable(*right_room)
    assert reach.is_reachable(1.0, 1.0)
    # closer than the robot radius to a wall: not reachable
    near_wall = grid.cell_to_world(20, 34)
    assert not reach.is_reachable(*near_wall)


def test_start_inside_obstacle_is_an_error(tmp_path):
    grid = OccupancyGrid.from_map_yaml(_write_map(tmp_path, _two_rooms()))
    with pytest.raises(ValueError, match="within"):
        ReachabilityMap(grid, start_xy=grid.cell_to_world(20, 35), robot_radius_m=0.2)


def test_sampling_is_deterministic_and_respects_constraints(tmp_path):
    path = _write_map(tmp_path, _two_rooms())
    a = sample_goal_set(
        path,
        (0.0, 1.0),
        8,
        seed=3,
        robot_radius_m=0.25,
        min_separation_m=0.5,
        min_start_distance_m=0.8,
    )
    b = sample_goal_set(
        path,
        (0.0, 1.0),
        8,
        seed=3,
        robot_radius_m=0.25,
        min_separation_m=0.5,
        min_start_distance_m=0.8,
    )
    assert a.to_dict() == b.to_dict()
    goals = [a.get(i) for i in a.ids()]
    reach = ReachabilityMap(OccupancyGrid.from_map_yaml(path), (0.0, 1.0), 0.25)
    for x, y, _ in goals:
        assert reach.is_reachable(x, y)
        assert math.hypot(x, y - 1.0) >= 0.8
    for i, g in enumerate(goals):
        for h in goals[i + 1 :]:
            assert math.hypot(g[0] - h[0], g[1] - h[1]) >= 0.5
    assert a.meta["map_image_sha256"] and a.meta["seed"] == 3


def test_sampling_fails_loudly_when_impossible(tmp_path):
    path = _write_map(tmp_path, _two_rooms())
    with pytest.raises(ValueError, match="only"):
        sample_goal_set(
            path,
            (0.0, 1.0),
            500,
            seed=0,
            robot_radius_m=0.25,
            min_separation_m=1.0,
            min_start_distance_m=0.0,
        )


def test_goal_set_yaml_round_trip(tmp_path):
    gs = GoalSet([NamedGoal("a", 1.0, 2.0, 0.5), NamedGoal("b", -1.0, 0.0, 0.0)], {"k": 1})
    gs.to_yaml(tmp_path / "g.yaml")
    back = GoalSet.from_yaml(tmp_path / "g.yaml")
    assert back.to_dict() == gs.to_dict()
    assert back.nearest(0.9, 2.1)[0] == "a"
    with pytest.raises(ValueError):
        GoalSet([NamedGoal("a", 0, 0, 0), NamedGoal("a", 1, 1, 0)])


def test_baseline_is_deterministic():
    gs = GoalSet([NamedGoal(f"g{i}", float(i), 0.0, 0.0) for i in range(10)])
    a = make_baseline(gs, 50, seed=1)
    b = make_baseline(gs, 50, seed=1)
    assert [config_hash(s) for s in a] == [config_hash(s) for s in b]
    assert len({s.scenario_id for s in a}) == 50
    assert all(s.family == "baseline" and s.goal_id in gs.ids() for s in a)
    c = make_baseline(gs, 50, seed=2)
    assert [config_hash(s) for s in a] != [config_hash(s) for s in c]

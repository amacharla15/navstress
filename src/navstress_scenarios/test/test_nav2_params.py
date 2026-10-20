import pytest
import yaml

from navstress_scenarios.nav2_params import reroute, write_rerouted

# The relevant shape of carter_navigation_params.yaml (IsaacSim-ros_workspaces tag
# IsaacSim-6.2.0), trimmed to the keys this module touches.
CARTER_LIKE = """
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["front_3d_lidar_layer", "front_2d_lidar_layer", "inflation_layer"]
      front_2d_lidar_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /front_2d_lidar/scan
          data_type: "LaserScan"
      front_3d_lidar_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        mark_threshold: 2
        observation_sources: hesai_mark
        hesai_mark:
          data_type: PointCloud2
          topic: /front_3d_lidar/lidar_points
          min_obstacle_height: -0.4
global_costmap:
  global_costmap:
    ros__parameters:
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
amcl:
  ros__parameters:
    scan_topic: scan
"""
RELAY = {"/front_3d_lidar/lidar_points": "/navstress/front_3d_lidar/lidar_points"}


def test_reroute_changes_only_the_mapped_topic():
    params = yaml.safe_load(CARTER_LIKE)
    out, changed = reroute(params, RELAY)
    lc = out["local_costmap"]["local_costmap"]["ros__parameters"]
    assert (
        lc["front_3d_lidar_layer"]["hesai_mark"]["topic"] == RELAY["/front_3d_lidar/lidar_points"]
    )
    assert lc["front_2d_lidar_layer"]["scan"]["topic"] == "/front_2d_lidar/scan"
    gc = out["global_costmap"]["global_costmap"]["ros__parameters"]
    assert gc["obstacle_layer"]["scan"]["topic"] == "/scan"
    assert changed == [
        "local_costmap.front_3d_lidar_layer.hesai_mark.topic: "
        "/front_3d_lidar/lidar_points -> /navstress/front_3d_lidar/lidar_points"
    ]
    # the input is not modified
    assert params != out
    assert yaml.safe_load(CARTER_LIKE) == params


def test_written_copy_is_content_named_and_stable(tmp_path):
    src = tmp_path / "carter.yaml"
    src.write_text(CARTER_LIKE)
    a, _ = write_rerouted(src, RELAY, tmp_path / "out")
    b, _ = write_rerouted(src, RELAY, tmp_path / "out")
    assert a == b and a.name.startswith("nav2_params_")
    text = a.read_text()
    assert text.startswith("# Generated from") and "hesai_mark.topic" in text
    loaded = yaml.safe_load(text)
    assert loaded["amcl"] == {"ros__parameters": {"scan_topic": "scan"}}


def test_missing_source_is_an_error(tmp_path):
    src = tmp_path / "p.yaml"
    src.write_text(CARTER_LIKE)
    with pytest.raises(ValueError, match="no observation source reads"):
        write_rerouted(src, {"/nope": "/x"}, tmp_path)

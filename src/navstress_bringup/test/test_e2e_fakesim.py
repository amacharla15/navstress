"""End to end against the fake simulator: agent -> RunBatch -> results -> report.

Launches fake_sim and every NavStress node (scenario_manager, noise_injector,
obstacle_mover, navstress_collision_monitor, eval_runner), then runs the real agent
loop with the mock LLM over the RunBatch action, and builds the report from the
files that landed on disk. The outcomes come from the fake's crude world rules; what
this test checks is that the pipeline is wired correctly, not how good Nav2 is.
"""

import contextlib
import os
import tempfile
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import pytest
import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from domain_coordinator import domain_id
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient

from navstress_agent.llm import MockLLM
from navstress_agent.loop import AGENT_LOG, Budget, StressAgent
from navstress_eval.batch_client import BatchClient, spin_in_background
from navstress_eval.results_io import read_metadata, read_rows
from navstress_interfaces.action import RunEpisode
from navstress_report.cli import build
from navstress_scenarios.conversions import spec_to_msg
from navstress_scenarios.goals import GoalSet
from navstress_scenarios.rosutil import wait_future
from navstress_scenarios.schema import validate_or_raise

# A private ROS domain for this test run, held until the process exits, so parallel
# tests on the same machine can't hear each other.
_DOMAIN = contextlib.ExitStack()
os.environ["ROS_DOMAIN_ID"] = str(_DOMAIN.enter_context(domain_id()))
RESULTS = Path(tempfile.mkdtemp(prefix="navstress_e2e_"))
RUN_ID = "agent_e2e"


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("navstress_bringup")
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "fakesim.launch.py")),
        launch_arguments={"results_dir": str(RESULTS)}.items(),
    )
    return launch.LaunchDescription([pipeline, launch_testing.actions.ReadyToTest()])


def _topic_counts(bag_dir: Path) -> dict[str, int]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    counts: dict[str, int] = {}
    while reader.has_next():
        topic, _, _ = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
    return counts


class TestEndToEnd(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("navstress_e2e_test")
        cls.executor, _ = spin_in_background(cls.node)
        cls.goals = GoalSet.from_yaml(
            Path(get_package_share_directory("navstress_fakesim")) / "config" / "goals.yaml"
        )

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def test_1_agent_batch_results_report(self):
        agent = StressAgent(
            llm=MockLLM(),
            run_batch=BatchClient(self.node, record_bags=True),
            goal_set=self.goals,
            default_start=(0.0, 0.0, 0.0),
            out_dir=RESULTS / RUN_ID,
            run_id=RUN_ID,
            budget=Budget(max_rounds=2, max_episodes_per_round=8, max_llm_calls=3),
        )
        result = agent.run()

        # Two rounds ran, and round 2 narrowed onto round 1's weakest family.
        self.assertEqual(result.stop_reason, "max rounds reached", result)
        r1, r2 = result.rounds
        self.assertIsNotNone(r1.weakest_family)
        self.assertEqual(r2.focus_family, r1.weakest_family)
        self.assertLess(r1.summaries[r1.weakest_family]["success_rate"], 1.0)

        # Every episode left a row, a bag and run metadata on disk.
        rows1 = read_rows(RESULTS / f"{RUN_ID}_r1")
        rows2 = read_rows(RESULTS / f"{RUN_ID}_r2")
        self.assertEqual(len(rows1), 8)
        self.assertEqual(len(rows2), r2.episodes)
        self.assertTrue(all(r["family"] == r2.focus_family for r in rows2))
        self.assertTrue(all(r["failure_reason"] != "setup_failed" for r in rows1 + rows2), rows1)
        meta = read_metadata(RESULTS / f"{RUN_ID}_r1")
        self.assertEqual(meta["simulator"], "fakesim")
        self.assertEqual(meta["episodes_completed"], 8)

        # Low obstacles are invisible to the fake lidar and block the fake robot.
        low = [r for r in rows1 if r["family"] == "low_obstacle"]
        self.assertTrue(low and not any(r["success"] for r in low), low)

        # The bags hold the relayed (noise-injector) scan and the monitor's output.
        bag = RESULTS / f"{RUN_ID}_r1" / rows1[0]["bag_path"]
        counts = _topic_counts(bag)
        for topic in ("/scan_raw", "/scan", "/odom", "/navstress/clearance"):
            self.assertGreater(counts.get(topic, 0), 0, f"{topic} missing from {bag}: {counts}")

        self.assertTrue((RESULTS / RUN_ID / AGENT_LOG).exists())

        # The report reads only those files and labels them as fakesim output.
        md = RESULTS / "docs" / "RESULTS.md"
        site = RESULTS / "site"
        build(
            runs=[RESULTS / f"{RUN_ID}_r1"], agents=[RESULTS / RUN_ID], before=None, after=None,
            out_md=md, assets_dir=RESULTS / "docs" / "img", site_dir=site, title="e2e",
        )  # fmt: skip
        text = md.read_text()
        self.assertIn("FAKESIM", text)
        self.assertIn(f"narrowing on `{r2.focus_family}`", text)
        self.assertTrue((site / "index.html").exists())

    def test_2_single_episode_action(self):
        client = ActionClient(self.node, RunEpisode, "/navstress/run_episode")
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))
        spec = validate_or_raise(
            {"scenario_id": "single/0", "family": "single", "goal": {"x": 8.5, "y": 1.2}}
        )
        goal = RunEpisode.Goal(scenario=spec_to_msg(spec), run_id="", record_bag=False)
        handle = wait_future(client.send_goal_async(goal), 30.0, "RunEpisode goal")
        self.assertTrue(handle.accepted)
        res = wait_future(handle.get_result_async(), 120.0, "RunEpisode result").result.result
        self.assertTrue(res.success, res)
        self.assertGreater(res.path_length_m, 7.0)  # straight line is ~8.6 m
        self.assertLess(res.final_distance_to_goal_m, 0.3)

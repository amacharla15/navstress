"""noise_injector: relay a range topic to Nav2 with Gaussian noise and dropout.

The simulator stays untouched; Nav2's input is remapped to this node's output. With
both noise parameters at zero the message is forwarded as-is, so the relay sits in
the same place for every scenario and baseline runs pay the same small latency.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from navstress_scenarios.noise import noisy_points, noisy_ranges
from navstress_scenarios.schema import NOISE_RANGES


def _range_descriptor(key: str, text: str) -> ParameterDescriptor:
    r = NOISE_RANGES[key]
    return ParameterDescriptor(
        description=text,
        floating_point_range=[FloatingPointRange(from_value=r.lo, to_value=r.hi, step=0.0)],
    )


class NoiseInjector(Node):
    def __init__(self) -> None:
        super().__init__("noise_injector")
        self.declare_parameter("message_type", "LaserScan")
        self.declare_parameter("input_topic", "scan_raw")
        self.declare_parameter("output_topic", "scan")
        # Same ranges as the scenario schema; ROS rejects out-of-range sets for us.
        self.std = self.declare_parameter(
            "gaussian_std_m", 0.0, _range_descriptor("gaussian_std_m", "range noise std-dev, m")
        ).value
        self.dropout = self.declare_parameter(
            "dropout_prob", 0.0, _range_descriptor("dropout_prob", "per-return drop probability")
        ).value
        self.seed = self.declare_parameter("seed", 0).value
        self.rng = np.random.default_rng(self.seed)
        self.add_post_set_parameters_callback(self._on_params)

        kind = self.get_parameter("message_type").value
        msg_type = {"LaserScan": LaserScan, "PointCloud2": PointCloud2}.get(kind)
        if msg_type is None:
            raise ValueError(f"message_type must be LaserScan or PointCloud2, got {kind!r}")
        # Sensor Data QoS (best effort) on both sides: Isaac Sim publishes sensors best
        # effort, and Nav2's costmap and AMCL subscribe with SensorDataQoS.
        self.pub = self.create_publisher(
            msg_type, self.get_parameter("output_topic").value, qos_profile_sensor_data
        )
        handler = self._on_scan if msg_type is LaserScan else self._on_cloud
        self.sub = self.create_subscription(
            msg_type, self.get_parameter("input_topic").value, handler, qos_profile_sensor_data
        )

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "gaussian_std_m":
                self.std = float(p.value)
            elif p.name == "dropout_prob":
                self.dropout = float(p.value)
            elif p.name == "seed":
                # Reseed per scenario so a scenario's noise sequence starts the same way
                # each time. Exact repeats also depend on message arrival order.
                self.seed = int(p.value)
                self.rng = np.random.default_rng(self.seed)
        self.get_logger().info(
            f"noise std={self.std:.3f} m dropout={self.dropout:.2f} seed={self.seed}"
        )
        return SetParametersResult(successful=True)

    def _active(self) -> bool:
        return self.std > 0.0 or self.dropout > 0.0

    def _on_scan(self, msg: LaserScan) -> None:
        if self._active():
            msg.ranges = noisy_ranges(
                np.asarray(msg.ranges), msg.range_min, msg.range_max, self.std, self.dropout,
                self.rng,
            ).tolist()  # fmt: skip
        self.pub.publish(msg)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not self._active():
            self.pub.publish(msg)
            return
        pts = point_cloud2.read_points(msg, skip_nans=False)
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=-1).reshape(-1, 3)
        new_xyz, keep = noisy_points(xyz, self.std, self.dropout, self.rng)
        flat = pts.reshape(-1).copy()
        for i, axis in enumerate("xyz"):
            flat[axis] = new_xyz[:, i].astype(flat.dtype[axis])
        # Dropping points breaks a 2D organized layout, so the output is unorganized.
        out = point_cloud2.create_cloud(msg.header, msg.fields, flat[keep], msg.point_step)
        out.is_dense = msg.is_dense
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NoiseInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

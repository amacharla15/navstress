"""obstacle_mover: drive moving obstacles back and forth with SetEntityState."""

from __future__ import annotations

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from simulation_interfaces.msg import EntityState, Result
from simulation_interfaces.srv import SetEntityState

from navstress_interfaces.msg import ScenarioSpec
from navstress_scenarios.conversions import yaw_to_quaternion
from navstress_scenarios.motion import map_to_world, position_at

# Latched so a node that starts late still gets the scenario that is currently active.
ACTIVE_SCENARIO_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class ObstacleMover(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_mover")
        self.rate_hz = self.declare_parameter("rate_hz", 10.0).value
        self.prefix = self.declare_parameter("entity_prefix", "/navstress_").value
        self.map_to_world = tuple(self.declare_parameter("map_to_world", [0.0, 0.0, 0.0]).value)
        service = self.declare_parameter("set_entity_state_service", "/set_entity_state").value
        group = ReentrantCallbackGroup()
        self.client = self.create_client(SetEntityState, service, callback_group=group)
        self.create_subscription(
            ScenarioSpec, "/navstress/active_scenario", self._on_scenario, ACTIVE_SCENARIO_QOS
        )
        self.movers: list = []
        self.t0 = self.get_clock().now()
        self.pending: dict[str, object] = {}
        self.create_timer(1.0 / self.rate_hz, self._tick, callback_group=group)

    def _on_scenario(self, msg: ScenarioSpec) -> None:
        self.movers = [o for o in msg.obstacles if o.moving and o.speed_mps > 0.0]
        self.pending.clear()
        # Motion time starts when the scenario is applied, so each run of the same
        # scenario sees the obstacle in the same place relative to the goal being sent.
        self.t0 = self.get_clock().now()
        if self.movers:
            names = ", ".join(o.name for o in self.movers)
            self.get_logger().info(f"{msg.scenario_id}: moving {names}")

    def _tick(self) -> None:
        if not self.movers or not self.client.service_is_ready():
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        for o in self.movers:
            fut = self.pending.get(o.name)
            if fut is not None and not fut.done():
                continue  # don't queue updates faster than the simulator answers
            if fut is not None and fut.done() and fut.result() is not None:
                res = fut.result().result
                if res.result != Result.RESULT_OK:
                    self.get_logger().warn(f"set_entity_state {o.name}: {res.error_message}")
            x, y = position_at((o.pose.x, o.pose.y), (o.end_x, o.end_y), o.speed_mps, t)
            wx, wy, wyaw = map_to_world(x, y, o.pose.yaw, self.map_to_world)
            state = EntityState()
            state.pose.position.x, state.pose.position.y = wx, wy
            state.pose.orientation = yaw_to_quaternion(wyaw)
            req = SetEntityState.Request(entity=self.prefix + o.name, state=state)
            self.pending[o.name] = self.client.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMover()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

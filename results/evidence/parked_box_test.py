import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from simulation_interfaces.msg import Resource, SpawnEntity
from simulation_interfaces.srv import SpawnEntities, DeleteEntity

rclpy.init()
node = Node("parked_box_test")
tf = Buffer()
listener = TransformListener(tf, node)
latest = {"grid": None, "clock": None}

def on_grid(msg):
    latest["grid"] = msg

def on_clock(msg):
    latest["clock"] = msg.clock.sec + msg.clock.nanosec * 1e-9

node.create_subscription(
    OccupancyGrid, "/local_costmap/front_3d_lidar_layer",
    on_grid, 10
)
node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
def spin_until(condition, timeout=30):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            raise TimeoutError("Timed out waiting for ROS data")
        rclpy.spin_once(node, timeout_sec=0.5)

def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )
def wait_sim(seconds):
    spin_until(lambda: latest["clock"] is not None)
    start = latest["clock"]
    spin_until(
        lambda: latest["clock"] >= start + seconds,
        timeout=max(60, seconds * 5)
    )
def call(client, request):
    if not client.wait_for_service(timeout_sec=15):
        raise RuntimeError("Simulator service unavailable")
    future = client.call_async(request)
    spin_until(lambda: future.done(), 30)
    if future.exception():
        raise future.exception()
    return future.result()

spin_until(lambda: latest["grid"] is not None and latest["clock"] is not None)
def get_pose(frame):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        try:
            return tf.lookup_transform(frame, "base_link", Time()).transform
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.5)
    raise RuntimeError("Unable to obtain robot TF")

map_pose = get_pose("map")
odom_pose = get_pose("odom")
map_yaw = yaw(map_pose.rotation)
odom_yaw = yaw(odom_pose.rotation)
mx = map_pose.translation.x + 1.5 * math.cos(map_yaw)
my = map_pose.translation.y + 1.5 * math.sin(map_yaw)
ox = odom_pose.translation.x + 1.5 * math.cos(odom_yaw)
oy = odom_pose.translation.y + 1.5 * math.sin(odom_yaw)

print(f"BOX MAP POSITION: ({mx:.3f}, {my:.3f})", flush=True)
print(f"BOX ODOM POSITION: ({ox:.3f}, {oy:.3f})", flush=True)
def occupied_cells():
    grid = latest["grid"]
    info = grid.info
    resolution = info.resolution
    origin = info.origin.position
    angle = yaw(info.origin.orientation)
    c, s = math.cos(angle), math.sin(angle)
    occupied = set()
    # Fixed world-space sample positions around the box.
    for i in range(-9, 10):
        for j in range(-9, 10):
            x = ox + i * 0.05
            y = oy + j * 0.05
            dx, dy = x - origin.x, y - origin.y
            gx = int(math.floor((c * dx + s * dy) / resolution))
            gy = int(math.floor((-s * dx + c * dy) / resolution))
            if 0 <= gx < info.width and 0 <= gy < info.height:
                value = grid.data[gy * info.width + gx]
                if value >= 65:
                    occupied.add((i, j))
    return occupied

print("Allowing old obstacle markings to decay...", flush=True)
wait_sim(20)
before = occupied_cells()
print("BEFORE BOX:", len(before), "occupied samples", flush=True)

asset = next(Path("/tmp/navstress_assets").glob("navstress_box_*.usda"))
name = "/navstress_parked_probe"
spawn_client = node.create_client(SpawnEntities, "/spawn_entities")
delete_client = node.create_client(DeleteEntity, "/delete_entity")
spawned = False

try:
    request = SpawnEntities.Request()
    pose = PoseStamped()
    pose.pose.position.x = mx
    pose.pose.position.y = my
    pose.pose.orientation.w = 1.0
    request.spawn_requests.append(
        SpawnEntity(
            name=name,
            allow_renaming=False,
            entity_resource=Resource(uri=asset.resolve().as_uri()),
            initial_pose=pose
        )
    )
    result = call(spawn_client, request)
    print("SPAWN:", result, flush=True)
    if result.result.result != 1 or any(
        r.result.result != 1 for r in result.results
    ):
        raise RuntimeError("Box spawn failed")
    spawned = True

    wait_sim(8)
    present = occupied_cells()
    newly_marked = present - before
    print("BOX PRESENT:", len(present), "occupied samples", flush=True)
    print("NEWLY MARKED:", len(newly_marked), flush=True)

finally:
    if spawned:
        result = call(delete_client, DeleteEntity.Request(entity=name))
        print("DELETE:", result, flush=True)
if spawned:
    previous = 0
    for seconds in (5, 20, 35):
        wait_sim(seconds - previous)
        previous = seconds
        current = occupied_cells()
        remaining = len(newly_marked & current)
        print(
            f"AFTER DELETE +{seconds}s:",
            f"{remaining}/{len(newly_marked)} newly marked samples remain",
            flush=True
        )

node.destroy_node()
rclpy.shutdown()

"""A synced RGB-D frame grabber, adapted from clair-robotics-stack's
ROSCameraSensor (examples/tamp/helpers.py).

Not a plain import: that class lives under clair-robotics-stack's
examples/ directory, which is a script folder, not part of the
`clair_robotics_stack` installed package namespace (only
clair_robotics_stack.vision/.camera/.planning/... are importable after
`pip install -e`) - so this is a direct adaptation of the same
working pattern (message_filters ApproximateTimeSynchronizer + cv_bridge,
spinning itself in a private executor/thread so it coexists with the
mission node's own rclpy usage in the same process), not a duplicate
reimplementation from scratch.

One behavioral difference from the original: depth_scale defaults to 1.0
here, not 0.001. The original's default matches a real RealSense's raw
uint16-millimeters depth encoding; Gazebo's rgbd_camera sensor publishes
depth as float32 meters already, so no rescaling is needed.
"""
import threading

import rclpy
import message_filters
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image


class SyncedCameraSensor(Node):

    def __init__(
        self,
        color_topic: str,
        depth_topic: str,
        depth_scale: float = 1.0,
        sync_slop: float = 0.05,
        timeout: float = 5.0,
        node_name: str = "synced_camera_sensor",
    ):
        super().__init__(node_name)
        self._bridge = CvBridge()
        self._depth_scale = depth_scale
        self._timeout = timeout
        self._latest = None
        self._lock = threading.Lock()
        self._event = threading.Event()

        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=sync_slop)
        self._sync.registerCallback(self._on_synced)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def _on_synced(self, color_msg: Image, depth_msg: Image):
        rgb = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype("float64")
        depth = depth * self._depth_scale
        with self._lock:
            self._latest = {"rgb": rgb, "depth": depth}
        self._event.set()

    def __call__(self) -> dict:
        self._event.clear()
        if not self._event.wait(timeout=self._timeout):
            raise TimeoutError(f"no synced camera frame within {self._timeout}s")
        with self._lock:
            return self._latest

    def shutdown(self):
        self._executor.shutdown()
        self._spin_thread.join(timeout=2.0)
        self.destroy_node()

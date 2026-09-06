"""Thin wrapper around a robot's DetachableJoint attach/detach topics
(bridged from Gazebo by demo.launch.py) - this is the "gripper" for this
demo: no simulated finger contact, just a rigid attach/detach of the box to
the wrist. See plan: "simple attach/detach plugin", mirroring the
"adhesive_gripper" concept already used in clair-robotics-stack's MuJoCo sim.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


class GraspController:

    def __init__(self, node: Node, robot_name: str):
        self._node = node
        self._attach_pub = node.create_publisher(Empty, f"/{robot_name}/gripper/attach", 10)
        self._detach_pub = node.create_publisher(Empty, f"/{robot_name}/gripper/detach", 10)
        # DetachableJoint's own output_topic reports its actual state
        # ("attached"/"detached", confirmed by echoing it directly) after
        # each transition - used below to turn attach()/detach() into a
        # real confirmed handshake instead of a fire-and-hope publish.
        self._state = None
        node.create_subscription(String, f"/{robot_name}/gripper/state", self._on_state, 10)

    def _on_state(self, msg: String):
        self._state = msg.data

    def attach(self, timeout_s: float = 15.0) -> bool:
        return self._publish_until_confirmed(self._attach_pub, "attached", timeout_s)

    def detach(self, timeout_s: float = 15.0) -> bool:
        return self._publish_until_confirmed(self._detach_pub, "detached", timeout_s)

    def _publish_until_confirmed(self, pub, expected_state: str, timeout_s: float) -> bool:
        # A one-shot (or even a fixed-count retried) publish can still lose
        # a discovery-timing race against the Gazebo-side DetachableJoint
        # subscriber - its gz-transport peer discovery is a separate
        # process from, and not implied by, ros2_control's controller
        # manager being active. Rather than guess how long that takes
        # (tried and failed twice: once assuming a few seconds was enough,
        # once gating this node's whole startup on the robot's own
        # controller spawner exiting - the box still got dragged on the
        # very first motion both times), retry-publish while actually
        # spinning the node so the state subscription above can fire, and
        # only report success once the plugin's own output_topic confirms
        # the transition really happened.
        self._node.get_logger().info(f"grasp: requesting {expected_state}")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            pub.publish(Empty())
            spin_deadline = min(deadline, time.time() + 0.3)
            while time.time() < spin_deadline:
                rclpy.spin_once(self._node, timeout_sec=0.05)
                if self._state == expected_state:
                    self._node.get_logger().info(f"grasp: confirmed {expected_state}")
                    return True
        self._node.get_logger().error(
            f"grasp: never confirmed {expected_state} after {timeout_s:.1f}s - proceeding anyway")
        return False

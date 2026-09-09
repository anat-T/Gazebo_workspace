
import time

import rclpy
from rclpy.node import Node

from onrobot_2fg7_interfaces.msg import Status
from onrobot_2fg7_interfaces.srv import Grip

STATUS_TOPIC = "onrobot_2fg7_status"
GRIP_SERVICE = "grip"


class TwoFG7GraspController:

    def __init__(
        self,
        node: Node,
        open_gap_mm: float = 70.0,
        close_gap_mm: float = 20.0,
        force_n: int = 50,
        speed_pct: int = 50,
        gripper_id: int = 0,
    ):
        self._node = node
        self.open_gap_mm = open_gap_mm
        self.close_gap_mm = close_gap_mm
        self.force_n = force_n
        self.speed_pct = speed_pct
        self.gripper_id = gripper_id

        self._client = node.create_client(Grip, GRIP_SERVICE)
        self._status = None
        node.create_subscription(Status, STATUS_TOPIC, self._on_status, 10)

    def _on_status(self, msg: Status):
        self._status = msg

    def _wait_for_first_status(self, timeout_s: float):
        deadline = time.time() + timeout_s
        while self._status is None and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def attach(self, timeout_s: float = 15.0) -> bool:
        return self._grip_to(self.close_gap_mm, "attach (close)", timeout_s)

    def detach(self, timeout_s: float = 15.0) -> bool:
        return self._grip_to(self.open_gap_mm, "detach (open)", timeout_s)

    def _grip_to(self, gap_mm: float, label: str, timeout_s: float) -> bool:
        self._node.get_logger().info(
            f"grasp: requesting {label}, gap={gap_mm}mm force={self.force_n}N speed={self.speed_pct}%"
        )

        # Nothing in this codebase validated gap_mm against what the
        # gripper can physically do before this - it was just sent
        # straight to the service, with no idea what an out-of-range
        # request actually does on this firmware (clamp? error? something
        # worse?), never confirmed against real hardware. The Status
        # message reports the device's OWN live range for external-grip
        # mode (min_external_width/max_external_width - "external" because
        # grip.py's underlying call is twofg_grip_external), so check
        # against that instead of trusting the configured constant blindly.
        if self._status is None:
            self._wait_for_first_status(timeout_s=min(timeout_s, 5.0))
        if self._status is not None:
            lo, hi = self._status.min_external_width, self._status.max_external_width
            if not (lo <= gap_mm <= hi):
                self._node.get_logger().error(
                    f"grasp: requested gap {gap_mm}mm is outside the gripper's own reported "
                    f"external-grip range [{lo:.1f}, {hi:.1f}]mm - refusing to send, fix "
                    "open_gap_mm/close_gap_mm rather than find out what happens"
                )
                return False
        else:
            self._node.get_logger().warn(
                f"grasp: no {STATUS_TOPIC} received yet - can't verify {gap_mm}mm is within the "
                "gripper's physical range, proceeding anyway, watch the hardware"
            )

        if not self._client.wait_for_service(timeout_sec=timeout_s):
            self._node.get_logger().error(
                f"grasp: '{GRIP_SERVICE}' service not available - is onrobot_2fg7's grip node running?"
            )
            return False

        request = Grip.Request()
        request.gap = float(gap_mm)
        request.id = self.gripper_id
        request.force = self.force_n
        request.speed = self.speed_pct
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            self._node.get_logger().error(f"grasp: {label} service call did not complete")
            return False
        status = future.result().status
        self._node.get_logger().info(f"grasp: {label} service accepted, status={status}")

        # The service returning just means the command was accepted, not
        # that the gripper has finished moving - wait for the status
        # topic's `busy` flag to clear before trusting it's actually done.
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._status is not None and not self._status.busy:
                break
        if self._status is None:
            self._node.get_logger().warn(
                f"grasp: no {STATUS_TOPIC} received - can't confirm the gripper actually finished "
                "moving (is the status publisher node running?), trusting the service response alone"
            )
            return status == 0
        if self._status.busy:
            self._node.get_logger().warn(f"grasp: {label} still reports busy after {timeout_s:.1f}s")
        self._node.get_logger().info(
            f"grasp: {label} settled - grip_detected={self._status.grip_detected}, "
            f"internal_width={self._status.internal_width:.1f}mm, "
            f"external_width={self._status.external_width:.1f}mm"
        )
        return status == 0

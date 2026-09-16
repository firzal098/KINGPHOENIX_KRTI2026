#!/usr/bin/env python3
"""
WebotsGripper ROS 2 Node.

Bridges ROS 2 gripper commands to Webots simulation via UDP datagrams.
Stores the boolean `opened` state and immediately sends UDP packets
("true" or "false") to the Webots simulation target port (default: 5504).
"""

import socket
import sys
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class WebotsGripper(Node):
    """
    ROS 2 Node for controlling the Webots Gripper via UDP.

    Manages the `opened` state (bool) and sends UDP commands to Webots.
    """

    def __init__(self):
        super().__init__('webots_gripper')

        # Declare parameters with defaults
        self.declare_parameter('target_ip', '127.0.0.1')
        self.declare_parameter('target_port', 5504)
        self.declare_parameter('initial_state', False)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('repeat_count', 1)

        # Read parameters
        self._target_ip = self.get_parameter('target_ip').get_parameter_value().string_value
        self._target_port = self.get_parameter('target_port').get_parameter_value().integer_value
        self._opened = self.get_parameter('initial_state').get_parameter_value().bool_value
        self._repeat_count = max(1, self.get_parameter('repeat_count').get_parameter_value().integer_value)
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Initialize UDP socket (reusable, pre-allocated for lowest latency)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)

        # QoS profile for state (transient local + reliable so latched state is available to new subscribers)
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publishers
        self._state_pub = self.create_publisher(Bool, '/gripper/state', state_qos)

        # Subscribers
        self._cmd_sub = self.create_subscription(
            Bool,
            '/gripper/command',
            self._command_callback,
            10
        )

        # Services
        self._set_state_srv = self.create_service(
            SetBool,
            '/gripper/set_state',
            self._set_state_callback
        )
        self._open_srv = self.create_service(
            Trigger,
            '/gripper/open',
            self._open_callback
        )
        self._close_srv = self.create_service(
            Trigger,
            '/gripper/close',
            self._close_callback
        )
        self._toggle_srv = self.create_service(
            Trigger,
            '/gripper/toggle',
            self._toggle_callback
        )

        # Timer for periodic state publication
        if publish_rate > 0.0:
            self._publish_timer = self.create_timer(1.0 / publish_rate, self._publish_state)
        else:
            self._publish_timer = None

        # Synchronize initial state to Webots and publish
        self._send_udp(self._opened)
        self._publish_state()

        self.get_logger().info(
            f"WebotsGripper node initialized. Target: {self._target_ip}:{self._target_port}, "
            f"Initial state: opened={self._opened}"
        )

    @property
    def opened(self) -> bool:
        """Return the current boolean opened state."""
        return self._opened

    def set_state(self, open_state: bool) -> Tuple[bool, str]:
        """
        Immediately updates state, transmits UDP command, and publishes new state.

        :param open_state: True to open the gripper, False to close.
        :return: Tuple of (success, status_message).
        """
        self._opened = bool(open_state)
        success, msg = self._send_udp(self._opened)

        # Publish immediately so any listening node or UI gets the update without delay
        self._publish_state()

        state_str = "OPENED" if self._opened else "CLOSED"
        if success:
            self.get_logger().info(f"Gripper state updated to {state_str} (sent to {self._target_ip}:{self._target_port})")
            return True, f"Gripper successfully set to {state_str}"
        else:
            self.get_logger().error(f"Failed to transmit UDP command to {self._target_ip}:{self._target_port}: {msg}")
            return False, f"Failed to set gripper state: {msg}"

    def _send_udp(self, state: bool) -> Tuple[bool, str]:
        """
        Send UDP datagram b"true" or b"false" to the target host and port.

        :param state: Boolean opened state.
        :return: Tuple of (success, error_message).
        """
        payload = b"true" if state else b"false"
        try:
            for _ in range(self._repeat_count):
                self._socket.sendto(payload, (self._target_ip, self._target_port))
            return True, ""
        except Exception as e:
            return False, str(e)

    def _publish_state(self):
        """Publish the current gripper state to /gripper/state."""
        msg = Bool()
        msg.data = self._opened
        self._state_pub.publish(msg)

    # ------------------ ROS Callbacks ------------------

    def _set_state_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Handle /gripper/set_state service request."""
        success, message = self.set_state(request.data)
        response.success = success
        response.message = message
        return response

    def _open_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Handle /gripper/open trigger service."""
        success, message = self.set_state(True)
        response.success = success
        response.message = message
        return response

    def _close_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Handle /gripper/close trigger service."""
        success, message = self.set_state(False)
        response.success = success
        response.message = message
        return response

    def _toggle_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Handle /gripper/toggle trigger service."""
        success, message = self.set_state(not self._opened)
        response.success = success
        response.message = message
        return response

    def _command_callback(self, msg: Bool):
        """Handle incoming command topic /gripper/command."""
        self.set_state(msg.data)

    def destroy_node(self):
        """Clean up resources on node destruction."""
        try:
            self._socket.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebotsGripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


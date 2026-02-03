import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class QosListener(Node):
    def __init__(self):
        super().__init__('qos_listener_py')

        # Parameters (match C++ experiment)
        self.declare_parameter('depth', 10)
        self.declare_parameter('reliable', True)
        self.declare_parameter('sub_sleep_ms', 20)  # experiment knob

        depth = int(self.get_parameter('depth').value)
        reliable = bool(self.get_parameter('reliable').value)
        sub_sleep_ms = int(self.get_parameter('sub_sleep_ms').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
        )

        self.sub_sleep_sec = sub_sleep_ms / 1000.0
        self.last = None

        self.sub = self.create_subscription(String, 'chatter', self.on_msg, qos)

        self.get_logger().info(
            f"Listener started (reliable={reliable}, depth={depth}, sub_sleep_ms={sub_sleep_ms})"
        )

    def on_msg(self, msg: String):
        # Optional: detect drops by checking sequence gaps
        try:
            n = int(msg.data.split()[-1])
            if self.last is not None and n != self.last + 1:
                self.get_logger().warn(f"DROP? expected {self.last + 1} but got {n}")
            self.last = n
        except Exception:
            pass

        self.get_logger().info(f"I heard: '{msg.data}'")

        # Make subscriber intentionally slow (to reproduce drop vs delay)
        time.sleep(self.sub_sleep_sec)


def main():
    rclpy.init()
    node = QosListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


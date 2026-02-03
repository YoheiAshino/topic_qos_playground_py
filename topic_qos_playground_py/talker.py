import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class QosTalker(Node):
    def __init__(self):
        super().__init__('qos_talker_py')

        # Parameters (match C++ experiment)
        self.declare_parameter('depth', 10)
        self.declare_parameter('reliable', True)
        self.declare_parameter('pub_period_ms', 10)  # experiment knob

        depth = int(self.get_parameter('depth').value)
        reliable = bool(self.get_parameter('reliable').value)
        pub_period_ms = int(self.get_parameter('pub_period_ms').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
        )

        self.pub = self.create_publisher(String, 'chatter', qos)
        self.count = 0

        self.timer = self.create_timer(pub_period_ms / 1000.0, self.on_timer)

        self.get_logger().info(
            f"Talker started (reliable={reliable}, depth={depth}, pub_period_ms={pub_period_ms})"
        )

    def on_timer(self):
        msg = String()
        msg.data = f"hello {self.count}"
        self.count += 1
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = QosTalker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


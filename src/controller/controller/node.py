from rclpy.node import Node
from geometry_msgs.msg import Twist


class ControlNode(Node):
    def __init__(self):
        super().__init__(node_name="controller", parameter_overrides=[])
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)

        self.control_timer = self.create_timer(0.1, self.control_callback)
        self.clock = self.get_clock()

    def now(self):
        secs, nsecs = self.clock.now().seconds_nanoseconds()
        return secs + nsecs / 1000000000

    def control_callback(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.5
        self.cmd_vel.publish(twist)

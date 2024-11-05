import math
from dataclasses import dataclass
from enum import Enum

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


@dataclass
class GearConfig:
    max_linear: float  # m/s
    max_angular: float  # rad/s


class Gears(Enum):
    FIRST = GearConfig(0.1, 0.07)
    SECOND = GearConfig(0.2, 0.15)
    THIRD = GearConfig(0.5, 0.5)


GEARS_LIST = list(Gears)


class RadiolinkNode(Node):
    def __init__(self):
        super().__init__(node_name="radiolink", parameter_overrides=[])
        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("max_angular_vel", 2.0)
        self.declare_parameter("threshold", 0.02)

        self.gear = Gears.FIRST
        self._is_on: bool = False
        self.last_gear_signal = None
        self.last_reset_signal = None
        self.last_mode_signal = None

        self.twist = Twist()
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        self.cmd_vel_timer = self.create_timer(0.02, self.send_cmd_vel)

    @property
    def max_linear_speed(self):
        return self.gear.value.max_linear

    @property
    def max_angular_speed(self):
        return self.gear.value.max_angular

    @property
    def is_on(self) -> bool:
        return self._is_on

    @is_on.setter
    def is_on(self, value):
        if isinstance(value, bool):
            self._is_on = value
        old_is_on = self._is_on
        self._is_on = value == 2
        if self._is_on != old_is_on:
            self.send_indicators()

    def send_reset(self):
        # TODO: some sort of reset?
        pass

    def send_indicators(self):
        # TODO: some sort of indication?
        if self._is_on:
            pass
        else:
            pass

    def set(self, gear, mode):
        if gear != 0:
            self.gear = GEARS_LIST[gear - 1]
        if mode != 0:
            self.is_on = mode

        self.get_logger().info(f"is_on: {self._is_on}, gear: {self.gear.name}")

    @staticmethod
    def parse_signal(signal, intervals=(1, 2, 3)):
        if signal is None:
            return 0
        res_sig = 1
        if -0.01 < signal < 0.01:
            res_sig = intervals[1]
        elif signal < 0:
            res_sig = intervals[0]
        else:
            res_sig = intervals[2]
        return res_sig

    def process_state(self, gear_signal, mode_signal):
        gear = self.parse_signal(gear_signal)
        mode = self.parse_signal(mode_signal, (1, 1, 2))

        try:
            self.set(gear=gear, mode=mode)
        except Exception as e:
            self.get_logger().error("Drive state call failed: %s" % e)

    def now(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000

    def joy_callback(self, data: Joy):
        gear_signal = data.axes[6]
        mode_signal = data.axes[4]
        reset_signal = data.axes[5]

        should_update_state = False

        if gear_signal != self.last_gear_signal:
            self.last_gear_signal = gear_signal
            should_update_state = True
        else:
            gear_signal = None

        if mode_signal != self.last_mode_signal:
            self.last_mode_signal = mode_signal
            should_update_state = True
        else:
            mode_signal = None

        if should_update_state:
            self.process_state(gear_signal, mode_signal)

        if reset_signal != self.last_reset_signal:
            self.last_reset_signal = reset_signal
            if reset_signal < 0:
                self.send_reset()

        threshold = self.get_parameter("threshold").get_parameter_value().double_value
        max_linear_vel = self.get_parameter("max_linear_vel").get_parameter_value().double_value
        max_angular_vel = self.get_parameter("max_angular_vel").get_parameter_value().double_value

        linear_x = 0
        if abs(data.axes[2]) > threshold:
            linear_x = -math.copysign(min(abs(data.axes[2]), max_linear_vel), data.axes[2]) * self.max_linear_speed

        angular_z = 0
        if abs(data.axes[0]) > threshold:
            angular_z = -math.copysign(min(abs(data.axes[0]), max_angular_vel), data.axes[0]) * self.max_angular_speed

        self.set_cmd_vel(linear_x, angular_z)

    def set_cmd_vel(self, linear_x, angular_z):
        self.twist.linear.x = float(linear_x)
        self.twist.angular.z = float(angular_z)

    def send_cmd_vel(self):
        if self.is_on:
            self.cmd_vel.publish(self.twist)

import rclpy
import rclpy.executors

from .node import RadiolinkNode


def main():
    rclpy.init()
    node = RadiolinkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()


if __name__ == "__main__":
    main()

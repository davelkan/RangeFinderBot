from more_msgs.msg import EstopStamped

import rclpy
from rclpy.node import Node


def main(estopped: bool = True):
    rclpy.init()
    node = Node()
    estop_publisher = node.create_publisher(EstopStamped, "estop", 10)
    estop_data = EstopStamped()
    estop_data.header.frame_id = "rangebot"
    estop_data.header.stamp = node.get_clock().now().to_msg()
    estop_data.estopped = estopped
    estop_publisher.publish(estop_data)
    node.destroy_node()
    rclpy.shutdown()


def publish_true():
    main(True)


def publish_false():
    main(False)


if __name__ == "__main__":
    main()

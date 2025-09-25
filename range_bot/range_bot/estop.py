from more_msgs.msg import EstopStamped

import rclpy
from rclpy.node import Node


def main(estopped: bool = True):
    rclpy.init()
    node = Node("estop_trigger")
    estop_publisher = node.create_publisher(EstopStamped, "estop", 10)
    estop_data = EstopStamped()
    estop_data.header.frame_id = "rangebot"
    estop_data.header.stamp = node.get_clock().now().to_msg()
    estop_data.estop_active = estopped
    estop_publisher.publish(estop_data)
    node.get_logger().info(f"estop value: {estopped}")
    node.destroy_node()
    rclpy.shutdown()


def publish_true():
    main(estopped=True)


def publish_false():
    main(estopped=False)


if __name__ == "__main__":
    main()

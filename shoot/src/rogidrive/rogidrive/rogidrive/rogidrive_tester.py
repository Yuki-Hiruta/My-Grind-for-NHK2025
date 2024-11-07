import rclpy
import rclpy.node
from rogidrive_msg.msg import RogidriveMessage


# publish rogidrivemessage to topic 'odrive_cmd'
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rogidrive_tester')
    pub = node.create_publisher(RogidriveMessage, 'odrive_cmd', 10)
    msg = RogidriveMessage()

    while True:
        msg.name = "TEST"
        msg.mode = int(input("input mode: "))
        if msg.mode == 0:
            msg.vel = float(input("input velocity: "))
        else:
            msg.vel = float(input("input vel limit: "))
            msg.pos = float(input("input position: "))

        # node.get_logger().info('Publishing: "%s"' % msg)
        pub.publish(msg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

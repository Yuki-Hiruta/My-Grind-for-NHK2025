import rclpy
from rclpy.node import Node
from rogilink_flex_lib import Publisher
from std_msgs.msg import Float64MultiArray, c_float

class Rogiflex(Node):
    def __init__(self):
        super().__init__('rogiflex_shoot')
        self.sub_goal = self.create_subscription(Float64MultiArray, 'shootVelocity', self.velocity_callback, 10)
        self.pub_velocity = Publisher(self, 'velocity', (c_float, c_float, c_float))

    def velocity_callback(self, msg):
        self.velocity = msg.data
        self.pub_goal.publish((float(self.velocity[0]), float(self.velocity[1])))
        # self.get_logger().info('Publishing goal: "%lf, %lf"' % (self.goal[0], self.goal[1]))

def main(args=None):
    rclpy.init(args=args)

    rogiflex_shoot = Rogiflex()

    rclpy.spin(rogiflex_shoot)

    rogiflex_shoot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rogilink_flex_lib import Publisher
from std_msgs.msg import Float64MultiArray, c_float

class Rogiflex(Node):
    def __init__(self):
        super().__init__('rogiflex_shoot')
        self.sub_goal = self.create_subscription(Float64MultiArray, 'shootVelocity', self.goal_callback, 10)
        self.pub_velocity = Publisher(self, 'velocity', (c_float, c_float c_float))

    def goal_callback(self, msg):
        self.velocity = msg.data
        self.pub_goal.publish((float(self.goal[0]), float(self.goal[1])))
        # self.get_logger().info('Publishing goal: "%lf, %lf"' % (self.goal[0], self.goal[1]))
    


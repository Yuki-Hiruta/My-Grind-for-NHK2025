import odrive
import rclpy
import rclpy.node
import json
import time
from std_msgs.msg import Bool
from rogidrive_msg.msg import RogidriveMessage
from rogidrive_msg.msg import RogidriveMultiArray

AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_IDLE = 1
VELOCITY_CONTROL = 2
POSITION_CONTROL = 3


class Rogidrive(rclpy.node.Node):

    def __init__(self):
        super().__init__('rogidrive')
        self.subscription = self.create_subscription(RogidriveMessage,
                                                     'odrive_cmd',
                                                     self.cmd_callback, 100)
        self.odrive_enable_sub = self.create_subscription(Bool, 'odrive_enable', self.odrive_enable_callback, 100)
        self.pub = self.create_publisher(
            RogidriveMultiArray, 'odrive_status', 100)
        self.status = RogidriveMultiArray()
        self.declare_parameter('config_path', 'default_value')
        self.config_path = self.get_parameter(
            'config_path').get_parameter_value().string_value
        self.is_calibrated = False

        with open(self.config_path, 'r') as json_file:
            self.config = json.load(json_file)['motors']

        self.connect_odrive()

        i = 0
        for axis in self.config:
            self.status.data.append(RogidriveMessage())
            i += 1
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def odrive_enable_callback(self, msg):
        self.calibration_check()
        if msg.data:
            if self.is_calibrated:   
                self.set_closed_loop()
                self.get_logger().info("entered closed loop")
            else:
                self.get_logger().error("Not Calibrated!")
            

    def connect_odrive(self):
        print(self.config)
        serial_set = set()
        for motor in self.config.items():
            print(motor[1]['serial'])
            serial_set.add(motor[1]['serial'])

        print(serial_set)
        self.devs = []
        print("connecting...")
        for serial in serial_set:
            self.devs.append(odrive.find_any(serial_number=serial))
            for motor in self.config.items():
                if motor[1]['serial'] == serial:
                    motor[1]['mode'] = 0
                    if motor[1]['axis'] == 0:
                        motor[1]['dev'] = self.devs[-1].axis0
                    elif motor[1]['axis'] == 1:
                        motor[1]['dev'] = self.devs[-1].axis1
            print(str(serial)+" connected")

        print(self.config)
        # print("connected")
        # print(devs)
        
    def calibration_check(self):
        num = 0
        self.get_logger().info("calibration check")
        for dev in self.devs:
            odrive.utils.dump_errors(dev)
        for axis in self.config:
            num += int(self.config[axis]['dev'].encoder.is_ready)
        self.get_logger().info("Calibrated: %s" % num)
        if num == len(self.config):
            self.is_calibrated = True
        else:
            self.is_calibrated = False
        
    def set_closed_loop(self):
        for axis in self.config:
            self.config[axis]['dev'].encoder.set_linear_count(0)
            time.sleep(0.1)
            self.config[axis]['dev'].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            # self.config[axis]['dev'].controller.config.control_mode = VELOCITY_CONTROL
            self.config[axis]['dev'].controller.config.input_mode = 1
            self.config[axis]['dev'].controller.input_vel = 0
            self.config[axis]['dev'].controller.input_pos = 0
            self.config[axis]['mode'] = 0
            self.get_logger().info("Closed loop set %s" % axis)
    
    
    def cmd_callback(self, cmd_msg):
        # self.get_logger().info('Received: "%s"' % cmd_msg)
        try:
            if cmd_msg.mode == 0:
                if cmd_msg.mode != self.config[cmd_msg.name]['mode']:
                    self.config[cmd_msg.name]['dev'].controller.config.control_mode = VELOCITY_CONTROL
                    self.config[cmd_msg.name]['mode'] = cmd_msg.mode
                self.config[cmd_msg.name]['dev'].controller.input_vel = cmd_msg.vel
            elif cmd_msg.mode == 1:
                if cmd_msg.mode != self.config[cmd_msg.name]['mode']:
                    self.config[cmd_msg.name]['dev'].controller.config.control_mode = POSITION_CONTROL
                    self.config[cmd_msg.name]['dev'].controller.config.input_mode = 5
                    self.config[cmd_msg.name]['mode'] = cmd_msg.mode
                self.config[cmd_msg.name]['dev'].controller.config.vel_limit = cmd_msg.vel
                self.config[cmd_msg.name]['dev'].trap_traj.config.vel_limit = cmd_msg.vel
                self.config[cmd_msg.name]['dev'].controller.input_pos = cmd_msg.pos
        except Exception as e:
            self.get_logger().error("Error: %s" % e)
            self.connect_odrive()
            

    def timer_callback(self):
        i = 0
        for axis in self.config:
            try:
                self.status.data[i].name = axis
                self.status.data[i].pos = self.config[axis]['dev'].encoder.pos_estimate
                self.status.data[i].vel = self.config[axis]['dev'].encoder.vel_estimate
                self.status.data[i].current = self.config[axis]['dev'].motor.current_control.Iq_measured
                i += 1
            except Exception as e:
                self.get_logger().error("Error: %s" % e)
                self.connect_odrive()
                # self.set_closed_loop()
        self.pub.publish(self.status)

    def __del__(self):
        pass
        # for axis in self.config:
        #     self.config[axis]['dev'].requested_state = AXIS_STATE_IDLE
        #     self.config[axis]['dev'].controller.input_vel = 0


def main():
    rclpy.init()
    rogidrive = Rogidrive()
    rclpy.spin(rogidrive)


if __name__ == '__main__':
    main()

from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from time import time

class cmdvel_to_wheelvel(Node):

    def __init__(self):
        super().__init__('cmdvel_to_wheelvel_node')

        # Declare Parameters
        self.declare_parameter('wheel_radius', 0.1524)
        self.declare_parameter('robot_width', 0.71755)

        # Read Parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.robot_width = self.get_parameter('robot_width').value

        # Create a Subscriber to subscibe to cmd_vel
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.subscription  # prevent unused variable warning

        # Create Publishers to publish encoder rpm
        self.motor_cmd_publisher = self.create_publisher(JointState, 'motor/cmd', 5)

        #last cmd_vel received
        self.last_cmd_vel_received = 0
        self.watchdog_timer = self.create_timer(1.0, self.cmd_vel_watchdog)

    def cmd_vel_watchdog(self):
        current_time = time()
        elapsed_time = current_time - self.last_cmd_vel_received

        if elapsed_time > 5.0:
            motor_cmd = JointState()
            motor_cmd.name = ["fr_motor", "fl_motor", "rr_motor", "rl_motor"]
            motor_cmd.velocity = [0.0, 0.0, 0.0, 0.0]
            motor_cmd.header.stamp = self.get_clock().now().to_msg()
            motor_cmd.header.frame_id = "" # TODO: fill this later

            # Publish
            self.motor_cmd_publisher.publish(motor_cmd)


    def cmd_vel_callback(self, twist):

        self.last_cmd_vel_received = time()

        motor_cmd = JointState()
        motor_cmd.name = ["fr_motor", "fl_motor", "rr_motor", "rl_motor"]
        
        twist.angular.z = -1 * twist.angular.z

        # Calculate individual wheel velocities from the cmd_vel
        left_speed =  ((2*twist.linear.x) - (twist.angular.z*self.robot_width)) / (2*self.wheel_radius) #rad/s
        right_speed =  ((2*twist.linear.x) + (twist.angular.z*self.robot_width)) / (2*self.wheel_radius)
        left_rpm = (left_speed / (2.0*pi)) * 60.0  # rad/s to rpm
        right_rpm = (right_speed / (2.0*pi)) * 60.0

        motor_cmd.velocity = [right_rpm, left_rpm, right_rpm, left_rpm]
        motor_cmd.header.stamp = self.get_clock().now().to_msg()
        motor_cmd.header.frame_id = "" # TODO: fill this later

        # Publish
        self.motor_cmd_publisher.publish(motor_cmd)



    
def main(args=None):
    rclpy.init(args=args)

    cmdvel_to_wheelvel_node = cmdvel_to_wheelvel()

    rclpy.spin(cmdvel_to_wheelvel_node)

    cmdvel_to_wheelvel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

from math import pi, cos, sin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
# import tf_transformations
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np

class cmdvel_to_wheelvel(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # Declare Parameters
        self.declare_parameter('ticks_meter', 28.008763607117015) # The number of wheel encoder ticks per meter of travel = ticks per rotation / (2*pi*r)
        self.declare_parameter('base_width', 0.71755) # The wheel base width in meters
        self.declare_parameter('encoder_min', -34000000000000000000000000000000000000.0) # float32 min
        self.declare_parameter('encoder_max', 34000000000000000000000000000000000000.0)  # float32 max
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_tf', True)

        # Read Parameters
        self.ticks_meter = self.get_parameter('ticks_meter').value
        self.base_width = self.get_parameter('base_width').value
        self.encoder_min = self.get_parameter('encoder_min').value
        self.encoder_max = self.get_parameter('encoder_max').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.pub_tf = self.get_parameter('publish_tf').value

        # Create a Subscriber to subscibe to cmd_vel
        self.subscription = self.create_subscription(JointState, 'motor/status', self.motor_feedback_callback, 10)
        self.subscription  # prevent unused variable warning

        # Create a Publisher to publish Odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # internal data
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min 
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min 
        self.enc_front_left = None        # wheel encoder readings
        self.enc_front_right = None
        self.enc_rear_left = None        # wheel encoder readings
        self.enc_rear_right = None
        self.front_left = 0               # actual values coming back from robot
        self.front_right = 0
        self.rear_left = 0               # actual values coming back from robot
        self.rear_right = 0
        self.fl_mult = 0
        self.fr_mult = 0
        self.rl_mult = 0
        self.rr_mult = 0
        self.prev_fl_encoder = 0
        self.prev_fr_encoder = 0
        self.prev_rl_encoder = 0
        self.prev_rr_encoder = 0
        self.x = 0.0                  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0                 # speeds in x/rotation
        self.dr = 0.0
        self.new_dataL = False
        self.new_dataR = False
        self.then = self.get_clock().now().to_msg()
        self.then = self.then.sec + self.then.nanosec*pow(10,-9)

    def encoder_data_range_wrap(self, enc, type):
        if type == "fl":
            if (enc < self.encoder_low_wrap and self.prev_fl_encoder > self.encoder_high_wrap):
                self.fl_mult = self.fl_mult + 1
            if (enc > self.encoder_high_wrap and self.prev_fl_encoder < self.encoder_low_wrap):
                self.fl_mult = self.fl_mult - 1
            self.front_left = 1.0 * (enc + self.fl_mult * (self.encoder_max - self.encoder_min))
            self.prev_fl_encoder = enc
        elif type == "fr":
            if (enc < self.encoder_low_wrap and self.prev_fr_encoder > self.encoder_high_wrap):
                self.fr_mult = self.fr_mult + 1
            if (enc > self.encoder_high_wrap and self.prev_fr_encoder < self.encoder_low_wrap):
                self.fr_mult = self.fr_mult - 1
            self.front_right = 1.0 * (enc + self.fr_mult * (self.encoder_max - self.encoder_min))
            self.prev_fr_encoder = enc
        elif type == "rl":
            if (enc < self.encoder_low_wrap and self.prev_rl_encoder > self.encoder_high_wrap):
                self.rl_mult = self.rl_mult + 1
            if (enc > self.encoder_high_wrap and self.prev_rl_encoder < self.encoder_low_wrap):
                self.rl_mult = self.rl_mult - 1
            self.rear_left = 1.0 * (enc + self.rl_mult * (self.encoder_max - self.encoder_min))
            self.prev_rl_encoder = enc
        elif type == "rr":
            if (enc < self.encoder_low_wrap and self.prev_rr_encoder > self.encoder_high_wrap):
                self.rr_mult = self.rr_mult + 1
            if (enc > self.encoder_high_wrap and self.prev_rr_encoder < self.encoder_low_wrap):
                self.rr_mult = self.rr_mult - 1
            self.rear_right = 1.0 * (enc + self.rr_mult * (self.encoder_max - self.encoder_min))
            self.prev_rr_encoder = enc

    def motor_feedback_callback(self, state):
        # Parse the msg
        for i, motor in enumerate(state.name):
            if motor == "fr_motor":
                self.raw_enc_fr = state.position[i]
            elif motor == "fl_motor":
                self.raw_enc_fl = state.position[i]
            elif motor == "rr_motor":
                self.raw_enc_rr = state.position[i]
            elif motor == "rl_motor":
                self.raw_enc_rl = state.position[i]
            else:
                pass
        
        # Correct for datatype range overflow
        self.encoder_data_range_wrap(self.raw_enc_fl, "fl")
        self.encoder_data_range_wrap(self.raw_enc_fr, "fr")
        self.encoder_data_range_wrap(self.raw_enc_rl, "rl")
        self.encoder_data_range_wrap(self.raw_enc_rr, "rr")

        # Find the elapsed time
        now = state.header.stamp
        now = now.sec + now.nanosec*pow(10,-9)
        elapsed = now - self.then
        self.then = now

        # calculate odometry
        if self.enc_front_left == None or self.enc_front_right == None or self.enc_rear_left == None or self.enc_rear_right == None:
            d_front_left = 0
            d_front_right = 0
            d_rear_left = 0
            d_rear_right = 0
        else:
            d_front_left =   (self.front_left - self.enc_front_left) / self.ticks_meter
            d_front_right =  (self.front_right - self.enc_front_right) / self.ticks_meter
            d_rear_left =   (self.front_left - self.enc_front_left) / self.ticks_meter
            d_rear_right =  (self.front_right - self.enc_front_right) / self.ticks_meter
        self.enc_front_left = self.front_left
        self.enc_front_right = self.front_right
        self.enc_rear_left = self.rear_left
        self.enc_rear_right = self.rear_right

        # distance traveled is the average of the two wheels
        d_left = (d_front_left + d_rear_left) / 2.0
        d_right = (d_front_right + d_rear_right) / 2.0
        d = -1.0 * ( d_left + d_right ) / 2.0
        # this approximation works (in radians) for small angles
        th = ( d_right - d_left ) / self.base_width
        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed

        if (d != 0):
            # calculate distance traveled in x and y
            x = cos( th ) * d
            y = -sin( th ) * d
            # calculate the final position of the robot
            self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
            self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
        if( th != 0):
            self.th = self.th + th

        # Publish tf
        if self.pub_tf:
            self.publish_tf()

        # Publish Odometry
        self.publish_odom()

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
    
    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = self.quaternion_from_euler(0, 0, self.th)
        # Convert a list to geometry_msgs.msg.Quaternion
        msg_quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom_msg.pose.pose.orientation = msg_quat
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.twist.twist.linear.x = self.dx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.dr
        self.odom_publisher.publish(odom_msg)

    def publish_tf(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id       
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = self.quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.br.sendTransform(t)

    
def main(args=None):
    rclpy.init(args=args)

    cmdvel_to_wheelvel_node = cmdvel_to_wheelvel()

    rclpy.spin(cmdvel_to_wheelvel_node)

    cmdvel_to_wheelvel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
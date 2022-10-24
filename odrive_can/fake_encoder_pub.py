import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class fakeEncoder(Node):

    def __init__(self):
        super().__init__('fake_encoder')

        self.feedback_publisher = self.create_publisher(JointState, 'motor/status', 10)
        timer_period = 1
        self.feedback_publisher_timer = self.create_timer(timer_period, self.feedback_publisher_callback)

        self.initial_pos = [0.0, 0.0, 0.0, 0.0]

    def feedback_publisher_callback(self):
        # Create the JointState msg
        feedback_msg = JointState()
        feedback_msg.name = ["fr_motor", "fl_motor", "rr_motor", "rl_motor"]
        self.initial_pos = [x+1.0 for x in self.initial_pos]
        feedback_msg.position = self.initial_pos

        # Populate the msg header
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.header.frame_id = "" # TODO: fill this later

        # Publish
        self.feedback_publisher.publish(feedback_msg)


def main(args=None):
    rclpy.init(args=args)

    odrive_can_node = fakeEncoder()

    rclpy.spin(odrive_can_node)

    odrive_can_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

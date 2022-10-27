import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
import can
import cantools
import time

class OdriveCAN(Node):

    def __init__(self):
        super().__init__('odrive_can')

        # Declare Parameters
        self.declare_parameter('axis_id_fr_motor', 0)
        self.declare_parameter('axis_id_fl_motor', 1)
        self.declare_parameter('axis_id_rr_motor', 2)
        self.declare_parameter('axis_id_rl_motor', 3)
        self.declare_parameter('dbc_file_path', 'odrive-cansimple.dbc')
        self.declare_parameter('gear_reduction_factor', 26.82)

        # Read Parameters
        self.axis_id_fr_motor = self.get_parameter('axis_id_fr_motor').value
        self.axis_id_fl_motor = self.get_parameter('axis_id_fl_motor').value
        self.axis_id_rr_motor = self.get_parameter('axis_id_rr_motor').value
        self.axis_id_rl_motor = self.get_parameter('axis_id_rl_motor').value
        self.dbc_file_path = self.get_parameter('dbc_file_path').value
        self.gear_reduction_factor = self.get_parameter('gear_reduction_factor').value

        # Instantiate the CAN bus
        self.db = cantools.database.load_file(self.dbc_file_path)
        self.bus = can.Bus("can0", bustype="socketcan")

        # Calibration Routine
        self.odrive_initial_setup(self.axis_id_fr_motor)
        self.odrive_initial_setup(self.axis_id_fl_motor)
        self.odrive_initial_setup(self.axis_id_rr_motor)
        self.odrive_initial_setup(self.axis_id_rl_motor)

        # Create a Publisher to publish encoder rpm
        self.feedback_publisher = self.create_publisher(JointState, 'motor/status', 10)
        timer_period = 0.009 # 0.009 seconds = 110 Hz
        self.feedback_publisher_timer = self.create_timer(timer_period, self.feedback_publisher_callback)

        # Create a Publisher to publish motor diagnostics
        self.diagnostics_publisher = self.create_publisher(Int32MultiArray, 'motor/diagnostics', 10)

        # Create a Subscriber to subscibe to velocity command
        self.subscription = self.create_subscription(JointState, 'motor/cmd', self.motor_cmd_subscriber_callback, 10)
        self.subscription  # prevent unused variable warning

    def motor_cmd_subscriber_callback(self, cmd):
        i = 0
        for motor in cmd.name:
            if motor == "fr_motor":
                rpm = cmd.velocity[i]
                rps = (rpm / 60.0) * self.gear_reduction_factor * -1.0
                data = self.db.encode_message('Set_Input_Vel', {'Input_Vel':rps, 'Input_Torque_FF':100.0})
                msg = can.Message(arbitration_id=self.axis_id_fr_motor << 5 | 0x00D, data=data, is_extended_id=False)
                self.bus.send(msg)
                i = i + 1
            elif motor == "fl_motor":
                rpm = cmd.velocity[i]
                rps = (rpm / 60.0) * self.gear_reduction_factor * -1.0
                data = self.db.encode_message('Set_Input_Vel', {'Input_Vel':rps, 'Input_Torque_FF':100.0})
                msg = can.Message(arbitration_id=self.axis_id_fl_motor << 5 | 0x00D, data=data, is_extended_id=False)
                self.bus.send(msg)
                i = i + 1
            elif motor == "rr_motor":
                rpm = cmd.velocity[i]
                rps = (rpm / 60.0) * self.gear_reduction_factor
                data = self.db.encode_message('Set_Input_Vel', {'Input_Vel':rps, 'Input_Torque_FF':100.0})
                msg = can.Message(arbitration_id=self.axis_id_rr_motor << 5 | 0x00D, data=data, is_extended_id=False)
                self.bus.send(msg)
                i = i + 1
            elif motor == "rl_motor":
                rpm = cmd.velocity[i]
                rps = (rpm / 60.0) * self.gear_reduction_factor
                data = self.db.encode_message('Set_Input_Vel', {'Input_Vel':rps, 'Input_Torque_FF':100.0})
                msg = can.Message(arbitration_id=self.axis_id_rl_motor << 5 | 0x00D, data=data, is_extended_id=False)
                self.bus.send(msg)
                i = i + 1
            else:
                pass

    def feedback_publisher_callback(self):
        # Create the JointState msg
        feedback_msg = JointState()
        feedback_msg.name = ["fr_motor", "fl_motor", "rr_motor", "rl_motor"]
        feedback_msg.position = [0.0, 0.0, 0.0, 0.0]
        feedback_msg.velocity = [0.0, 0.0, 0.0, 0.0]

        # Flags to check if we had received data from all four motors
        flag_fl = False
        flag_fr = False
        flag_rl = False
        flag_rr = False

        # Check the ID and populate the position and velocity fields
        while not (flag_fl and flag_fr and flag_rl and flag_rr):
            msg = self.bus.recv()
            cmd_id = msg.arbitration_id & 31 # last 5 bits
            node_id = (msg.arbitration_id & 2016) >> 5 # first 6 bits

            if cmd_id == 9:
                encoder_estimates = self.db.decode_message('Get_Encoder_Estimates', msg.data)
                vel_estimate = encoder_estimates['Vel_Estimate'] * 60.0 / self.gear_reduction_factor
                pos_estimate = encoder_estimates['Pos_Estimate']
                if node_id == 0:
                    feedback_msg.velocity[0] = vel_estimate
                    feedback_msg.position[0] = pos_estimate
                    flag_fr = True
                elif node_id == 1:
                    feedback_msg.velocity[1] = vel_estimate
                    feedback_msg.position[1] = pos_estimate
                    flag_fl = True
                elif node_id == 2:
                    feedback_msg.velocity[2] = vel_estimate
                    feedback_msg.position[2] = pos_estimate
                    flag_rr = True
                elif node_id == 3:
                    feedback_msg.velocity[3] = vel_estimate
                    feedback_msg.position[3] = pos_estimate
                    flag_rl = True
                else:
                    self.get_logger().warn("Unkown Node ID %s" % node_id)

            elif cmd_id == 1:
                    error_code = self.db.decode_message('Heartbeat', msg.data)['Axis_Error']
                    axis_current_state = self.db.decode_message('Heartbeat', msg.data)['Axis_State']
                    motor_state = self.db.decode_message('Heartbeat', msg.data)['Motor_Flags']
                    encoder_state = self.db.decode_message('Heartbeat', msg.data)['Encoder_Flags']
                    controller_status = self.db.decode_message('Heartbeat', msg.data)['Controller_Flags']

                    diag_msg = Int32MultiArray()
                    diag_msg.data = [node_id, error_code, axis_current_state, controller_status, motor_state, encoder_state]
                    self.diagnostics_publisher.publish(diag_msg)

            else:
                self.get_logger().warn("Unkown Cmd ID %s" % cmd_id)

        # Populate the msg header
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.header.frame_id = "" # TODO: fill this later

        # Publish
        self.feedback_publisher.publish(feedback_msg)

    def odrive_initial_setup(self, axisID):
        # Transition to MOTOR_CALIBRATION state
        self.get_logger().info('Transitioning Axis %s into MOTOR_CALIBRATION (0x04)' % axisID)
        msg = self.db.get_message_by_name('Set_Axis_State')
        data = msg.encode({'Axis_Requested_State': 0x04})
        msg = can.Message(arbitration_id=msg.frame_id | axisID << 5, is_extended_id=False, data=data)

        try:
            self.bus.send(msg)
            # print("Message sent on {}".format(self.bus.channel_info))
        except can.CanError:
            # print("Message NOT sent!  Please verify can0 is working first")
            pass # TODO: Handle these cases

        self.get_logger().info("Waiting for calibration to finish for Axis %s... " % axisID)
        # Read messages infinitely and wait for the right ID to show up
        while True:
            msg = self.bus.recv()
            if msg.arbitration_id == ((axisID << 5) | self.db.get_message_by_name('Heartbeat').frame_id):
                current_state = self.db.decode_message('Heartbeat', msg.data)['Axis_State']
                if current_state == 0x1:
                    self.get_logger().info("Calibration Done. Axis %s has returned to Idle state." % axisID)
                    break

        # Check for any errors
        for msg in self.bus:
            if msg.arbitration_id == ((axisID << 5) | self.db.get_message_by_name('Heartbeat').frame_id):
                errorCode = self.db.decode_message('Heartbeat', msg.data)['Axis_Error']
                if errorCode == 0x00:
                    self.get_logger().info("No Axis errors")
                else:
                    self.get_logger().info("Axis error!  Error code: "+str(hex(errorCode)))
                break

        in_closed_loop = False
        def set_closed_loop():
            # Transition to CLOSED_LOOP_CONTROL state
            self.get_logger().info("Transitioning Axis %s into CLOSED_LOOP_CONTROL (0x08)" % axisID)
            data = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x08})
            msg = can.Message(arbitration_id=0x07 | axisID << 5, is_extended_id=False, data=data)

            try:
                self.bus.send(msg)
                # print("Message sent on {}".format(self.bus.channel_info))
            except can.CanError:
                print("Message NOT sent!") # TODO: Handle these cases


        while not in_closed_loop:

            set_closed_loop()
            # Check if axis has transitioned into CLOSED_LOOP_CONTROL
            for msg in self.bus:
                if msg.arbitration_id == 0x01 | axisID << 5:
                    msg = self.db.decode_message('Heartbeat', msg.data)
                    if msg['Axis_State'] == 0x8:
                        self.get_logger().info("Axis %s has entered closed loop" % axisID)
                        in_closed_loop = True
                    else:
                        self.get_logger().error("Axis  %s failed to enter closed loop" % axisID)
                    break



def main(args=None):
    rclpy.init(args=args)

    odrive_can_node = OdriveCAN()

    rclpy.spin(odrive_can_node)

    odrive_can_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

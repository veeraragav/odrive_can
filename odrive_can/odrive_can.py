import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import can
import cantools
import time
import threading

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

        # Create a Publisher to publish encoder rpm
        self.feedback_publisher = self.create_publisher(JointState, 'motor/status', 10)

        # Create a Publisher to publish motor diagnostics
        self.diagnostics_publisher = self.create_publisher(Int32MultiArray, 'motor/diagnostics', 10)

        # Instantiate the CAN bus
        self.db = cantools.database.load_file(self.dbc_file_path)
        self.bus = can.ThreadSafeBus("can0", bustype="socketcan")

        # locks
        self.velocity_cmd_lock = threading.Lock()

        # Create Motor objects
        self.motors = {
            "fr_motor": Motor(self.bus, self.db, self.axis_id_fr_motor, Motor.REVERSE, self.gear_reduction_factor),
            "fl_motor": Motor(self.bus, self.db, self.axis_id_fl_motor, Motor.REVERSE, self.gear_reduction_factor),
            "rr_motor": Motor(self.bus, self.db, self.axis_id_rr_motor, Motor.FORWARD, self.gear_reduction_factor),
            "rl_motor": Motor(self.bus, self.db, self.axis_id_rl_motor, Motor.FORWARD, self.gear_reduction_factor),
        }

        # configure timer to send velocity cmds to the motor
        self.velocity_can_sender_cb_group = MutuallyExclusiveCallbackGroup()
        velocity_can_sender_timer_period = 0.1 # 10 Hz
        self.velocity_can_sender_timer = self.create_timer(velocity_can_sender_timer_period, self.velocity_can_sender, callback_group=self.velocity_can_sender_cb_group)

        # configure timer to perform initial setup
        self.initial_setup_cb_group = MutuallyExclusiveCallbackGroup()        
        self.odrive_initial_setup_timer = self.create_timer(0.1, self.odrive_inital_setup_timer_callback, callback_group=self.initial_setup_cb_group)

        # create a thread to read CAN msgs and publish the data
        self.feedback_thread = threading.Thread(target=self.feedback_publisher_callback)
        self.feedback_thread.daemon = True
        self.feedback_thread.start()

        # Create a Subscriber to subscibe to velocity command
        self.motor_cmd_subscriber_cb_group = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(JointState, 'motor/cmd', self.motor_cmd_subscriber_callback, 10, callback_group=self.motor_cmd_subscriber_cb_group)
        self.subscription  # prevent unused variable warning

        #last cmd_vel received
        self.last_cmd_vel_received = 0
        self.watchdog_cb_group = MutuallyExclusiveCallbackGroup()
        self.watchdog_timer = self.create_timer(0.5, self.vel_cmd_watchdog, callback_group=self.watchdog_cb_group)

    def vel_cmd_watchdog(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_cmd_vel_received

        if elapsed_time > 0.5:
            self.get_logger().info("velocity command watchdog activated" )
            with self.velocity_cmd_lock:
                for motor_name, motor in self.motors.items():
                    motor.velocity_cmd_rpm = 0.0         

    def motor_cmd_subscriber_callback(self, cmd):
        # self.get_logger().info("motor_cmd_subscriber_callback" )
        self.last_cmd_vel_received = time.time()

        if len(cmd.name) != len(cmd.velocity):
            self.get_logger().error('Invalid command. Length of joint names'
                                    ' and commands do not match. '
                                    f'{len(cmd.name)} != {len(cmd.velocity)}')
            return

        # Else update all motors:
        with self.velocity_cmd_lock:
            for motor_name, motor_rpm in zip(cmd.name, cmd.velocity):
                try:
                    self.motors[motor_name].velocity_cmd_rpm = motor_rpm
                except KeyError as kerr:
                    # TODO: log error
                    self.get_logger().error(f'Invalid motor name: {kerr}')
                    pass
        

    def odrive_inital_setup_timer_callback(self):
        # Calibration Routine
        for motor_name, motor in self.motors.items():
            motor.odrive_initial_setup()    
        self.odrive_initial_setup_timer.cancel()

    def velocity_can_sender(self):
        with self.velocity_cmd_lock:
            for motor_name, motor in self.motors.items():
                motor.set_rpm()   

    def feedback_publisher_callback(self):
        self.get_logger().info("Starting feedback publisher thread" )

        # Create the JointState msg
        feedback_msg = JointState()
        feedback_msg.name = ["fr_motor", "fl_motor", "rr_motor", "rl_motor"]
        feedback_msg.position = [0.0, 0.0, 0.0, 0.0]
        feedback_msg.velocity = [0.0, 0.0, 0.0, 0.0]

        while True:
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
                        print(f"Unkown node ID {node_id}", flush=True)

                elif cmd_id == 1:
                        error_code = self.db.decode_message('Heartbeat', msg.data)['Axis_Error']
                        axis_current_state = self.db.decode_message('Heartbeat', msg.data)['Axis_State']
                        motor_state = self.db.decode_message('Heartbeat', msg.data)['Motor_Flags']
                        encoder_state = self.db.decode_message('Heartbeat', msg.data)['Encoder_Flags']
                        controller_status = self.db.decode_message('Heartbeat', msg.data)['Controller_Flags']

                        diag_msg = Int32MultiArray()
                        diag_msg.data = [node_id, error_code, axis_current_state, controller_status, motor_state, encoder_state]
                        self.diagnostics_publisher.publish(diag_msg)

                        # set axis state for all motors
                        if node_id == 0:
                            self.motors["fr_motor"].axis_state = axis_current_state
                        elif node_id == 1:
                            self.motors["fl_motor"].axis_state = axis_current_state
                        elif node_id == 2:
                            self.motors["rr_motor"].axis_state = axis_current_state
                        elif node_id == 3:
                            self.motors["rl_motor"].axis_state = axis_current_state
                        else:
                            print(f"Unkown node ID {node_id}", flush=True)

                else:
                    print(f"Unkown Cmd ID {cmd_id}", flush=True)

            # Populate the msg header
            feedback_msg.header.stamp = self.get_clock().now().to_msg()
            feedback_msg.header.frame_id = "" # TODO: fill this later

            # Publish
            self.feedback_publisher.publish(feedback_msg)


class Motor:
    FORWARD = 1.0
    REVERSE = -1.0
    STOP = 0.0
    DEFAULT_GEAR_REDUCTION_FACTOR = 1.0

    def __init__(self, bus, db, axis_id: int, direction: float = FORWARD,
                 gear_reduction_factor: float = DEFAULT_GEAR_REDUCTION_FACTOR):
        self.bus = bus
        self.db = db
        self.axis_id = axis_id
        self.direction = direction
        self.gear_reduction = gear_reduction_factor
        self.axis_state = 0 # 0 = UNDEFINED
        self.velocity_cmd_rpm = 0.0 
    
    def set_rpm(self):
        # print(f'Setting Axis {self.axis_id} rpm {self.velocity_cmd_rpm}', flush=True)
        rps = (self.velocity_cmd_rpm / 60.0) * self.gear_reduction * self.direction
        data = self.db.encode_message('Set_Input_Vel', {'Input_Vel': rps, 'Input_Torque_FF': 100.0})
        msg = can.Message(arbitration_id=self.axis_id << 5 | 0x00D, data=data, is_extended_id=False)
        # TODO: mutex lock the can bus for concurrent access?
        # May conflict with feedback loop
        self.bus.send(msg)

    def odrive_initial_setup(self):
        # Transition to MOTOR_CALIBRATION state
        print(f'Transitioning Axis {self.axis_id} into MOTOR_CALIBRATION (0x04)', flush=True)
        msg = self.db.get_message_by_name('Set_Axis_State')
        data = msg.encode({'Axis_Requested_State': 0x04})
        msg = can.Message(arbitration_id=msg.frame_id | self.axis_id << 5, is_extended_id=False, data=data)

        try:
            self.bus.send(msg)
            # print("Message sent on {}".format(self.bus.channel_info))
        except can.CanError:
            # print("Message NOT sent!  Please verify can0 is working first")
            pass # TODO: Handle these cases

        print(f"Waiting for calibration to finish for Axis {self.axis_id} ... ", flush=True)
        while self.axis_state != 1:
            time.sleep(0.01)
        print(f"Calibration finished for Axis {self.axis_id} ... ", flush=True)
        
        
        
        # Transition to CLOSED_LOOP_CONTROL state
        print(f"Transitioning Axis {self.axis_id} into CLOSED_LOOP_CONTROL (0x08)",  flush=True)
        data = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x08})
        msg = can.Message(arbitration_id=0x07 | self.axis_id << 5, is_extended_id=False, data=data)

        try:
            self.bus.send(msg)
            # print("Message sent on {}".format(self.bus.channel_info))
        except can.CanError:
            print("Message NOT sent!") # TODO: Handle these cases

        
        print(f"Waiting for {self.axis_id} to go into CLOSED_LOOP_CONTROL (0x08) ... ", flush=True)
        while self.axis_state != 8:
            time.sleep(0.01)
        print(f"Axis {self.axis_id} set to CLOSED_LOOP_CONTROL (0x08) ... ", flush=True)
        

            


def main(args=None):
    # rclpy.init(args=args)

    # odrive_can_node = OdriveCAN()

    # rclpy.spin(odrive_can_node)

    # odrive_can_node.destroy_node()
    # rclpy.shutdown()
    rclpy.init(args=args)

    odrive_can_node = OdriveCAN()

    executor = MultiThreadedExecutor()
    executor.add_node(odrive_can_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        odrive_can_node.get_logger().info('Keyboard interrupt, shutting down.\n')

    odrive_can_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

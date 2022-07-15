#!/usr/bin/env python3

"""
This script is used to control the actuators/pedals of a New Eagle DBW enabled car using a joystick. It controls the
actuators/pedals using percentages (as opposed to direct speed or torque based control).

The script subscribes to a sensor_msgs/Joy message and publishes a ... message.

NOTE: This script has been tested with a Logitech Wireless Gamepad F710 and expects the X-input mode (not DirectInput)
      and will fail otherwise.
"""

# import legacy modules first (compulsorily at the top)
from __future__ import division, print_function, absolute_import

# standard library imports
import math
from math import sin, cos, tan, pi, fabs
import sys

# third-party library imports
import rospy

# import ros messages
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

# import local modules
# from new_eagle_dbw import NewEagleDbw
from raptor_dbw_msgs.msg import AcceleratorPedalCmd, BrakeCmd, SteeringCmd, GearCmd, MiscCmd, GlobalEnableCmd
from raptor_dbw_msgs.msg import ActuatorControlMode, Gear, TurnSignal


class JoystickActuatorControlNode:
    """docstring for ClassName"""

    def __init__(self, ):
        """Constructor for JoystickActuatorControlNode"""
        super(JoystickActuatorControlNode, self).__init__()

        # get parameters. Todo: test new dbw_config.yaml file
        self.ignore = rospy.get_param('~ignore', False)  # Ignore driver overrides
        self.enable = rospy.get_param('~enable', True)  # Use enable and disable buttons
        self.count = rospy.get_param('~count', False)  # Increment counter to enable watchdog
        self.svel = rospy.get_param('~svel', 0.0)  # Steering command speed
        self.MAX_ACCELERATION_ACTUATION = rospy.get_param('~max_acceleration_actuation', 20)  # in percentage. 100%

        # setup loop frequency. Will be enforced if the loop completes execution within the limit, otherwise ignored.
        self.loop_rate = 50.0  # Hz
        self.loop_sample_time = 1.0 / self.loop_rate  # s
        self.rate = rospy.Rate(self.loop_rate)

        # setup threads to run asynchronously
        period = 0.02  # a.k.a. duration in s (or 1/Hz)
        self.joystick_timer = rospy.Timer(rospy.Duration(period), self.cmd_callback, reset=True)

        # one thread running at a low frequency to act as a watchdog
        self.timer_watchdog = rospy.Timer(rospy.Duration(2), self.watchdog, reset=True)

        # instantiate subscribers
        subscriber_queue_size = 1
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback, queue_size=subscriber_queue_size)

        # instantiate publishers
        publisher_queue_size = 1
        self.pub_accelerator_pedal = rospy.Publisher("accelerator_pedal_cmd", AcceleratorPedalCmd,
                                                     queue_size=publisher_queue_size)
        self.pub_brake = rospy.Publisher("brake_cmd", BrakeCmd, queue_size=publisher_queue_size)
        self.pub_misc = rospy.Publisher("misc_cmd", MiscCmd, queue_size=publisher_queue_size)
        self.pub_steering = rospy.Publisher("steering_cmd", SteeringCmd, queue_size=publisher_queue_size)
        self.pub_global_enable = rospy.Publisher("global_enable_cmd", GlobalEnableCmd, queue_size=publisher_queue_size)
        self.pub_gear = rospy.Publisher("gear_cmd", GearCmd, queue_size=publisher_queue_size)

        if self.enable:
            self.pub_enable = rospy.Publisher("enable", Empty, queue_size=publisher_queue_size)
            self.pub_disable = rospy.Publisher("disable", Empty, queue_size=publisher_queue_size)

        # joystick data
        '''joystick_data is called data_ in JoystickDemo.h with type JoystickDataStruct.'''
        self.joystick_data = {'stamp': rospy.Time.now(),
                              'brake_joy': 0.0,
                              'accelerator_pedal_joy': 0.0,
                              'steering_joy': 0.0,
                              'steering_mult': False,
                              'gear_cmd': Gear.NONE,
                              'turn_signal_cmd': TurnSignal.NONE,
                              'joy_accelerator_pedal_valid': False,
                              'joy_brake_valid': False}

        # New Eagle Header definitions
        self.BTN_PARK = 3
        self.BTN_REVERSE = 1
        self.BTN_NEUTRAL = 2
        self.BTN_DRIVE = 0
        self.BTN_ENABLE = 5
        self.BTN_DISABLE = 4
        self.BTN_STEER_MULT_1 = 6
        self.BTN_STEER_MULT_2 = 7
        self.BTN_COUNT = 11
        self.AXIS_ACCELERATOR_PEDAL = 5
        self.AXIS_BRAKE = 2
        self.AXIS_STEER_1 = 0
        self.AXIS_STEER_2 = 3
        self.AXIS_TURN_SIG = 6
        self.AXIS_COUNT = 8

        # to get the current ROS time. Use this function call as it works both for simulation and wall time.
        self.current_time = rospy.Time.now()
        self.counter = 0

        # miscellaneous
        self.empty = Empty()
        self.joy = Joy()
        self.joy.axes = [0] * self.AXIS_COUNT
        self.joy.buttons = [0] * self.BTN_COUNT

    # def run(self):
    #     while not rospy.is_shutdown():
    #         # to enforce rate/sample time
    #         self.rate.sleep()
    #
    #     # whatever is written here will also be processed once a shutdown is initiated but is not guaranteed.
    #     # Use the shutdown method instead.
    #     pass

    def cmd_callback(self, event):
        """
        Similar to cmdCallback in JoystickDemo.cpp and JoystickDemo.h
        :param event: rospy.TimerEvent instance passed by the Timer object
        :return:
        """
        # Detect joy timeouts and reset
        if event.current_real - self.joystick_data.get('stamp') > rospy.Duration(0.1):
            self.joystick_data['joy_accelerator_pedal_valid'] = False
            self.joystick_data['joy_brake_valid'] = False

        # Optional watchdog counter
        if self.count:
            self.counter += 1

            # counter reset/overflow
            if self.counter > 15:
                self.counter = 0

        # Accelerator Pedal
        accelerator_pedal_msg = AcceleratorPedalCmd()
        accelerator_pedal_msg.enable = True  # self.enable
        accelerator_pedal_msg.ignore = self.ignore
        accelerator_pedal_msg.rolling_counter = self.counter
        # max is 100 but set to 20 for safety
        accelerator_pedal_msg.pedal_cmd = self.joystick_data['accelerator_pedal_joy'] * self.MAX_ACCELERATION_ACTUATION
        accelerator_pedal_msg.control_type.value = ActuatorControlMode.open_loop
        self.pub_accelerator_pedal.publish(accelerator_pedal_msg)

        # Brake
        brake_msg = BrakeCmd()
        brake_msg.enable = True
        brake_msg.rolling_counter = self.counter
        brake_msg.pedal_cmd = self.joystick_data['brake_joy'] * 100
        brake_msg.control_type.value = ActuatorControlMode.open_loop
        self.pub_brake.publish(brake_msg)

        # Steering
        steering_msg = SteeringCmd()
        steering_msg.enable = True
        steering_msg.ignore = self.ignore
        steering_msg.rolling_counter = self.counter
        steering_msg.angle_cmd = self.joystick_data['steering_joy']
        steering_msg.angle_velocity = self.svel
        steering_msg.control_type.value = ActuatorControlMode.closed_loop_actuator
        if not self.joystick_data['steering_mult']:
            steering_msg.angle_cmd *= 0.5
        self.pub_steering.publish(steering_msg)

        # Gear
        gear_msg = GearCmd()
        gear_msg.enable = True
        gear_msg.cmd.gear = self.joystick_data['gear_cmd']
        gear_msg.rolling_counter = self.counter
        self.pub_gear.publish(gear_msg)

        # Turn Signal
        misc_msg = MiscCmd()
        misc_msg.cmd.value = self.joystick_data['turn_signal_cmd']
        misc_msg.rolling_counter = self.counter
        self.pub_misc.publish(misc_msg)

        # Global Enable
        global_enable_msg = GlobalEnableCmd()
        global_enable_msg.global_enable = True
        global_enable_msg.enable_joystick_limits = True
        global_enable_msg.rolling_counter = self.counter
        self.pub_global_enable.publish(global_enable_msg)

    def joystick_callback(self, msg):
        """
        Similar to recvJoy in JoystickDemo.cpp and JoystickDemo.h
        :param msg: sensor_msgs/Joy message
        :return:
        """
        # Check for expected sizes
        if len(msg.axes) != self.AXIS_COUNT:
            rospy.logerr("Expected {} joy axis count, received {}".format(self.AXIS_COUNT, len(msg.axes)))
            return

        if len(msg.buttons) != self.BTN_COUNT:
            rospy.logerr("Expected {} joy button count, received {}".format(self.BTN_COUNT, len(msg.buttons)))
            return

        # Handle joystick startup
        if msg.axes[self.AXIS_ACCELERATOR_PEDAL] != 0.0:
            self.joystick_data["joy_accelerator_pedal_valid"] = True

        if msg.axes[self.AXIS_BRAKE] != 0.0:
            self.joystick_data["joy_brake_valid"] = True

        # Accelerator pedal
        if self.joystick_data["joy_accelerator_pedal_valid"]:
            self.joystick_data["accelerator_pedal_joy"] = 0.5 - 0.5 * msg.axes[self.AXIS_ACCELERATOR_PEDAL]

        # Brake
        if self.joystick_data["joy_brake_valid"]:
            self.joystick_data["brake_joy"] = 0.5 - 0.5 * msg.axes[self.AXIS_BRAKE]

        # Gear
        if msg.buttons[self.BTN_PARK]:
            self.joystick_data["gear_cmd"] = Gear.PARK
        elif msg.buttons[self.BTN_REVERSE]:
            self.joystick_data["gear_cmd"] = Gear.REVERSE
        elif msg.buttons[self.BTN_DRIVE]:
            self.joystick_data["gear_cmd"] = Gear.DRIVE
        elif msg.buttons[self.BTN_NEUTRAL]:
            self.joystick_data["gear_cmd"] = Gear.NEUTRAL
        else:
            self.joystick_data["gear_cmd"] = Gear.NONE

        # Steering
        axis_steer_1 = msg.axes[self.AXIS_STEER_1]
        axis_steer_2 = msg.axes[self.AXIS_STEER_2]
        steer_comparison = (fabs(axis_steer_1) > fabs(axis_steer_2))
        self.joystick_data["steering_joy"] = 470.0 * pi / 180.0 * (axis_steer_1 if steer_comparison else axis_steer_2)
        self.joystick_data["steering_mult"] = msg.buttons[self.BTN_STEER_MULT_1] or msg.buttons[self.BTN_STEER_MULT_2]

        # Turn signal
        # todo: current bug - turn signal is never turned back off even upon button repetition
        if msg.axes[self.AXIS_TURN_SIG] != self.joy.axes[self.AXIS_TURN_SIG]:
            # python 3.6 and below do not have switch statements
            def turn_signal_none(data):
                if data.axes[self.AXIS_TURN_SIG] < -0.5:
                    self.joystick_data["turn_signal_cmd"] = TurnSignal.RIGHT
                elif data.axes[self.AXIS_TURN_SIG] > 0.5:
                    self.joystick_data["turn_signal_cmd"] = TurnSignal.LEFT
                return

            def turn_signal_left(data):
                if data.axes[self.AXIS_TURN_SIG] < -0.5:
                    self.joystick_data["turn_signal_cmd"] = TurnSignal.RIGHT
                elif data.axes[self.AXIS_TURN_SIG] > 0.5:
                    self.joystick_data["turn_signal_cmd"] = TurnSignal.NONE
                return

            def turn_signal_right(data):
                if data.axes[self.AXIS_TURN_SIG] < -0.5:
                    self.joystick_data["turn_signal_cmd"] = TurnSignal.NONE
                elif data.axes[self.AXIS_TURN_SIG] > 0.5:
                    self.joystick_data["turn_signal_cmd"] = TurnSignal.LEFT
                return

            def default(data):
                print("Invalid option. Using default value of {} instead.".format(TurnSignal.NONE))
                self.joystick_data["turn_signal_cmd"] = TurnSignal.NONE
                return

            turn_signal_switcher = {TurnSignal.NONE: turn_signal_none, TurnSignal.LEFT: turn_signal_left,
                                    TurnSignal.RIGHT: turn_signal_right}
            turn_signal_switcher.get(self.joystick_data["turn_signal_cmd"], default)(msg)

        # Optional enable and disable buttons
        if self.enable:
            if msg.buttons[self.BTN_ENABLE]:
                self.pub_enable.publish(self.empty)
            if msg.buttons[self.BTN_DISABLE]:
                self.pub_disable.publish(self.empty)

        self.joystick_data['stamp'] = rospy.Time.now()
        self.joy = msg

    # def timer_callback(self, event):
    #     rospy.loginfo("This is called every (period seconds).")

    def watchdog(self, event):
        """
        This callback is called, e.g. to monitor a topic, parameter, notify the user that a subscriber does is not
        publishing instead of using a rospy.wait_for_message since that is thread blocking.
        Should run at a low frequency.
        :param event:
        :return:
        """
        pass

    def shutdown(self):
        rospy.loginfo("Beginning clean shutdown routine...")
        # perform shutdown tasks here
        self.joystick_timer.shutdown()
        rospy.loginfo("Shutting down...")


def main(args):
    # args will be a list of commands passed
    optional_nodename = 'joystick_actuator_control_node'
    rospy.init_node('{}'.format(optional_nodename))
    nodename = rospy.get_name()  # this gets the actual nodename whether the node is launched or run separately
    rospy.loginfo("{} node started.".format(nodename))  # just log that the node has started.
    percentage_based_instance = JoystickActuatorControlNode()

    # use rospy.on_shutdown() to perform clean shutdown tasks, e.g. saving a file, shutting down motors, etc.
    rospy.on_shutdown(percentage_based_instance.shutdown)
    # could also use atexit.register(sample_instance.shutdown) to avoid trusting rospy

    try:
        # run the main functions here
        # percentage_based_instance.run()
        rospy.spin()  # only necessary if not publishing (i.e. subscribing only)
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        # this exception block most likely won't be called since there is a shutdown method in the class that will
        # override this and shutdown however is needed but is here just in case.
        rospy.loginfo('Encountered {}. Shutting down.'.format(e))

        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)  # ROS compatible way to handle command line arguments, i.e main(sys.argv)

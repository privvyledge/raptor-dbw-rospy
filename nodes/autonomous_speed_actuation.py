#!/usr/bin/env python3

"""
This script is used to control the speed of a New Eagle DBW enabled car using a set speed command. It controls the
actuators using an internal DBW-ECM module controller (as opposed to actuator based percentage commands).

The script subscribes to a ... message and publishes a ... message.

NOTE: This script has been tested with a Logitech Wireless Gamepad F710 and expects the X-input mode (not DirectInput)
      and will fail otherwise.

ToDo: Use up and down arrow for speed, assign button to arm autonomous mode, include manual joystick override,
      setup reversing gear shift when speed is negative, setup park when speed is zero and goal is reached,
      setup switching to drive.
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


class AutonomousSpeedActuationNode(object):
    """docstring for ClassName"""

    def __init__(self, ):
        """Constructor for AutonomousSpeedActuationNode"""
        super(AutonomousSpeedActuationNode, self).__init__()

        # get parameters
        self.ignore = rospy.get_param('~ignore', False)  # Ignore driver overrides
        self.enable = rospy.get_param('~enable', True)  # Use enable and disable buttons
        self.count = rospy.get_param('~count', False)  # Increment counter to enable watchdog
        self.svel = rospy.get_param('~svel', 0.0)  # Steering command speed
        # safety limits
        self.MAX_SPEED = rospy.get_param('~max_speed', 10)  # MPH. The maximum speed the car can drive
        # amount to raise or decrease the speed by. Translates to 1 MPH
        self.SPEED_INCREMENT = rospy.get_param('~speed_increment', 0.44704)

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
        # todo: subscribe to separate controller node (maybe Autoware) with autonomy_callback

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
                              'joy_brake_valid': False,
                              'accel_decel_limits': 3.0}

        # autonomous data
        self.autonomous_data = {'stamp': rospy.Time.now(),
                                'brake_joy': 0.0,
                                'accelerator_pedal_joy': 0.0,
                                'steering_joy': 0.0,
                                'steering_mult': False,
                                'gear_cmd': Gear.NONE,
                                'turn_signal_cmd': TurnSignal.NONE,
                                'joy_accelerator_pedal_valid': False,
                                'joy_brake_valid': False,
                                'accel_decel_limits': 3.0}

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
        # todo: initialize empty message of the autonomy_callback message type

    def cmd_callback(self, event):
        """
        Similar to cmdCallback in JoystickDemo.cpp and JoystickDemo.h
        :param event: rospy.TimerEvent instance passed by the Timer object
        :return:
        ToDo: takes commands from Joystick and high level control node. Prioritizes Joystick.
        """
        pass

    def joystick_callback(self, msg):
        """
        Similar to recvJoy in JoystickDemo.cpp and JoystickDemo.h
        :param msg: sensor_msgs/Joy message
        :return:
        ToDo: Use up and down arrow for speed, assign button to arm autonomous mode, include manual joystick override
        """
        pass

    def autonomy_callback(self, msg):
        """
        Receives actuator (speed and steering angle) commands from a high level control node, e.g. Autoware and sets
        the appropriate commands for Autonole. Uses autonomous_data dictionary similar to joystick_data.
        :param msg:
        :return:
        ToDo: setup reversing gear shift when speed is negative, setup park when speed is zero and goal is reached,
              setup switching to drive
        """
        pass

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
    optional_nodename = 'autonomous_speed_actuation_node'
    rospy.init_node('{}'.format(optional_nodename))
    nodename = rospy.get_name()  # this gets the actual nodename whether the node is launched or run separately
    rospy.loginfo("{} node started.".format(nodename))  # just log that the node has started.
    autonomous_speed_based_instance = AutonomousSpeedActuationNode()

    # use rospy.on_shutdown() to perform clean shutdown tasks, e.g. saving a file, shutting down motors, etc.
    rospy.on_shutdown(autonomous_speed_based_instance.shutdown)
    # could also use atexit.register(sample_instance.shutdown) to avoid trusting rospy

    try:
        # run the main functions here
        # autonomous_speed_based_instance.run()
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

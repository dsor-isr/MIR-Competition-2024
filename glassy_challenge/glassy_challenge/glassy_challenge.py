import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import std_srvs.srv as std_srvs

import glassy_msgs.msg as glassy_msgs

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# allowed libraries
import numpy as np
import scipy as sp


class GlassyChallenge(Node):
    def __init__(self):
        """
        Class
        """
        super().__init__('glassy_challenge')

        # initialize all the variables to be used
        self.yaw_rate = 0.0
        self.surge = 0.0
        self.sway = 0.0
        self.yaw = 0.0

        self.initial_yaw = 0.0

        self.x = 0.0
        self.y = 0.0

        self.initial_x = 0.0
        self.initial_y = 0.0

        self.is_active = False


        #********************************************************************************
        #
        #
        #
        #
        #       HERE YOU CAN ADD MORE VARIABLES, WHICH YOU CAN USE TO STORE THE PREVIOUS ERRORS,
        #                                   INTEGRAL OF ERRORS, ETC.
        #
        #
        #
        #
        #
        #*********************************************************************************




        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )



        # create publishers for torque and thrust setpoints
        self.actuators_publisher_ = self.create_publisher(glassy_msgs.Actuators, 'glassy/actuators', 1)

        # create subscriber for the state of the vehicle
        self.state_odometry_subscriber_ = self.create_subscription(glassy_msgs.State, 'glassy/state', self.state_subscription_callback, qos_profile)

        # create subscriber for the mission status
        self.mission_status_subscriber_ = self.create_subscription(glassy_msgs.MissionInfo, 'glassy/mission_status', self.mission_status_subscription_callback, 1)


        # create timer
        self.timer_control_ = self.create_timer(1.0/30.0, self.myChallengeController)
        self.timer_control_.cancel();
    
        self.time_of_last_mission_status_msg_received = None;



    def state_subscription_callback(self, msg):
        """
        Receives the state message and updates the local variables that store the state of the vehicle.
        """

        # Get the yaw and yaw rate (most important for the challenge)
        self.yaw = msg.yaw
        self.yaw_rate = msg.yaw_rate

        self.surge = msg.surge
        self.sway = msg.sway

        self.x = msg.p_ned[0]
        self.y = msg.p_ned[1]
        




    def mission_status_subscription_callback(self, msg):
        """
        Checks whether the mission is active or not.
        """
        if self.is_active:
            if msg.mission_mode != glassy_msgs.MissionInfo.SUMMER_CHALLENGE:
                self.timer_control_.cancel()
                self.is_active = False

        else:
            if msg.mission_mode == glassy_msgs.MissionInfo.SUMMER_CHALLENGE:
                self.timer_control_.reset()
                self.is_active = True

                # reset the initial mission values
                self.initial_x = self.x
                self.initial_y = self.y
                self.initial_yaw = self.yaw
        


    def myChallengeController(self):
        """
        Implement the controller for the challenge here.
        (it will run at 30Hz)
        """
        # Implement your controller here 
        # You can use the variables self.yaw, self.yaw_rate, self.surge, self.sway, self.x, self.y (only a subset of these is actually needed)
        # You also have access to the initial position and yaw (self.initial_x, self.initial_y, self.initial_yaw)
        # You can also use the libraries numpy and scipy
        # You can add more variables to the class to keep for example the previous errors/error integral,...
        # Please do not overcomplicate, the challenge is simple (~ 20 lines of code should be enough).
        # Please, use this function and if you need, add variables to the class constructor, do not change ANY other function.

        #********************************************************************************
        #
        #
        #
        #
        #                            YOUR CALCULATIONS GO HERE
        #                                   GOOD LUCK!
        #
        #
        #
        #
        #
        #*********************************************************************************


        # After finishing your calculations, fill the following variables with the values you want to publish, and thats it, you are done.

        # Fill these values in please (motor should be between [0,1]) (max thrust is reduced, to avoid accidents)
        #currently, the motor is a constant value, just to test that everything is working
        motor_value = 0.3

        # Fill (rudder should be between [-1,1])
        # currently, the rudder is a sinusoidal function of time, just to test that everything is working
        rudder_value = np.sin(self.get_clock().now().nanoseconds/1000000000)


        self.publish_actuators(motor_value, rudder_value)



    #
    def publish_actuators(self, motor_value, rudder_value):
        """
        Takes the motor and rudder values. Clips and publishes them.
        """
        #clip values
        motor_value = np.clip(motor_value, 0.0, 1.0)
        rudder_value = np.clip(rudder_value, -1.0, 1.0)

        msg_actuators = glassy_msgs.Actuators()

        msg_actuators.header.stamp = self.get_clock().now().to_msg()
        msg_actuators.thrust = motor_value
        msg_actuators.rudder = rudder_value


        self.actuators_publisher_.publish(msg=msg_actuators)

    

def main(args=None):
    rclpy.init(args=args)

    test = GlassyChallenge()

    test.get_logger().info('Glassy Challenge node started')
    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()
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

        self.initial_yaw = 0.0

        self.x = 0.0
        self.y = 0.0

        self.initial_x = 0.0
        self.initial_y = 0.0

        self.yaw = 0.0

        self.quaternion_attitude = sp.spatial.transform.Rotation.from_quat([0, 0, 0, 1])
        self.euler_attitude = self.quaternion_attitude.as_euler('xyz')

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

        # create service for the challenge
        self.start_stop_service_ = self.create_service(std_srvs.SetBool, 'start_stop_challenge', self.start_stop_service_callback)

        # create timer
        self.timer_control_ = self.create_timer(1.0/30.0, self.myChallengeController)

        self.timer_control_.cancel();



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
        




    def start_stop_service_callback(self, request, response):
        self.is_active = request.data
        if self.is_active:
            self.timer_control_.reset()

            #we introduce an offset to complicate things a bit (in real life, external disturbances, model errors, etc. would do this for us)
            self.initial_yaw = self.yaw+0.2
            self.initial_x = self.x
            self.initial_y = self.y
        else:
            self.timer_control_.cancel()

        response.success = True
        return response



    def myChallengeController(self):
        """
        Implement the controller for the challenge here.
        (it will run at 30Hz)
        """
        # Implement your controller here 
        # You can use the variables self.yaw, self.yaw_rate, self.surge, self.sway, self.x, self.y (only a small subset of these is actually needed)
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



        # Fill these values in please (motor should be between [0,1]) (max thrust is reduced, to avoid accidents)
        #currently, the motor is a constant value, just to test that everything is working
        motor_value = 0.3

        # Fill (rudder should be between [-1,1])
        # currently, the rudder is a sinusoidal function of time, just to test that everything is working
        rudder_value = np.sin(self.get_clock().now().nanoseconds/1000000000)



        self.publish_actuators(motor_value, rudder_value)



    def publish_actuators(self, motor_value, rudder_value):
        """
        Publish the actuator values.
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
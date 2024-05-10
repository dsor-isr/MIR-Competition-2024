import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import std_srvs.srv as std_srvs

import glassy_msgs.msg as glassy_msgs


# allowed libraries
import numpy as np


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

        # used to calculate the score... ( you may test different coeficients to see how they affect the score, and different max_speeds)

        self.max_velocity = 3.0
        self.total_dist_coef = 1.0
        self.cross_dist_coef = 0.001
        self.vel_coef = 2.0


        #auxiliary variables
        self.time_prev = 0.0
        self.cross_track_distance = 0.0
        self.velocity_above_max = 0.0



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

        # create publishers for torque and thrust setpoints
        self.actuators_publisher_ = self.create_publisher(glassy_msgs.Actuators, 'glassy/actuators', 1)

        # create subscriber for the state of the vehicle
        self.state_subscriber_ = self.create_subscription(glassy_msgs.State, 'glassy/state', self.state_subscription_callback, 1)

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

        self.surge = msg.v_body[0]
        self.sway = msg.v_body[1]

        self.x = msg.p_ned[0]
        self.y = msg.p_ned[1]

        current_time = self.get_clock().now().nanoseconds/1e9
        dt = (current_time - self.time_prev)

        # calculate the integral of the cross track distance
        self.cross_track_distance += np.abs(( np.cos(self.initial_yaw) * (self.initial_y-self.y) - np.sin(self.initial_yaw) * (self.initial_x - self.x)))**2 * dt

        # calculate the integral of the velocity above the max velocity
        self.velocity_above_max += np.maximum(np.sqrt(self.surge**2 + self.sway**2) - self.max_velocity, 0)**2 * dt


        self.time_prev = current_time

        




    def mission_status_subscription_callback(self, msg):
        """
        Checks whether the mission is active or not.
        """
        if self.is_active:
            if msg.mission_mode != glassy_msgs.MissionInfo.SUMMER_CHALLENGE:
                self.timer_control_.cancel()
                self.is_active = False


                get_projection_on_line = np.dot([self.x - self.initial_x, self.y - self.initial_y], [np.cos(self.initial_yaw), np.sin(self.initial_yaw)])


                self.get_logger().info('Mission ended')
                self.get_logger().info('ALONG TRACK DISTANCE (larger is better): ' + str(get_projection_on_line))
                self.get_logger().info('CROSS TRACK DISTANCE SQUARED INTEGRAL (lower is better): ' + str(self.cross_track_distance))
                self.get_logger().info('VELOCITY OVER MAX SQUARED INTEGRAL (lower is better): ' + str(self.velocity_above_max))
                self.get_logger().info('ALONG TRACK DISTANCE SCORE: ' + str(get_projection_on_line * self.total_dist_coef))
                self.get_logger().info('CROSS TRACK DISTANCE INTEGRAL SCORE: ' + str(- self.cross_dist_coef * self.cross_track_distance))
                self.get_logger().info('VELOCITY OVER MAX INTEGRAL SCORE: ' + str(- self.vel_coef * self.velocity_above_max))
                self.get_logger().info('TOTAL SCORE: ' + str(get_projection_on_line * self.total_dist_coef - self.cross_dist_coef * self.cross_track_distance - self.vel_coef * self.velocity_above_max))

        else:
            if msg.mission_mode == glassy_msgs.MissionInfo.SUMMER_CHALLENGE:
                self.timer_control_.reset()
                self.is_active = True

                # reset the initial mission values
                # HERE YOU CAN ADD SLIGHT OFFSETS TO THE INITIAL VALUES, TO MAKE THE CHALLENGE MORE INTERESTING/COMPLICATED, AND TEST YOUR CONTROLLER BETTER
                # Ex: self.initial_yaw = self.initial_yaw + 0.1 (around 5º offset)
                self.initial_x = self.x
                self.initial_y = self.y
                self.initial_yaw = self.yaw

                # these are used for evaluation purposes
                self.time_prev = self.get_clock().now().nanoseconds/1e9
                self.cross_track_distance = 0.0
                self.velocity_above_max = 0.0
        


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

        # Variables available, and corresponding units:
        # yaw -> [-pi, pi] (rad)
        # yaw_rate (rad/s)
        # surge (m/s)
        # sway (m/s)
        # x, y (m)

        # You also have access to the initial yaw, and position:
        # initial_yaw [-pi, pi] (rad)
        # initial_x, initial_y (m)




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
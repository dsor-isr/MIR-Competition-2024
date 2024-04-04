import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import std_srvs.srv as std_srvs

import glassy_interfaces.msg as glassy_interfaces

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
        self.torque_publisher_ = self.create_publisher(px4_msgs.VehicleTorqueSetpoint, 'fmu/in/vehicle_torque_setpoint', 1)
        self.thrust_publisher_ = self.create_publisher(px4_msgs.VehicleThrustSetpoint, 'fmu/in/vehicle_thrust_setpoint', 1)
        self.offboard_control_mode_publisher_ = self.create_publisher(px4_msgs.OffboardControlMode, 'fmu/in/offboard_control_mode', 1)

        # create subscriber for the state of the vehicle
        self.state_odometry_subscriber_ = self.create_subscription(px4_msgs.VehicleOdometry, 'fmu/out/vehicle_odometry', self.state_odometry_callback, qos_profile)

        # create service for the challenge
        self.start_stop_service_ = self.create_service(std_srvs.SetBool, 'start_stop_challenge', self.start_stop_service_callback)

        # create timer
        self.timer_control_ = self.create_timer(1.0/30.0, self.myChallengeController)
        self.timer_offboard_mode_ = self.create_timer(1.0/10.0, self.publish_offboard_control_mode)

        self.timer_control_.cancel();
        self.timer_offboard_mode_.cancel();



    def state_odometry_callback(self, msg):
        """
        Receives the odometry message and updates the local variables that store the state of the vehicle.
        """

        # Get the attitude of the vehicle
        self.quaternion_attitude = sp.spatial.transform.Rotation.from_quat([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        self.euler_attitude = self.quaternion_attitude.as_euler('xyz')

        # Get the yaw and yaw rate (most important for the challenge)
        self.yaw = self.euler_attitude[2]
        self.yaw_rate = msg.angular_velocity[2]

        # Get the velocity of the vehicle
        #check the reference frame of the velocity, and act accordingly
        if(msg.velocity_frame == msg.VELOCITY_FRAME_BODY_FRD):
            self.surge = msg.velocity[0]
            self.sway = msg.velocity[1]
        elif(msg.velocity_frame == msg.VELOCITY_FRAME_NED):
            #NEED TO ROTATE THE STUFF...
            velocity = [msg.velocity[0], msg.velocity[1], 0]
            # apply the body rotation to the velocity, to get the velocity in body frame
            velocity = self.quaternion_attitude.apply(velocity, False)

            # get the most important components of the velocity in body frame
            self.surge = velocity[0]
            self.sway = velocity[1]
        else:
            self.get_logger().info('UNKNOWN VELOCITY REFFERENCE FRAME: %s' % msg.velocity_frame)
            
        if(msg.pose_frame == msg.POSE_FRAME_NED):
            self.x = msg.position[0]
            self.y = msg.position[1]
        else:
            self.get_logger().info('UNKNOWN VELOCITY REFFERENCE FRAME: %s' % msg.velocity_frame)


    def start_stop_service_callback(self, request, response):
        self.is_active = request.data
        if self.is_active:
            self.timer_control_.reset()
            self.timer_offboard_mode_.reset()

            #we introduce an offset to complicate things a bit (in real life, external disturbances, model errors, etc. would do this for us)
            self.initial_yaw = self.yaw+0.2
        else:
            self.timer_control_.cancel()
            self.timer_offboard_mode_.cancel()

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
        motor_value = np.clip(motor_value, 0.0, 1)
        rudder_value = np.clip(rudder_value, -1.0, 1.0)

        msg_torque = px4_msgs.VehicleTorqueSetpoint()
        msg_thrust = px4_msgs.VehicleThrustSetpoint()

        msg_thrust.xyz[0] = motor_value
        msg_thrust.xyz[1] = 0.0
        msg_thrust.xyz[2] = 0.0

        msg_torque.xyz[0] = 0.0
        msg_torque.xyz[1] = 0.0
        msg_torque.xyz[2] = rudder_value

        msg_thrust.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg_torque.timestamp = msg_thrust.timestamp
        msg_thrust.timestamp_sample = msg_thrust.timestamp
        msg_torque.timestamp_sample = msg_thrust.timestamp

        self.thrust_publisher_.publish(msg=msg_thrust)
        self.torque_publisher_.publish(msg=msg_torque)

        
    def publish_offboard_control_mode(self):
        msg_offboard_control_mode = px4_msgs.OffboardControlMode()
        msg_offboard_control_mode.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg_offboard_control_mode.position = False
        msg_offboard_control_mode.velocity = False
        msg_offboard_control_mode.acceleration = False
        msg_offboard_control_mode.attitude = False
        msg_offboard_control_mode.body_rate = False
        msg_offboard_control_mode.thrust_and_torque = True
        self.offboard_control_mode_publisher_.publish(msg=msg_offboard_control_mode)
    

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
/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _StateManager_
#define _StateManager_

#include <std_srvs/srv/set_bool.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp> 
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <glassy_msgs/msg/state.hpp>
#include <glassy_msgs/msg/actuators.hpp>
#include <glassy_msgs/msg/mission_info.hpp>
#include <glassy_msgs/srv/set_mission.h>

#include <glassy_utils/GlassyGeneralUtils.h>

#include "mission_types.h"

#include <chrono>
#include <iostream>
#include <algorithm>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;



extern const std::vector<uint8_t> MissionTypes;


/**
* @brief StateManager class:
* is responsible for managing the state of the ASV. It publishes offboard control mode and setpoints, allowing the User to * arm and enter offboard mode while using a ground control station. 
* It detects the vehicle status, and if the vehicle is armed, and in offboard mode, it will initialize a mission, which will provide * the necessary offboard setopints to the vehicle.
*/
class StateManager : public rclcpp::Node
{
public:
	StateManager();
	void disarm();

private:

	/*
		Timer
	*/
	rclcpp::TimerBase::SharedPtr timer_actuator_publishing_;
	rclcpp::TimerBase::SharedPtr timer_state_publishing_;
	rclcpp::TimerBase::SharedPtr timer_offboard_control_mode_publishing_;
	rclcpp::TimerBase::SharedPtr timer_mission_info_publishing_;
	rclcpp::TimerBase::SharedPtr timer_;

	
	/*
		Timer callbacks
	*/
	void publish_state_callback();
	void publish_offboard_control_mode();
    void publish_offboard_actuator_signals();
	void publish_mission_info();


    /*
        Publishers 
    */
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_setpoint_publisher_;
	rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr torque_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<glassy_msgs::msg::State>::SharedPtr state_px4_publisher_;
	rclcpp::Publisher<glassy_msgs::msg::MissionInfo>::SharedPtr mission_info_publisher_;


    /*
        Subscribers
    */
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_;
	rclcpp::Subscription<glassy_msgs::msg::Actuators>::SharedPtr actuator_glassy_subscriber_;

	/*
		Subscriber callbacks	
	*/
    void vehicle_control_mode_callback(const VehicleControlMode::SharedPtr msg);
	void vehicle_odometry_callback(const VehicleOdometry::SharedPtr msg);
	void actuator_glassy_callback(const glassy_msgs::msg::Actuators::SharedPtr msg);


	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped



	/*
		Publishing msgs structure variables
	*/
	std::shared_ptr<glassy_msgs::msg::State> state_px4_msg_;
	std::shared_ptr<glassy_msgs::msg::Actuators> actuators_msg_;
	std::shared_ptr<VehicleThrustSetpoint> thrust_msg_;
    std::shared_ptr<VehicleTorqueSetpoint> torque_msg_;
	std::shared_ptr<glassy_msgs::msg::MissionInfo> mission_info_msg_;



	/*
		Parameters
	*/
	uint8_t mission_type_ = glassy_msgs::msg::MissionInfo::SUMMER_CHALLENGE;

	// in nanoseconds
	long int timeout_actuators_ = 1500000000;

	// in nanoseconds
	long int last_time_received_actuators_;

	long int mission_timeout_ = 20000000000;
	long int time_mission_start_ = 0;

	// Actuator variables
	float thrust_ = 0.0;
	float rudder_ = 0.0;

	float thrust_upper_limit_ = 1.0;

	float rudder_max_abs_input_ = 1.0;

	float thrust_trim_ = 0.0;
	float rudder_trim_ = 0.0;

	// simulatior or real vehicle settings
	bool is_gazebo_simulator_ = true;


	/*
		Parameter changes Callbacks
	*/
	void mission_type_callback(const rclcpp::Parameter & p);
	void mission_timeout_callback(const rclcpp::Parameter & p);
	void thrust_upper_limit_callback(const rclcpp::Parameter & p);
	void rudder_max_abs_input_callback(const rclcpp::Parameter & p);
	void print_param_change_info(const rclcpp::Parameter & p);

	// define callback handlers
	std::shared_ptr<rclcpp::ParameterCallbackHandle> mission_type_callback_handler_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> mission_timeout_callback_handler_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> thrust_upper_limit_callback_handler_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> rudder_max_abs_input_callback_handler_;

	// define one param subscriber
	std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;



	// Vehicle status variables
    bool offboard_mode_=false;
    bool is_armed_=false;  
	bool mission_is_on_=false;

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void start_mission();
    void stop_mission();
};

#endif
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
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "mission_types.h"

#include <glassy_msgs/msg/state.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


/**
* @brief StateManager class
* This class is responsible for managing the state of the ASV. It publishes offboard control mode and setpoints, allowing the User to * arm and enter offboard mode while using a ground control station. 
* It detects the vehicle status, and if the vehicle is armed, and in offboard mode, it will initialize a mission, which will provide * the necessary offboard setopints to the vehicle.
*/
class StateManager : public rclcpp::Node
{
public:
	StateManager();

    void vehicle_control_mode_callback(const VehicleControlMode::SharedPtr msg);
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

    /*
        Publishers 
    */
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_setpoint_publisher_;
	rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr torque_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;


    /*
        Subscribers
    */
    rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_control_mode_;

	/*
		Clients
	*/
	rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_mission_summer_challenge_client_;
	rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr path_following_client_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	MissionType mission_type_ = MissionType::SUMMER_SCHOOL_CHALLENGE;

    bool offboard_mode_=false;
    bool is_armed_=false;  
	bool mission_is_on_=false;

	void publish_offboard_control_mode();

    void publish_offboard_signals();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    bool start_mission();
    bool stop_mission();
	bool start_stop_mission(bool start);

	bool start_stop_path_following(bool start);
	bool start_stop_summer_challenge(bool start);
    
};

#endif
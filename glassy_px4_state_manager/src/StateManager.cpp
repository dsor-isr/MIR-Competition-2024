
/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#include "StateManager.h"


/**
 * @brief State Manager constructor
*/
StateManager::StateManager() : Node("glassy_state_manager")
{

    // Initialize the parameters
    this->declare_parameter("mission_type", 2);
    this->declare_parameter("types/PATH_FOLLOWING", 1);
    this->declare_parameter("types/SUMMER_SCHOOL_CHALLENGE", 2);

    // Get the parameters
    int mission = this->get_parameter("mission_type").as_int();

    // Initialize the mission type based on the parameter
    if(mission == this->get_parameter("types/PATH_FOLLOWING").as_int()){
        mission_type_ = PATH_FOLLOWING;
        RCLCPP_INFO(this->get_logger(), "Mission type: PATH_FOLLOWING");
    }
    else if(mission == this->get_parameter("types/SUMMER_SCHOOL_CHALLENGE").as_int()){
        mission_type_ = SUMMER_SCHOOL_CHALLENGE;
        RCLCPP_INFO(this->get_logger(), "Mission type: SUMMER_SCHOOL_CHALLENGE");
    }
    else{
        mission_type_ = SUMMER_SCHOOL_CHALLENGE;
    }

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    thrust_setpoint_publisher_ = this->create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);
    torque_setpoint_publisher_ = this->create_publisher<VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_control_mode_ = this->create_subscription<VehicleControlMode>("fmu/out/vehicle_control_mode", qos, std::bind(&StateManager::vehicle_control_mode_callback, this, std::placeholders::_1));

    // define the clients for the services
    path_following_client_ = this->create_client<std_srvs::srv::SetBool>("/fmu/in/path_following");
    start_mission_summer_challenge_client_ = this->create_client<std_srvs::srv::SetBool>("start_stop_challenge");


    timer_ = this->create_wall_timer(100ms, std::bind(&StateManager::publish_offboard_signals, this));

}



// /**
//  * @brief This takes the state of the vehicle from px4 and turns in into a more readable format. Easier to debug/addition of * body velocities/...
//  * 
//  * 
//  */
// void to_be_done(const VehicleOdometry::SharedPtr msg){
//     //TODO
// }


/**
 * @brief Monitor state of vehicle, in case of armed and in offboard, start mission,
 * else stop it.
 */
void StateManager::vehicle_control_mode_callback(const VehicleControlMode::SharedPtr msg)
{
    offboard_mode_ = msg->flag_control_offboard_enabled;
    is_armed_ = msg->flag_armed;

    std::cout<< "is armed : "<< is_armed_<<std::endl;
    std::cout<< "is offboard : "<< offboard_mode_<<std::endl;
    std::cout<< "is mission : "<< mission_is_on_<<std::endl;

    if (offboard_mode_ && is_armed_ && !mission_is_on_)
    {
        // Initialize mission
        mission_is_on_ = this->start_mission();
    }
    else if ((!offboard_mode_ || !is_armed_) && mission_is_on_)
    {
        // Stop mission
        mission_is_on_ = !this->stop_mission();
    } 
};

/**
 * @brief Disarm the vehicle, mostly used when mission is finished or in case of emergency.
 */
void StateManager::disarm()
{
	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        We are always interested in torque and thrust setpoints in our case.
 */
void StateManager::publish_offboard_control_mode()
{
    // This is always published, so the mission does not need to publish it.
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
    msg.thrust_and_torque = true;
    msg.direct_actuator = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish null thrust and torque setpoint.
 *  This ensures offboard mode can be entered from a Ground station.       
 */
void StateManager::publish_offboard_signals()
{   
    
    if(mission_is_on_){
        std::cout<<"returning in timer callback"<<std::endl;
        return;
    }
    // in case of offboard mode and armed, do not publish, as the vehicle will be controlled by the mission.
    publish_offboard_control_mode();

    // create and populate the thrust and torque setpoint messages
    VehicleThrustSetpoint thrust_msg{};
    VehicleTorqueSetpoint torque_msg{};
    thrust_msg.xyz[0] = 0.0;
    thrust_msg.xyz[1] = 0.0;
    thrust_msg.xyz[2] = 0.0;
    torque_msg.xyz[0] = 0.0;
    torque_msg.xyz[1] = 0.0;
    torque_msg.xyz[2] = 0.0;

	thrust_msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
	torque_msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;

    thrust_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    torque_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // publish the messages
    thrust_setpoint_publisher_->publish(thrust_msg);
    torque_setpoint_publisher_->publish(torque_msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1 (default: 0.0)
 * @param param2    Command parameter 2 (default: 0.0)
 */
void StateManager::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Start a selected mission, this mission is selected by the parameter mission_type
 * 
 */
bool StateManager::start_mission(){
    return start_stop_mission(true);
}

/**
 * @brief Stop the current mission.
 * 
 */
bool StateManager::stop_mission(){
    return start_stop_mission(false);
}

/**
 * @brief Start or stop a mission
 * @param start     True to start, False to stop
 */
bool StateManager::start_stop_mission(bool start){

    if(mission_type_ == PATH_FOLLOWING){
        return this->start_stop_path_following(start);
    }
    else if(mission_type_ == SUMMER_SCHOOL_CHALLENGE){
        RCLCPP_INFO(this->get_logger(), "Starting/stoping summer_challenge, %d", start);
        return this->start_stop_summer_challenge(start);
    }
    else{
        return false;
        RCLCPP_INFO(this->get_logger(), "Problem starting/stoping mission, unknown mission type");
    }

}

/**
 * @brief Start or stop path following mission
 * @param start     True to start, False to stop
 */
bool StateManager::start_stop_path_following(bool start){
    std_srvs::srv::SetBool::Request::SharedPtr request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = start;
    auto result = path_following_client_->async_send_request(request);
    if(result.get()->success){
        return true;
    }
    else{
        return false;
    }
}

/**
 * @brief Start or stop summer challenge
 * @param start     True to start, False to stop
 */
bool StateManager::start_stop_summer_challenge(bool start){

    bool is_service_available = start_mission_summer_challenge_client_->wait_for_service(0.1s);
    if (!is_service_available) {
        RCLCPP_ERROR(this->get_logger(), "Service not available");
        this->disarm();
        return false;
    }

    std_srvs::srv::SetBool::Request::SharedPtr request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = start;
    auto result = start_mission_summer_challenge_client_->async_send_request(request);

    return true;
}
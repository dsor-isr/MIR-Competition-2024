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
    this->declare_parameter("rates/state_publishing", 50);
    this->declare_parameter("rates/actuator_publishing", 40);
    this->declare_parameter("rates/mission_info_publishing", 2);
    this->declare_parameter("timeouts/actuator_timeout", 1500000000);
    this->declare_parameter("timeouts/mission_timeout", 20000000000);
    this->declare_parameter("thrust_upper_limit", 0.5);
    this->declare_parameter("thrust_trim", 0.0);
    this->declare_parameter("rudder_trim", 0.0);
    this->declare_parameter("rudder_max_abs_input", 1.0);
    this->declare_parameter("gazebo_simulation", true);



    // get parameters
    int state_publishing_rate = this->get_parameter("rates/state_publishing").as_int();
    int actuator_publishing_rate = this->get_parameter("rates/actuator_publishing").as_int();
    int mission_info_publishing_rate = this->get_parameter("rates/mission_info_publishing").as_int();
    int mission = this->get_parameter("mission_type").as_int();
    timeout_actuators_ = this->get_parameter("timeouts/actuator_timeout").as_int();
    mission_timeout_ = this->get_parameter("timeouts/mission_timeout").as_int();
    thrust_trim_ = this->get_parameter("thrust_trim").as_double();
    rudder_trim_ = this->get_parameter("rudder_trim").as_double();
    thrust_upper_limit_ = this->get_parameter("thrust_upper_limit").as_double();
    rudder_max_abs_input_ = this->get_parameter("rudder_max_abs_input").as_double();
    is_gazebo_simulator_ = this->get_parameter("gazebo_simulation").as_bool();


    // initialize parameter handlers
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // define parameter callbacks
    mission_type_callback_handler_ =param_subscriber_->add_parameter_callback("mission_type", std::bind(&StateManager::mission_type_callback, this, std::placeholders::_1));
    mission_timeout_callback_handler_= param_subscriber_->add_parameter_callback("timeouts/mission_timeout", std::bind(&StateManager::mission_timeout_callback, this, std::placeholders::_1));
    thrust_upper_limit_callback_handler_ = param_subscriber_->add_parameter_callback("thrust_upper_limit", std::bind(&StateManager::thrust_upper_limit_callback, this, std::placeholders::_1));
    rudder_max_abs_input_callback_handler_ = param_subscriber_->add_parameter_callback("rudder_max_abs_input", std::bind(&StateManager::rudder_max_abs_input_callback, this, std::placeholders::_1));


    // check that the mission type is valid

    // Initialize the mission type based on the parameter
    if(std::find(MissionTypes.begin(), MissionTypes.end(), mission) != MissionTypes.end()){
        
        mission_type_ = mission;
    } else{
        mission_type_ = MissionInfo::SUMMER_CHALLENGE;
    }


    // Initialize the variables
    state_px4_msg_ = std::make_shared<glassy_msgs::msg::State>();
    actuators_msg_ = std::make_shared<glassy_msgs::msg::Actuators>();
    thrust_msg_ = std::make_shared<VehicleThrustSetpoint>();
    torque_msg_ = std::make_shared<VehicleTorqueSetpoint>();
    mission_info_msg_ = std::make_shared<glassy_msgs::msg::MissionInfo>();


    // Initialize the mission status
    mission_info_msg_->mission_mode = glassy_msgs::msg::MissionInfo::MISSION_OFF;

    // Initialize publishers
    state_px4_publisher_ = this->create_publisher<glassy_msgs::msg::State>("/glassy/state", 1);
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    thrust_setpoint_publisher_ = this->create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);
    torque_setpoint_publisher_ = this->create_publisher<VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    mission_info_publisher_ = this->create_publisher<glassy_msgs::msg::MissionInfo>("/glassy/mission_status", 10);

    //subriber profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    //Initialize subscribers
    vehicle_control_mode_ = this->create_subscription<VehicleControlMode>("fmu/out/vehicle_control_mode", qos, std::bind(&StateManager::vehicle_control_mode_callback, this, std::placeholders::_1));

    vehicle_odometry_ = this->create_subscription<VehicleOdometry>("fmu/out/vehicle_odometry", qos, std::bind(&StateManager::vehicle_odometry_callback, this, std::placeholders::_1));

    actuator_glassy_subscriber_ = this->create_subscription<glassy_msgs::msg::Actuators>("/glassy/actuators",1,  std::bind(&StateManager::actuator_glassy_callback, this, std::placeholders::_1));



    // define timers for state publishing and actuator publishing
    timer_actuator_publishing_ = this->create_wall_timer((1.0s/actuator_publishing_rate), std::bind(&StateManager::publish_offboard_actuator_signals, this));

    timer_offboard_control_mode_publishing_ = this->create_wall_timer(1.0s/10, std::bind(&StateManager::publish_offboard_control_mode, this));

    timer_state_publishing_ = this->create_wall_timer((1.0s/state_publishing_rate), std::bind(&
    StateManager::publish_state_callback, this));

    timer_mission_info_publishing_ = this->create_wall_timer(1.0s/mission_info_publishing_rate, std::bind(&StateManager::publish_mission_info, this));

}



/**
 * @brief This takes the state of the vehicle from px4 and turns in into a more readable format. Easier to debug/addition of * body velocities/...
 * The odometry message should be received at around 125 hz, so the state is updated at that rate.
 */
void StateManager::vehicle_odometry_callback(const VehicleOdometry::SharedPtr msg){

    state_px4_msg_->header.stamp = this->get_clock()->now();

    // populate an eigen quaternion with the info from the odometry msg 
    Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    
    // convert the quaternion to euler angles
    Eigen::Vector3d euler_angles= quat_to_euler_ZYX(q);

    state_px4_msg_->yaw = euler_angles[2];
    state_px4_msg_->pitch = euler_angles[1];
    state_px4_msg_->roll = euler_angles[0];


    state_px4_msg_->yaw_rate = msg->angular_velocity[2];
    state_px4_msg_->pitch_rate = msg->angular_velocity[1];
    state_px4_msg_->roll_rate = msg->angular_velocity[0];



    if(msg->pose_frame == VehicleOdometry::POSE_FRAME_NED){
        state_px4_msg_->p_ned[0] = msg->position[0];
        state_px4_msg_->p_ned[1] = msg->position[1];
        state_px4_msg_->p_ned[2] = msg->position[2];
    } else{
        RCLCPP_WARN(this->get_logger(), "Unknown pose frame, not updating NED position");
    }

    if(msg->velocity_frame == VehicleOdometry::VELOCITY_FRAME_BODY_FRD){
        // populate the body velocities
        state_px4_msg_->v_body[0] = msg->velocity[0];
        state_px4_msg_->v_body[1] = msg->velocity[1];
        state_px4_msg_->v_body[2] = msg->velocity[2];
    } else if (msg->velocity_frame == VehicleOdometry::VELOCITY_FRAME_NED){

        // get the vector in ned, then transform it to body frame
        Eigen::Vector3d v_ned = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
        Eigen::Vector3d v_body = q.conjugate()._transformVector(v_ned);

        // populate the body velocities
        state_px4_msg_->v_body[0] = v_body[0];
        state_px4_msg_->v_body[1] = v_body[1];
        state_px4_msg_->v_body[2] = v_body[2];
    }
    else{
        RCLCPP_WARN(this->get_logger(), "Unknown velocity frame, not updating body velocities");
    }

}

/**
 * @brief This callback is used to update the thrust and rudder values of the vehicle.
 * @param msg   The actuators message containing the thrust and rudder values.
 */
void StateManager::actuator_glassy_callback(const glassy_msgs::msg::Actuators::SharedPtr msg){
    thrust_ = msg->thrust;
    rudder_ = msg->rudder;
    last_time_received_actuators_ = rclcpp::Time(msg->header.stamp).nanoseconds();
}


/**
 * @brief Monitor state of vehicle, in case of armed and in offboard, start mission,
 * else stop it.
 */
void StateManager::vehicle_control_mode_callback(const VehicleControlMode::SharedPtr msg)
{
    offboard_mode_ = msg->flag_control_offboard_enabled;
    is_armed_ = msg->flag_armed;

    if (offboard_mode_ && is_armed_ && !mission_is_on_)
    {
        // Initialize mission
        this->start_mission();
    }
    else if ((!offboard_mode_ || !is_armed_) && mission_is_on_)
    {
        // Stop mission
        this->stop_mission();
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
    // get the current time
    long int time_now = this->get_clock()->now().nanoseconds();

    // check if the actuators have been received (done here due to the timer frequency)
    if(mission_is_on_){
        if(time_now - last_time_received_actuators_ > timeout_actuators_){
            RCLCPP_WARN(this->get_logger(), "TIMEOUT -> No actuator inputs received for %f s, disarming", float(timeout_actuators_)/1e9);
            thrust_ = 0.0;
            rudder_ = 0.0;
            this->disarm();
        }
        if(time_now - time_mission_start_ > mission_timeout_){
            RCLCPP_INFO(this->get_logger(), "MISSION OVER BY TIMEOUT (%fs), disarming", double(mission_timeout_/1e9));
            thrust_ = 0.0;
            rudder_ = 0.0;
            this->disarm();
        }
    }

    // This is always published, so the mission does not need to publish it.
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
    msg.thrust_and_torque = true;
    msg.direct_actuator = false;
	msg.timestamp = time_now / 1000;
	offboard_control_mode_publisher_->publish(msg);
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
void StateManager::start_mission(){
    last_time_received_actuators_ = this->get_clock()->now().nanoseconds();
    time_mission_start_ = last_time_received_actuators_;

    mission_info_msg_->mission_mode = mission_type_;
    mission_is_on_ = true;

    
}

/**
 * @brief Stop the current mission.
 * 
 */
void StateManager::stop_mission(){
    mission_info_msg_->mission_mode = glassy_msgs::msg::MissionInfo::MISSION_OFF;
    mission_is_on_ = false;
}



/* ------------------------------
    Timer Callbacks
-------------------------------*/


/**
 * @brief Publish null thrust and torque setpoint.
 *  This ensures offboard mode can be entered from a Ground station.       
 */
void StateManager::publish_offboard_actuator_signals()
{   
    float thrust = 0.0;
    float rudder = 0.0;

    // get the current time
    long int time_now = this->get_clock()->now().nanoseconds();


    // check if the actuators have been received 
    if(mission_is_on_){
        if(is_gazebo_simulator_){
            // to get a linear relationship
            thrust = sqrt(thrust_) + thrust_trim_;
        }
        else{
            thrust = thrust_ + thrust_trim_;
        }
        rudder = rudder_ + rudder_trim_;
    } 
    // if mission not on, send 0 thrust and rudder
    else{
        thrust = 0.0;
        rudder = 0.0;
    }

    thrust = std::max(std::min(thrust, thrust_upper_limit_), 0.f);
    rudder = std::max(std::min(rudder, rudder_max_abs_input_), -rudder_max_abs_input_);


    // create and populate the thrust and torque setpoint messages
    thrust_msg_->xyz[0] = thrust;
    thrust_msg_->xyz[1] = 0.0;
    thrust_msg_->xyz[2] = 0.0;
    torque_msg_->xyz[0] = 0.0;
    torque_msg_->xyz[1] = 0.0;
    torque_msg_->xyz[2] = rudder;

	thrust_msg_->timestamp_sample = time_now / 1000;
	torque_msg_->timestamp_sample = thrust_msg_->timestamp_sample;

    thrust_msg_->timestamp = thrust_msg_->timestamp_sample;
    torque_msg_->timestamp = thrust_msg_->timestamp_sample;

    // publish the messages
    thrust_setpoint_publisher_->publish(*thrust_msg_);
    torque_setpoint_publisher_->publish(*torque_msg_);
}

/**
 * @brief Publish the state of the vehicle in a compatible format to the glassy code stack (layer of independence to the px4 stack)
 */
void StateManager::publish_state_callback(){
    state_px4_publisher_->publish(*state_px4_msg_);
}

/**
 * @brief Publish the mission status of the vehicle
 */
void StateManager::publish_mission_info(){
    mission_info_publisher_->publish(*mission_info_msg_);
}



/*----------------------------------*
*                                   *
*   PARAMETER CHANGES CALLBACKS     *
*                                   *
------------------------------------*/

/**
 * @brief Callback for the mission type parameter change
 * @param param 
 */
void StateManager::mission_type_callback(const rclcpp::Parameter & p){
    print_param_change_info(p);
    try{
        int mission = p.as_int();
        if(std::find(MissionTypes.begin(), MissionTypes.end(), mission) != MissionTypes.end()){
            mission_type_ = mission;
        } else{
            RCLCPP_ERROR(this->get_logger(), "Invalid mission type, not changing");
        }
    } catch (const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Error converting parameter to int: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "Mission Type: %s", MissionNames[mission_type_].c_str());
}

/**
 * @brief Callback for the mission timeout parameter change
 * @param param 
 */
void StateManager::mission_timeout_callback(const rclcpp::Parameter & p){
    print_param_change_info(p);
    try{
        mission_timeout_ = p.as_int();
    } catch (const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Error converting parameter to int: %s", e.what());
    }
}

/**
 * @brief Callback for the thrust upper limit parameter change
 * @param param 
 */
void StateManager::thrust_upper_limit_callback(const rclcpp::Parameter & p){
    print_param_change_info(p);
    try{
        thrust_upper_limit_ = p.as_double();
    } catch (const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Error converting parameter to double: %s", e.what());
    }
}

/**
 * @brief Callback for the rudder max abs input parameter change
 * @param param 
 */
void StateManager::rudder_max_abs_input_callback(const rclcpp::Parameter & p){
    print_param_change_info(p);
    try{
        rudder_max_abs_input_ = p.as_double();
    } catch (const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Error converting parameter to double: %s", e.what());
    }    
}


/**
 * @brief Print information when receiving a parameter change
 * @param param pointer
*/
void StateManager::print_param_change_info(const rclcpp::Parameter & p){
    RCLCPP_INFO(this->get_logger(), "Received an update to parameter \"%s\" of type %s", p.get_name().c_str(), p.get_type_name().c_str());
    RCLCPP_INFO(this->get_logger(), "New value: %s", p.value_to_string().c_str());
}
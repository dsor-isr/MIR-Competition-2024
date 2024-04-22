/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _MissionTypes_
#define _MissionTypes_

#include <glassy_msgs/msg/mission_info.hpp>
#include <vector>
#include <string>
using namespace glassy_msgs::msg;

const std::vector<uint8_t> MissionTypes = {
    MissionInfo::MISSION_OFF, 
    MissionInfo::PATH_FOLLOWING,
    MissionInfo::TRAJECTORY_TRACKING,
    MissionInfo::SUMMER_CHALLENGE
};

const std::vector<std::string> MissionNames = {
    "MISSION_OFF",
    "PATH_FOLLOWING",
    "TRAJECTORY_TRACKING",
    "SUMMER_CHALLENGE"
};

#endif
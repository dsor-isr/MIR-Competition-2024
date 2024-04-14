/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _MissionTypes_
#define _MissionTypes_

#include <glassy_msgs/msg/mission_info.hpp>
#include <vector>
using namespace glassy_msgs::msg;

std::vector<uint8_t> missions_available = {
    MissionInfo::MISSION_OFF, 
    MissionInfo::PATH_FOLLOWING,
    MissionInfo::TRAJECTORY_TRACKING,
    MissionInfo::SUMMER_CHALLENGE
};

#endif
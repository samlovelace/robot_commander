#ifndef COMMANDERCOMMS_H
#define COMMANDERCOMMS_H

#include "robot_idl/msg/joint_position_waypoint.hpp"
#include "robot_idl/msg/enable.hpp"
#include "robot_idl/msg/task_position_waypoint.hpp"
#include "robot_idl/msg/task_velocity_waypoint.hpp"
#include "robot_idl/msg/joint_velocity_waypoint.hpp"
#include "robot_idl/msg/plan_command.hpp"

#include "robot_idl/msg/command.hpp"
 
class CommanderComms 
{ 
public:
    CommanderComms();
    ~CommanderComms();

    bool start(); 
    bool stop(); 

private:
   
};
#endif //COMMANDERCOMMS_H   
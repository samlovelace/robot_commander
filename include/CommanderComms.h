#ifndef COMMANDERCOMMS_H
#define COMMANDERCOMMS_H

#include "arm_idl/msg/joint_position_waypoint.hpp"
#include "arm_idl/msg/enable.hpp"
#include "arm_idl/msg/task_position_waypoint.hpp"
#include "arm_idl/msg/command.hpp"
 
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

#include "CommanderComms.h"
#include "RosTopicManager.hpp"
#include "arm_idl/msg/joint_position_waypoint.hpp"
#include "arm_idl/msg/enable.hpp"
#include "arm_idl/msg/task_position_waypoint.hpp"
#include "arm_idl/msg/command.hpp"

CommanderComms::CommanderComms()
{
    rclcpp::init(0, nullptr); 
}

CommanderComms::~CommanderComms()
{

}

bool CommanderComms::start()
{

    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createPublisher<arm_idl::msg::JointPositionWaypoint>("arm/joint_position_waypoint"); 
    topicManager->createPublisher<arm_idl::msg::Enable>("arm/enable"); 
    topicManager->createPublisher<arm_idl::msg::TaskPositionWaypoint>("arm/task_position_waypoint"); 
    topicManager->createPublisher<arm_idl::msg::Command>("arm/command"); 
    
    topicManager->spinNode(); 

    return true; 
}

bool CommanderComms::stop()
{
    rclcpp::shutdown(); 
}
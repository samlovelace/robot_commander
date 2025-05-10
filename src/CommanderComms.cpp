
#include "CommanderComms.h"
#include "RosTopicManager.hpp"
#include "arm_idl/msg/joint_position_waypoint.hpp"
#include "arm_idl/msg/enable.hpp"

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
    
    topicManager->spinNode(); 

    return true; 
}

bool CommanderComms::stop()
{
    rclcpp::shutdown(); 
}
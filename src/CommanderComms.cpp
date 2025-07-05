
#include "CommanderComms.h"
#include "RosTopicManager.hpp"


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
    topicManager->createPublisher<arm_idl::msg::PlanCommand>("arm/command"); 
    topicManager->createPublisher<vision_idl::msg::Command>("vision/command"); 
    
    topicManager->spinNode(); 

    return true; 
}

bool CommanderComms::stop()
{
    rclcpp::shutdown(); 
}
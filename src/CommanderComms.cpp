
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
    topicManager->createPublisher<robot_idl::msg::JointPositionWaypoint>("arm/joint_position_waypoint"); 
    topicManager->createPublisher<robot_idl::msg::Enable>("arm/enable"); 
    topicManager->createPublisher<robot_idl::msg::TaskPositionWaypoint>("arm/task_position_waypoint");
    topicManager->createPublisher<robot_idl::msg::TaskVelocityWaypoint>("arm/task_velocity_waypoint"); 
    topicManager->createPublisher<robot_idl::msg::JointVelocityWaypoint>("arm/joint_velocity_waypoint");  
    topicManager->createPublisher<robot_idl::msg::PlanCommand>("arm/plan"); // TODO: remove or update 
    topicManager->createPublisher<robot_idl::msg::Command>("vision/command"); 
    topicManager->createPublisher<robot_idl::msg::AbvCommand>("abv/command");
    topicManager->createPublisher<robot_idl::msg::GpcGoal>("gpc/goal"); 
    topicManager->createPublisher<robot_idl::msg::ManipulationCommand>("arm/command");
    
    topicManager->spinNode(); 

    return true; 
}

bool CommanderComms::stop()
{
    rclcpp::shutdown(); 
}
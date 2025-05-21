
#include "InputHandler.h"
#include <vector> 
#include <iostream>
#include <string> 

#include "RosTopicManager.hpp"
#include "arm_idl/msg/joint_position_waypoint.hpp"
#include "arm_idl/msg/task_position_waypoint.hpp"
#include "arm_idl/msg/enable.hpp"

InputHandler::InputHandler()
{

}

InputHandler::~InputHandler()
{

}

void InputHandler::handle(const std::string& anInput)
{
    if("waypoint" == anInput)
    {
        std::string waypointType; 
        std::cout << GREEN << "Type: "; 
        std::getline(std::cin, waypointType);
        
        if("jointPos" == waypointType)
        {
            std::cout << "Num Joints: "; 
            std::string numJointsStr; 
            std::getline(std::cin, numJointsStr);

            int numJoints = std::stoi(numJointsStr); 
            std::vector<double> jntPosCmd(numJoints);
            std::vector<double> jntTol(numJoints, 0.1);

            for(int i = 0; i < numJoints; i++)
            {
                std::string posStr; 
                std::cout << "Joint " << i << ": "; 
                std::getline(std::cin, posStr);
                
                jntPosCmd[i] = std::stod(posStr); 
            }

            arm_idl::msg::JointPositionWaypoint cmd; 
            cmd.set__positions(jntPosCmd); 
            cmd.set__tolerances(jntTol); 
            RosTopicManager::getInstance()->publishMessage<arm_idl::msg::JointPositionWaypoint>("arm/joint_position_waypoint", cmd); 
            std::cout << CYAN << "Published joint position waypoint cmd!" << std::endl; 
        }
        else if ("taskPos" == waypointType)
        {
            std::vector<std::string> axes = {"x", "y", "z", "qw", "qx", "qy", "qz"}; 
            std::vector<double> taskPos(7); 
            for(int i = 0; i < axes.size(); i++)
            {
                std::string tmp; 
                std::cout << axes[i] << ": "; 
                std::getline(std::cin, tmp); 

                taskPos[i] = std::stod(tmp); 
            }

            std::vector<double> jntTol(7, 0.1);
            std::string commandFrame = "base"; 

            std_msgs::msg::String cmdFrame;
            cmdFrame.set__data(commandFrame);  
            
            geometry_msgs::msg::Point pt; 
            pt.set__x(taskPos[0]); 
            pt.set__y(taskPos[1]); 
            pt.set__z(taskPos[2]); 

            geometry_msgs::msg::Quaternion q; 
            q.set__w(taskPos[3]);
            q.set__x(taskPos[4]);
            q.set__y(taskPos[5]);
            q.set__z(taskPos[6]); 

            geometry_msgs::msg::Pose pose; 
            pose.set__orientation(q); 
            pose.set__position(pt); 

            arm_idl::msg::TaskPositionWaypoint wp; 
            wp.set__command_frame(cmdFrame); 
            wp.set__tolerance(jntTol);
            wp.set__pose(pose);

            RosTopicManager::getInstance()->publishMessage<arm_idl::msg::TaskPositionWaypoint>("arm/task_position_waypoint", wp); 
            std::cout << CYAN << "Published task position waypoint cmd!" << std::endl; 

        }
        else{
            std::cout << RED << "Unsupported waypoint type!" << std::endl; 
        }
    }
    else if ("enable" == anInput)
    {
        std::cout << GREEN << "Hardware:";
        std::string hardwareName; 
        std::getline(std::cin, hardwareName); 

        arm_idl::msg::Enable cmd; 
        cmd.set__enabled(true); 

        RosTopicManager::getInstance()->publishMessage<arm_idl::msg::Enable>("arm/enable", cmd); 
        std::cout << CYAN << "Sent enable command for " << hardwareName << std::endl; 
    }
    else if ("disable" == anInput)
    {
        std::cout << GREEN << "Hardware:";
        std::string hardwareName; 
        std::getline(std::cin, hardwareName); 

        arm_idl::msg::Enable cmd; 
        cmd.set__enabled(false); 

        RosTopicManager::getInstance()->publishMessage<arm_idl::msg::Enable>("arm/enable", cmd); 
        std::cout << CYAN << "Sent disable command for " << hardwareName << std::endl; 

    }
    else if ("stow" == anInput)
    {
        // send stow cmd
    }
    else if ("deploy" == anInput)
    {
        // send deploy cmd
    }
    else{
        std::cout << RED << "Unsupported command!" << std::endl; 
    }
}

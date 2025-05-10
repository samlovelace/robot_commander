
#include "InputHandler.h"
#include <vector> 
#include <iostream>
#include <string> 

#include "RosTopicManager.hpp"
#include "arm_idl/msg/joint_position_waypoint.hpp"

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
                
                jntPosCmd[i] = std::stoi(posStr); 
            }

            arm_idl::msg::JointPositionWaypoint cmd; 
            cmd.set__positions(jntPosCmd); 
            cmd.set__tolerances(jntTol); 
            RosTopicManager::getInstance()->publishMessage<arm_idl::msg::JointPositionWaypoint>("joint_position_waypoint", cmd); 
            std::cout << CYAN << "Published joint position waypoint cmd!" << std::endl; 
        }
        else{
            std::cout << RED << "Unsupported waypoint type!" << std::endl; 
        }
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

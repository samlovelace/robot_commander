
#include "InputHandler.h"
#include <vector> 
#include <iostream>
#include <string> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "RosTopicManager.hpp"
#include "CommanderComms.h"
#include "PointCloudHandler.h"

InputHandler::InputHandler()
{
    mPackagePath = ament_index_cpp::get_package_share_directory("robot_commander");
    std::cout << "Package path: " << mPackagePath << std::endl; 

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
            std::vector<double> jntTol(numJoints, 0.05);

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
    else if ("plan" == anInput)
    {
        std::cout << GREEN << "Task: "; 
        std::string taskType; 
        std::getline(std::cin, taskType);

        if("pick" == taskType)
        {
            std::cout << GREEN << "Object: "; 
            std::string objectType; 
            std::getline(std::cin, objectType); 

            // read in point cloud of object
            // TODO: Move this somehwere else because it will grow 
            std::string cloudFile; 
            if("test" == objectType || "rectPrism" == objectType || "rectangularPrism" == objectType)
            {    
                cloudFile = "rectangularPrism.ply"; 
            }
            else if ("simple" == objectType)
            {
                cloudFile = "simple.ply"; 
            }
            else if ("box" == objectType)
            {
                cloudFile = "Box.ply"; 
            }
            else if ("cylinder" == objectType)
            {
                cloudFile = "Cylinder.ply"; 
            }
            else if ("banana" == objectType)
            {
                cloudFile = "banana.ply"; 
            }
            else 
            {
                std::cerr << "Object type: " << objectType << " not supported. Please add .ply file to models dir"; 
                return; 
            }

            std::string file = mPackagePath + "/objects/" + cloudFile; 
            sensor_msgs::msg::PointCloud2 cloud; 

            if(!PointCloudHandler::fromFile(file, cloud))
            {
                std::cerr << "Failed to send pick plan request"; 
                return; 
            }

            arm_idl::msg::PlanCommand cmd;  
            cmd.set__operation_type(arm_idl::msg::PlanCommand::PICK);

            cmd.set__object_id(objectType);
            cmd.set__object_type(objectType);  

            // TODO: get this from config somehow 
            geometry_msgs::msg::Point centroid_gl; 
            centroid_gl.set__x(0); 
            centroid_gl.set__y(0); 
            centroid_gl.set__z(0); 

            cmd.set__pick_obj_centroid_gl(centroid_gl); 
            cmd.set__pick_obj_point_cloud_gl(cloud); 

            cmd.set__object_type("box"); 
            geometry_msgs::msg::Pose placePose; 
            cmd.set__place_pose(placePose); 

            RosTopicManager::getInstance()->publishMessage<arm_idl::msg::PlanCommand>("arm/command", cmd); 
        }
        
    }
    else if (anInput == "help" || anInput == "--help" || anInput == "-h") 
    {
        std::cout << R"(
        Arm Commander Tool
        -----------------------------
        Interact with the arm control module by sending waypoints and control commands.
        
        Usage:
        <tool_name> [command]
        
        Available Commands:
        
        waypoint         Send a waypoint to the arm.s
                            • jointPos  - Joint-space position command
                            • taskPos   - Task-space pose command (x, y, z, quaternion)
        
        enable           Enable a specific hardware
        disable          Disable a specific hardware
        stow             (Not yet implemented)
        deploy           (Not yet implemented)
        
        Type 'help' or '--help' to show this message again.
        )" << std::endl;
        std::string test; 
        std::getline(std::cin, test); 
    }       
    else{
        std::cout << RED << "Unsupported command!" << std::endl; 
    }
}

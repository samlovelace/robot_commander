
#include "InputHandler.h"
#include <vector> 
#include <iostream>
#include <string> 
#include <vector> 
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

            robot_idl::msg::JointPositionWaypoint cmd; 
            cmd.set__positions(jntPosCmd); 
            cmd.set__tolerances(jntTol); 
            RosTopicManager::getInstance()->publishMessage<robot_idl::msg::JointPositionWaypoint>("arm/joint_position_waypoint", cmd); 
            std::cout << CYAN << "Published joint position waypoint cmd!" << std::endl; 
        }
        else if ("jointVel" == waypointType)
        {
            std::cout << "Num Joints: "; 
            std::string numJointsStr; 
            std::getline(std::cin, numJointsStr);

            int numJoints = std::stoi(numJointsStr); 
            std::vector<double> jntVelCmd(numJoints);
            std::vector<double> jntTol(numJoints, 0.05);

            for(int i = 0; i < numJoints; i++)
            {
                std::string posStr; 
                std::cout << "Joint " << i << ": "; 
                std::getline(std::cin, posStr);
                
                jntVelCmd[i] = std::stod(posStr); 
            }

            robot_idl::msg::JointVelocityWaypoint cmd; 
            cmd.set__velocities(jntVelCmd); 
            cmd.set__tolerances(jntTol); 
            RosTopicManager::getInstance()->publishMessage<robot_idl::msg::JointVelocityWaypoint>("arm/joint_velocity_waypoint", cmd); 
            std::cout << CYAN << "Published joint velocity waypoint cmd!" << std::endl; 
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

            robot_idl::msg::TaskPositionWaypoint wp; 
            wp.set__command_frame(cmdFrame); 
            wp.set__tolerance(jntTol);
            wp.set__pose(pose);

            RosTopicManager::getInstance()->publishMessage<robot_idl::msg::TaskPositionWaypoint>("arm/task_position_waypoint", wp); 
            std::cout << CYAN << "Published task position waypoint cmd!" << std::endl; 

        }
        else if ("taskVel" == waypointType)
        {
            std::vector<std::string> axes = {"x", "y", "z", "wx", "wy", "wz"}; 
            std::vector<double> twist(6); 
            
            for(int i = 0; i < axes.size(); i++)
            {
                std::string tmp; 
                std::cout << axes[i] << ": "; 
                std::getline(std::cin, tmp); 

                twist[i] = std::stod(tmp); 
            }

            std::vector<double> taskTol(6, 0.1);
            std::string commandFrame = "base"; 

            std_msgs::msg::String cmdFrame;
            cmdFrame.set__data(commandFrame);  
            
            geometry_msgs::msg::Vector3 linTol; 
            linTol.set__x(taskTol[0]); 
            linTol.set__y(taskTol[1]);
            linTol.set__z(taskTol[2]);

            geometry_msgs::msg::Vector3 angTol; 
            angTol.set__x(taskTol[3]); 
            angTol.set__y(taskTol[4]);
            angTol.set__z(taskTol[5]);
            
            geometry_msgs::msg::Twist tol; 
            tol.set__angular(angTol); 
            tol.set__linear(linTol);
            
            geometry_msgs::msg::Vector3 linGoal; 
            linGoal.set__x(twist[0]); 
            linGoal.set__y(twist[1]);
            linGoal.set__z(twist[2]);

            geometry_msgs::msg::Vector3 angGoal; 
            angGoal.set__x(twist[3]); 
            angGoal.set__y(twist[4]);
            angGoal.set__z(twist[5]);
            
            geometry_msgs::msg::Twist goal; 
            goal.set__angular(angGoal); 
            goal.set__linear(linGoal);

            robot_idl::msg::TaskVelocityWaypoint wp; 
            wp.set__command_frame(cmdFrame); 
            wp.set__tolerance(tol);
            wp.set__goal(goal); 

            RosTopicManager::getInstance()->publishMessage<robot_idl::msg::TaskVelocityWaypoint>("arm/task_velocity_waypoint", wp); 
            std::cout << CYAN << "Published task velocity waypoint cmd!" << std::endl;

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

        robot_idl::msg::Enable cmd; 
        cmd.set__enabled(true); 

        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::Enable>("arm/enable", cmd); 
        std::cout << CYAN << "Sent enable command for " << hardwareName << std::endl; 
    }
    else if ("disable" == anInput)
    {
        std::cout << GREEN << "Hardware:";
        std::string hardwareName; 
        std::getline(std::cin, hardwareName); 

        robot_idl::msg::Enable cmd; 
        cmd.set__enabled(false); 

        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::Enable>("arm/enable", cmd); 
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
    else if ("pick" == anInput)
    {
        // TODO: add full implementation 
        robot_idl::msg::ManipulationCommand cmd; 
        cmd.set__cmd(robot_idl::msg::ManipulationCommand::CMD_PICK); 

        RosTopicManager::getInstance()->publishMessage("arm/command", cmd); 
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
            else if ("cube" == objectType)
            {
                cloudFile = "cube.ply"; 
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

            robot_idl::msg::PlanCommand cmd;  
            cmd.set__operation_type(robot_idl::msg::PlanCommand::PICK);

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

            RosTopicManager::getInstance()->publishMessage<robot_idl::msg::PlanCommand>("arm/command", cmd); 
        }
        
    }
    else if ("find_object" == anInput)
    {
        std::cout << GREEN << "Type: "; 
        std::string objectType; 
        std::getline(std::cin, objectType); 

        std_msgs::msg::String action;
        action.set__data("find_object"); 

        std_msgs::msg::String type;
        type.set__data(objectType);  

        robot_idl::msg::Command cmd; 
        cmd.set__command(action); 
        cmd.set__object_type(type); 

        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::Command>("vision/command", cmd); 
    }
    else if ("stopVision" == anInput)
    {
        std_msgs::msg::String action;
        action.set__data("disable"); 

        std_msgs::msg::String type;
        type.set__data("none");  

        robot_idl::msg::Command cmd; 
        cmd.set__command(action); 
        cmd.set__object_type(type); 

        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::Command>("vision/command", cmd); 
    }
    else if ("vehPose" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Enter goal pose (x, y, yaw): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        robot_idl::msg::AbvVec3 data; 
        data.set__x(vehPose[0]); 
        data.set__y(vehPose[1]); 
        data.set__yaw(vehPose[2]); 

        robot_idl::msg::AbvCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("pose");
        
        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::AbvCommand>("abv/command", cmd); 
        std::cout << "Published goal waypoint (x y yaw): " << vehPose[0] << ", " << vehPose[1] << ", " << vehPose[2] << std::endl; 
    }
    else if ("vehVel" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Enter goal velocity (xd, yd, yawd): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        robot_idl::msg::AbvVec3 data; 
        data.set__x(vehPose[0]); 
        data.set__y(vehPose[1]); 
        data.set__yaw(vehPose[2]); 

        robot_idl::msg::AbvCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("velocity");
        
        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::AbvCommand>("abv/command", cmd); 
        std::cout << "Published goal velocity waypoint (xd yd yawd): " << vehPose[0] << ", " << vehPose[1] << ", " << vehPose[2] << std::endl;
    }
    else if ("gpcGoal" == anInput)
    {
        std::string mode; 
        std::cout << GREEN << "Mode: "; 
        std::getline(std::cin, mode);
        
        if("setpoint" == mode)
        {
            std::cout << GREEN << "Goal Size: "; 
            int goalSize = 0; 
            std::string goalSizeStr; 
            std::getline(std::cin, goalSizeStr); 
            goalSize = std::stoi(goalSizeStr); 

            std::string goalString; 
            std::cout << "Goal: "; 
            std::getline(std::cin, goalString);

            std::vector<double> goalVec; 
            goalVec.resize(goalSize); 
            std::istringstream iss(goalString);
            
            for (int i = 0; i < goalSize; ++i)
            {
                if (!(iss >> goalVec[i]))
                {
                    std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                    return;
                }
            }

            robot_idl::msg::GpcGoal goal; 
            goal.set__mode(0); 
            goal.set__x_ref(goalVec); 

            RosTopicManager::getInstance()->publishMessage<robot_idl::msg::GpcGoal>("gpc/goal", goal); 

            std::cout << "Published GPC SETPOINT of ";  
            for(int i = 0; i < goalVec.size(); i++)
            {
                std::cout << goalVec[i] << ", "; 
            }
            std::cout << std::endl; 
            
        }
        else
        {
            std::cout << RED << "Unsupported GPC mode" << std::endl; 
            return; 
        }
    }
    else if ("abvWp" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Goal pose (x, y, yaw): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        std::vector<double> goalVec = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        goalVec[0] = vehPose[0]; // x
        goalVec[1] = vehPose[1]; // y
        goalVec[6] = vehPose[2]; // yaw

        robot_idl::msg::GpcGoal goal; 
        goal.set__mode(0); // always setpoint for wp 
        goal.set__x_ref(goalVec); 

        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::GpcGoal>("gpc/goal", goal); 

        std::cout << "Published GPC SETPOINT of ";  
        for(int i = 0; i < goalVec.size(); i++)
        {
            std::cout << goalVec[i] << ", "; 
        }
        std::cout << std::endl; 
    }
    else if ("abvTraj" == anInput)
    {
        std::string trajType; 
        std::cout << GREEN << "Type: "; 
        std::getline(std::cin, trajType);

        robot_idl::msg::GpcGoal goal; 
        goal.set__mode(robot_idl::msg::GpcGoal::MODE_TRAJECTORY); 

        // default
        auto trajTypeToSet = robot_idl::msg::GpcGoal::TRAJ_LINE2D; 
        std::string trajTypeForLog = "DEFAULT"; 

        if("circle" == trajType)
        {
            trajTypeToSet = robot_idl::msg::GpcGoal::TRAJ_CIRCLE2D;
            trajTypeForLog = "Circle2D";  
        }
        else if ("line" == trajType)
        {
            trajTypeToSet = robot_idl::msg::GpcGoal::TRAJ_LINE2D;
            trajTypeForLog = "Line2D"; 
        }
        else
        {
            std::cout << RED << "Unsupported trajectory type" << std::endl;
            return; 
        }

        goal.set__trajectory_type(trajTypeToSet); 

        RosTopicManager::getInstance()->publishMessage<robot_idl::msg::GpcGoal>("gpc/goal", goal); 
        std::cout << "Sent " << trajTypeForLog << " trajectory command for ABV!" << std::endl; 
    }
    else if (anInput == "help" || anInput == "--help" || anInput == "-h") 
    {
        std::cout << R"(
        Robot Commander Tool
        -----------------------------
        Interact with the arm control module by sending waypoints and control commands.
        
        Usage:
        <tool_name> [command]
        
        Available Commands:
        
        ####################### A.R.M #####################
        
        waypoint         Send a waypoint to the arm.s
                            • jointPos  - Joint-space position command
                            • taskPos   - Task-space pose command (x, y, z, quaternion)

        plan             Send a point cloud to do manipulation planning 
        
        enable           Enable a specific hardware
        disable          Disable a specific hardware
        stow             (Not yet implemented)
        deploy           (Not yet implemented)

        ####################### Vision #######################

        find_object         command vision system to look for a certain type of object
        stopVision          disable vision system
        
        Type 'help' or '--help' to show this message again.
        )" << std::endl;
        std::string test; 
        std::getline(std::cin, test); 
    }       
    else{
        std::cout << RED << "Unsupported command!" << std::endl; 
    }
}

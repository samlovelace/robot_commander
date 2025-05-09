
#include "InputHandler.h"

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
            std::cout << CYAN << "Sending Joint Position Waypoint!" << std::endl;
        }
    }
    else{
        std::cout << RED << "Unsupported command!" << std::endl; 
    }
}

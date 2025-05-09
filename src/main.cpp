#include <iostream> 
#include "Colors.hpp"
#include "InputHandler.h"

int main()
{
    std::cout << R"(
     ___      .______      .___  ___.      ______   ______   .___  ___. .___  ___.      ___      .__   __.  _______   _______ .______      
    /   \     |   _  \     |   \/   |     /      | /  __  \  |   \/   | |   \/   |     /   \     |  \ |  | |       \ |   ____||   _  \     
   /  ^  \    |  |_)  |    |  \  /  |    |  ,----'|  |  |  | |  \  /  | |  \  /  |    /  ^  \    |   \|  | |  .--.  ||  |__   |  |_)  |    
  /  /_\  \   |      /     |  |\/|  |    |  |     |  |  |  | |  |\/|  | |  |\/|  |   /  /_\  \   |  . `  | |  |  |  ||   __|  |      /     
 /  _____  \  |  |\  \----.|  |  |  |    |  `----.|  `--'  | |  |  |  | |  |  |  |  /  _____  \  |  |\   | |  '--'  ||  |____ |  |\  \----.
/__/     \__\ | _| `._____||__|  |__|     \______| \______/  |__|  |__| |__|  |__| /__/     \__\ |__| \__| |_______/ |_______|| _| `._____|
    )" << std::endl;

    std::string command; 
    InputHandler inputHandler; 

    while(true)
    {
        std::cout << GREEN << "Command: "; 
        std::getline(std::cin, command); 

        if("exit" == command)
        {
            std::cout << "Exiting..." << std::endl; 
            break; 
        }

        inputHandler.handle(command); 
    }
}

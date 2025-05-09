#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H
 
#include <iostream>
#include <string> 
#include "Colors.hpp"
 
class InputHandler 
{ 
public:
    InputHandler();
    ~InputHandler();

    void handle(const std::string& anInput); 

private:
   
};
#endif //INPUTHANDLER_H
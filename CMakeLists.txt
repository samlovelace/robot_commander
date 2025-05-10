cmake_minimum_required(VERSION 3.8)
project(arm_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_idl REQUIRED)

set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/CommanderComms.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/InputHandler.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

include_directories(${PROJECT_NAME} 
    ${CMAKE_CURRENT_LIST_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  arm_idl
)

install(TARGETS ${PROJECT_NAME} 
	DESTINATION lib/${PROJECT_NAME}
)
ament_package()

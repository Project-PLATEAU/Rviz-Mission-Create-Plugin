cmake_minimum_required(VERSION 3.0.2)
project(mission_plan_msg)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        message_generation
        nav_msgs
        roscpp
        std_msgs
        geometry_msgs
        )


add_message_files(
        FILES
        MissionPlan.msg
)


generate_messages(
        DEPENDENCIES
        nav_msgs std_msgs geometry_msgs
)


catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES mission_plan_msg
        CATKIN_DEPENDS  nav_msgs roscpp std_msgs message_runtime
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)


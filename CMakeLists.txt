cmake_minimum_required(VERSION 2.8.3)
project(cdxbot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslaunch
    std_msgs
    geometry_msgs
    message_generation
    )
# roslaunch_add_file_check(launch)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(Confuse REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    gc_cmd.msg
    gc_out.msg
    pc_cmd.msg
    pc_out.msg
    sc_cmd.msg
    sc_out.msg
    vc_cmd.msg
    vc_out.msg
    )

## Generate services in the 'srv' folder
add_service_files(
    FILES
    gantryEmergencyStop.srv
    gantryGetCurrentPosition.srv
    gantryHome.srv
    gantryMotorsToggle.srv
    gantryMove.srv
    gantrySetAccelerations.srv
    gantrySetAxisStepsPerUnit.srv
    gantrySetFeedrates.srv
    gantrySetUnits.srv
    nodeInit.srv
    nodeShutdown.srv
    pipetterAspirate.srv
    pipetterDispense.srv
    pipetterEjectTip.srv
    pipetterEmergencyStop.srv
    pipetterHome.srv
    pipetterMakeContainerGeometry.srv
    pipetterMakeDeckGeometry.srv
    pipetterMakeLiquidClass.srv
    pipetterMoveZ.srv
    pipetterPickUpTip.srv
    shakerReset.srv
    pipetterSetLLDActive.srv
    shakerSetFreq.srv
    shakerSetPower.srv
    shakerStart.srv
    shakerStop.srv
    gc_cmd_s.srv
    pc_cmd_s.srv
    vc_cmd_s.srv
    pipetterGetContainerVolume.srv
    )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES smoothie zeus eppendorf
    #  CATKIN_DEPENDS other_catkin_pkg
    # DEPENDS system_lib
    CATKIN_DEPENDS message_runtime qt_startcommand)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(container
    # src/${PROJECT_NAME}/Container.cpp
    Container.cpp)

add_library(cdxbot
    CDXBot.cpp
    Container.cpp)

add_library(zeus
    libzeus.cpp)

add_library(eppendorf
    libeppendorf.cpp)

add_library(smoothie
    libsmoothie.cpp)

add_library(ramps
    libramps.cpp)

add_library(teleshake
    libteleshake.cpp)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(cdxbot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(shakerControllerNode shakerControllerNode.cpp)
add_executable(gantryControllerNode gantryControllerNode.cpp)
add_executable(pipetterControllerNode pipetterControllerNode.cpp)
add_executable(visionControllerNode visionControllerNode.cpp)
add_executable(CDXBotNode Container.cpp CDXBot.cpp CDXBotNode.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(shakerControllerNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gantryControllerNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pipetterControllerNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(visionControllerNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(CDXBotNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(CDXBotNode
    ${catkin_LIBRARIES}
    container
    cdxbot
    )
target_link_libraries(gantryControllerNode
    ${catkin_LIBRARIES}
    dl
    )

target_link_libraries(pipetterControllerNode
    ${catkin_LIBRARIES}
    dl
    )
target_link_libraries(visionControllerNode
    ${catkin_LIBRARIES}
    )
target_link_libraries(shakerControllerNode
    ${catkin_LIBRARIES}
    dl
    )
target_link_libraries(smoothie
    ${catkin_LIBRARIES}
    pthread
    )

SET(GCC_LINKER_FLAGS "-l -E")
target_link_libraries(smoothie
    ${catkin_LIBRARIES}
    pthread
    )
ADD_DEFINITIONS(${GCC_LINKER_FLAGS})
target_link_libraries(smoothie
    ${catkin_LIBRARIES}
    pthread
    )
target_link_libraries(ramps
    ${catkin_LIBRARIES}
    pthread
    )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS cdxbot CDXBotNode
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #LIBRARY DESTINATION "./lib"
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cdxbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
ADD_DEFINITIONS(-std=c++11 -g)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ldl -fPIC -lpthread")

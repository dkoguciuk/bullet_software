cmake_minimum_required(VERSION 2.8.3)
project(bullet_kinematics)

#####################################################################
######################### CATKIN PACKAGES ###########################
#####################################################################

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
  bullet_core
)
find_package(orocos_kdl)

#####################################################################
############################# C++11 #################################
#####################################################################

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#####################################################################
############################# BOOST #################################
#####################################################################

find_package(Boost 1.54 REQUIRED system filesystem)
if (Boost_FOUND)
    message("-- Boost ${Boost_VERSION} was found")
    include_directories(${Boost_INCLUDE_DIRS})
else(Boost_FOUND)
    message(FATAL_ERROR "Boost 1.54 was not found")
endif(Boost_FOUND)

#####################################################################
########################### CATKIN MAGIC ############################
#####################################################################

# Generate messages
add_message_files(FILES LegPosition.msg RobotPose.msg)

# Generate services
add_service_files(FILES ConvertPointFrame.srv ZeroJoints.srv SolveFwd.srv)

# Generate all
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

# Catkin magic here
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES bullet_kinematics
#  CATKIN_DEPENDS geometry_msgs orocos_kdl roscpp std_msgs message_runtime
  DEPENDS bullet_core
)

#####################################################################
############################### EXEC ################################
#####################################################################

# Include dirs
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} include/bullet_kinematics)

# Declare a cpp executable
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}  FOLLOW_SYMLINKS include/bullet_kinematics/*.h include/bullet_kinematics/*.hpp src/*.cpp)
add_executable(kinematics_solver ${SOURCES})
add_dependencies(kinematics_solver bullet_kinematics_generate_messages_cpp bullet_core)
target_link_libraries(kinematics_solver ${catkin_LIBRARIES})

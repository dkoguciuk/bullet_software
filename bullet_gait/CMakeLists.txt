cmake_minimum_required(VERSION 2.8.3)
project(bullet_gait)

#####################################################################
######################### CATKIN PACKAGES ###########################
#####################################################################

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
  bullet_kinematics
  bullet_core
)

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

# Generate services
add_service_files(FILES StandUpLayDown.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Catkin magic here
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES bullet_gait
  CATKIN_DEPENDS message_runtime
  DEPENDS bullet_core bullet_kinematics
)

#####################################################################
############################### EXEC ################################
#####################################################################

# Include dirs
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} include/bullet_gait)

## Declare a C++ executable
add_executable(gait_generator src/main_gait.cpp src/gait_generator.cpp src/gait_generator_simple.cpp include/bullet_gait/gait_generator.h include/bullet_gait/gait_generator_simple.h)
add_dependencies(gait_generator bullet_kinematics_generate_messages_cpp bullet_core)
target_link_libraries(gait_generator ${catkin_LIBRARIES})

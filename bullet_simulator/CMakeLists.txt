cmake_minimum_required(VERSION 2.8.3)
project(bullet_simulator)

#####################################################################
######################### CATKIN PACKAGES ###########################
#####################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  control_msgs
  bullet_core
)

#####################################################################
############################# C++11 #################################
#####################################################################

# C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
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

# Catkin magic here
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bullet_gazebo
#  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS bullet_core
)

#####################################################################
############################### EXEC ################################
#####################################################################

# Include dirs
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

# Declare a cpp executable
add_executable(pid_test src/main_pid_test.cpp)
add_dependencies(pid_test bullet_core)
target_link_libraries(pid_test ${catkin_LIBRARIES})

# Declare a cpp executable
add_executable(bullet_kinematics_translator src/main_bullet_kinematics_translator.cpp)
add_dependencies(bullet_kinematics_translator bullet_core)
target_link_libraries(bullet_kinematics_translator ${catkin_LIBRARIES})

#####################################################################
############################# PLUGINS ###############################
#####################################################################

add_subdirectory(plugins)

cmake_minimum_required(VERSION 2.8.3)
project(bullet_driver)

#####################################################################
######################### CATKIN PACKAGES ###########################
#####################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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
########################### CATKIN MAGIC ############################
#####################################################################

# Catkin magic here
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES bullet_driver
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

#####################################################################
############################### EXEC ################################
#####################################################################

include_directories(
    ${catkin_INCLUDE_DIRS}
    include/bullet_driver
)

# qtcreator hack
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h include/*.hpp src/*.hpp src/stm_communication_manager.cpp src/bullet_robot.cpp src/bullet_spi.cpp)

## Configure servo
add_executable(configure_servo src/main_configure_servo.cpp ${SOURCES} )
add_executable(configure_servo_id_br src/main_configure_servo_id_br.cpp ${SOURCES})

## Connect to servo
add_executable(connect_to_servo src/main_connect_to_servo.cpp ${SOURCES})

## Bullet driver
add_executable(bullet_driver src/main_bullet_driver.cpp ${SOURCES})
target_link_libraries(bullet_driver ${catkin_LIBRARIES})

## Reset all servos
add_executable(reset_all_servos_positions src/main_reset.cpp ${SOURCES})
target_link_libraries(reset_all_servos_positions ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(bullet_core)

#####################################################################
######################### CATKIN PACKAGES ###########################
#####################################################################

find_package(catkin REQUIRED COMPONENTS roscpp)

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
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#####################################################################
########################### CATKIN MAGIC ############################
#####################################################################

# Catkin magic here
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES bullet_core
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

#####################################################################
############################### EXEC ################################
#####################################################################

include_directories(${catkin_INCLUDE_DIRS})
add_library(bullet_core include/bullet_core/bullet_core.hpp)
set_target_properties(bullet_core PROPERTIES LINKER_LANGUAGE CXX)

#ifndef BULLET_CORE_HPP
#define BULLET_CORE_HPP


#include <iostream>
#include <ros/ros.h>


#define COUT_RESET   "\033[0m"                   /* Reset */
#define COUT_BLACK   "\033[30m"                  /* Black */
#define COUT_RED     "\033[31m"                  /* Red */
#define COUT_GREEN   "\033[32m"                  /* Green */
#define COUT_YELLOW  "\033[33m"                  /* Yellow */
#define COUT_BLUE    "\033[34m"                  /* Blue */
#define COUT_MAGENTA "\033[35m"                  /* Magenta */
#define COUT_CYAN    "\033[36m"                  /* Cyan */
#define COUT_WHITE   "\033[37m"                  /* White */
#define COUT_BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define COUT_BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define COUT_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define COUT_BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define COUT_BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define COUT_BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define COUT_BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define COUT_BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


/**
 * @brief bullet_info                   Print debug info with the node name in the beginning.
 * @param msg                           Message to print.
 */
inline void bullet_info(const std::string &msg)
{
    std::cout << COUT_BOLDGREEN << ros::this_node::getName() << COUT_RESET << " " << msg << std::endl;
}

/**
 * @brief bullet_warning                Print debug warning with the node name in the beginning.
 * @param msg                           Message to print.
 */
inline void bullet_warning(const std::string &msg)
{
    std::cout << COUT_BOLDYELLOW << ros::this_node::getName() << COUT_RESET << " " << msg << std::endl;
}

/**
 * @brief bullet_error                  Print debug error with the node name in the beginning.
 * @param msg                           Message to print.
 */
inline void bullet_error(const std::string &msg)
{
    std::cout << COUT_BOLDRED << ros::this_node::getName() << COUT_RESET << " " << msg << std::endl;
}


#endif //BULLET_CORE_HPP

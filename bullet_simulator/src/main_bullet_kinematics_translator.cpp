#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include  "bullet_core/bullet_core.hpp"

using namespace std;

// ============================================================================================
// ======================================== GLOBAL VARS =======================================
// ============================================================================================

/**
 * @brief pub_joints                        Joint publisher of the node.
 */
ros::Publisher pub_joints[18];

/**
 * @brief sub_state                         Joint state subscriber.
 */
ros::Subscriber sub_joints;

// ============================================================================================
// ========================================= CALLBACKS ========================================
// ============================================================================================

/**
 * @brief jointsCallback                    Callback for
 * @param msg                               Received message.
 */
void jointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for (int i=0; i<18; ++i)
    {
        if (i==0)
        {
            cout << "LEG_1_1 " << msg->data[i] << endl;
        }
        std_msgs::Float64 out;
        out.data = msg->data[i];
        pub_joints[i].publish(out);
    }
//    cout << "SET POINT = " << msg->set_point << endl;
//    cout << "VALUE     = " << msg->process_value << endl;
//    cout << "ERROR     = " << msg->error << endl;
}

// ============================================================================================
// ========================================== SIGINT ==========================================
// ============================================================================================

/**
 * @brief cleanAndExit              Replacement SIGINT handler.
 * @param sig                       Signal.
 */
void cleanAndExit(int sig=0)
{
    bullet_info("Exiting node");
    ros::shutdown();
    exit(0);
}

// ============================================================================================
// ======================================= MAIN ROUTINE =======================================
// ============================================================================================

/**
 * @brief Entry point of node
 */
int main(int argc, char** argv)
{
    // Init this node
    ros::init(argc, argv, "bullet_kinematics_translator", ros::init_options::NoSigintHandler);
    signal(SIGINT, cleanAndExit);
    bullet_info("Starting...");

    // ROS nodehandle
    ros::NodeHandle n("");

    // Namespace
    std::string ns = n.getNamespace();
    std::string parent_ns = ros::names::parentNamespace(ns);

    // Create publishers and subscribers
    for (int leg=1; leg<=6; leg++)
    {
        for (int joint=1; joint<=3; joint++)
        {
            //string topic = string("/" + parent_ns +"/leg_") + std::to_string(leg) + "_" + std::to_string(joint) + string("_ctrl/command");
            string topic = string("leg_") + std::to_string(leg) + "_" + std::to_string(joint) + string("_ctrl/command");
            pub_joints[(leg-1)*3+(joint-1)] = n.advertise<std_msgs::Float64>(topic, 1);
        }
    }
    sub_joints = n.subscribe("joint_goals", 1, jointsCallback);
    bullet_info("Topics connected");

    // Spin
    ros::spin();

    // Exit
    return 0;
}



#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include  "bullet_core/bullet_core.hpp"

using namespace std;

// ============================================================================================
// ======================================== GLOBAL VARS =======================================
// ============================================================================================

/**
 * @brief pub_joints                        Joint publisher of the node.
 */
ros::Publisher pub_joint;

/**
 * @brief sub_state                         Joint state subscriber.
 */
ros::Subscriber sub_state;

// ============================================================================================
// ========================================= CALLBACKS ========================================
// ============================================================================================

/**
 * @brief timerCallback                     Timer callback to send a value to controller.
 */
void timerCallback(const ros::TimerEvent&)
{
    static bool positive = true;
    std_msgs::Float64 msg;
    if (positive)
    {
        cout << "POS" << endl;
        msg.data = 0.5;
        positive = false;
    } else
    {
        cout << "NEG" << endl;
        msg.data = -0.5;
        positive = true;
    }
    pub_joint.publish(msg);
}

/**
 * @brief globalRobotPoseCallback           Callback for gait generation in world frame.
 * @param msg                               Received message.
 */
void stateCallback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    cout << "SET POINT = " << msg->set_point << endl;
    cout << "VALUE     = " << msg->process_value << endl;
    cout << "ERROR     = " << msg->error << endl;
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
    bullet_info("Zero the position");
    std_msgs::Float64 msg;
    msg.data = 0.0;
    pub_joint.publish(msg);

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
    ros::init(argc, argv, "pid_test", ros::init_options::NoSigintHandler);
    signal(SIGINT, cleanAndExit);
    bullet_info("Starting...");

    // ROS nodehandle
    ros::NodeHandle n("~");

    // Create publishers and subscribers
    sub_state = n.subscribe("/bullet/leg_1_1_ctrl/state", 1, stateCallback);
    pub_joint = n.advertise<std_msgs::Float64>("/bullet/leg_1_1_ctrl/command", 1);
    bullet_info("Waiting for subscribers..");
    while (!pub_joint.getNumSubscribers());
    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);
    bullet_info("Topics connected");

    // Spin
    bullet_info("Initialization done - publishing..");
    ros::spin();

    // Exit
    return 0;
}



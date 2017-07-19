#include "ros/ros.h"
#include "solver.h"
#include "kinematics_services.h"
#include "bullet_kinematics/RobotPose.h"
#include "kdl/frames.hpp"
#include "boost/format.hpp"


#include  "bullet_core/bullet_core.hpp"

using namespace std;

// ============================================================================================
// ======================================== GLOBAL VARS =======================================
// ============================================================================================

/**
 * @brief pub_joints                        Joints publisher of the node.
 */
ros::Publisher pub_joints;

// ============================================================================================
// ========================================= CALLBACKS ========================================
// ============================================================================================

/**
 * @brief globalRobotPoseCallback           Callback for gait generation in world frame.
 * @param msg                               Received message.
 */
void globalRobotPoseCallback(const bullet_kinematics::RobotPose::ConstPtr& msg)
{
    // Prepare temporary variables
    std_msgs::Float32MultiArray response_message;
    response_message.data.reserve(18);

    KDL::Vector body_position;
    KDL::Rotation body_orientation;

    // Extract body position from message
    body_position.data[0] = msg->body_pose.position.x;
    body_position.data[1] = msg->body_pose.position.y;
    body_position.data[2] = msg->body_pose.position.z;

    cout << "X" << msg->leg_position[0].x << endl;
    cout << "Y" << msg->leg_position[0].y << endl;
    cout << "Z" << msg->leg_position[0].z << endl;

    // Extract body orientation from message
    body_orientation = KDL::Rotation::Quaternion(msg->body_pose.orientation.x,
                                                 msg->body_pose.orientation.y,
                                                 msg->body_pose.orientation.z,
                                                 msg->body_pose.orientation.w);

    // Extract all legs and solve
    for(int i =0; i<6; i++)
    {
        KDL::JntArray result(3);
        KDL::Vector leg_tip_position;
        leg_tip_position.data[0] = msg->leg_position[i].x;
        leg_tip_position.data[1] = msg->leg_position[i].y;
        leg_tip_position.data[2] = msg->leg_position[i].z;

        // Solve inverse kinematics
        try
        {
            Solver::getInstance()->solveInv(leg_tip_position,body_position,body_orientation,result, i);
            cout << "XS" << result(0) << endl;
            cout << "YS" << result(1) << endl;
            cout << "ZS" << result(2) << endl;
            response_message.data.push_back(result(0));
            response_message.data.push_back(result(1));
            response_message.data.push_back(-result(2));
        } catch (std::runtime_error e)
        {
            bullet_error(string("Given pose cannot be reached: ") + string(e.what()));
            return;
        }
    }

    // Publish results
    pub_joints.publish(response_message);
    return;
}

/**
 * @brief bodyRobotPoseCallback             Callback for gait generation in body frame.
 * @param msg                               Received message.
 */
void bodyRobotPoseCallback(const bullet_kinematics::RobotPose::ConstPtr& msg)
{
    // Prepare temporary variables
    std_msgs::Float32MultiArray response_message;
    response_message.data.reserve(18);

    // Extract all legs and solve
    for(int i =0; i<6; i++)
    {
        KDL::JntArray result(3);
        KDL::Vector leg_tip_position;
        leg_tip_position.data[0] = msg->leg_position[i].x;
        leg_tip_position.data[1] = msg->leg_position[i].y;
        leg_tip_position.data[2] = msg->leg_position[i].z;

        // Solve inverse kinematics
        try
        {
            Solver::getInstance()->solveInvBody(leg_tip_position, result, i);
            response_message.data.push_back(result(0));
            response_message.data.push_back(result(1));
            response_message.data.push_back(-result(2));
        } catch (std::runtime_error e)
        {
            bullet_error(string("Given pose cannot be reached: ") + string(e.what()));
            return;
        }
    }

    // Publish results
    pub_joints.publish(response_message);
    return;
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
    ros::init(argc, argv, "kinematics_solver");
    bullet_info("Starting...");

    // ROS nodehandle
    ros::NodeHandle n;
    ros::Subscriber sub_robot_pose;

    // Namespace
    std::string ns = n.getNamespace();
    std::string parent_ns = ros::names::parentNamespace(ns);

    // Read robot description from parameter server
    std::vector<double> leg_len, hip_pos_x, hip_pos_y, hip_angle;
    if(!n.getParam("kinematics_description/leg_segment_lenghts", leg_len))
    {
        bullet_error("Leg segment legths not found on server!");
        return -1;
    }
    if(!n.getParam("kinematics_description/leg_x_pos_in_root_frame", hip_pos_x))
    {
        bullet_error("Hips x postitions not found on server!");
        return -1;
    }
    if(!ros::param::get("kinematics_description/leg_y_pos_in_root_frame", hip_pos_y))
    {
        bullet_error("Hips y postitions not found on server!");
        return -1;
    }
    if(!ros::param::get("kinematics_description/leg_angle_in_root_frame", hip_angle))
    {
        bullet_error("Hips angle in the root frame not found on server!");
        return -1;
    }
    bullet_info("Params read.");

    // Set robot params
    try
    {
        Solver::getInstance()->setRobotParameters(leg_len, hip_pos_x, hip_pos_y, hip_angle);
    } catch(std::invalid_argument e)
    {
        bullet_error(string("Unable to set robot parameters. Solver thrown exception: ") + string(e.what()));
        return -1;
    }
    bullet_info("Params set.");

    /// Start service servers
    ros::ServiceServer zeroJointsService = n.advertiseService(ros::this_node::getName() + "/zero_joints", KinematicServices::zeroJointsHandler);
    ros::ServiceServer convertFrameService = n.advertiseService(ros::this_node::getName() + "/convert_frame", KinematicServices::convertPointFrameHandler);
    ros::ServiceServer solveFwdService = n.advertiseService(ros::this_node::getName() + "/solve_fwd", KinematicServices::solveForwardKinematicsHandler);
    bullet_info("Service servers started.");

    // Get mode
    string mode;
    if(!ros::param::get("kinematics_description/kinematics_mode", mode))
    {
        bullet_error("Kinematics mode not found on server. Add parameter \"body\" or \"global\" to /bullet/kinematics_description/kinematics_mode");
        return -1;
    }

    /// Create joint state advertiser
    pub_joints = n.advertise<std_msgs::Float32MultiArray>("joint_goals",1000);
    bullet_info("Joint command topics created.");

    // Subscribe
    bullet_info("Subscribing to robot_pose...");
    if(mode == "global") sub_robot_pose = n.subscribe("robot_pose", 1000, globalRobotPoseCallback);
    else if(mode == "body") sub_robot_pose = n.subscribe("robot_pose", 1000, bodyRobotPoseCallback);
    else return -1;

    // Spin
    bullet_info("Initialization done - waiting for messages..");
    ros::spin();

    // Exit
    bullet_info("Exiting kinematics node");
    return 0;
}



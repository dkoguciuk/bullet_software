#include "kinematics_services.h"


KinematicServices::KinematicServices()
{
}

bool KinematicServices::solveForwardKinematicsHandler(bullet_kinematics::SolveFwd::Request &req, bullet_kinematics::SolveFwd::Response &res)
{
    KDL::Vector result, result_hip;
    KDL::JntArray j;
    j.resize(3);
    for(int i=0; i<3; i++)
    {
        double d = req.joints.at(i).data;
        j(i) = d;
    }
    Solver::getInstance()->solveFwd(j,result, result_hip, req.leg_index);
    res.result.x = result.data[0];
    res.result.y = result.data[1];
    res.result.z = result.data[2];

    res.result_hip.x = result_hip.data[0];
    res.result_hip.y = result_hip.data[1];
    res.result_hip.z = result_hip.data[2];

    return true;
}

bool KinematicServices::convertPointFrameHandler(bullet_kinematics::ConvertPointFrame::Request &req, bullet_kinematics::ConvertPointFrame::Response &res)
{
    KDL::Vector input(req.pt.x, req.pt.y, req.pt.z);
    KDL::Vector output;

    if(req.command.data == "BODY_TO_HIP")

    {
        output = Solver::getInstance()->bodyToHip(input, req.leg_index);
    }
    else if(req.command.data == "HIP_TO_BODY")
    {
        std::cout << "HIP TO BODY!" << std::endl;
        output = Solver::getInstance()->hipToBody(input, req.leg_index);
    }
    else bullet_error("Wrong command");

    res.result.x = output.data[0];
    res.result.y = output.data[1];
    res.result.z = output.data[2];
    return true;
}

bool KinematicServices::zeroJointsHandler(bullet_kinematics::ZeroJoints::Request &req, bullet_kinematics::ZeroJoints::Response &res)
{    
    extern ros::Publisher pub_joints;

    std_msgs::Float32MultiArray msg;
    msg.data.reserve(18);

    for (int i=0; i<18; i++)
        msg.data.push_back(0);

    pub_joints.publish(msg);

    return true;
}


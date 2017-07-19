#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "bullet_kinematics/RobotPose.h"
#include "bullet_kinematics/SolveFwd.h"
#include "bullet_kinematics/ConvertPointFrame.h"
#include "bullet_kinematics/ZeroJoints.h"
#include "bullet_gait/StandUpLayDown.h"
#include "gait_generator_simple.h"
#include "bullet_core/bullet_core.hpp"

/// Globals (necessary for services)

/**
 * @brief pub                           Robot pose publisher.
 */
ros::Publisher pub_robot_pose;


std::vector<KDL::Vector> base;
ros::ServiceClient svc_zero_joints;
ros::ServiceClient svc_hip_2_body;
ros::ServiceClient svc_solve_fwd;

KDL::Vector body_position = KDL::Vector(0,0,0);
KDL::Rotation body_orientation = KDL::Rotation::RPY(0,0,0);

/**
 * @brief convertToMessage Util function to convert KDL vector to message
 * @param position KDL vector with robot base position
 * @param orientation KDL rotation matrix with robot base orietation
 * @param legs vector of legs posiontns
 * @return ROS message */
bullet_kinematics::RobotPose convertToMessage(KDL::Vector &position, KDL::Rotation &orientation, std::vector<KDL::Vector> &legs)
{
    bullet_kinematics::RobotPose retval;
    retval.body_pose.position.x = position.x();
    retval.body_pose.position.y = position.y();
    retval.body_pose.position.z = position.z();

    double x,y,z,w;
    orientation.GetQuaternion(x,y,z,w);
    retval.body_pose.orientation.w = w;
    retval.body_pose.orientation.x = x;
    retval.body_pose.orientation.y = y;
    retval.body_pose.orientation.z = z;

    for(int i = 0; i < 6; i++){
        retval.leg_position[i].x=legs[i].x();
        retval.leg_position[i].y=legs[i].y();
        retval.leg_position[i].z=legs[i].z();
    }
    return retval;
}

/**
 * @brief moveUpDown Routine of standing up and laying down robot
 * @param zero_jnts - service client
 * @param robot_pose_publisher RobotPose publisher
 * @param destination Destination pose (usually gait base)
 * @param time Time to do motion
 * @param standup If true, robot will stand up, otherwise lay down */
void moveUpDown(ros::ServiceClient &zero_jnts,
                ros::ServiceClient &solve_fwd,
                ros::ServiceClient &cvt_frame,
                ros::Publisher &robot_pose_publisher,
                std::vector<KDL::Vector> &destination, double time, bool standup)
{

   bullet_kinematics::ZeroJoints::Request zj_req;
   bullet_kinematics::ZeroJoints::Response zj_res;
   bullet_kinematics::SolveFwd::Request sfwd_req;
   bullet_kinematics::SolveFwd::Response sfwd_res;
   bullet_kinematics::ConvertPointFrame::Request cpf_req;
   bullet_kinematics::ConvertPointFrame::Response cpf_res;

   std::vector<std::vector<KDL::Vector> > trajectory;
   std::vector<KDL::Vector> laying_pose(6, KDL::Vector::Zero());
   std::vector<KDL::Vector> middle_pose(6, KDL::Vector::Zero());


   for(int i = 0; i<6; i++)
   {
       sfwd_req.joints[0].data=0;
       sfwd_req.joints[1].data=0;
       sfwd_req.joints[2].data=0;
       sfwd_req.leg_index=i;
       solve_fwd.call(sfwd_req,sfwd_res);

       // 0.005 is to assert inverse kinematics solution will be possible
       sfwd_res.result_hip.x-=0.005;

       cpf_req.command.data = "HIP_TO_BODY";
       cpf_req.leg_index = i;
       cpf_req.pt = sfwd_res.result_hip;
       cvt_frame.call(cpf_req, cpf_res);

       laying_pose[i] = KDL::Vector(cpf_res.result.x,
                                    cpf_res.result.y,
                                    cpf_res.result.z);
       middle_pose[i] = destination[i];
       middle_pose[i].data[2] = 0.01;
   }

//   trajectory.push_back(laying_pose);
   if(standup)
   {
       trajectory.push_back(middle_pose);
       trajectory.push_back(std::vector<KDL::Vector>(destination));
       zero_jnts.call(zj_req, zj_res);
       GaitGeneratorSimple::getInstance()->setRobotPose(laying_pose);
   }
   else
   {
       trajectory.push_back(middle_pose);
       trajectory.push_back(laying_pose);
   }


   GaitGeneratorSimple::getInstance()->setCustumTrajectoryPoints(trajectory);
   GaitGeneratorSimple::getInstance()->setCustomTrajectoryTime(time);

   KDL::Vector position;
   KDL::Rotation orientation;
   std::vector<KDL::Vector> leg_tips(6);
   bullet_kinematics::RobotPose pose_msg;
   double tp = GaitGeneratorSimple::getInstance()->getSamplingPeriod();
   ros::Rate rate(1/tp);

   bool finished = false;
   while(!finished)
   {
       finished = GaitGeneratorSimple::getInstance()->generateTrajectory(GaitGeneratorSimple::TRAJECTORY_CUSTOM);
       GaitGeneratorSimple::getInstance()->getRobotPose(leg_tips);
       pose_msg =  convertToMessage(position, orientation, leg_tips);
       robot_pose_publisher.publish(pose_msg);
       std::cout << pose_msg.leg_position[0].x << std::endl;
       std::cout << pose_msg.leg_position[0].y << std::endl;
       std::cout << pose_msg.leg_position[0].z << std::endl;

       ros::spinOnce();
       rate.sleep();
   }
   if(!standup)
       svc_zero_joints.call(zj_req, zj_res);
}


/**
 * @brief standup_service_handler StandUp service handler */
bool standup_service_handler(bullet_gait::StandUpLayDown::Request &req, bullet_gait::StandUpLayDown::Response &res)
{
    bullet_info("Stand up service called!");
    moveUpDown(svc_zero_joints, svc_solve_fwd, svc_hip_2_body, pub_robot_pose, base, 1, true);
    return true;
}

/**
 * @brief laydown_service_handler LayDown service handler */
bool laydown_service_handler(bullet_gait::StandUpLayDown::Request &req, bullet_gait::StandUpLayDown::Response &res)
{
    bullet_info("Lay down service called!");
    moveUpDown(svc_zero_joints, svc_solve_fwd, svc_hip_2_body, pub_robot_pose, base, 1, false);
    return true;
}


/**
 * @brief generateGaitBase Generates gait base
 * @param node Node handle (for service access)
 * @param body_heigth heigh of body in gait base
 * @param leg_tip_position_radius radial displacement of leg (how close to body leg tip is)
 * @return gait base */
std::vector<KDL::Vector> generateGaitBase(ros::ServiceClient &solve_fwd,
                                          ros::ServiceClient &cvt_frame,
                                          double body_heigth, double leg_tip_position_radius)
{
    std::vector<KDL::Vector> gait_base(6, KDL::Vector(0,0,0));
    bullet_kinematics::SolveFwd::Request sfwd_req;
    bullet_kinematics::SolveFwd::Response sfwd_res;
    bullet_kinematics::ConvertPointFrame::Request cpf_req;
    bullet_kinematics::ConvertPointFrame::Response cpf_res;

    for(int i=0; i<gait_base.size(); i++)
    {
        /// Solve forward kinematics for zero angles
        sfwd_req.joints[0].data=0;
        sfwd_req.joints[1].data=0;
        sfwd_req.joints[2].data=0;
        sfwd_req.leg_index=i;
        solve_fwd.call(sfwd_req,sfwd_res);

        sfwd_res.result_hip.x=leg_tip_position_radius;


        cpf_req.command.data = "HIP_TO_BODY";
        cpf_req.leg_index = i;
        cpf_req.pt = sfwd_res.result_hip;
        cvt_frame.call(cpf_req, cpf_res);

        gait_base[i] = KDL::Vector(cpf_res.result.x, cpf_res.result.y, -1*body_heigth);
    }
    return gait_base;
}

/**
 * @brief teleop_body_relative_cb Teleoperator-BodyRelative callback */
void teleop_body_relative_cb(const geometry_msgs::TwistConstPtr& msg)
{
    KDL::Vector lin_vel = KDL::Vector(msg->linear.x, msg->linear.y, 0);

    GaitGeneratorSimple::getInstance()->setLinearVelocity(lin_vel);
    GaitGeneratorSimple::getInstance()->setAngularVelocity(msg->angular.z);
}

void bodyPositionCB (const geometry_msgs::Vector3::ConstPtr& msg)
{
    body_position = KDL::Vector (msg->x, msg->y, msg->z);
}

void bodyOrientationCB (const geometry_msgs::Vector3::ConstPtr& msg)
{
    body_orientation = KDL::Rotation::RPY(msg->x, msg->y, msg->z);
}

// ============================================================================================
// ======================================= MAIN ROUTINE =======================================
// ============================================================================================

int main(int argc, char** argv)
{
    // Init this node
    ros::init(argc, argv, "gait_generator");
    bullet_info("Starting...");

    // ROS nodehandle
    ros::NodeHandle n;
    ros::Subscriber sub_robot_pose;

    // Namespace
    std::string ns = n.getNamespace();
    std::string parent_ns = ros::names::parentNamespace(ns);

    /// Read gait params from parameter server
    double tp, step_period, step_heigth, body_heigth, leg_radius;
    if(!ros::param::get("gait/body_heigth", body_heigth)    ||
       !ros::param::get("gait/step_heigth", step_heigth)    ||
       !ros::param::get("gait/step_period", step_period)    ||
       !ros::param::get("gait/leg_tip_pos_r", leg_radius)   ||
       !ros::param::get("gait/generation/tp", tp))
    {
        bullet_error("Parameters not found.");
        return -1;
    }
    bullet_info("Params read.");

    // Connect publishers and subscribers
    pub_robot_pose = n.advertise<bullet_kinematics::RobotPose>("robot_pose", 1000);
    ros::Subscriber sub_body_relative_vel = n.subscribe("teleop/body_relative", 100, teleop_body_relative_cb);
    ros::Subscriber sub_body_position = n.subscribe("body_position", 10, bodyPositionCB);
    ros::Subscriber sub_body_orientation = n.subscribe("body_orientation", 10, bodyOrientationCB);
    bullet_info("Publishers/subscribers connected.");

    ros::ServiceServer layDownService = n.advertiseService("stand_up", standup_service_handler);
    ros::ServiceServer standUpService = n.advertiseService("lay_down", laydown_service_handler);
    bullet_info("Service servers started.");

    svc_zero_joints = n.serviceClient<bullet_kinematics::ZeroJoints>("kinematics_solver/zero_joints");
    svc_solve_fwd = n.serviceClient<bullet_kinematics::SolveFwd>("kinematics_solver/solve_fwd");
    svc_hip_2_body = n.serviceClient<bullet_kinematics::ConvertPointFrame>("kinematics_solver/convert_frame");
    bullet_info("Service clients connected.");

    /// Gait params
    double angular_vel = 0.0;
    KDL::Vector lin_vel(0, 0, 0);
    base = generateGaitBase(svc_solve_fwd, svc_hip_2_body, body_heigth, leg_radius);
    bullet_info("Gait params set.");

    /// Generator preparations
    GaitGeneratorSimple::getInstance()->setGaitBase(base);
    GaitGeneratorSimple::getInstance()->setSamplingPeriod(tp);
    GaitGeneratorSimple::getInstance()->setGaitParameters(step_heigth, step_period);
    GaitGeneratorSimple::getInstance()->setLinearVelocity(lin_vel);
    GaitGeneratorSimple::getInstance()->setAngularVelocity(angular_vel);
    bullet_info("Generator prepared.");


    std::vector<KDL::Vector> leg_tips(6);
    bullet_kinematics::RobotPose pose_msg;
    bullet_info("Generator preparedA.");
    ros::Rate rate(1/tp);
    bullet_info("Generator preparedB.");


    /// No matter what real pose of robot is, in GaitGenerator internals all legs lies in body frame origin
    /// This is to avoid messing robot leg due to this condition.
    moveUpDown(svc_zero_joints, svc_solve_fwd, svc_hip_2_body, pub_robot_pose, base, 1, false);
    bullet_info("MoveUpDown.");

    while(ros::ok())
    {
        GaitGeneratorSimple::getInstance()->generateTrajectory(GaitGeneratorSimple::TRAJECTORY_GAIT);
        GaitGeneratorSimple::getInstance()->getRobotPose(leg_tips);
        pose_msg =  convertToMessage(body_position, body_orientation, leg_tips);
        pub_robot_pose.publish(pose_msg);
        ros::spinOnce();
        rate.sleep();
    }

}


void test_loop_translations(ros::NodeHandle &node, ros::Publisher &robot_pose_publisher)
{
    bullet_kinematics::RobotPose pose;
    pose.body_pose.position.x=0.01;
    pose.body_pose.position.y=0.00;
    ros::Rate rate(10);
    while(ros::ok())
    {

        pose.body_pose.orientation.w=1;
        pose.body_pose.orientation.x=0;
        pose.body_pose.orientation.y=0;
        pose.body_pose.orientation.z=0;
//        pose.body_pose.position.x=0.01;

        pose.body_pose.position.z= 0.04;

        for(int i=0; i<6; i++)
        {
            ros::ServiceClient svc_solve_fwd = node.serviceClient<bullet_kinematics::SolveFwd>("/bullet/solve_fwd");
            ros::ServiceClient svc_hip_2_body = node.serviceClient<bullet_kinematics::ConvertPointFrame>("/bullet/convert_frame");
            bullet_kinematics::SolveFwd::Request req;
            bullet_kinematics::SolveFwd::Response res;
            req.joints[0].data=0;
            req.joints[1].data=0;
            req.joints[2].data=0;
            req.leg_index=i;
            svc_solve_fwd.call(req,res);
            res.result_hip.x-=0.12;

            bullet_kinematics::ConvertPointFrame::Request cpf_req;
            bullet_kinematics::ConvertPointFrame::Response cpf_res;
            cpf_req.command.data = "HIP_TO_BODY";
            cpf_req.leg_index = i;
            cpf_req.pt = res.result_hip;
            svc_hip_2_body.call(cpf_req, cpf_res);

            pose.leg_position[i] = cpf_res.result;


        }
        static int dir = 1;
        pose.body_pose.position.y+= dir*0.001;
        double d =pose.body_pose.position.y;
        if(fabs(d)>0.04)
            dir*=-1;
        robot_pose_publisher.publish(pose);
        ros::spinOnce();

        rate.sleep();


    }
}

RobotPose test_brutal_stand_up(ros::NodeHandle &node, ros::Publisher &robot_pose_publisher)
{
    bullet_kinematics::RobotPose pose;
    RobotPose retval;
    pose.body_pose.position.x= 0.00;
    pose.body_pose.position.y= 0.00;
    pose.body_pose.position.z= 0.05;
    for(int i=0; i<6; i++)
    {
        ros::ServiceClient svc_solve_fwd = node.serviceClient<bullet_kinematics::SolveFwd>("/bullet/solve_fwd");
        ros::ServiceClient svc_hip_2_body = node.serviceClient<bullet_kinematics::ConvertPointFrame>("/bullet/convert_frame");
        bullet_kinematics::SolveFwd::Request req;
        bullet_kinematics::SolveFwd::Response res;
        req.joints[0].data=0;
        req.joints[1].data=0;
        req.joints[2].data=0;
        req.leg_index=i;
        svc_solve_fwd.call(req,res);
        res.result_hip.x-=0.12;

        bullet_kinematics::ConvertPointFrame::Request cpf_req;
        bullet_kinematics::ConvertPointFrame::Response cpf_res;
        cpf_req.command.data = "HIP_TO_BODY";
        cpf_req.leg_index = i;
        cpf_req.pt = res.result_hip;
        svc_hip_2_body.call(cpf_req, cpf_res);

        pose.leg_position[i] = cpf_res.result;
        retval.leg_tip_position[i] = KDL::Vector(cpf_res.result.x, cpf_res.result.y, cpf_res.result.z);
    }
    robot_pose_publisher.publish(pose);

    retval.body_orientation = KDL::Rotation::Identity();
    retval.body_position = KDL::Vector(0,0,0.06);

    ros::spinOnce();
    return retval;
}

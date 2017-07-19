#ifndef KINEMATICS_SERVICES_H
#define KINEMATICS_SERVICES_H


#include "solver.h"
#include "ros/ros.h"
#include "boost/noncopyable.hpp"
#include "bullet_core/bullet_core.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "bullet_kinematics/SolveFwd.h"
#include "bullet_kinematics/ZeroJoints.h"
#include "bullet_kinematics/ConvertPointFrame.h"


/**
 * @brief The KinematicServices class           Class containing all defined services of the kinetics_solver node.
 */
class KinematicServices: private boost::noncopyable
{
public:

    /**
     * @brief solveForwardKinematicsHandler     Solve forward kinematics service handler.
     * @param req                               Request message of the service.
     * @param res                               Result message of the service.
     * @return                                  True if succeed.
     */
    static bool solveForwardKinematicsHandler(bullet_kinematics::SolveFwd::Request &req, bullet_kinematics::SolveFwd::Response &res);

    /**
     * @brief convertPointFrameHandler          Convert a point from one frame to another (body and hip)
     * @param req                               Request message of the service.
     * @param res                               Result message of the service.
     * @return                                  True if succeed.
     */
    static bool convertPointFrameHandler(bullet_kinematics::ConvertPointFrame::Request &req, bullet_kinematics::ConvertPointFrame::Response &res);

    /**
     * @brief zeroJointsHandler                 Zero the position of all joints.
     * @param req                               Request message of the service.
     * @param res                               Result message of the service.
     * @return                                  True if succeed.
     */
    static bool zeroJointsHandler(bullet_kinematics::ZeroJoints::Request &req, bullet_kinematics::ZeroJoints::Response &res);

private:

    /**
     * @brief KinematicServices                 Default constructor (class with only static methods).
     */
    KinematicServices();

};


#endif // KINEMATICS_SERVICES_H

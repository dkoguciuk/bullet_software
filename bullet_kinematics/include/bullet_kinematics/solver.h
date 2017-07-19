#ifndef SOLVER_H
#define SOLVER_H


#include "leg_solver_kdl.h"
#include "leg_solver_analitical.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/frames.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainiksolverpos_nr.hpp"
#include "boost/shared_array.hpp"


using namespace KDL;


/**
 * @brief The Solver class. This class is a singleton.
 */
class Solver
{
private:

    /**
     * @brief Solver                    Default constructor.
     */
    Solver();

    static Solver* s_instance;


    Tree tree_;

    /**
     * @brief hips_pose_                Pose of robot hips (joint 1) in Body Coordinate Frame.
     */
    std::vector<KDL::Frame> hips_pose_;

    std::vector<boost::shared_ptr<LegSolverKDL> > leg_solvers_kdl_; /**< DEPRECATED */

    /**
     * @brief leg_solver_analitical_    Analitical IK solver.
     */
    boost::shared_ptr<LegSolverAnalitical>  leg_solver_analitical_; /**<  */

    /**
     * @brief fwd_leg_solver_           Numerical FK solver.
     */
    boost::shared_ptr<LegSolverKDL> fwd_leg_solver_;
    
public:

    /**
     * @brief getInstance returns instance of Solver class
     * @return pointer to instance of class
     */
    static Solver* getInstance();

    /**
     * @brief setRobotParameters generates KDL Tree object from URDF model
     * @param robot_description Reference to string representation of URDF model
     * @return 0 if succes, otherwise error code
     */
    int setRobotParameters(std::string &robot_description);

    /**
     * @brief setRobotParameters Prepares analitical solver and numerical solver for forward kinematics
     * @param leg_length Reference to leg segments length vector
     * @param hip_pos_x Reference to hips x position in root frame
     * @param hip_pos_y Reference to hips y position in root frame
     * @param hip_angle Reference to root-hip angle vector
     * @return 0 if succes, otherwise -1
     */
    int setRobotParameters(std::vector<double> &leg_length, std::vector<double> &hip_pos_x, std::vector<double> &hip_pos_y, std::vector<double> &hip_angle);

    /**
     * @brief solve Solves inverse kinematics problem for given leg.
     * @param point Destination point represented as KDL::Vector
     * @param joints Output array of joints
     * @param leg_index Index of leg to be solved
     * @return Always 0 (TODO: error codes)
     */
    int solveInv(KDL::Vector &leg_tip, KDL::Vector &body, KDL::Rotation &body_orientation, KDL::JntArray &joints, int leg_index);

    /**
     * @brief solveInvBody Solves inverse kinematics for point given in body frame
     * @param leg_tip Destination point represented as KDL::Vector
     * @param joints Output array of joints
     * @param leg_index Index of leg to be solved
     * @return Always 0 (TODO: error codes)
     */
    int solveInvBody(KDL::Vector &leg_tip, KDL::JntArray &joints, int leg_index);

    /**
     * @brief solveFwd Solver forward kinematics problem for given leg
     * @param joints Reference to joint array
     * @param point Reference to result point
     * @param leg_index Index of leg to which kinematics will be solved
     * @return Always zero.
     */
    int solveFwd(KDL::JntArray &joints, KDL::Vector &point, Vector &point_hip, int leg_index);

    /**
     * @brief hipToGlobal Converts vector from Hip Frame to Body Frame
     * @param point Point to convert (reference)
     * @param leg_index Index of leg
     * @return Converted point
     */
    KDL::Vector hipToBody(KDL::Vector &point, int leg_index);

    /**
     * @brief globalToHip Converts vector from Body Frame to Hip Frame
     * @param point Point to convert (reference)
     * @param leg_index Index of leg
     * @return Converted point
     */
    KDL::Vector bodyToHip(KDL::Vector &point, int leg_index);

    /**
     * @brief test_solveForward Unit test of kinematics solution. Dirty&bad
     */
    void test_solveForward();

    /**
     * @brief test_frameTest Unit test for frame manipulation
     */
    void test_frameTest();
};


#endif // SOLVER_H

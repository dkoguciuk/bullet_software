#ifndef LEG_SOLVER_H
#define LEG_SOLVER_H

#include "kdl_parser/kdl_parser.hpp"
#include "kdl/frames.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainiksolvervel_wdls.hpp"
#include "kdl/chainiksolverpos_nr.hpp"
#include "kdl/solveri.hpp"
#include "boost/scoped_ptr.hpp"

class LegSolverKDL
{
private:

    KDL::Chain chain_;                                                  /**< Kinematic chain of leg*/
    KDL::JntArray joint_states_;                                        /**< Joint angles array */
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;  /**< Fwd kinematics solver instance */
    boost::scoped_ptr<KDL::/*ChainIkSolverVel_pinv*/ChainIkSolverVel_wdls> ik_vel_solver_;       /**< Inv kinematics solver for velocity */
    boost::scoped_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;         /**< Inv kinematics solver for position */

    bool first_run_;                            /**< Flag to indicate first iteration of solving */

    static const short int JOINT_NUMBER = 3;    /**< Constant number of segments in leg */

    /** \brief  Create new kinematics solvers.
     *          To use when new chain is created */
    void createSolvers();
public:
    /** \brief Constructor for LegSolverKDL
     *  \param chain Reference to kinematic chain
     *  \throws std::invalid_argument When chain contains invalid number of joints. */
    LegSolverKDL(KDL::Chain &chain) throw(std::invalid_argument);

    /** \brief Constructor for LegSolverKDL
     *  \param tree Reference to kinematic tree
     *  \param root Name of root segment
     *  \param tip Name of tip segment of chain
     *  \throws std::invalid_argument When chain contains invalid number of joints or no chain with given segment names found */
    LegSolverKDL(KDL::Tree &tree, const std::string &root, const std::string &tip) throw(std::invalid_argument);

    /** \brief Solves inverse kinematic problem
     *  \param point_f Destination point given as KDL frame
     *  \param result Result array of joints
     *  \throws std::runtime_error when given point has degraded solution
     *  \return true if solved, false if failed */
    bool solveLeg(KDL::Frame &point_f, KDL::JntArray &result) throw(std::runtime_error);

    /** \brief Solves inverse kinematic problem
     *  \param point Destination point given as KDL vector
     *  \param result Result array of joints
     *  \throws std::runtime_error when given point has degraded solution
     *  \return true if solved, false if failed */
    bool solveLeg(KDL::Vector &point, KDL::JntArray &result) throw(std::runtime_error);

    bool solveLegForward(KDL::JntArray &joints, KDL::Frame &point);// throw (std::runtime_error);
};

#endif // LEG_SOLVER_H

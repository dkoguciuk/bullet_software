#include "leg_solver_kdl.h"


LegSolverKDL::LegSolverKDL(KDL::Chain &chain) throw(std::invalid_argument)
{
    if(chain.getNrOfJoints() != JOINT_NUMBER )
        throw std::invalid_argument("Invalid joint number. Wrong chain given.");
    this->chain_ = chain;
    createSolvers();

    KDL::Segment s1, s2, s3;
    s1 = chain.getSegment(0);
    s2 = chain.getSegment(1);
    s3 = chain.getSegment(2);

    KDL::Frame f1, f2, f3;

    f1 = s1.getFrameToTip();
    f2 = s2.getFrameToTip();
    f3 = s3.getFrameToTip();


}

LegSolverKDL::LegSolverKDL(KDL::Tree &tree, const std::string &root, const std::string &tip) throw(std::invalid_argument)
{
    if(!tree.getChain(root, tip, chain_))
        throw std::invalid_argument("Unable to get chain. Wrong root or tip name given.");
    if(chain_.getNrOfJoints() != JOINT_NUMBER)
        throw std::invalid_argument("Invalid joint number. Wrong chain given.");
    createSolvers();

}

bool LegSolverKDL::solveLeg(KDL::Frame &point_f, KDL::JntArray &result) throw(std::runtime_error)
{
    KDL::JntArray init_joint_states_;
    init_joint_states_.resize(JOINT_NUMBER);

    /// Solver demands initial joint states
    /// For fist run joints angles will be set to 0
    if(first_run_)
    {
        init_joint_states_(0) = 0.0;
        init_joint_states_(1) = 0.0;
        init_joint_states_(2) = 0.0;
    }
    else
    {
        init_joint_states_ = joint_states_;
    }

    int ik_result = ik_pos_solver_->CartToJnt(init_joint_states_, point_f, joint_states_);
    if(ik_result == KDL::SolverI::E_NOERROR)
    {
        result = joint_states_;
    }
    else if(ik_result == KDL::SolverI::E_DEGRADED)
    {
        result = joint_states_;
        throw std::runtime_error("Degraded solution computed.");
    }
    else
    {
        throw std::runtime_error("No solution for given point found.");
        return false;
    }

    first_run_ = false;
    return true;

}

bool LegSolverKDL::solveLeg(KDL::Vector &point, KDL::JntArray &result) throw(std::runtime_error)
{
    KDL::Frame fr = KDL::Frame(point);
    return solveLeg(fr, result);
}

bool LegSolverKDL::solveLegForward(KDL::JntArray &joints, KDL::Frame &point)
{
    int ik_result = fk_pos_solver_->JntToCart(joints, point);
//    if(ik_result == KDL::SolverI::E_NOERROR)
//    {
//        result = joint_states_;
//    }
//    else if(ik_result == KDL::SolverI::E_DEGRADED)
//    {
//        result = joint_states_;
//        throw std::runtime_error("Degraded solution computed.");
//    }
//    else
//    {
//        throw std::runtime_error("No solution for given point found.");
//        return false;
//    }
}


void LegSolverKDL::createSolvers()
{
    first_run_ = true;
    joint_states_.resize(JOINT_NUMBER);
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    ik_vel_solver_.reset(new KDL::/*ChainIkSolverVel_pinv*/ChainIkSolverVel_wdls(chain_/*, 0.00001, 1000*/));

    /// Set weights of position to 1, of orientation to 0
    Eigen::MatrixXd weigth_ts =  Eigen::MatrixXd::Identity(6,6);
//    weigth_ts(3,3) = 0.01;
//    weigth_ts(4,4) = 0.01;
//    weigth_ts(5,5) = 0.01;
//    weigth_ts[1] = weigth_ts[2] = weigth_ts[3] = 1.0; //X  Y  Z
//    weigth_ts[3] = weigth_ts[4] = weigth_ts[5] = 0.0; //Rx Ry Rz
    ik_vel_solver_->setWeightTS(weigth_ts);
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR(chain_, *fk_pos_solver_, *ik_vel_solver_));

}


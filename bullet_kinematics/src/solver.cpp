#include "solver.h"

Solver::Solver() : hips_pose_(6, KDL::Frame())
{
}

Solver* Solver::getInstance()
{
    if(!s_instance)
        s_instance = new Solver;
    return s_instance;
}

Solver *Solver::s_instance = 0;

int Solver::setRobotParameters(std::vector<double> &leg_length, std::vector<double> &hip_pos_x, std::vector<double> &hip_pos_y, std::vector<double> &hip_angle)
{
    if(leg_length.size() != 3)
    {
        throw std::invalid_argument("Wrong size of leg lengths array.");
        return -1;
    }
    /// Create analitical solver
    leg_solver_analitical_.reset(new LegSolverAnalitical(leg_length));
    leg_solver_analitical_->setSolutionCriteria(-1,-1);

    /// Create numeric forward solver
    KDL::Chain ch;
    ch.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(leg_length[0],0.0,0.0))));
    ch.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,leg_length[1]))));
    ch.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,leg_length[2]))));
    fwd_leg_solver_.reset(new LegSolverKDL(ch));

    if(hip_angle.size() != 6 || hip_pos_x.size() != 6 || hip_pos_y.size() != 6)
    {
        throw std::invalid_argument("Wrong size hip positions/angle array");
        return -1;
    }

    for(int i = 0; i < 6; i++)
    {
        hips_pose_[i] = KDL::Frame( KDL::Vector( - hip_pos_x[i], - hip_pos_y[i], 0 ) );
        KDL::Frame tmp = KDL::Frame( KDL::Rotation::RPY( 0, 0, - hip_angle[i] ) );
        hips_pose_[i] = tmp* hips_pose_[i];
    }
    return 0;

}

int Solver::setRobotParameters(std::string &robot_description)
{
    int retval = kdl_parser::treeFromString(robot_description, tree_);
    if( !retval )
        return -1;

    leg_solvers_kdl_.clear();

    /// Get kinematic chains from tree and create KDL solvers
    KDL::Chain ch;
    for(int i=0; i<6; i++)
    {

        std::stringstream ss;
        ss << "leg_" << i+1 << "_3";
        std::string leg_name = ss.str();
        if(!tree_.getChain("body", leg_name, ch))
            return -1;

        boost::shared_ptr<LegSolverKDL> ptr;
        try
        {
            ptr.reset(new LegSolverKDL(ch));
        } catch (std::invalid_argument e)
        {
            throw;
            return -1;
        }
        leg_solvers_kdl_.push_back(ptr);
    }


    /// Prepare analitical solver
    std::vector<double> segment_lengths(3);
    for(int i = 0; i<3; i++)
    {
        double x,y,z;
        x = ch.getSegment(i).getFrameToTip().p.data[0];
        y = ch.getSegment(i).getFrameToTip().p.data[1];
        z = ch.getSegment(i).getFrameToTip().p.data[2];
        segment_lengths[i] = sqrt(x*x + y*y + z*z);
    }
    leg_solver_analitical_.reset(new LegSolverAnalitical(segment_lengths));
    leg_solver_analitical_->setSolutionCriteria(-1, 1);
    KDL::Chain testChain;
    testChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(segment_lengths[0],0.0,0.0))));
    testChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,segment_lengths[1]))));
    testChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,segment_lengths[2]))));
    fwd_leg_solver_.reset(new LegSolverKDL(testChain));



    return 0;
}


int Solver::solveInv(KDL::Vector &leg_tip, KDL::Vector &body, KDL::Rotation &body_orientation, KDL::JntArray &joints, int leg_index)
{
    KDL::Vector l(leg_tip);
    /// Translation from Main Frame to Body Frame
    l = l - body;

    /// Rotation to Body Frame
    l = body_orientation * l;

    /// Transformation to Hip Frame
    l = hips_pose_[leg_index] * l;

    try
    {
       leg_solver_analitical_->solveLeg(l, joints);
    } catch (std::domain_error e)
    {
        std::string s = "Unable to solve. Solver returned: "+ std::string(e.what());
        throw std::runtime_error(s);
        return -1;
    }
}

int Solver::solveInvBody(KDL::Vector &leg_tip, KDL::JntArray &joints, int leg_index)
{
    KDL::Vector l(leg_tip);

    /// Transformation to Hip Frame
    l = hips_pose_[leg_index] * l;
    /// Transformations done, try to solve kinematics.
    try
    {
       leg_solver_analitical_->solveLeg(l, joints);
    } catch (std::domain_error e)
    {
        std::string s = "Unable to solve. Solver returned: "+ std::string(e.what());
        throw std::runtime_error(s);
        return -1;
    }
}

int Solver::solveFwd( KDL::JntArray &joints, KDL::Vector &point, KDL::Vector &point_hip, int leg_index)
{
//     KDL::Frame fr(point);
//     leg_solvers_kdl_[leg_index]->solveLegForward(joints, fr);
    leg_solver_analitical_->solveLegFwd(point_hip, joints);
    point = point_hip - hips_pose_[leg_index].p;
    point = hips_pose_[leg_index].M.Inverse() * point;

}

KDL::Vector Solver::hipToBody(Vector &point, int leg_index)
{
    KDL::Vector ret;
    ret = point - hips_pose_[leg_index].p;
    ret = hips_pose_[leg_index].M.Inverse() * ret;
    return ret;
}

KDL::Vector Solver::bodyToHip(Vector &point, int leg_index)
{
    return hips_pose_[leg_index]*point;
}

void Solver::test_solveForward()
{
    KDL::JntArray jarr;
    jarr.resize(3);
    jarr(0) = 0.0;
    jarr(1) = 0;
    jarr(2) = 0.0;

    KDL::Rotation test = this->hips_pose_[0].M.Inverse();

    KDL::Vector result;
    KDL::Vector result_hip;
    this->solveFwd(jarr, result, result_hip, 0);
//    result = test * result;
    KDL::Vector glo, hip;
//    hip = this->hips_pose_[0].M.Inverse() * result;
    glo = hipToBody(result_hip, 0);
    hip = bodyToHip(glo, 0);
    hip+=hip;
//    fwd_leg_solver_->solveLegForward(jarr, result);
//    leg_solver_analitical_->solveLegFwd(result,jarr);
    KDL::JntArray jarr_res;
    jarr_res.resize(3);
    leg_solver_analitical_->solveLeg(result_hip, jarr_res);
    double a = jarr_res(0);
    double b = jarr_res(1);
    double c = jarr_res(2);
    std::cout<<"done:"<< a <<"\t"<<b<<"\t"<<c<<std::endl;
}

void Solver::test_frameTest()
{
    KDL::Frame fr;
    fr.M.DoRotZ(M_PI/4);
    fr.p = fr.M*KDL::Vector(1,1,0);


    KDL::Vector v1(5,5,0);

    KDL::Vector v2 = fr * v1;
    std::cout<<" "<<std::endl;
 }


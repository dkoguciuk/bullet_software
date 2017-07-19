#ifndef LEG_SOLVER_ANALITICAL_H
#define LEG_SOLVER_ANALITICAL_H
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "math.h"
#include <vector>
#include <stdexcept>

class LegSolverAnalitical
{
private:
    double l1_;
    double l2_;
    double l3_;

    double delta1_;
    double delta2_;

    double joints_[3];

    double arcSinCos(double s, double c);
    bool assertSqrt(double sqrt);
    bool assertTrigonometric(double val);

public:
    LegSolverAnalitical(std::vector<double> &segment_lengths) throw(std::invalid_argument);
    bool setSolutionCriteria(int d1, int d2) throw(std::invalid_argument);
    bool solveLeg(KDL::Frame &point, KDL::JntArray &result) throw(std::runtime_error);
    bool solveLeg(KDL::Vector &point, KDL::JntArray &result) throw(std::runtime_error);
    bool solveLegFwd(KDL::Vector &point, KDL::JntArray &joints) throw(std::runtime_error);

};

#endif // LEG_SOLVER_ANALITICAL_H

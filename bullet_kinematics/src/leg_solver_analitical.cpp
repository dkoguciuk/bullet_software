#include "leg_solver_analitical.h"


LegSolverAnalitical::LegSolverAnalitical(std::vector<double> &segment_lengths) throw(std::invalid_argument) :
    l1_(segment_lengths[0]),
    l2_(segment_lengths[1]),
    l3_(segment_lengths[2]),
    delta1_(1),
    delta2_(1)

{

}

bool LegSolverAnalitical::setSolutionCriteria(int d1, int d2) throw(std::invalid_argument)
{
    if( d1 != 1 && d1 != -1 && d2 != 1 && d2 != -1)
    {
        throw std::invalid_argument("Criterium must be equal -1 or 1.");
        return false;
    }
    delta1_ = d1;
    delta2_ = d2;
    return true;
}

bool LegSolverAnalitical::solveLeg(KDL::Vector &point, KDL::JntArray &result) throw(std::runtime_error)
{
    double x,y,z;
    double j1, j2, j3;
    double s1,c1,s2,c2,s3,c3;

    x = point.data[0];
    y = point.data[1];
    z = point.data[2];
    /*
    double sqrt_x_y = sqrt(x*x + y*y);
    if(!assertSqrt(sqrt_x_y))
    {
        throw std::domain_error("Unable to find solution. Sqrt from negative.");
        return false;
    }
    /// First joint angle sin and cos
    s1 = y / sqrt_x_y;
    s1 *= delta1_;
    c1 = x / sqrt_x_y;
    c1 *= delta1_;

    if(!assertTrigonometric(s1) || !assertTrigonometric(c1))
    {
        throw std::domain_error("Unable to find solution. Sin or cos greater than 1.");
        return false;
    }

    /// Second joint angle sin and cos
    double a = -1*l1_ + delta1_ * sqrt_x_y;
    double b = (a*a + z*z +  l2_*l2_ - l3_* l3_)/(2*l2_);
    double sqrt_a_z_b = sqrt(a*a + z*z - b*b);
    if(!assertSqrt(sqrt_a_z_b))
    {
        throw std::domain_error("Unable to find solution. Sqrt from negative.");
        return false;
    }
    s2 = 1/(a*a + z*z) * (z*b + delta2_*a*sqrt_a_z_b);
    c2 = 1/(a*a + z*z) * (a*b + delta2_*z*sqrt_a_z_b);
    if(!assertTrigonometric(s2) || !assertTrigonometric(c2))
    {
        throw std::domain_error("Unable to find solution. Sin or cos greater than 1.");
        return false;
    }

    /// Thrid joint angle sin and cos
    s3 = (z * c2 - a* s2) / l3_;
    c3 = (a * c2 + z* s2 - l2_) / l3_;
    if(!assertTrigonometric(s3) || !assertTrigonometric(c3))
    {
        throw std::domain_error("Unable to find solution. Sin or cos greater than 1.");
        return false;
    }
    */

    j1 = atan2(y,x);
    s1 = sin(j1);
    c1 = cos(j1);

    double d = sqrt( (x-l1_*c1)*(x-l1_*c1) + (y - l1_*s1)*(y - l1_*s1) + z*z );

    double b = acos((d*d + l2_*l2_ - l3_*l3_ )/(2*l2_*d));
    j2 = asin(-1*z/d) - b;
    j2 *= -1;
    double ca = acos((l2_ * sin(b))/l3_);
    double cb = M_PI/2 - b;
    j3 = M_PI - ca - cb;

    if(std::isnan(j1) || std::isnan(j2) || std::isnan(j3))
    {
        throw std::runtime_error("Cannot solve for this point.");
        return false;
    }
    /// Calculate joint angles
    result(0) = j1;
    result(1) = j2;
    result(2) = j3;
//    result(0) = arcSinCos(s1, c1);
//    result(1) = arcSinCos(s2, c2);
//    result(2) = arcSinCos(s3, c3);

    return true;
}

bool LegSolverAnalitical::solveLegFwd(KDL::Vector &point, KDL::JntArray &joints) throw(std::runtime_error)
{
    point.data[0] = cos(joints(0)) * (l1_ + l2_*cos(joints(1)) + l3_*cos(joints(1)+joints(2)));
    point.data[1] = sin(joints(0)) * (l1_ + l2_*cos(joints(1)) + l3_*cos(joints(1)+joints(2)));
    point.data[2] = -1*l2_*sin(joints(1))+l3_*sin(joints(1)+joints(2));
    return true;
}

double LegSolverAnalitical::arcSinCos(double s, double c)
{
    if(abs(s) >= abs(c))
    {
        return asin(s);
    }
    else return acos(c);
}

bool LegSolverAnalitical::assertSqrt(double sqrt)
{
    if(sqrt == NAN || sqrt < 0)
        return false;
    else return true;
}

bool LegSolverAnalitical::assertTrigonometric(double val)
{
    if(val > 1)
        return false;
    else return true;
}

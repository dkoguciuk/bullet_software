#include "gait_generator.h"
#include "kdl/frames.hpp"
#include "iostream"

bool GaitGenerator::g_instance_initialised;
GaitGenerator GaitGenerator::g_instance;

GaitGenerator::GaitGenerator() :    trajectory_resolution_(10),
                                    dt_(0.01),
                                    linear_velocity_(0,0,0),
                                    angular_velocity_(0)
{
    g_instance_initialised = true;
}

GaitGenerator::~GaitGenerator(){}
GaitGenerator* GaitGenerator::getInstance()
{
    return g_instance_initialised? &g_instance : NULL;
}

bool GaitGenerator::setGaitParameters(double step_heigth, double time_period)
{
    if(step_heigth<=0 || time_period<=0)
        return false;
    step_heigth_ = step_heigth;
    t_ = time_period;
    trajectory_resolution_ = t_/dt_;
    return true;
}

bool GaitGenerator::setSamplingPeriod(double period)
{
    if(period <= 0)
        return false;

    dt_ = period;
    trajectory_resolution_ = t_/dt_;
    return true;
}

bool GaitGenerator::setLinearVelocity(KDL::Vector &linear_velocity)
{
//    if(almostEqual(linear_velocity_.Norm(), 0))
//        phase_= -1*trajectory_resolution_/2;
    linear_velocity_ = linear_velocity;

    return true;
}

bool GaitGenerator::setAngularVelocity(double angular_velocity_z)
{
    angular_velocity_ = angular_velocity_z;
    return true;
}

bool GaitGenerator::setBodyHeigth(double height, double v_z)
{
    body_heigth_ = height;
}

bool GaitGenerator::setBodyRP(double r, double p, double v)
{
    if(!std::isnan(r))
    {
        body_roll_ = r;
    }
    if(!std::isnan(p))
    {
        body_pitch_ = p;
    }
}

bool GaitGenerator::setRobotPose(KDL::Vector &body_position, KDL::Rotation &body_orientation, std::vector<KDL::Vector> &leg_tip_position)
{
    if(leg_tip_position.size() != 6)
        return false;
    current_pose_.body_orientation=body_orientation;
    current_pose_.body_position=body_position;
    current_pose_.leg_tip_position = leg_tip_position;
    base_position_ = current_pose_;
    return true;
}

bool GaitGenerator::setLegBase(std::vector<KDL::Vector> &base)
{
    if(base.size() != 6)
        return false;
    leg_base_ = std::vector<KDL::Vector>(base);
    return true;
}

void GaitGenerator::getRobotPose(KDL::Vector &body_position, KDL::Rotation &body_orientation, std::vector<KDL::Vector> &leg_tips)
{
    body_position = current_pose_.body_position;
    body_orientation = current_pose_.body_orientation;
    leg_tips = current_pose_.leg_tip_position;
}

bool GaitGenerator::generate()
{
    RobotPose pos = current_pose_;

//    if(phase_ == 0 && half_step_== false)
//        base_position_ = current_pose_;
    switch(state_)
    {
        case STATE_WALK:
        {
            if(linear_velocity_.Norm() < 0.001 && abs(angular_velocity_) < 0.001)
                return true;
            /// Step beginning
            if(phase_ == 0)
            {
                /// Recalculate setpoints
                base_position_ = current_pose_;
                for(int i =0; i <3; i++)
                {
                    int index = half_step_ ? 2*i : 2*i+1;
                    KDL::Vector leg_SP = current_pose_.body_orientation * KDL::Rotation::RotZ(angular_velocity_*t_) * leg_base_[index] + 0.5 * linear_velocity_*t_;
                    leg_SP = KDL::Rotation::RotZ(0.5*angular_velocity_*t_) * leg_SP;
                    leg_SP = leg_SP + (current_pose_.body_position+linear_velocity_*t_);
//                    setpoint_position_.leg_tip_position[index] = /*(current_pose_.body_orientation * KDL::Rotation::RotZ(-1*angular_velocity_*t_)) * leg_SP;*/
                }
            }
            /// Condition to eliminate drift (body faster than leg)
            if(phase_ != trajectory_resolution_-1)
            {
                /// Calculate body position
                pos.body_position = current_pose_.body_position + linear_velocity_*dt_;
                /// Calculate body orientation
//                pos.body_orientation = current_pose_.body_orientation * KDL::Rotation::RotZ(-1*angular_velocity_*dt_);
            }

            /// Calculate sweeping legs
            pos.leg_tip_position = current_pose_.leg_tip_position;
            for(int i = 0; i<3; i++)
            {
                if(half_step_ == true)
                {
                    calculateLegTips(pos.leg_tip_position, 2*i);
                }
                else
                {
                    calculateLegTips(pos.leg_tip_position, 2*i+1);
                }
            }
            phase_++;
            current_pose_ = pos;

            /// Step end operation
            if(phase_ > trajectory_resolution_)
            {
//                base_position_ = current_pose_;
                phase_ = 0;
                half_step_ = half_step_ ? false : true;
            }
            return true;
        }
        case STATE_STAND_UP:
        {
//            static int
        }
        case STATE_LAY_DOWN:
        {

        }
        case STATE_CHANGE_LEG_BASE:
        {

        }
    }

    /// State-independent activities
    if(!almostEqual(current_pose_.body_position.z(), body_heigth_))
    {

    }
    double r,p,y;
//    current_pose_.body_orientation.GetRPY(r,p,y);
    if(!almostEqual(r, body_roll_))
    {
//        double rot = r + body_orientation_vel_*
//        current_pose_.body_orientation *= KDL::Rotation::DoRotX(rot)
    }
    if(!almostEqual(p, body_pitch_))
    {

    }

}

void GaitGenerator::calculateLegTips(std::vector<KDL::Vector> &legs, int index)
{
    static double dx;
    static double dy;
    if(phase_ == 0)
    {
            dx = setpoint_position_.leg_tip_position[index].x()-base_position_.leg_tip_position[index].x();

            dy = setpoint_position_.leg_tip_position[index].y()-base_position_.leg_tip_position[index].y();
    }
    legs[index].data[0] =   base_position_.leg_tip_position[index].x() + dx * 0.5 *
                            (1-cos(M_PI * (double)phase_/trajectory_resolution_));
    legs[index].data[1] =   base_position_.leg_tip_position[index].y() + dy * 0.5 *
                            (1-cos(M_PI * (double)phase_/trajectory_resolution_));
//    legs[index].data[0] = base_position_.leg_tip_position[index].x() +
//                                    1*linear_velocity_.x()*trajectory_resolution_*dt_*
//                                    (1-cos(M_PI * (double)phase_/trajectory_resolution_));
//    legs[index].data[1] = base_position_.leg_tip_position[index].y() +
//                                    1*linear_velocity_.y()*trajectory_resolution_*dt_*
//                                    (1-cos(M_PI * (double)phase_/trajectory_resolution_));
    legs[index].data[2] =   step_heigth_*(1-cos(2*M_PI*(double)phase_/trajectory_resolution_));
}

inline bool GaitGenerator::almostEqual(double a, double b, double e)
{
    return abs(a-b)<e ? true : false;
}

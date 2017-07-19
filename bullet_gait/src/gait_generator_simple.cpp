#include "gait_generator_simple.h"
#include "kdl/frames.hpp"
#include "iostream"

#include "ros/ros.h"

bool GaitGeneratorSimple::g_instance_initialised;
GaitGeneratorSimple GaitGeneratorSimple::g_instance;

GaitGeneratorSimple::GaitGeneratorSimple() :    gait_trajectory_resolution_(10),
                                    dt_(0.01),
                                    linear_velocity_(0,0,0),
                                    angular_velocity_(0),
                                    leg_setpoint_(6, KDL::Vector(0,0,0)),
                                    leg_position_delta_(6, KDL::Vector(0,0,0)),
                                    current_pose_(6)
{
    g_instance_initialised = true;
}

GaitGeneratorSimple::~GaitGeneratorSimple(){}
GaitGeneratorSimple* GaitGeneratorSimple::getInstance()
{
    return g_instance_initialised? &g_instance : NULL;
}

bool GaitGeneratorSimple::setGaitParameters(double step_heigth, double time_period)
{
    if(step_heigth<=0 || time_period<=0)
        return false;
    step_heigth_ = step_heigth;
    t_ = time_period;
    gait_trajectory_resolution_ = t_/dt_;
    return true;
}


bool GaitGeneratorSimple::setCustomTrajectoryTime(double time)
{
    if(time<0)
        return false;
    trajectory_t_ = time;
    custom_trajectory_resolution_ = trajectory_t_/dt_;
}

bool GaitGeneratorSimple::setSamplingPeriod(double period)
{
    if(period <= 0)
        return false;

    dt_ = period;
    gait_trajectory_resolution_ = t_/dt_;
    return true;
}

double GaitGeneratorSimple::getSamplingPeriod()
{
    return dt_;
}

bool GaitGeneratorSimple::setLinearVelocity(KDL::Vector &linear_velocity)
{
    linear_velocity_ = linear_velocity;
    return true;
}

bool GaitGeneratorSimple::setAngularVelocity(double angular_velocity_z)
{
    angular_velocity_ = angular_velocity_z;
    return true;
}

bool GaitGeneratorSimple::setRobotPose(std::vector<KDL::Vector> &leg_tip_position)
{
    if(leg_tip_position.size() != 6)
        return false;
    current_pose_ = leg_tip_position;
    return true;
}

bool GaitGeneratorSimple::setGaitBase(std::vector<KDL::Vector> &base)
{
    if(base.size() != 6)
        return false;
    gait_base_ = std::vector<KDL::Vector>(base);
//    current_pose_ = std::vector<KDL::Vector>(base);
    return true;
}

void GaitGeneratorSimple::getRobotPose(std::vector<KDL::Vector> &leg_tips)
{
    leg_tips = current_pose_;
}

bool GaitGeneratorSimple::setCustumTrajectoryPoints(std::vector<std::vector<KDL::Vector> > &points)
{
    if(points.size() < 2)
        return false;
    for(int i = 0; i< points.size(); i++)
        if(points[i].size() != 6)
            return false;

    custom_trajectory_ = points;
    trajectory_point_index_ = 0;
//    current_pose_ = std::vector<KDL::Vector>(points[0]);
    return true;
}

bool GaitGeneratorSimple::generateTrajectory(TrajectoryType type)
{

    std::vector<KDL::Vector> pos(current_pose_);
    switch(type)
    {
        case TRAJECTORY_GAIT:
        {
            if(linear_velocity_.Norm() < 0.001 && fabs(angular_velocity_) < 0.001)
                return true;
            /// Step beginning
            if(step_phase_ == 0)
            {
                /// Assing new step base
                step_base_ = current_pose_;

                /// Prepare setpoints
                for(int i =0; i <3; i++)
                {
                    int index_standing, index_sweeping;
                    /// Select standing and sweeping legs indices
                    if(half_step_)
                    {
                        index_standing = 2*i;
                        index_sweeping = 2*i+1;
                    } else
                    {
                        index_standing = 2*i+1;
                        index_sweeping = 2*i;
                    }
                    
                    /// Compute leg destination points
                    KDL::Rotation leg_standing_rotation = KDL::Rotation::RotZ(-0.5*angular_velocity_*t_);
                    KDL::Rotation leg_sweeping_rotation = KDL::Rotation::RotZ(0.5*angular_velocity_*t_);
                    KDL::Vector leg_standing_setpoint = (leg_standing_rotation*gait_base_[index_standing]) - 0.5*linear_velocity_*t_;
                    KDL::Vector leg_sweeping_setpoint = (leg_sweeping_rotation*gait_base_[index_sweeping]) + 0.5*linear_velocity_*t_;
                    
                    /// Setpoints
                    leg_setpoint_[index_standing] = leg_standing_setpoint;
                    leg_setpoint_[index_sweeping] = leg_sweeping_setpoint;

                    /// Position delta - actual displacement to be done during cycle
                    leg_position_delta_[index_standing] = leg_standing_setpoint - current_pose_[index_standing];
                    leg_position_delta_[index_sweeping] = leg_sweeping_setpoint - current_pose_[index_sweeping];
               }

            }

            /// Move legs
            for(int i =0; i <3; i++)
            {
                int index_standing, index_sweeping;
                /// Select standing and sweeping legs indices
                if(half_step_)
                {
                    index_standing = 2*i;
                    index_sweeping = 2*i+1;
                } else
                {
                    index_standing = 2*i+1;
                    index_sweeping = 2*i;
                }
                /// Calculate leg movement
                calculateLegTips(pos, index_standing, LEG_STANDING);
                calculateLegTips(pos, index_sweeping, LEG_SWEEPING);
            }

            /// Increment phase
            step_phase_++;
            current_pose_ = pos;

            /// Step end condition
            if(step_phase_ > gait_trajectory_resolution_)
            {
                step_phase_ = 0;
                half_step_ = half_step_ ? false : true;
            }
            return true;
        } break;
        case TRAJECTORY_CUSTOM:
        {
            if(trajectory_phase_ == 0)
            {
                step_base_ = current_pose_;
                for(int i =0; i <6; i++)
                {
                    leg_setpoint_[i] = custom_trajectory_[trajectory_point_index_][i];
                    leg_position_delta_[i] = leg_setpoint_[i]-current_pose_[i];
                }

            }
            for(int i =0; i <6; i++)
            {
                /// Generate trajectory
                double phi = (double)trajectory_phase_/custom_trajectory_resolution_;
                double cos_phase = (1 - cos(M_PI * phi))*0.5; /**< Only najstarsi gorale know why I have to multiply it by 0.5 */
                pos[i].data[0] = step_base_[i].data[0] + leg_position_delta_[i].data[0] * cos_phase;
                pos[i].data[1] = step_base_[i].data[1] + leg_position_delta_[i].data[1] * cos_phase;
                pos[i].data[2] = step_base_[i].data[2] + leg_position_delta_[i].data[2] * cos_phase;
            }

            trajectory_phase_++;
            current_pose_ = pos;
            if(trajectory_phase_ > custom_trajectory_resolution_)
            {
                /// Zero phase and prepare to take next trajectory point
                trajectory_phase_ = 0;
                trajectory_point_index_++;
                /// If no more points in trajectory, return true to indicate finish
                if(trajectory_point_index_ >= custom_trajectory_.size())
                {
                    return true;
                }
                return false;
            }
            return false;
        } break;
    }


}

void GaitGeneratorSimple::calculateLegTips(std::vector<KDL::Vector> &legs, int index, LegMotionType motion_type)
{
    double phi = (double)step_phase_/gait_trajectory_resolution_;
    double z_cos_phase = 1-cos(2 * M_PI * phi);


    /// Sweeping leg needs Z axis motion
    if(motion_type == LEG_SWEEPING)
    {
        double cos_phase = (1 - cos(M_PI * phi))*0.5; /**< Only najstarsi gorale know why I have to multiply it by 0.5 */
        legs[index].data[0] = step_base_[index].data[0] + leg_position_delta_[index].data[0] * cos_phase;
        legs[index].data[1] = step_base_[index].data[1] + leg_position_delta_[index].data[1] * cos_phase;
        legs[index].data[2] = gait_base_[index].data[2] + (step_heigth_ * z_cos_phase);
    }
    else if(motion_type == LEG_STANDING)
    {
        double cos_phase = (1 - cos(M_PI * phi))*0.5;
//        legs[index].data[0] = step_base_[index].data[0] + leg_position_delta_[index].data[0] * cos_phase;
//        legs[index].data[1] = step_base_[index].data[1] + leg_position_delta_[index].data[1] * cos_phase;
        legs[index].data[0] = step_base_[index].data[0] + leg_position_delta_[index].data[0] * phi;
        legs[index].data[1] = step_base_[index].data[1] + leg_position_delta_[index].data[1] * phi;
        legs[index].data[2] = gait_base_[index].data[2];
    }

}


inline bool GaitGeneratorSimple::almostEqual(double a, double b, double e)
{
    return abs(a-b)<e ? true : false;
}

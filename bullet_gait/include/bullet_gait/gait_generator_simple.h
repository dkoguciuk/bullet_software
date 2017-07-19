#include "boost/noncopyable.hpp"
#include "kdl/frames.hpp"
#include <vector>
#include "gait_generator.h"


class GaitGeneratorSimple : private boost::noncopyable
{
public:
    enum TrajectoryType{
        TRAJECTORY_GAIT,
        TRAJECTORY_CUSTOM
    };

    static GaitGeneratorSimple* getInstance();

    /**
     * @brief setVelocity Sets linear velocity vector
     * @param linear_velocity Linear velocity vector
     * @return Result of operation     */
    bool setLinearVelocity(KDL::Vector &linear_velocity);

    /**
     * @brief setVelocity Sets angular velocity vector
     * @param angular_velocity Angular velocity vector
     * @return Result of operation     */
    bool setAngularVelocity(double angular_velocity_z);

    /**
     * @brief setGaitParameters sets gait generation constans
     * @param step_heigth height of step (how high sweeping leg will be raised)
     * @param time_period how long will last every half-step
     * @return Result of operation*/
    bool setGaitParameters(double step_heigth, double time_period);

    bool setCustomTrajectoryTime(double time);

    /**
     * @brief setRobotPose Sets robot pose to given
     * @param leg_tip_position vector of leg KDL::Vectors
     * @return result of operation */
    bool setRobotPose(std::vector<KDL::Vector> &leg_tip_position);

    /**
     * @brief setGaitBase Sets base of gait (layout of legs when standing still)
     * @param base vector of KDL::Vectors of leg positions in BODY frame
     * @return result of operation */
    bool setGaitBase(std::vector<KDL::Vector> &base);

    /**
     * @brief setSamplingPeriod Sets time quantum for generating gait. Trajectory resolution depends of it
     * @param period time period in seconds
     * @return result of operation  */
    bool setSamplingPeriod(double period);

    /**
     * @brief getSamplingPeriod Gets time quantum of gait generation.
     * @return sampling period */
    double getSamplingPeriod();
    /**
     * @brief getRobotPose Returns reference to generated leg tips positions
     * @param leg_tips reference to leg tips vector */
    void getRobotPose(std::vector<KDL::Vector> &leg_tips);

    /**
     * @brief setCostumTrajectoryPoints Sets points of costum trajectory
     * @param points vector of vectors of KDL::Vectors. That is, a list of trajectory points (each trajectory point is described by 6 KDL::Vectors)
     * @return result of operation */
    bool setCustumTrajectoryPoints(std::vector<std::vector<KDL::Vector> > &points);

    /**
     * @brief generate Main worker function. Generates single vector of trajectory points, which may be get by getRobotPose()
     * @return result of operation */
    bool generateTrajectory(TrajectoryType type);




private:

    /**
     * @brief The LegMotionType enum enumerates possible motions of leg
     */
    enum LegMotionType{
        LEG_SWEEPING,   /**< Leg is sweeping in gait */
        LEG_STANDING,   /**< Leg is standing in gait */
        LEG_OTHER       /**< Leg does custom move */
    };

    GaitGeneratorSimple();
    ~GaitGeneratorSimple();

    void calculateLegTips(std::vector<KDL::Vector> &legs, int index, LegMotionType motion_type);
    bool almostEqual(double a, double b, double e = 0.001);

    /// Singleton magic
    static bool g_instance_initialised;
    static GaitGeneratorSimple g_instance;


    /// State variables
    std::vector<KDL::Vector> current_pose_;         /**< Current pose of robot - updated everytime generate() is called */
    std::vector<KDL::Vector> step_base_;            /**< Value of current_pose_ at beginning of step - base of particular step */
    std::vector<KDL::Vector> gait_base_;            /**< Base position of legs given in BODY frame. It is pose of robot when it stands still */
    std::vector<KDL::Vector> leg_position_delta_;   /**< Difference between setpoint and actual position of legs at cycle beginning */
    std::vector<KDL::Vector> leg_setpoint_;         /**< Leg setpoint position */

    /// Custom trajectory points
    std::vector<std::vector<KDL::Vector> > custom_trajectory_; /**< Points of custom trajectory */

    /// Setpoints
    KDL::Vector linear_velocity_;
    double angular_velocity_;       /**< Angular velocity for Z axis */
    double body_heigth_;            /**< Body heigth */

    /// Gait parameters
    double dt_;                      /**< Sampling period */
    double t_;                       /**< Gait period */
    double trajectory_t_;             /**< Custom trajectory realisation time */
    double step_heigth_;             /**< Step height (how high leg will be elevated during sweep)*/
    int gait_trajectory_resolution_; /**< Calculated from gait period and sampling period */
    int custom_trajectory_resolution_;/**< */

    /// Gait helpers
    int step_phase_;
    bool half_step_;

    /// Custom trajectory helpers
    int trajectory_phase_;
    int trajectory_point_index_;

};

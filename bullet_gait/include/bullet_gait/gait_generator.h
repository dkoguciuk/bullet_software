#include "boost/noncopyable.hpp"
#include "kdl/frames.hpp"
#include <vector>

struct RobotPose
{
    RobotPose() : leg_tip_position(std::vector<KDL::Vector>(6)){}
    KDL::Vector body_position;                 /**< Robot body position */
    KDL::Rotation body_orientation;            /**< Robot body orientation */
    std::vector<KDL::Vector> leg_tip_position; /**< Robot legs position */
};

class GaitGenerator : private boost::noncopyable
{
public:
    static GaitGenerator* getInstance();

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
     * @brief setBodyHeigth Sets body heigth.
     * @detail Will be reached with given velocity
     * @param height Target height of body
     * @param v_z Velocity
     * @return Result of operation */
    bool setBodyHeigth(double height, double v_z);

    /**
     * @brief setBodyRP Sets body roll and pitch angles
     * @param r Roll angle. If nan, wont be changed
     * @param p Pitch angle. If nan, wont be changed
     * @return Resutl of operation
     */
    bool setBodyRP(double r = 0, double p = 0, double v = 1);

    /**
     * @brief setGaitParameters sets gait generation constans
     * @param k1 distance to time ratio for translation
     * @param k2 angle to time ratio for rotation
     * @return Result of operation*/
    bool setGaitParameters(double step_heigth, double time_period);

    bool setRobotPose(KDL::Vector &body_position, KDL::Rotation &body_orientation, std::vector<KDL::Vector> &leg_tip_position);

    bool setLegBase(std::vector<KDL::Vector> &base);

    bool setSamplingPeriod(double period);

    bool generate();

    void getRobotPose(KDL::Vector &body_position, KDL::Rotation &body_orientation, std::vector<KDL::Vector> &leg_tips);

private:
    enum MotionState{
        STATE_WALK,
        STATE_STAND_UP,
        STATE_LAY_DOWN,
        STATE_CHANGE_LEG_BASE
    };

    GaitGenerator();
    ~GaitGenerator();

    void calculateLegTips(std::vector<KDL::Vector> &legs, int index);
    bool almostEqual(double a, double b, double e = 0.001);

    /// Singleton magic
    static bool g_instance_initialised;
    static GaitGenerator g_instance;


    /// State variables
    MotionState state_;             /**< Current state of generator */
    RobotPose current_pose_;        /**< Current pose of robot */
    RobotPose base_position_;       /**< Base position for step */
    RobotPose setpoint_position_;    /**< Target position (for end of step) */
    std::vector<KDL::Vector> leg_base_; /**< Base position of legs given in BODY frame */


    /// Setpoints
    KDL::Vector linear_velocity_;
    double angular_velocity_;       /**< Angular velocity for Z axis */
    double body_heigth_;            /**< Body heigth */
    double body_pitch_;             /**< Body pitch */
    double body_roll_;              /**< Body roll */

    /// Gait parameters
    double dt_;             /**< Sampling period */
    double t_;              /**< Gait period */
    double step_heigth_;     /**< Step height (how high leg will be elevated during sweep)*/
    int trajectory_resolution_; /**< Calculated from gait period and sampling period */

    /// Gait helpers
    int phase_;
    bool half_step_;

};

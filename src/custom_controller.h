#include <tocabi_controller/data_container.h>
#include <tocabi_controller/link.h>
#include "math_type_define.h"

class CustomController
{
public:
    CustomController(DataContainer &dc,RobotData &rd, Wholebody_controller &wc);
    Eigen::VectorQd getControl();
    void compute_slow();
    void compute_fast();
    
    DataContainer &dc_;
    RobotData &rd_;
    Wholebody_controller &wc_;
    void setWalkingParameter(double walking_duration, double walking_speed, double step_width, double knee_target_angle);


private:
    void getRobotData();
    void getProcessedRobotData();
    void walkingStateManager();
    void motionGenerator();
    void getCOMTrajectory();
    void getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired);
    void computeIk(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des);
    Eigen::VectorQd comVelocityControlCompute();
    Eigen::VectorQd jointTragectoryPDControlCompute();
    bool balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d);
    void computeZmp();
    void savePreData();

    
    Eigen::VectorQd ControlVal_;
    
    int foot_contact_;                                      // 1:left,   -1:right,   0:double
    bool foot_swing_trigger_;                               // trigger in on when the absolute value of the com velocity is over the threshold
    double stop_vel_threshold_;

    double walking_duration_;
    double walking_phase_;
    double turning_phase_;
    
    double current_time_;
    double pre_time_;
    double start_time_;
    double dt_;

    double walking_speed_;
    double yaw_angular_vel_;
    double knee_target_angle_;

    double step_width_;
    double step_length_;

    // CoM variables
    Eigen::Vector3d com_pos_desired_; 
    Eigen::Vector3d com_vel_desired_;
    Eigen::Vector3d com_pos_current_;
    Eigen::Vector3d com_vel_current_; 
    Eigen::Vector3d com_pos_init_;
    Eigen::Vector3d com_vel_init_;

    // Joint related variables
    Eigen::VectorQd current_q_;
    Eigen::VectorQd current_q_dot_;
    Eigen::VectorQd current_q_ddot_;
    Eigen::VectorQd desired_q_;
    Eigen::VectorQd desired_q_dot_;
    Eigen::VectorQd desired_q_ddot_;
    Eigen::VectorQd pre_q_;

    Eigen::VectorQd pd_control_mask_;

    Eigen::Vector2d target_foot_landing_from_pelv_;
    Eigen::Vector2d target_foot_landing_from_sup_;
    Eigen::Vector3d swing_foot_pos_trajectory_from_pelv_;
    Eigen::Vector3d swing_foot_pos_trajectory_from_sup_;
    Eigen::Vector6d swing_foot_vel_trajectory_from_pelv_;
    Eigen::Vector6d swing_foot_vel_trajectory_from_sup_;
    Eigen::Matrix3d swing_foot_rot_trajectory_from_pelv_;
    Eigen::Matrix3d swing_foot_rot_trajectory_from_sup_;

    Eigen::Vector3d swing_foot_pos_init_;
    Eigen::Matrix3d swing_foot_rot_init_;

    Eigen::MatrixXd jac_com_;
    Eigen::MatrixXd jac_com_pos_;
    Eigen::MatrixXd jac_rhand_;
    Eigen::MatrixXd jac_lhand_;
    Eigen::MatrixXd jac_rfoot_;
    Eigen::MatrixXd jac_lfoot_;


    Eigen::Isometry3d lfoot_transform_init_from_pelv_;
    Eigen::Isometry3d rfoot_transform_init_from_pelv_;
    Eigen::Isometry3d lfoot_transform_init_from_sup_;
    Eigen::Isometry3d rfoot_transform_init_from_sup_;

    Eigen::Isometry3d lfoot_transform_current_from_pelv_;
    Eigen::Isometry3d rfoot_transform_current_from_pelv_;
    Eigen::Isometry3d lfoot_transform_current_from_sup_;
    Eigen::Isometry3d rfoot_transform_current_from_sup_;
    
    Eigen::Vector2d zmp_l_;
    Eigen::Vector2d zmp_r_;

    Eigen::Vector2d zmp_measured_;
    Eigen::Vector2d zmp_measured_pre_;
    Eigen::Vector2d zmp_measured_ppre_;
    
    Eigen::Vector6d l_ft_;
    Eigen::Vector6d r_ft_;

};
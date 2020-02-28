#include "custom_controller.h"
#include "tocabi_controller/wholebody_controller.h"


CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd)
{
    ControlVal_.setZero();

    
    stop_vel_threshold_ =  0.03;
    walking_duration_ = 1;
    walking_speed_ = 0.0;
    step_width_ = 0.20;
    knee_target_angle_ = M_PI/40;                               //4.5degree
    yaw_angular_vel_ = 1;                                       //   rad/s

    foot_swing_trigger_ == false;
    foot_contact_ = 1;

    jac_com_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_com_pos_.setZero(3, MODEL_DOF_VIRTUAL);
    jac_rhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_rfoot_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lfoot_.setZero(6, MODEL_DOF_VIRTUAL);
    //set init pre data
    pre_time_ = current_time_;
    
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::compute_slow()
{
    //rd_.control_time_; current time
    //rd_.link_[Right_Foot].Jac : current rightfoot jac
    //rd_.q_dot_ : current q velocity

    //rd_.link_[Right_Foot]

    getRobotData();
    walkingStateManager();
    getProcessedRobotData();

    motionGenerator();
    getCOMTrajectory();
    getSwingFootXYTrajectory(walking_phase_, com_pos_current_, com_vel_current_, com_vel_desired_);

    ControlVal_ += comVelocityControlCompute();             //support control for COM velocity control
    ControlVal_ += jointTragectoryPDControlCompute();       //upper body motion + swing foot control + knee angle control
    
    savePreData();
}

void CustomController::compute_fast()
{

}

void CustomController::setWalkingParameter(double walking_duration, double walking_speed, double step_width, double knee_target_angle)
{
    walking_duration_ = walking_duration;
    walking_speed_ = walking_speed;
    step_width_ = step_width;
    knee_target_angle_ = knee_target_angle;

    walking_duration = DyrosMath::minmax_cut(walking_duration, 0.5, 1.5);
    walking_speed_ = DyrosMath::minmax_cut(walking_speed_, -0.5, 1.0);
    step_width_ = DyrosMath::minmax_cut(step_width_, 0.17, 0.25);
    knee_target_angle_ = DyrosMath::minmax_cut(knee_target_angle_, 0.0, 1.5);
}

void CustomController::getRobotData()
{
    // Wholebody_controller wc_(dc_, rd_);

    current_time_ = rd_.control_time_;
    dt_ = current_time_ - pre_time_;


    current_q_ = rd_.q_;
    current_q_dot_ = rd_.q_dot_;

    com_pos_current_ = rd_.link_[COM_id].xpos;
    com_vel_current_ = rd_.link_[COM_id].v;

    jac_com_ = rd_.link_[COM_id].Jac;
    jac_com_pos_ = rd_.link_[COM_id].Jac_COM_p;
    jac_rhand_ = rd_.link_[Right_Hand].Jac;
    jac_lhand_ = rd_.link_[Left_Hand].Jac;
    jac_rfoot_ = rd_.link_[Right_Foot].Jac;
    jac_lfoot_ = rd_.link_[Left_Foot].Jac;

    // rd_.ZMP_ft = wc_.GetZMPpos(rd_);
    zmp_measured_ = rd_.ZMP_ft.segment<2>(0);

    l_ft_ = rd_.LH_FT;
    r_ft_ = rd_.RH_FT; 
}

void CustomController::walkingStateManager()
{
    if(walking_speed_ == 0)
    {
        if(walking_phase_ == 1)
        {
            if( balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0)) ) //gonna be fall
            {
                foot_swing_trigger_ = true;         
                start_time_ = current_time_;    
                foot_contact_ *= -1;                        //support foot change
            }
            else
            {
                foot_swing_trigger_ = false;
            }
        }
        
    }
    else
    {
        if(walking_phase_ == 1)
        {
                foot_swing_trigger_ = true;         
                start_time_ = current_time_;    
                foot_contact_ *= -1;     
        }
    }

    
 
    walking_phase_ = (current_time_-start_time_)/walking_duration_;
    walking_phase_ = DyrosMath::minmax_cut(walking_phase_, 0, 1);
}

bool CustomController::balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d)
{
    bool trigger;
    Vector2d capture_point_2d;
    double omega;
    omega = sqrt(com_pos_current_(2)/GRAVITY);
    capture_point_2d = com_pos_2d + com_vel_2d/omega;

    if(capture_point_2d.norm() > stop_vel_threshold_)
    {
        trigger = true;
    }

    return trigger;
}

void CustomController::getProcessedRobotData()
{
    if(current_time_ = start_time_)
    {
        com_pos_init_ = com_pos_current_;
        com_vel_init_ = com_vel_current_;

        if(foot_contact_ == 1) // left support foot
        {
            swing_foot_pos_init_ = rd_.link_[Right_Foot].xpos;
            swing_foot_rot_init_ = rd_.link_[Right_Foot].Rotm;
        }
        else if(foot_contact_ == -1) //right support foot
        {
            swing_foot_pos_init_ = rd_.link_[Left_Foot].xpos;
            swing_foot_rot_init_ = rd_.link_[Left_Foot].Rotm;
        }
    }    
}



void CustomController::motionGenerator()
{   


    desired_q_dot_.setZero();
    desired_q_.setZero();
    pd_control_mask_.setZero();

    /////////////////////KNEE///////////////////
    if(foot_swing_trigger_ == true)
    {
        if(foot_contact_ == 1) //left support
        {
            desired_q_(3) = knee_target_angle_;
            pd_control_mask_(3) = 1;
        }
        else if (foot_contact_ == -1) //right support
        {
            desired_q_(9) = knee_target_angle_;
            pd_control_mask_(9) = 1;
        }
    }
    else
    {
        desired_q_(3) = knee_target_angle_;
        pd_control_mask_(3) = 1;
        desired_q_(9) = knee_target_angle_;
        pd_control_mask_(9) = 1;
    }
    //////////////////////////////////////////////
    
    ///////////////////////WAIST/////////////////////////
    desired_q_(12) = 0;//yaw
    desired_q_(13) = 0;//pitch
    desired_q_(14) = 0;//roll
    pd_control_mask_(12) = 1;
    pd_control_mask_(13) = 1;
    pd_control_mask_(14) = 1;
    /////////////////////////////////////////////////////

    ///////////////////////HEAD/////////////////////////
    desired_q_(23) = 0;//way                     
    desired_q_(24) = 0;//pitch
    pd_control_mask_(23) = 1;
    pd_control_mask_(24) = 1;
    /////////////////////////////////////////////////////

    ///////////////////////ARM/////////////////////////
    //////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
    desired_q_(15) = 0.3;                     
    desired_q_(16) = 0.3;
    desired_q_(17) = 1.5;
    desired_q_(18) = -1.27;                     
    desired_q_(19) = -1.0;
    desired_q_(20) = 0.0;
    desired_q_(21) = -1.0;                     
    desired_q_(22) = 0.0;
    pd_control_mask_(15) = 1;
    pd_control_mask_(16) = 1;
    pd_control_mask_(17) = 1;
    pd_control_mask_(18) = 1;
    pd_control_mask_(19) = 1;
    pd_control_mask_(20) = 1;
    pd_control_mask_(21) = 1;
    pd_control_mask_(22) = 1;
    //////////////////////
    /////RIFHT ARM////////-0.3 -0.3 -1.5 1.27 1 0 1 0
    desired_q_(25) = -0.3;
    desired_q_(26) = -0.3;
    desired_q_(27) = -1.5;
    desired_q_(28) = 1.27;
    desired_q_(29) = 1.0;
    desired_q_(30) = 0.0;
    desired_q_(31) = 1.0;
    desired_q_(32) = 0.0;
    pd_control_mask_(25) = 1;
    pd_control_mask_(26) = 1;
    pd_control_mask_(27) = 1;
    pd_control_mask_(28) = 1;
    pd_control_mask_(29) = 1;
    pd_control_mask_(30) = 1;
    pd_control_mask_(31) = 1;
    pd_control_mask_(32) = 1;
    /////////////////////////////////////////////////////
    if(walking_phase_<0.5)
    {
        swing_foot_pos_trajectory_from_pelv_(2) = DyrosMath::QuinticSpline(walking_phase_, 0, 0.5, swing_foot_pos_init_(2)+0, 0, 0, swing_foot_pos_init_(2)+0.05, 0, 0)(0);
        swing_foot_vel_trajectory_from_pelv_(2) = DyrosMath::QuinticSpline(walking_phase_, 0, 0.5, swing_foot_pos_init_(2)+0, 0, 0, swing_foot_pos_init_(2)+0.05, 0, 0)(1); 
    }
    else
    {
        swing_foot_pos_trajectory_from_pelv_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.5, 1.0, swing_foot_pos_init_(2)+0.05, 0, 0, swing_foot_pos_init_(2)+0, 0, 0)(0);
        swing_foot_vel_trajectory_from_pelv_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.5, 1.0, swing_foot_pos_init_(2)+0.05, 0, 0, swing_foot_pos_init_(2)+0, 0, 0)(1); 
    }   
}

void CustomController::getCOMTrajectory()
{
    com_vel_desired_(0) = walking_speed_;
    com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;

    if(foot_swing_trigger_ == true)
    {
        com_pos_desired_(1) = DyrosMath::cubic(current_time_, start_time_, start_time_+walking_duration_, com_pos_init_(1), com_pos_init_(1)-step_width_*foot_contact_, 0, 0);  
        com_vel_desired_(1) = DyrosMath::cubicDot(current_time_, start_time_, start_time_+walking_duration_, com_pos_init_(1), com_pos_init_(1)-step_width_*foot_contact_, 0, 0, 1/dt_);  
    }
    else
    {
        com_pos_desired_(1) = com_pos_init_(1);
        com_vel_desired_(1) = 0;
    }
}

void CustomController::getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired)
{   
    Eigen::Vector2d d;
    Eigen::Vector2d d_prime;
    double d_temp;
    double alpha = 0.05;

    if(foot_swing_trigger_ == true)
    {
        // x axis
        d_temp = sqrt(com_pos_current(2)/GRAVITY + pow(com_vel_current(0)/(2*GRAVITY), 2) );
        d(0) = com_vel_current(0)*d_temp;
        // y axis
        d_temp = sqrt(com_pos_current(2)/GRAVITY + pow(com_vel_current(1)/(2*GRAVITY), 2) );
        d(1) = com_vel_current(1)*d_temp;

        d_prime(0) = d(0) - alpha*com_vel_desired(0);
        d_prime(1) = d(1);
        // d_prime(1) = d(1) - alpha*com_vel_desired(1);

        d_prime(0) = DyrosMath::minmax_cut(d_prime(0), -0.3, 0.5);


        if(foot_contact_ == 1) //left support
        {
            d_prime(0) = DyrosMath::minmax_cut(d_prime(0), lfoot_transform_current_from_pelv_.translation()(0)-0.5, lfoot_transform_current_from_pelv_.translation()(0) + 0.5);
            d_prime(1) = DyrosMath::minmax_cut(d_prime(1), lfoot_transform_current_from_pelv_.translation()(1)-0.5, lfoot_transform_current_from_pelv_.translation()(1)-0.17);
        }
        else if (foot_contact_ == -1) // right support
        {
            d_prime(0) = DyrosMath::minmax_cut(d_prime(0), rfoot_transform_current_from_pelv_.translation()(0)-0.5, rfoot_transform_current_from_pelv_.translation()(0) + 0.5);
            d_prime(1) = DyrosMath::minmax_cut(d_prime(1), rfoot_transform_current_from_pelv_.translation()(1)+0.17, rfoot_transform_current_from_pelv_.translation()(1)+0.5);
        }
        

        target_foot_landing_from_pelv_ = com_pos_current.segment<2>(0) + d_prime;


        //retrun 
        swing_foot_pos_trajectory_from_pelv_.segment<2>(0) = (1 - phase)*swing_foot_pos_init_.segment<2>(0) + phase*target_foot_landing_from_pelv_;
        swing_foot_vel_trajectory_from_pelv_.segment<2>(0) = (target_foot_landing_from_pelv_ - swing_foot_pos_init_.segment<2>(0))/walking_duration_;
        swing_foot_rot_trajectory_from_pelv_.setIdentity();  //no orientation
    }
    else
    {   
        swing_foot_pos_trajectory_from_pelv_ = swing_foot_pos_init_;
        swing_foot_vel_trajectory_from_pelv_.setZero();
        swing_foot_rot_trajectory_from_pelv_ = swing_foot_rot_init_;
    }
}

Eigen::VectorQd CustomController::comVelocityControlCompute()
{
    Wholebody_controller wc_(dc_, rd_);

    Eigen::VectorQd torque;

    Eigen::Vector6d f_com;
    Eigen::MatrixXd jac_com_xy;
    jac_com_xy.setZero(6, MODEL_DOF_VIRTUAL);
    f_com.setZero();
    
    double kp = 400;
    double kv = 40;

    f_com(0) = kv*(com_vel_desired_(0) - com_vel_current_(0));                                                          //X axis D control
    f_com(1) = kv*(com_vel_desired_(1) - com_vel_current_(1)) + kp*(com_pos_desired_(1)- com_pos_current_(1));          //Y axis PD control

    jac_com_xy = jac_com_;
    jac_com_xy.block(0, 0, 3, MODEL_DOF_VIRTUAL) = jac_com_pos_;
    jac_com_xy.block(0, 21, 6, 8).setZero(); // Exclude Left hand Jacobian
    jac_com_xy.block(0, 31, 6, 8).setZero(); // Exclude Right hand Jacobian
    // jac_com_xy.block(0, 29, 2, 2).setZero(); // Exclude Head Jacobian
    if(foot_contact_ == 1) //left support
    {
        jac_com_xy.block(0, 12, 6, 8).setZero(); //Exclude Rifht Leg Jacobian
    }
    else if(foot_contact_ == -1) //right support
    {
        jac_com_xy.block(0, 6, 6, 8).setZero(); //Exclude Left Leg Jacobian
    }

    torque = wc_.task_control_torque_QP2(rd_, jac_com_xy, f_com);  //jacobian control + gravity torque

    return torque;
}

Eigen::VectorQd CustomController::jointTragectoryPDControlCompute()
{
    Eigen::VectorQd torque;
    Eigen::Vector12d desired_q_leg;
    Eigen::Isometry3d pelv_transform_from_pelv;
    Eigen::Isometry3d lleg_transform_from_pelv;
    Eigen::Isometry3d rleg_transform_from_pelv;

    double kp_j = 9;
    double kv_j = 6;

    pelv_transform_from_pelv.setIdentity();
    if(foot_swing_trigger_ == true)
    {
        if(foot_contact_ == 1) //left support
        {
            rleg_transform_from_pelv.translation() = swing_foot_pos_trajectory_from_pelv_;
            rleg_transform_from_pelv.linear() = swing_foot_rot_trajectory_from_pelv_;

            computeIk(pelv_transform_from_pelv, lleg_transform_from_pelv, rleg_transform_from_pelv, desired_q_leg);

            for(int i = 0; i<6; i++)
            {
                desired_q_(i) = desired_q_leg(i);
                pd_control_mask_(i);
            }
        }
        else if(foot_contact_ == -1) //right support
        {

            lleg_transform_from_pelv.translation() = swing_foot_pos_trajectory_from_pelv_;
            lleg_transform_from_pelv.linear() = swing_foot_rot_trajectory_from_pelv_;

            computeIk(pelv_transform_from_pelv, lleg_transform_from_pelv, rleg_transform_from_pelv, desired_q_leg);

            for(int i = 6; i<12; i++)
            {
                desired_q_(i) = desired_q_leg(i);
                pd_control_mask_(i);
            }
        }
    }
    else
    {
        torque.setZero();
    }
    
    for(int i = 0; i<MODEL_DOF; i++)
    {
        torque(i) = kp_j*(desired_q_(i) - current_q_(i)) + kv_j*(desired_q_dot_(i) - current_q_dot_(i));
        torque(i) *= pd_control_mask_(i);   // masking for joint pd control 
    }
    
    return torque;
}

void CustomController::computeIk(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des)
{
  //float = World/ trunk = pelvis
  // 명주 정리 (KAJITA 책 <-> Code 구성)
  // float_trunk_transform.rotation() : float에서 바라본 pelvis rotation -> R1
  // float_trunk_transform.translation() : float에서 바라본 pelvis 좌표 -> P1
  // float_rleg_transform.rotation() : float에서 바라본 발끝 rotation -> R7 
  // float_rleg_transform.translation() : float에서 바라본 발끝 좌표 -> P7
  // float_trunk_transform.translation() + float_trunk_transform.rotation()*D  : float에서 바라본 pelvis 좌표 + float 좌표계로 변환 * pelvis 좌표계에서 바라본 pelvis ~ hip까지 거리-> P2
  
  // R7.transpose * (P2 - P7) , P2 = P1 + R1*D


  Eigen::Vector3d R_r, R_D, L_r, L_D ;

  L_D << 0 , 0.1025, -0.1225;
  R_D << 0 , -0.1025, -0.1225;
  
  L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*L_D - float_lleg_transform.translation());
  R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*R_D - float_rleg_transform.translation());
  
  double R_C = 0, L_C = 0, L_upper = 0.35, L_lower = 0.35 , R_alpha = 0, L_alpha = 0;

  L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
  R_C = sqrt( pow(R_r(0),2) + pow(R_r(1),2) + pow(R_r(2),2) );
  
  q_des(3) = (-acos((pow(L_upper,2) + pow(L_lower,2) - pow(L_C,2)) / (2*L_upper*L_lower))+ M_PI) ;
  q_des(9) = (-acos((pow(L_upper,2) + pow(L_lower,2) - pow(R_C,2)) / (2*L_upper*L_lower))+ M_PI) ;
  L_alpha = asin(L_upper / L_C * sin(M_PI - q_des(3)));
  R_alpha = asin(L_upper / R_C * sin(M_PI - q_des(9)));

  q_des(4)  = -atan2(L_r(0), sqrt(pow(L_r(1),2) + pow(L_r(2),2))) - L_alpha ;
  q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) - R_alpha ;
  
  // trunk_lleg_rotation -> R1.transpose * R7 
  // Ryaw * Rroll * Rpitch = R1.transpose * R7 * ~ 
  Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
  Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
  Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

  L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3)-q_des(4));
  L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
  R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9)-q_des(10));
  R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11)); 
  
  L_Hip_rot_mat.setZero(); R_Hip_rot_mat.setZero();

  L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat; 
  R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

  q_des(0) = -atan2(-L_Hip_rot_mat(0,1),L_Hip_rot_mat(1,1)); // Hip yaw
  q_des(1) =  atan2(L_Hip_rot_mat(2,1), -L_Hip_rot_mat(0,1) * sin(q_des(0)) + L_Hip_rot_mat(1,1)*cos(q_des(0))); // Hip roll
  q_des(2) =  atan2(-L_Hip_rot_mat(2,0), L_Hip_rot_mat(2,2)) ; // Hip pitch
  q_des(3) =  q_des(3) ; // Knee pitch
  q_des(4) =  q_des(4) ; // Ankle pitch
  q_des(5) =  atan2( L_r(1), L_r(2) ); // Ankle roll

  q_des(6) = -atan2(-R_Hip_rot_mat(0,1),R_Hip_rot_mat(1,1));
  q_des(7) =  atan2(R_Hip_rot_mat(2,1), -R_Hip_rot_mat(0,1) * sin(q_des(6)) + R_Hip_rot_mat(1,1)*cos(q_des(6)));
  q_des(8) = atan2(-R_Hip_rot_mat(2,0), R_Hip_rot_mat(2,2)) ;
  q_des(9) = q_des(9) ;
  q_des(10) = q_des(10) ; 
  q_des(11) =  atan2( R_r(1), R_r(2) );
}


void CustomController::savePreData()
{
    pre_time_ = current_time_;
    pre_q_ = current_q_;
    zmp_measured_ppre_ = zmp_measured_pre_;
    zmp_measured_pre_ = zmp_measured_;
}
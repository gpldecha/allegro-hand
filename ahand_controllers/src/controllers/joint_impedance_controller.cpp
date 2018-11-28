#include "ahand_controllers/controllers/joint_impedance_controller.h"
#include <algorithm>
#include <memory>
#include <functional>
#include "utils/filters.h"
#include "utils/utils.h"
#include <string>
#include <iostream>


JointImpedanceController::JointImpedanceController(ros::NodeHandle &nh): BaseController(nh){

    joint_des_position_.setZero();
    joint_filtered_position_.setZero();
    joint_filtered_velocity_.setZero();

    kp_.resize(n_joints_, 0.0);
    kd_.resize(n_joints_, 0.0);

    std::string name_space = nh.getNamespace();

    // position command topic
    sub_command_pose_ = nh.subscribe("joint_impedance/position_command", 10, &JointImpedanceController::command_callback, this);

    // Dynamic reconfigure
    nh_gains_pd_ = ros::NodeHandle("gains_pd");
    dynamic_server_gains_dp_param_.reset( new dynamic_reconfigure::Server< ahand_controllers::gains_pd_paramConfig>(nh_gains_pd_) );
    dynamic_server_gains_dp_param_->setCallback(boost::bind(&JointImpedanceController::gains_pd_callback, this, _1, _2));

}

void JointImpedanceController::start(const hand_info& hand_info, const ros::Duration& period){

}


void JointImpedanceController::update(const hand_info& hand_info, Vector16d& torque_commands, const ros::Duration& period){

    const Vector16d& measured_joint_positions = hand_info.measured_joint_positions;
    const Vector16d& estimated_joint_velocities = hand_info.estimated_joint_velocities;

    // filter
    for(size_t i=0; i< measured_joint_positions.size(); i++) {
        joint_filtered_position_[i] = ahand_controllers::exponentialSmoothing(measured_joint_positions[i], joint_filtered_position_[i], 0.2);
        joint_filtered_velocity_[i] = ahand_controllers::exponentialSmoothing(estimated_joint_velocities[i], joint_filtered_velocity_[i], 0.2);
    }

    // joint impedance controller
    for(size_t i=0; i< hand_info.measured_joint_positions.size(); i++) {
        torque_commands[i] = kp_[i]*(joint_des_position_[i] - joint_filtered_position_[i]) - kd_[i]*joint_filtered_velocity_[i];
    }

}

void JointImpedanceController::command_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
      std::copy(msg->data.begin(), msg->data.end(), joint_des_position_.data());
}

void JointImpedanceController::gains_pd_callback(ahand_controllers::gains_pd_paramConfig& config, uint32_t level){
    kp_[0]  = config.p00; kp_[1]  = config.p01; kp_[2]  = config.p02; kp_[3]  = config.p03;
    kp_[4]  = config.p10; kp_[5]  = config.p11; kp_[6]  = config.p12; kp_[7]  = config.p13;
    kp_[8]  = config.p20; kp_[9]  = config.p21; kp_[10] = config.p22; kp_[11] = config.p23;
    kp_[12] = config.p30; kp_[13] = config.p31; kp_[14] = config.p32; kp_[15] = config.p33;

    kd_[0]  = config.d00; kd_[1]  = config.d01; kd_[2]  = config.d02; kd_[3]  = config.d03;
    kd_[4]  = config.d10; kd_[5]  = config.d11; kd_[6]  = config.d12; kd_[7]  = config.d13;
    kd_[8]  = config.d20; kd_[9]  = config.d21; kd_[10] = config.d22; kd_[11] = config.d23;
    kd_[12] = config.d30; kd_[13] = config.d31; kd_[14] = config.d32; kd_[15] = config.d33;
    for(std::size_t j = 0; j < n_joints_; j++){
        kp_[j] = kp_[j]/1000.0;
        kd_[j] = kd_[j]/1000.0;
    }
}


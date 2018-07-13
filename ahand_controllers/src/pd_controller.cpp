#include "ahand_controllers/pd_controller.h"
#include <algorithm>
#include <memory>
#include <functional>
#include "utils/filters.h"
#include <string>
#include <iostream>

 std::array<double, 16> home_pose = {
   0.0,  35.0, 35.0, 35.0,
   0.0,  35.0, 35.0, 35.0,
   0.0,  35.0, 35.0, 35.0,
   40.0, 0.0,  35.0, 45.0
};


ahand_controllers::PDController::PDController(){}


bool ahand_controllers::PDController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &nh){

    n_joints_ = robot->getNames().size();
    ROS_INFO("Initisalising PDController");
    ROS_INFO_STREAM("PDController number of joints: "  << n_joints_);
    for(std::size_t i = 0; i < n_joints_; i++){
        joint_handles_.push_back(robot->getHandle("ahand_joint_" + boost::lexical_cast<std::string>(i)));
    }

    // convert deg to rad
    for(std::size_t i = 0; i < home_pose.size(); i++){
        home_pose[i] = home_pose[i]*M_PI/180.0;
    }

    tau_cmd_.resize(n_joints_, 0.0);
    joint_msr_position_.resize(n_joints_, 0.0);
    joint_msr_velocity_.resize(n_joints_, 0.0);
    joint_des_position_.resize(n_joints_, 0.0);
    joint_filtered_position_.resize(n_joints_, 0.0);
    joint_filtered_velocity_.resize(n_joints_, 0.0);

    kp_.resize(n_joints_, 0.0);
    kd_.resize(n_joints_, 0.0);

    realtime_publisher.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(nh,"pd_controller",1) );
    data_msg_.layout.dim.resize(1);
    data_msg_.data.resize(n_joints_);

    // Dynamic reconfigure
    nh_gains_pd_ = ros::NodeHandle("gains_pd");
    dynamic_server_gains_dp_param_.reset( new dynamic_reconfigure::Server< ahand_controllers::gains_pd_paramConfig>(nh_gains_pd_) );
    dynamic_server_gains_dp_param_->setCallback(boost::bind(&ahand_controllers::PDController::gains_pd_callback, this, _1, _2));
    ROS_INFO("Sucessfuly initialised PDController");
    return true;
}

void ahand_controllers::PDController::starting(const ros::Time& time){
    // read
    ROS_INFO("PDController starting");
    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_msr_position_[i] = joint_handles_[i].getPosition();
        joint_msr_velocity_[i] = joint_handles_[i].getVelocity();
        joint_des_position_[i] = home_pose[i];
        joint_filtered_position_[i] = joint_msr_position_[i];
        joint_filtered_velocity_[i] = joint_msr_velocity_[i];
    }
    ROS_INFO("PDController started!");

}

void ahand_controllers::PDController::update(const ros::Time& time, const ros::Duration& period){
    // read
    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_msr_position_[i] = joint_handles_[i].getPosition();
        joint_msr_velocity_[i] = joint_handles_[i].getVelocity();
        joint_filtered_position_[i] = ahand_controllers::exponentialSmoothing(joint_msr_position_[i], joint_filtered_position_[i], 0.2);
        joint_filtered_velocity_[i] = ahand_controllers::exponentialSmoothing(joint_msr_velocity_[i], joint_filtered_velocity_[i], 0.2);
    }

    // pd controller
    for(size_t i=0; i<joint_handles_.size(); i++) {
      tau_cmd_[i] = kp_[i]*(joint_des_position_[i] - joint_filtered_position_[i]) - kd_[i]*joint_filtered_velocity_[i];
    }

    // write
    for(size_t i=0; i<joint_handles_.size(); i++) {
        joint_handles_[i].setCommand(tau_cmd_[i]);
    }
}


void ahand_controllers::PDController::gains_pd_callback(ahand_controllers::gains_pd_paramConfig& config, uint32_t level){
    kp_[0]  = config.p00; kp_[1]  = config.p01; kp_[2]  = config.p02; kp_[3]  = config.p03;
    kp_[4]  = config.p10; kp_[5]  = config.p11; kp_[6]  = config.p12; kp_[7]  = config.p13;
    kp_[8]  = config.p20; kp_[9]  = config.p21; kp_[10] = config.p22; kp_[11] = config.p23;
    kp_[12] = config.p30; kp_[13] = config.p31; kp_[14] = config.p32; kp_[15] = config.p33;

    kd_[0]  = config.d00; kd_[1]  = config.d01; kd_[2]  = config.d02; kd_[3]  = config.d03;
    kd_[4]  = config.d10; kd_[5]  = config.d11; kd_[6]  = config.d12; kd_[7]  = config.d13;
    kd_[8]  = config.d20; kd_[9]  = config.d21; kd_[10] = config.d22; kd_[11] = config.d23;
    kd_[12] = config.d30; kd_[13] = config.d31; kd_[14] = config.d32; kd_[15] = config.d33;

}


PLUGINLIB_EXPORT_CLASS(ahand_controllers::PDController, controller_interface::ControllerBase)

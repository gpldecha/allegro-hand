#include "ahand_controllers/controllers/bhand_controller.h"

BhandController::BhandController(ros::NodeHandle& nh): BaseController(nh), cddynamics(n_joints_, 0.003, 1){
    bhand = bhCreateRightHand();
    bhand->SetTimeInterval(0.003);
    bhand->SetMotionType(eMotionType_JOINT_PD);
    start_t = std::chrono::steady_clock::now();

    next_joint_positions.resize(n_joints_);

    velocity_limits.setOnes();
    velocity_limits = velocity_limits*2.5;
    cddynamics.SetVelocityLimits(velocity_limits);
    // position command topic
    sub_command_pose_ = nh.subscribe("bhand/position_command", 10, &BhandController::command_callback, this);
}

void BhandController::set_target_joint_position(Vector16d& _target_joint_positions){
    bhand->SetJointDesiredPosition(_target_joint_positions.data());
}

void BhandController::start(const hand_info& hand_info, const ros::Duration& period){
    cddynamics.SetState(hand_info.measured_joint_positions);
    c = 0;
    ROS_WARN("BhandController started!");
}

void BhandController::update(const hand_info& hand_info, Vector16d& torque_commands, const ros::Duration& period){

    Vector16d& measured_joint_positions = const_cast<Vector16d&>(hand_info.measured_joint_positions);

    cddynamics.Update();
    cddynamics.GetState(next_joint_positions);

    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_t).count();

    bhand->SetJointPosition(measured_joint_positions.data());
    bhand->SetJointDesiredPosition(next_joint_positions.data());
    bhand->UpdateControl(t);
    bhand->GetJointTorque(torque_commands.data());
    if(c < 100){
        torque_commands = 0.1*torque_commands;
        c++;
    }
}

void BhandController::command_callback(const ahand_controllers::bhand::ConstPtr& msg){
    velocity_limits.setOnes();
    velocity_limits = velocity_limits*(msg->velocity);
    cddynamics.SetVelocityLimits(velocity_limits);
    std::copy(msg->positions.begin(), msg->positions.end(), command_joint_position.data());
    cddynamics.SetTarget(command_joint_position);
}
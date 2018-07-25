#include "ahand_controllers/empty_controller.h"


ahand_controllers::EmptyController::EmptyController(){

}

bool ahand_controllers::EmptyController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle& nh){
    std::size_t n_joints_ = robot->getNames().size();
    ROS_INFO("Initisalising Empty");
    ROS_INFO_STREAM("PDController number of joints: "  << n_joints_);
    for(std::size_t i = 0; i < n_joints_; i++){
        joint_handles_.push_back(robot->getHandle("ahand_joint_" + std::to_string(i)));
    }
    return true;
}

void ahand_controllers::EmptyController::starting(const ros::Time& time){

}

void ahand_controllers::EmptyController::update(const ros::Time& time, const ros::Duration& period){
    for(auto joint_handle_ : joint_handles_){
        joint_handle_.setCommand(0);
    }
}

PLUGINLIB_EXPORT_CLASS(ahand_controllers::EmptyController, controller_interface::ControllerBase)

#include "ahand_controllers/ros_controller.h"

bool ahand_controllers::ROSController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle& nh){
    ROS_INFO("Initisalising ROSController");
    std::size_t n_joints_ = robot->getNames().size();
    for(std::size_t i = 0; i < n_joints_; i++){
        joint_handles_.push_back(robot->getHandle("ahand_joint_" + std::to_string(i)));
    }
    sub_cmd_ = nh.subscribe("/ahand/command", 1, &ahand_controllers::ROSController::command_callback, this);
    return true;
}

void ahand_controllers::ROSController::starting(const ros::Time& time){
    for(std::size_t i = 0; i < tau_cmd_.size(); i++){
        tau_cmd_[i] = 0.0;
    }
}

void ahand_controllers::ROSController::update(const ros::Time& time, const ros::Duration& period){
    for(std::size_t i = 0; i < tau_cmd_.size(); i++){
        joint_handles_[i].setCommand(tau_cmd_[i]);
    }
}

void ahand_controllers::ROSController::command_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if(msg->data.size()  != tau_cmd_.size()){
        ROS_ERROR_STREAM_THROTTLE(1.0, msg->data.size() << " != " << tau_cmd_.size());
        return;
    }
    for(std::size_t i = 0; i < tau_cmd_.size(); i++){
        tau_cmd_[i] = msg->data[i];
    }
}

PLUGINLIB_EXPORT_CLASS(ahand_controllers::ROSController, controller_interface::ControllerBase)
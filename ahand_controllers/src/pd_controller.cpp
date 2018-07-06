#include "ahand_controllers/pd_controller.h"


ahand_controllers::PDController::PDController(){}


bool ahand_controllers::PDController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh){

}

void ahand_controllers::PDController::starting(const ros::Time& time){

}

void ahand_controllers::PDController::update(const ros::Time& time, const ros::Duration& period){

}

void ahand_controllers::PDController::stopping(const ros::Time& time){

}


PLUGINLIB_EXPORT_CLASS(ahand_controllers::PDController, controller_interface::ControllerBase)

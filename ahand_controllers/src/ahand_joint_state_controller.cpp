#include "ahand_controllers/ahand_joint_state_controller.h"


bool ahand_controllers::AhandJointStateController::init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh){
    ROS_INFO("Initialising JointStateController");

    // get all joint names from the hardware interface
    const std::vector<std::string>& joint_names = hw->getNames();
    for (std::size_t i=0; i<joint_names.size(); i++)
        ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
        ROS_ERROR("Parameter 'publish_rate' not set");
        return false;
    }

    // realtime publisher
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));

    // get joints and allocate message
    for (std::size_t i=0; i<joint_names.size(); i++){
        joint_state_.push_back(hw->getHandle(joint_names[i]));
        realtime_pub_->msg_.name.push_back(joint_names[i]);
        realtime_pub_->msg_.position.push_back(0.0);
        realtime_pub_->msg_.velocity.push_back(0.0);
        realtime_pub_->msg_.effort.push_back(0.0);
    }
    ROS_INFO("JointStateController is initialised");
    return true;
}

void ahand_controllers::AhandJointStateController::starting(const ros::Time& time){
    // initialize time
    last_publish_time_ = time;
}

void ahand_controllers::AhandJointStateController::update(const ros::Time& time, const ros::Duration& period){
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time) {
        // try to publish
        if (realtime_pub_->trylock()) {
            // we're actually publishing, so increment time
            last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

            // populate joint state message
            realtime_pub_->msg_.header.stamp = time;
            for (unsigned i=0; i<joint_state_.size(); i++){
                realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
                realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
                realtime_pub_->msg_.effort[i] = joint_state_[i].getEffort();
            }
            realtime_pub_->unlockAndPublish();
        }
    }
}

void ahand_controllers::AhandJointStateController::stopping(const ros::Time&){}



PLUGINLIB_EXPORT_CLASS(ahand_controllers::AhandJointStateController,  controller_interface::ControllerBase)

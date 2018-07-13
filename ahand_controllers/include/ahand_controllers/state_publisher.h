#ifndef AHAND_CONTROLLERS__JOINT_STATE_CONTROLLER_H
#define AHAND_CONTROLLERS__JOINT_STATE_CONTROLLER_H

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>

#include <hardware_interface/joint_state_interface.h>

#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/JointState.h>

#include <realtime_tools/realtime_publisher.h>


namespace ahand_controllers{

class StatePublisher : public controller_interface::Controller<hardware_interface::JointStateInterface>{

    public:

        StatePublisher() : publish_rate_(0.0) {}

        virtual bool init(hardware_interface::JointStateInterface* hw,
                          ros::NodeHandle&                         root_nh,
                          ros::NodeHandle&                         controller_nh);

        virtual void starting(const ros::Time& time);

        virtual void update(const ros::Time& time, const ros::Duration& period);

        virtual void stopping(const ros::Time& time);

    private:

        std::vector<hardware_interface::JointStateHandle> joint_state_;

        // Publishers
        boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;

        ros::Time last_publish_time_;
        double publish_rate_;
        std::size_t num_hw_joints_;

};

}

#endif

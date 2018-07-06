#ifndef AHAND_CONTROLLERS__PD_CONTROLLER_H
#define AHAND_CONTROLLERS__PD_CONTROLLER_H

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>

#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>


namespace ahand_controllers{

class PDController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{

    public:

        PDController();

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);

        void stopping(const ros::Time& time);

    private:

         ros::Subscriber sub_command_pose_;

};

}

#endif

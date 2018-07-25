//
// Created by guillaume on 25/07/18.
//

#ifndef EMPTY_CONTROLLER_H
#define EMPTY_CONTROLLER_H

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>

#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

namespace ahand_controllers {

    class EmptyController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

    public:

        EmptyController();

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle& nh) override;

        void starting(const ros::Time& time) override;

        void update(const ros::Time& time, const ros::Duration& period) override;

    private:

        std::vector<hardware_interface::EffortJointInterface::ResourceHandleType> joint_handles_;

    };
}

#endif //PROJECT_EMPTY_CONTROLLER_H

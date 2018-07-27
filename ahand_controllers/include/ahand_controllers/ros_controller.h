#ifndef AHAND_CONTROLLERS__ROS_CONTROLLER_H
#define AHAND_CONTROLLERS__ROS_CONTROLLER_H

#include <array>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>


namespace ahand_controllers {

    class ROSController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        public:

            ROSController() = default;

            bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle& nh) override;

            void starting(const ros::Time& time) override;

            void update(const ros::Time& time, const ros::Duration& period) override;

        private:

            void command_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

        private:

            std::vector<hardware_interface::EffortJointInterface::ResourceHandleType> joint_handles_;
            ros::Subscriber sub_cmd_;
            std::array<double, 16> tau_cmd_;

    };
}

#endif //AHAND_CONTROLLERS__ROS_CONTROLLER_H

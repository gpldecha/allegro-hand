#ifndef AHAND_CONTROLLERS__ROS_CONTROLLER_H
#define AHAND_CONTROLLERS__ROS_CONTROLLER_H

// STL
#include <array>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ahand_controllers/Command.h>

// package

#include "controllers/ahand_types.h"
#include "controllers/joint_impedance_controller.h"
#include "controllers/bhand_controller.h"


namespace ahand_controllers {

    class ROSController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

        public:

            ROSController() = default;

            bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle& nh) override;

            void starting(const ros::Time& time) override;

            void update(const ros::Time& time, const ros::Duration& period) override;

        private:

            void command_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

            bool service_command_callback(Command::Request& request, Command::Response& response);

    private:

            std::vector<hardware_interface::EffortJointInterface::ResourceHandleType> joint_handles_;
            CONTROL_TYPE control_type, control_type_prev;
            hand_info measurements;

            std::shared_ptr<JointImpedanceController> joint_impedance_controller;
            std::shared_ptr<BhandController> bhand_controller;

            Vector16d torque_commands;
            bool bPrintTorques;

            ros::ServiceServer service_server_;

            ros::Subscriber sub_cmd_;

    };
}

#endif //AHAND_CONTROLLERS__ROS_CONTROLLER_H

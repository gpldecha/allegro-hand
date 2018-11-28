#ifndef AHAND_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_H
#define AHAND_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_H

// ROS

#include <ahand_controllers/Command.h>
#include <dynamic_reconfigure/server.h>
#include <ahand_controllers/gains_pd_paramConfig.h>
#include <std_msgs/Float32MultiArray.h>

// STL

#include <memory>
#include <array>


// package

#include "controllers/base_controller.h"
#include <ahand_controllers/Command.h>


class JointImpedanceController : public BaseController {

    public:

        JointImpedanceController(ros::NodeHandle& nh);

        void start(const hand_info& hand_info, const ros::Duration& period) override;

        void update(const hand_info& hand_info, Vector16d& torque_commands, const ros::Duration& period) override;

    private:

        void gains_pd_callback(ahand_controllers::gains_pd_paramConfig& config, uint32_t level);

        void command_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    private:

         Vector16d joint_des_position_;
         Vector16d joint_filtered_position_, joint_filtered_velocity_;

         std::vector<double> kp_, kd_;

         std::unique_ptr< dynamic_reconfigure::Server<ahand_controllers::gains_pd_paramConfig>> dynamic_server_gains_dp_param_;

         ros::Subscriber sub_command_pose_;
         ros::ServiceServer service_server_;
         ros::NodeHandle nh_gains_pd_;

};


#endif

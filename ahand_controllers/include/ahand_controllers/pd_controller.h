#ifndef AHAND_CONTROLLERS__PD_CONTROLLER_H
#define AHAND_CONTROLLERS__PD_CONTROLLER_H

// ROS

#include <ahand_controllers/Command.h>
#include <dynamic_reconfigure/server.h>
#include <ahand_controllers/gains_pd_paramConfig.h>

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>

#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>


// STL

#include <memory>
#include <array>

namespace ahand_controllers{

class PDController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{

    public:

        PDController();

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle& nh);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);

    private:

        void gains_pd_callback(ahand_controllers::gains_pd_paramConfig& config, uint32_t level);

        bool command_callback(Command::Request& request, Command::Response& response);

    private:

         std::size_t n_joints_;

         std::vector<double> tau_cmd_;
         std::vector<double> joint_msr_position_, joint_msr_velocity_;
         std::vector<double> joint_des_position_;
         std::vector<double> joint_filtered_position_, joint_filtered_velocity_;

         std::vector<double> kp_, kd_;

         std::vector<hardware_interface::EffortJointInterface::ResourceHandleType> joint_handles_;

         std::unique_ptr< dynamic_reconfigure::Server<ahand_controllers::gains_pd_paramConfig>> dynamic_server_gains_dp_param_;

         ros::Subscriber sub_command_pose_;
         ros::ServiceServer service_server_;
         ros::NodeHandle nh_gains_pd_;



};

}

#endif

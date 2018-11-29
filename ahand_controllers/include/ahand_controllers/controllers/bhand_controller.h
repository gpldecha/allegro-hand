//
// Created by guillaume on 23/11/18.
//

#ifndef BHAND_CONTROLLER_H
#define BHAND_CONTROLLER_H

#include "bhand/BHand.h"
#include "controllers/base_controller.h"
#include "motion/CDDynamics.h"


// ROS

#include <ros/ros.h>
#include <chrono>
#include <ahand_controllers/bhand.h>


class BhandController : public BaseController {

public:

    BhandController(ros::NodeHandle& nh);

    void set_target_joint_position(Vector16d& target_joint_positions);

    void start(const hand_info& hand_info, const ros::Duration& period) override;

    void update(const hand_info& hand_info, Vector16d& torque_commands, const ros::Duration& period) override;

private:

    void command_callback(const ahand_controllers::bhand::ConstPtr& msg);

private:

    BHand* bhand;
    CDDynamics cddynamics;
    ros::Subscriber sub_command_pose_;
    std::chrono::steady_clock::time_point start_t;
    Vector16d velocity_limits;
    Vector16d command_joint_position;
    Eigen::VectorXd next_joint_positions;
    std::size_t c;

};

#endif
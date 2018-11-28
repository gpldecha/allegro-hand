//
// Created by guillaume on 26/11/18.
//

#ifndef AHAND_BASE_CONTROLLER_H
#define AHAND_BASE_CONTROLLER_H

#include <ros/ros.h>

#include "controllers/ahand_types.h"

class BaseController{

public:

    BaseController(ros::NodeHandle& nh):n_joints_(16){}

    virtual void start(const hand_info& hand_info, const ros::Duration& period) = 0;

    virtual void update(const hand_info& hand_info, Vector16d& torque_commands, const ros::Duration& period) = 0;

    std::size_t n_joints_;

};

#endif //AHAND_BASE_CONTROLLER_H

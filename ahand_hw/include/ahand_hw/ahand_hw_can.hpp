#ifndef AHAND_HW_CAN_H
#define AHAND_HW_CAN_H

// ROS

#include <angles/angles.h>


#include "ahand_hw/ahand_hw.h"
#include "ahand_driver/ahandDriver.h"
#include "ahand_hw/ahand_filters.h"


class AhandHWCAN : public AhandHW {

public:

    AhandHWCAN() : AhandHW() {
        for(std::size_t j=0; j < n_joints_; j++){
            raw_positions_[j] = 0.0;
            raw_prev_positions_[j] = 0.0;
            median_filter_[j] = new filters::Median(5);
            sg_filters_[j] = new filters::SavitzkyGolay(15, 2);
        }
    }

    void stop(){
        ahandDriver_->stop();
        delete ahandDriver_;
        ahandDriver_ = NULL;
    }

    bool init() override {
        ahandDriver_ = new AhandDriver();
        return ahandDriver_->isIntialised();
    }

    inline bool is_joint_within_limits(std::size_t joint_idx){
        return (raw_positions_[joint_idx] <= (joint_upper_limits_[joint_idx]+angle_error)) && (raw_positions_[joint_idx] >= (joint_lower_limits_[joint_idx]-angle_error));
    }

    inline bool is_impulse_noise(const double& joint_velocity){
        return joint_velocity > max_radial_velocity;
    }

    void read(ros::Time time, ros::Duration period) override {
        ahandDriver_->getJointInfo(raw_positions_);
        std::size_t finger_idx;
        std::size_t joint_idx;
        double joint_velocity=0;

        for(std::size_t j=0; j < n_joints_; j++){
            finger_idx = j/n_fingers_;
            joint_idx = j%n_fingers_;

            raw_positions_[j] = median_filter_[j]->get(raw_positions_[j]);
            joint_velocity = (raw_positions_[j] - raw_prev_positions_[j])/period.toSec();

            if(is_joint_within_limits(j) && !is_impulse_noise(joint_velocity)){
                joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], raw_positions_[j]);
                sg_filters_[j]->update(joint_position_[j]);
                joint_position_[j] = sg_filters_[j]->position;
                joint_velocity_[j] = sg_filters_[j]->velocity/period.toSec();
                joint_position_kdl_[finger_idx](joint_idx) = joint_position_[j];
            }
            raw_prev_positions_[j] = raw_positions_[j];
            joint_effort_[j]   = 0.0;
        }
    }

    void write(ros::Time time, ros::Duration period) override {
        for(std::size_t finger_idx = 0; finger_idx < n_fingers_; finger_idx++){
            f_dyn_solvers_[finger_idx]->JntToGravity(joint_position_kdl_[finger_idx], gravity_effort_[finger_idx]);
        }
        std::size_t finger_idx;
        std::size_t joint_idx;
        for(std::size_t j = 0; j < n_joints_;j++){
            finger_idx = j/n_fingers_;
            joint_idx = j%n_fingers_;
            joint_effort_command_[j] += grav_fudge[joint_idx] * gravity_effort_[finger_idx](joint_idx);
        }
        ahandDriver_->setTorque(&joint_effort_command_[0]);
    }

private:

    AhandDriver* ahandDriver_;
    filters::Median* median_filter_[n_joints_];
    filters::SavitzkyGolay* sg_filters_[n_joints_];

    double raw_positions_[n_joints_];
    double raw_prev_positions_[n_joints_];
    const double angle_error = 8.0*M_PI/180;
    const double max_radial_velocity = 4.0;
    const double grav_fudge[4] = {2.0, 2.0, 2.0, 0.0};

};


#endif

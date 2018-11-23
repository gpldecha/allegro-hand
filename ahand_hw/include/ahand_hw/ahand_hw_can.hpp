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
        raw_positions_.setZero();
        raw_prev_positions_.setZero();
        for(std::size_t j=0; j < n_joints_; j++){
            median_filter_[j] = new filters::Median(5);
            sg_filters_[j] = new filters::SavitzkyGolay(5, 2);
        }
        last = ros::Time(0, 0);
        duration_vel = ros::Duration(1.0);
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
        return (raw_positions_(joint_idx) <= (joint_upper_limits_[joint_idx]+angle_error)) && (raw_positions_(joint_idx) >= (joint_lower_limits_[joint_idx]-angle_error));
    }

    inline bool is_impulse_noise(const double& joint_velocity){
        return joint_velocity > max_radial_velocity;
    }

    void read(ros::Time time, ros::Duration period) override {
        ahandDriver_->getJointInfo(raw_positions_);
        std::size_t finger_idx;
        std::size_t joint_idx;
        double joint_velocity=0;

        if(count == 10) {
            duration_vel = time - last;
            last = time;
        }

        for(std::size_t j=0; j < n_joints_; j++){
            finger_idx = j/n_fingers_;
            joint_idx = j%n_fingers_;

            raw_positions_[j] = median_filter_[j]->get(raw_positions_[j]);
            joint_velocity = (raw_positions_[j] - raw_prev_positions_[j])/period.toSec();

            if(is_joint_within_limits(j) && !is_impulse_noise(joint_velocity)){
                measured_joint_position_[j] += angles::shortest_angular_distance(measured_joint_position_[j], raw_positions_[j]);

                estimated_joint_velocity_[j] = filters::exponentialSmoothing((measured_joint_position_[j] - measured_joint_position_prev[j])/period.toSec(), estimated_joint_velocity_[j], 0.005);
                //sg_filters_[j]->update(measured_joint_position_[j]);
                //estimated_joint_velocity_[j] = sg_filters_[j]->velocity/(duration_vel.toSec()+0.1);

                measured_joint_position_prev[j] = measured_joint_position_[j];

                //joint_position_[j] = sg_filters_[j]->position;
                /*if(count == 10){
                    measured_joint_position_prev[j] = measured_joint_position_[j];
                }*/

                joint_position_kdl_[finger_idx](joint_idx) = measured_joint_position_[j];
            }
            raw_prev_positions_[j] = raw_positions_[j];
        }

        if(count==10){count=0;}
        count++;
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
        ahandDriver_->setTorque(joint_effort_command_);
        joint_effort_command_.setZero();
    }

private:

    AhandDriver* ahandDriver_;
    filters::Median* median_filter_[n_joints_];
    filters::SavitzkyGolay* sg_filters_[n_joints_];


    Vector16d raw_positions_;
    Vector16d raw_prev_positions_;
    const double angle_error = 8.0*M_PI/180;
    const double max_radial_velocity = 4.0;
    const double grav_fudge[4] = {2.0, 2.0, 2.0, 0.0};
    int count=0;
    ros::Time last;
    ros::Duration duration_vel;

};


#endif

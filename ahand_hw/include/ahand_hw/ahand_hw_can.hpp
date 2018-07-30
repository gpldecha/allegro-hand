#ifndef AHAND_HW_CAN_H
#define AHAND_HW_CAN_H

// ROS

#include <angles/angles.h>


#include "ahand_hw/ahand_hw.h"
#include "ahand_driver/ahandDriver.h"


class AhandHWCAN : public AhandHW {

public:

    AhandHWCAN() : AhandHW() {
        wait_valid_state_ = true;
    }

    void stop(){
        ahandDriver_->stop();
        delete ahandDriver_;
        ahandDriver_ = NULL;
        wait_valid_state_=true;
    }

    bool init() override {
        ahandDriver_ = new AhandDriver();
        return ahandDriver_->isIntialised();
    }

    void read(ros::Time time, ros::Duration period) override {
        ahandDriver_->getJointInfo(positions_);
        std::size_t finger_idx;
        std::size_t joint_idx;
        for(std::size_t j=0; j < n_joints_; j++){
            finger_idx = j/n_fingers_;
            joint_idx = j%n_fingers_;
            joint_position_prev_[j] = joint_position_[j];
            if( wait_valid_state_ || (positions_[j] <= (joint_upper_limits_[j]+angle_error) && positions_[j] >= (joint_lower_limits_[j]-angle_error))){
                joint_position_[j] = positions_[j];//+= angles::shortest_angular_distance(joint_position_[j], positions_[j]);
                joint_position_kdl_[finger_idx](joint_idx) = joint_position_[j];
                if(wait_valid_state_){count++;}
            }
            joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
            joint_effort_[j]   = 0.0;
        }
        if(count > 10){
            wait_valid_state_=false;
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
            joint_effort_command_[j] += 2500.0*gravity_effort_[finger_idx](joint_idx);
        }

        if(!wait_valid_state_){
            ahandDriver_->setTorque(&joint_effort_command_[0]);
        }else{
            ROS_WARN_STREAM_THROTTLE(1.0, "state is invalid");
        }
    }

private:

    AhandDriver* ahandDriver_;
    double positions_[n_joints_];
    const double angle_error = 8.0*M_PI/180;
    bool wait_valid_state_;
    int count=0;

};


#endif

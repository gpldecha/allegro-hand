#ifndef AHAND_HW_GAZEBO_H
#define AHAND_HW_GAZEBO_H

// ROS
#include <angles/angles.h>

// Gazebo hook
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include "ahand_hw/ahand_hw.h"

class AHandHWGazebo : public AhandHW{

    public:

        AHandHWGazebo(){}

        void setParentModel(gazebo::physics::ModelPtr parent_model){parent_model_ = parent_model; parent_set_ = true;}

        // Init, read, and write, with Gazebo hooks
        bool init() override {
            ROS_INFO("init gazebo_plugin");

            if(!(parent_set_)){
                std::cout << "Did you forget to set the parent model?" << std::endl << "You must do that before init()" << std::endl << "Exiting..." << std::endl;
                return false;
            }

          gazebo::physics::JointPtr joint;
          for(int j=0; j < n_joints_; j++){
            joint = parent_model_->GetJoint(joint_names_[j]);
            if (!joint){
              std::cout << "This robot has a joint named \"" << joint_names_[j] << "\" which is not in the gazebo model." << std::endl;
              return false;
            }
            sim_joints_.push_back(joint);
          }
          return true;
        }

        void read(ros::Time time, ros::Duration period) override {
            std::size_t finger_idx;
            std::size_t joint_idx;
            for(std::size_t j=0; j < n_joints_; ++j){
                finger_idx = j/n_fingers_;
                joint_idx = j%n_fingers_;
                joint_position_prev_[j] = joint_position_[j];
                joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], sim_joints_[j]->Position(0));
                joint_position_kdl_[finger_idx](joint_idx) = joint_position_[j];
                joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
                joint_effort_[j]  = sim_joints_[j]->GetForce((unsigned int)(0));
            }
        }

        void write(ros::Time time, ros::Duration period) override {
            for(std::size_t finger_idx = 0; finger_idx < n_fingers_; finger_idx++){
                f_dyn_solvers_[finger_idx]->JntToGravity(joint_position_kdl_[finger_idx], gravity_effort_[finger_idx]);
            }
            std::size_t finger_idx;
            std::size_t joint_idx;
            for(std::size_t j = 0; j < n_joints_; j++){
                finger_idx = j/n_fingers_;
                joint_idx = j%n_fingers_;
                sim_joints_[j]->SetForce(0, joint_effort_command_[j] + gravity_fudge_factor*gravity_effort_[finger_idx](joint_idx));
             }
        }


private:

  // Gazebo stuff
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::ModelPtr parent_model_;
  bool parent_set_ = false;
  static constexpr double gravity_fudge_factor = 2.0;


};


#endif

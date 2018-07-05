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
        bool init(){
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

        void read(ros::Time time, ros::Duration period){
            for(int j=0; j < n_joints_; ++j){
                joint_position_prev_[j] = joint_position_[j];
                joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], sim_joints_[j]->GetAngle(0).Radian());
                //joint_position_kdl_(j) = joint_position_[j];
                // derivate velocity as in the real hardware instead of reading it from simulation
                joint_velocity_[j]    = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
                joint_effort_[j]      = sim_joints_[j]->GetForce((int)(0));
                //joint_stiffness_[j]   = joint_stiffness_command_[j];
                //joint_damping_[j]     = joint_damping_command_[j];
            }
        }

        void write(ros::Time time, ros::Duration period){

        }




private:

  // Gazebo stuff
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::ModelPtr parent_model_;
  bool parent_set_ = false;


};


#endif

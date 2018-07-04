#include "ahand_hw/ahand_hw.h"


AhandHW::AhandHW(){

}

void AhandHW::create(std::string name, std::string urdf_string){

    std::cout<< "Allegro Hand called: " << name << std::endl;

    robot_namespace_ = name;
    urdf_string_ = urdf_string;

    joint_names_.reserve(n_joints_);
    joint_names_.push_back(  std::string("f1_dof0_joint") );
    joint_names_.push_back(  std::string("f1_dof1_joint") );
    joint_names_.push_back(  std::string("f1_dof2_joint") );
    joint_names_.push_back(  std::string("f1_dof3_joint") );

    joint_names_.push_back(  std::string("f2_dof0_joint") );
    joint_names_.push_back(  std::string("f2_dof1_joint") );
    joint_names_.push_back(  std::string("f2_dof2_joint") );
    joint_names_.push_back(  std::string("f2_dof3_joint") );

    joint_names_.push_back(  std::string("f3_dof0_joint") );
    joint_names_.push_back(  std::string("f3_dof1_joint") );
    joint_names_.push_back(  std::string("f3_dof2_joint") );
    joint_names_.push_back(  std::string("f3_dof3_joint") );

    joint_names_.push_back(  std::string("thumb_dof0_joint") );
    joint_names_.push_back(  std::string("thumb_dof1_joint") );
    joint_names_.push_back(  std::string("thumb_dof2_joint") );
    joint_names_.push_back(  std::string("thumb_dof3_joint") );

    joint_position_.resize(n_joints_);
    joint_velocity_.resize(n_joints_);
    joint_effort_.resize(n_joints_);
    joint_effort_command_.resize(n_joints_);

    reset();

    // GET TRANSMISSIONS THAT BELONG TO ALLEGRO HAND
    if (!parseTransmissionsFromURDF(urdf_string_)){
        std::cout << "ahand_hw: " << "Error parsing URDF in allegro_hand_hw.\n" << std::endl;
        return;
    }

    std::cout<< "Register interface" << std::endl;

    const urdf::Model *const urdf_model_ptr = urdf_model_.initString(urdf_string_) ? &urdf_model_ : NULL;

    registerInterfaces(urdf_model_ptr, transmissions_);
}

void AhandHW::reset(){
    for (std::size_t j = 0; j < n_joints_; ++j){
      joint_position_[j] = 0.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 0.0;

      joint_effort_command_[j] = 0.0;
    }
}


void AhandHW::registerInterfaces(const urdf::Model *const urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions){
    std::cout<< "transmissions.size(): " << transmissions.size() << std::endl;
    for(std::size_t j = 0; j < n_joints_; j++){
        // Check that this transmission has one joint
        if(transmissions[j].joints_.size() == 0){
          std::cout << "ahand_hw: " << "Transmission " << transmissions[j].name_
                    << " has no associated joints." << std::endl;
          continue;
        }else if(transmissions[j].joints_.size() > 1){
          std::cout << "ahand_hw: " << "Transmission " << transmissions[j].name_
            << " has more than one joint, and they can't be controlled simultaneously"
            << std::endl;
          continue;
        }
        std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
        if( joint_interfaces.empty() ){
            std::cout << "ahand_hw: " << "Joint " << transmissions[j].joints_[0].name_ <<
              " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
              "You need to, otherwise the joint can't be controlled." << std::endl;
            continue;
        }

        const std::string& hardware_interface = joint_interfaces.front();
        // Debug
        std::cout << "\x1B[37m" << "ahand_hw: " << "Loading joint '" << joint_names_[j]
          << "' of type '" << hardware_interface << "'" << "\x1B[0m" << std::endl;

        // Create joint state interface for all joints
        state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

        // Decide what kind of command interface this actuator/joint has
        hardware_interface::JointHandle joint_handle_effort;
        joint_handle_effort = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]), &joint_effort_command_[j]);
        effort_interface_.registerHandle(joint_handle_effort);
    }
}


bool AhandHW::parseTransmissionsFromURDF(const std::string& urdf_string){
    std::vector<transmission_interface::TransmissionInfo> transmissions;
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions);
    // Now iterate and save only transmission from this robot
    for (int j = 0; j < n_joints_; ++j){
        //std::cout << "Check joint " << joint_names_[j] << std::endl;
        std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions.begin();
        for(; it != transmissions.end(); ++it){
           // std::cout << "With transmission " << it->name_ << std::endl;
           if (joint_names_[j].compare(it->joints_[0].name_) == 0){
            transmissions_.push_back( *it );
            std::cout << "Found a match for transmission " << it->name_ << std::endl;
           }
        }
     }

     if( transmissions_.empty() ){
         std::cout << "ahand_hw: " << "There are no transmission in this robot, all are non-driven joints? " << std::endl;
         return false;
     }else{
        return true;
     }
}

bool AhandHW::initKDLdescription(const urdf::Model *const urdf_model){

}

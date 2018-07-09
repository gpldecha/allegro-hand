#include "ahand_hw/ahand_hw.h"


AhandHW::AhandHW(){

}

void AhandHW::create(std::string name, std::string urdf_string){

    std::cout<< "Allegro Hand called: " << name << std::endl;

    robot_namespace_ = name;
    urdf_string_ = urdf_string;

    joint_names_.reserve(n_joints_);
    joint_names_.push_back( std::string("_joint_0") );
    joint_names_.push_back( std::string("_joint_1") );
    joint_names_.push_back( std::string("_joint_2") );
    joint_names_.push_back( std::string("_joint_3") );

    joint_names_.push_back( std::string("_joint_4") );
    joint_names_.push_back( std::string("_joint_5") );
    joint_names_.push_back( std::string("_joint_6") );
    joint_names_.push_back( std::string("_joint_7") );

    joint_names_.push_back(  std::string("_joint_8") );
    joint_names_.push_back(  std::string("_joint_9") );
    joint_names_.push_back(  std::string("_joint_10") );
    joint_names_.push_back(  std::string("_joint_11") );

    joint_names_.push_back(  std::string("_joint_12") );
    joint_names_.push_back(  std::string("_joint_13") );
    joint_names_.push_back(  std::string("_joint_14") );
    joint_names_.push_back(  std::string("_joint_15") );
    for(std::size_t i = 0; i < joint_names_.size(); i++){
        joint_names_[i] = robot_namespace_ + joint_names_[i];
    }

    joint_position_.resize(n_joints_);
    joint_position_prev_.resize(n_joints_);
    joint_velocity_.resize(n_joints_);
    joint_effort_.resize(n_joints_);
    joint_effort_command_.resize(n_joints_);

    reset();

    // GET TRANSMISSIONS THAT BELONG TO ALLEGRO HAND
    if (!parseTransmissionsFromURDF(urdf_string_)){
        std::cout << "ahand_hw: " << "Error parsing URDF in ahand_hw.\n" << std::endl;
        return;
    }

    std::cout<< "Register interface" << std::endl;

    const urdf::Model *const urdf_model_ptr = urdf_model_.initString(urdf_string_) ? &urdf_model_ : NULL;

    registerInterfaces(urdf_model_ptr, transmissions_);
}

void AhandHW::reset(){
    for (std::size_t j = 0; j < n_joints_; ++j){
      joint_position_[j] = 0.0;
      joint_position_prev_[j] = 0.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 0.0;
      joint_effort_command_[j] = 0.0;
    }
}

bool AhandHW::canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const{
    ROS_INFO("can switch");
    for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it ){
        std::cout<< "name: " << it->name << std::endl;

    }
    return true;
}

void AhandHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list){
    ROS_INFO("doSwitch");
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

        // Debug
        std::cout << "\x1B[37m" << "ahand_hw: " << "Loading joint '" << joint_names_[j] << "' of type '" << joint_interfaces.front() << "'" << "\x1B[0m" << std::endl;

        state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

        hardware_interface::JointHandle joint_handle_effort;
        joint_handle_effort = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),&joint_effort_command_[j]);
        effort_interface_.registerHandle(joint_handle_effort);
    }

    // Register interfaces
    registerInterface(&state_interface_);
    registerInterface(&effort_interface_);

}


bool AhandHW::parseTransmissionsFromURDF(const std::string& urdf_string){
    std::vector<transmission_interface::TransmissionInfo> transmissions;
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions);
    // Now iterate and save only transmission from this robot
    for (int j = 0; j < n_joints_; ++j){
        //std::cout << "Check joint " << joint_names_[j] << std::endl;
        std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions.begin();

        for(; it != transmissions.end(); ++it){
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

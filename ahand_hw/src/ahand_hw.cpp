#include "ahand_hw/ahand_hw.h"


AhandHW::AhandHW(){}

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

    joint_lower_limits_.resize(n_joints_);
    joint_upper_limits_.resize(n_joints_);


    reset();

    // GET TRANSMISSIONS THAT BELONG TO ALLEGRO HAND
    if (!parseTransmissionsFromURDF(urdf_string_)){
        std::cout << "ahand_hw: " << "Error parsing URDF in ahand_hw.\n" << std::endl;
        return;
    }

    std::cout<< "Register interface" << std::endl;

    const urdf::Model *const urdf_model_ptr = urdf_model_.initString(urdf_string_) ? &urdf_model_ : nullptr;

    registerInterfaces(urdf_model_ptr, transmissions_);

    std::cout<< "Initialise KDL variables ..." << std::endl;
    initKDLdescription(urdf_model_ptr);
    std::cout<< "Successfully create abstract Allegro Hand with interface to ROS control" << std::endl;
}

void AhandHW::reset(){
    measured_joint_position_.setZero();
    measured_joint_position_prev.setZero();
    estimated_joint_velocity_.setZero();
    measured_joint_effort_.setZero();

    joint_effort_command_.setZero();
}

bool AhandHW::canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const{
    ROS_INFO("can switch");
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

        state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[j], &measured_joint_position_[j], &estimated_joint_velocity_[j], &measured_joint_effort_[j]));

        hardware_interface::JointHandle joint_handle_effort;
        joint_handle_effort = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),&joint_effort_command_[j]);
        effort_interface_.registerHandle(joint_handle_effort);

        registerJointLimits(joint_names_[j], &urdf_model_, &joint_lower_limits_[j], &joint_upper_limits_[j]);
    }
    // Register interfaces
    registerInterface(&state_interface_);
    registerInterface(&effort_interface_);
}

void AhandHW::registerJointLimits(const std::string& joint_name,
                                  const urdf::Model *const urdf_model,
                                  double *const lower_limit,
                                  double *const upper_limit){

    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    joint_limits_interface::JointLimits limits;
    bool has_limits = false;

    if (urdf_model != NULL){
        const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
        if (urdf_joint != NULL){
            if (joint_limits_interface::getJointLimits(urdf_joint, limits)){
                has_limits = true;
            }else{
                std::cout<< "does not have joint limit" << std::endl;
            }
        }else{
            std::cerr<< "urdf_joint is NULL" << std::endl;
        }

    }else{
        std::cerr<< "urdf_model is NULL" << std::endl;
    }
    if (!has_limits){
        std::cout<< "joint " << joint_name << " does not have a limit" << std::endl;
       return;
    }

    if(limits.has_position_limits){
        std::cout<< "\t limite (" << limits.min_position << " " << limits.max_position << ")" << std::endl;
        *lower_limit = limits.min_position;
        *upper_limit = limits.max_position;
     }else{
        std::cout<< "joint " << joint_name << " does not have a position limit" << std::endl;
    }
}

bool AhandHW::parseTransmissionsFromURDF(const std::string& urdf_string){
    std::vector<transmission_interface::TransmissionInfo> transmissions;
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions);
    // Now iterate and save only transmission from this robot
    for (int j = 0; j < n_joints_; ++j){
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
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    std::cout << "Ahand kinematic successfully parsed with "
              << kdl_tree.getNrOfJoints()
              << " joints, and "
              << kdl_tree.getNrOfJoints()
              << " segments." << std::endl;
    std::cout<< "get_param: " <<std::string("/") + robot_namespace_ + std::string("/root_name") << std::endl;

    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    std::string root_name = robot_namespace_ + "_palm_link";
    std::string tip_name;
    std::cout<< "creating kdl chain " << std::endl;
    for(std::size_t i = 0; i < n_fingers_; i++){
        tip_name = robot_namespace_ + "_link_" + std::to_string(i*4+3) + "_tip";
        std::cout<< "  " << root_name << " ----> " << tip_name << std::endl;
        if(!kdl_tree.getChain(root_name, tip_name, ahand_chains_[i])) {
            ROS_ERROR("Failed to get KDL chain from tree: ");
            return false;
        }
        f_dyn_solvers_[i] = std::make_unique<KDL::ChainDynParam>(KDL::ChainDynParam(ahand_chains_[i], gravity_));
        joint_position_kdl_[i] = KDL::JntArray(ahand_chains_[i].getNrOfJoints());
        gravity_effort_[i] = KDL::JntArray(ahand_chains_[i].getNrOfJoints());

        joint_position_kdl_[i].data.setZero();
        gravity_effort_[i].data.setZero();
    }

}

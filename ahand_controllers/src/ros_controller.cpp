#include "ahand_controllers/ros_controller.h"

bool ahand_controllers::ROSController:: init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle& nh){
    ROS_INFO("Initisalising ROSController");
    std::size_t n_joints_ = robot->getNames().size();
    for(std::size_t i = 0; i < n_joints_; i++){
        joint_handles_.push_back(robot->getHandle("ahand_joint_" + std::to_string(i)));
    }

    sub_cmd_ = nh.subscribe("torque_command", 1, &ahand_controllers::ROSController::command_callback, this);
    service_server_ = nh.advertiseService("service_command", &ahand_controllers::ROSController::service_command_callback, this);

    control_type = NONE;
    control_type_prev = NONE;

    joint_impedance_controller.reset(new JointImpedanceController(nh));
    bhand_controller.reset(new BhandController(nh));

    bPrintTorques = true;

    return true;
}

void ahand_controllers::ROSController::starting(const ros::Time& time){
    torque_commands.setZero();
}

void ahand_controllers::ROSController::update(const ros::Time& time, const ros::Duration& period){
    safety.check_time();
    for(std::size_t i = 0; i < joint_handles_.size(); i++){
        measurements.measured_joint_positions[i] = joint_handles_[i].getPosition();
        measurements.estimated_joint_velocities[i] = joint_handles_[i].getVelocity();
    }

    if(control_type == JOINT_IMPEDANCE){
        if(control_type != control_type_prev){
            joint_impedance_controller->start(measurements, period);
            control_type_prev = control_type;
        }
        joint_impedance_controller->update(measurements, torque_commands, period);
    }else if(control_type == BHAND){
        if(control_type != control_type_prev) {
            bhand_controller->start(measurements, period);
            control_type_prev = control_type;
        }else {
            bhand_controller->update(measurements, torque_commands, period);
        }
    }else if(control_type == NONE) {
        if (control_type != control_type_prev) {
            control_type_prev = control_type;
            torque_commands.setZero();
        }
    }

    for(std::size_t i = 0; i < torque_commands.size(); i++){
        //safety.check_torque(torque_commands[i]);
        joint_handles_[i].setCommand(torque_commands[i]);
    }

    if(bPrintTorques) {
        ROS_INFO_STREAM_THROTTLE(1.0, "torques \nf1: "
        << torque_commands[0] << " " << torque_commands[1] << " " << torque_commands[2] << " " << torque_commands[3] <<
        "\nf2: " << torque_commands[4] << " " << torque_commands[5] << " " << torque_commands[6] << " " << torque_commands[7] <<
        "\nf3: " << torque_commands[8] << " " << torque_commands[9] << " " << torque_commands[10] << " " << torque_commands[11] <<
        "\nth: " << torque_commands[12] << " " << torque_commands[13] << " " << torque_commands[14] << " " << torque_commands[15]);
    }
}

void ahand_controllers::ROSController::command_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if(msg->data.size()  != torque_commands.size()){
        ROS_ERROR_STREAM_THROTTLE(1.0, msg->data.size() << " != " << torque_commands.size());
        return;
    }
    std::copy(msg->data.begin(), msg->data.end(), torque_commands.data());
    safety.update_time();
}

bool ahand_controllers::ROSController::service_command_callback(Command::Request &request, Command::Response &response){
    std::string msg =request.message;
    std::string res;
    if(msg == "controller:none"){
        ROS_WARN("control type: NONE");
        control_type = NONE;
        res = "NONE";
    } else if(msg == "controller::joint_impedance"){
        ROS_WARN("control type: JOINT_IMPEDANCE");
        control_type = JOINT_IMPEDANCE;
        res = "JOINT_IMPEDANCE";
    }else if(msg == "controller:bhand"){
        ROS_WARN("control type: BHAND");
        control_type = BHAND;
        res = "BHAND";
    }else{
        ROS_WARN_STREAM("option [" << msg << "] not implemented");
        res = "FAILED!";
    }
    response.response = "control_type = " + res;
    return true;
}


PLUGINLIB_EXPORT_CLASS(ahand_controllers::ROSController, controller_interface::ControllerBase)
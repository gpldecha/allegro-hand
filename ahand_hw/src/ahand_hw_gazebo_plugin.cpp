// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <controller_manager/controller_manager.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "ahand_hw/ahand_hw_gazebo.hpp"


class AHandHWsimPlugin : public gazebo::ModelPlugin{

    public:

        AHandHWsimPlugin() : gazebo::ModelPlugin() {}

        virtual ~AHandHWsimPlugin(){
           //gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
        }

        virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf){
            std::cout << "ahand_hw" << "Loading ahand_hw plugin" << std::endl;

            // Save pointers to the model
            parent_model_ = parent;
            sdf_ = sdf;

            // Error message if the model couldn't be found
            if (!parent_model_){
              ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
              return;
            }

            // Check that ROS has been initialized
            if(!ros::isInitialized()){
                ROS_FATAL_STREAM_NAMED("ahand_hw","A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            // Get namespace for nodehandle
            if(sdf_->HasElement("robotNamespace")){
                robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
            }else{
                robot_namespace_ = parent_model_->GetName(); // default
            }

            // Get robot_description ROS param name
            if (sdf_->HasElement("robotParam")){
                robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
            }else {
                robot_description_ = "robot_description"; // default
            }

            // Get the Gazebo simulation period
            ros::Duration gazebo_period(parent_model_->GetWorld()->Physics()->GetMaxStepSize());

            // Decide the plugin control period
            if(sdf_->HasElement("controlPeriod")){
                control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));
                // Check the period against the simulation period
                if( control_period_ < gazebo_period ){
                  ROS_ERROR_STREAM_NAMED("ahand_hw","Desired controller update period ("<<control_period_  <<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
                } else if( control_period_ > gazebo_period ) {
                  ROS_WARN_STREAM_NAMED("ahand_hw","Desired controller update period ("<<control_period_ <<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
                }
            }else{
                control_period_ = gazebo_period;
                ROS_DEBUG_STREAM_NAMED("ahand_hw","Control period not found in URDF/SDF, defaulting to Gazebo period of " << control_period_);
            }

            // Get parameters/settings for controllers from ROS param server
            model_nh_ = ros::NodeHandle(robot_namespace_);
            ROS_INFO_NAMED("ahand_hw", "Starting lwr_hw plugin in namespace: %s", robot_namespace_.c_str());

            // Read urdf from ros parameter server then
            // setup actuators and mechanism control node.
            // This call will block if ROS is not properly initialized.
            const std::string urdf_string = getURDF(robot_description_);

            // Load the LWRHWsim abstraction to interface the controllers with the gazebo model
            robot_hw_sim_.reset( new AHandHWGazebo() );
            robot_hw_sim_->create(robot_namespace_, urdf_string);
            robot_hw_sim_->setParentModel(parent_model_);
            if(!robot_hw_sim_->init()){
              ROS_FATAL_NAMED("ahand_hw","Could not initialize robot simulation interface");
              return;
            }

            // Create the controller manager
            ROS_INFO_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
            controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_sim_.get(), model_nh_));

            // Listen to the update event. This event is broadcast every simulation iteration.
            update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&AHandHWsimPlugin::Update, this));
            ROS_INFO_NAMED("ahand_hw", "Loaded ahand_hw.");
        }

        // Called by the world update start event
        void Update(){
            // Get the simulation time and period
            gazebo::common::Time gz_time_now = parent_model_->GetWorld()->SimTime();
            ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
            ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

            // Check if we should update the controllers
            if(sim_period >= control_period_){
              // Store this simulation time
              last_update_sim_time_ros_ = sim_time_ros;

              // Update the robot simulation with the state of the gazebo model
              robot_hw_sim_->read(sim_time_ros, sim_period);

              // Compute the controller commands
              controller_manager_->update(sim_time_ros, sim_period);
            }

            // Update the gazebo model with the result of the controller
            // computation
            robot_hw_sim_->write(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
            last_write_sim_time_ros_ = sim_time_ros;
        }

        // Called on world reset
        virtual void Reset(){
          // Reset timing variables to not pass negative update periods to controllers on world reset
          last_update_sim_time_ros_ = ros::Time();
          last_write_sim_time_ros_ = ros::Time();
        }

    private:

        // Get the URDF XML from the parameter server
        std::string getURDF(std::string param_name) const {
          std::string urdf_string;

          // search and wait for robot_description on param server
          while (urdf_string.empty()){
            std::string search_param_name;
            if (model_nh_.searchParam(param_name, search_param_name)){
              ROS_INFO_ONCE_NAMED("LWRHWsim", "LWRHWsim plugin is waiting for model"
               " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());
              model_nh_.getParam(search_param_name, urdf_string);
            }else{
              ROS_INFO_ONCE_NAMED("LWRHWsim", "LWRHWsim plugin is waiting for model"
                " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());
              model_nh_.getParam(param_name, urdf_string);
            }
            usleep(100000);
          }
          ROS_DEBUG_STREAM_NAMED("LWRHWsim", "Recieved urdf from param server, parsing...");
          return urdf_string;
        }

    private:

        // Pointer to the model
        gazebo::physics::ModelPtr parent_model_;
        sdf::ElementPtr sdf_;

        // Pointer to the update event connection
        gazebo::event::ConnectionPtr update_connection_;

        // Node Handles
        ros::NodeHandle model_nh_;

        // Strings
        std::string robot_namespace_;
        std::string robot_description_;

        // Robot simulator interface
        std::string robot_hw_sim_type_str_;
        boost::shared_ptr<AHandHWGazebo> robot_hw_sim_;

        // Controller manager
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        // Timing
        ros::Duration control_period_;
        ros::Time last_update_sim_time_ros_;
        ros::Time last_write_sim_time_ros_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AHandHWsimPlugin)

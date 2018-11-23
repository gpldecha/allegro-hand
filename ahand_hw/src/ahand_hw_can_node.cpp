// SYS

#include <sys/mman.h>
#include <signal.h>
#include <stdexcept>
#include <chrono>

// ROS

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "ahand_hw/ahand_hw_can.hpp"

// Boost

#include <boost/program_options.hpp>


// Get the URDF XML from the parameter server
std::string getURDF(ros::NodeHandle &model_nh_, std::string param_name)
{
  std::string urdf_string;
  std::string robot_description = "/robot_description";

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("AHAND_HW_CAN_NODE", "AHAND_HW_CAN_NODE is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("AHAND_HW_CAN_NODE", "AHAND_HW_CAN_NODE is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("AHAND_HW_CAN_NODE", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

namespace po = boost::program_options;


int main(int argc, char** argv){

    po::options_description description("ahand_hw_can options");
    description.add_options()
            ("robot_name,n", po::value<std::string>(), "robot name" );

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);
    po::notify(vm);

    std::string namespace_ = vm["robot_name"].as<std::string>();

    // initialize ROS
    ros::init(argc, argv, "ahand_hw_interface", ros::init_options::NoSigintHandler);

    // ros spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // create a node
    ros::NodeHandle ahand_nh(namespace_);

    // get params or give default values
    std::string file;
    std::string name;

    // get the general robot description, the lwr class will take care of parsing what's useful to itself
    std::string urdf_string = getURDF(ahand_nh, namespace_+ "/robot_description");

    // construct and start the real ahand
    AhandHWCAN ahand_robot;
    ahand_robot.create("ahand", urdf_string);

    if(!ahand_robot.init()){
       ROS_FATAL_NAMED("ahand_hw","Could not initialize robot real interface");
       return -1;
    }

    // timer variables
    struct timespec ts = {0, 0};
    ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
    ros::Duration period(1.0);

    //the controller manager
    controller_manager::ControllerManager manager(&ahand_robot, ahand_nh);

    while( true ){
      // get the time / period
      if (!clock_gettime(CLOCK_REALTIME, &ts)){
        now.sec = ts.tv_sec;
        now.nsec = ts.tv_nsec;
        period = now - last;
        last = now;
      }else{
        ROS_FATAL("Failed to poll realtime clock!");
        break;
      }

      // read the state from the lwr
      ahand_robot.read(now, period);

      // update the controllers
      manager.update(now, period);

      // write the command to the lwr
      ahand_robot.write(now, period);
      std::this_thread::sleep_for(std::chrono::milliseconds(3));

    }

    std::cerr<<"Stopping spinner..."<<std::endl;
    spinner.stop();

    std::cerr<<"Stopping ahand_hw..."<<std::endl;
    ahand_robot.stop();

    std::cerr<<"This node was killed!"<<std::endl;

    return 0;
}

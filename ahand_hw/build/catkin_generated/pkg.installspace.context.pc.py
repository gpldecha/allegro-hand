# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "/usr/local/include".split(';') if "/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "control_toolbox;controller_interface;controller_manager;hardware_interface;realtime_tools;joint_limits_interface;roscpp;tf;urdf;cmake_modules;pluginlib;kdl_parser;transmission_interface;gazebo_ros;std_msgs;ahand_driver".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lahand_hw;-lahand_hw_can;-lahand_hw_gazebo_plugin".split(';') if "-lahand_hw;-lahand_hw_can;-lahand_hw_gazebo_plugin" != "" else []
PROJECT_NAME = "ahand_hw"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.0.0"

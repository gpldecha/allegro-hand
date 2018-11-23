#ifndef AHAND_DRIVER_H
#define AHAND_DRIVER_H

#include <thread>
#include <mutex>

#include "can_api/canAPI.h"
#include "can_api/canDef.h"
#include "ahand_driver/rDeviceAllegroHandCANDef.h"
#include <boost/lockfree/spsc_queue.hpp>

// Eigen

#include <Eigen/Dense>


typedef Eigen::Matrix<double, 16, 1> Vector16d;

class AhandDriver{

public:

    AhandDriver();

    ~AhandDriver() = default;

    bool isIntialised();

    void setTorque(const Vector16d& torques);

    bool getJointInfo(Vector16d& joint_positions);

    void stop();

private:

    bool openCAN();

    void closeCAN();

    void updateCAN();

    int getCANChannelIndex(const char* cname);

private:

    int CAN_Ch = 0;
    const double delT = 0.003;
    // USER HAND CONFIGURATION
    const bool	RIGHT_HAND = true;
    const int	HAND_VERSION = 3;

    const double tau_cov_const_v2 = 800.0; // 800.0 for SAH020xxxxx
    const double tau_cov_const_v3 = 1200.0; // 1200.0 for SAH030xxxxx
    double tau_cov_const;

    const double enc_dir[MAX_DOF] = { // SAH030F049
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0
    };
    const double motor_dir[MAX_DOF] = { // SAH030F049
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0
    };
    const int enc_offset[MAX_DOF] = { // SAH030F049
      -6212, 3628, 733, 172, // f1
      -6347, -38, -4683, -4669,
      -3509, -2094, 312, -12,
      8234, -7615, 504, -165
    };

    AllegroHand_DeviceMemory_t vars;

    Vector16d tau_des;
    Vector16d cur_des;
    Vector16d q;
    Vector16d q_des;

    boost::lockfree::spsc_queue<Vector16d> torque_command_buffer;
    boost::lockfree::spsc_queue<Vector16d> sensor_buffer;

    std::thread updated_thread_;
    bool ioThreadRun = false;
    bool isInitialised = false;


};



#endif

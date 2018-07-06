#ifndef AHAND_DRIVER_H
#define AHAND_DRIVER_H

#include <BHand/BHand.h>

#include <thread>

#include "can_api/canAPI.h"
#include "can_api/canDef.h"
#include "ahand_driver/rDeviceAllegroHandCANDef.h"

class AhandDriver{

public:

    AhandDriver();

    ~AhandDriver();

    void setTorque(double *torque);

    void getJointInfo(double *position);

    BHand* const getBHand();

    double* getDesiredJointPosition();


private:

    bool openCAN();

    void closeCAN();

    void updateCAN();

    void computeTorque();

    bool createBHandAlgorithm();

    void destroyBHandAlgorithm();

    int getCANChannelIndex(const char* cname);

private:

    int CAN_Ch = 0;
    const double delT = 0.003;
    // USER HAND CONFIGURATION
    const bool	RIGHT_HAND = true;
    const int	HAND_VERSION = 3;

    const double tau_cov_const_v2 = 800.0; // 800.0 for SAH020xxxxx
    const double tau_cov_const_v3 = 1200.0; // 1200.0 for SAH030xxxxx

    const double enc_dir[MAX_DOF] = { // SAH030xxxxx
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0
    };
    const double motor_dir[MAX_DOF] = { // SAH030xxxxx
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0
    };
    const int enc_offset[MAX_DOF] = { // SAH030C033R
      -1591, -277, 545, 168,
      -904, 53, -233, -1476,
      2, -987, -230, -106,
      -1203, 361, 327, 565
    };

    BHand* pBHand = NULL;

    AllegroHand_DeviceMemory_t vars;
    double q[MAX_DOF];
    double q_des[MAX_DOF];
    double tau_des[MAX_DOF];
    double cur_des[MAX_DOF];

    std::thread updated_thread_;
    bool ioThreadRun = false;


};



#endif

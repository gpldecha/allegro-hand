#ifndef AHAND_DRIVER_H
#define AHAND_DRIVER_H

#include "ahand_driver/canAPI.h"
#include "ahand_driver/canDef.h"
#include "ahand_driver/rDeviceAllegroHandCANDef.h"

class AhandDriver{

public:

private:

    bool openCAN();

    void communcationCAN();

private:

    int CAN_Ch = 0;
    const double delT = 0.003;


};



#endif

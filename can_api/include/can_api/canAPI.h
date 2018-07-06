/*
 *\brief API for communication over CAN bus 
 *\detailed The API for communicating with the various motor controllers
 *          over the CAN bus interface on the robot hand
 *
 *$Author: Sangyup Yi $
 *$Date: 2012/5/11 23:34:00 $
 *$Revision: 1.0 $
 */ 

#ifndef _CANDAPI_H
#define _CANDAPI_H

#include "canDef.h"

// CANAPI_BEGIN

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

/*=====================*/
/*       Defines       */
/*=====================*/
//constants
#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (5)
#define RX_TIMEOUT          (5)
#define mbxID               (0)
#define BASE_ID             (0)
#define MAX_BUS             (256)

/******************/
/* CAN device API */
/******************/

class CANAPI{

    public:

        static int initCAN(int bus);

        static int command_can_open(int ch);
        static int command_can_open_ex(int ch, int type, int index);
        static int command_can_reset(int ch);
        static int command_can_close(int ch);

        static int command_can_query_id(int ch); // since v3.0
        static int command_can_sys_init(int ch, int period_msec);
        static int command_can_start(int ch);
        static int command_can_stop(int ch);

        static int command_can_AHRS_set(int ch, unsigned char rate, unsigned char mask); // since v3.0

        static int write_current(int ch, int findex, short* pwm);

        static int get_message(int ch, char* cmd, char* src, char* des, int* len, unsigned char* data, int blocking);

    private:

        static int canReadMsg(int bus, int *id, int *len, unsigned char *data, int blocking);
        static int canSendMsg(int bus, int id, char len, unsigned char *data, int blocking);

        static int freeCAN(int bus);
};

//CANAPI_END

#endif

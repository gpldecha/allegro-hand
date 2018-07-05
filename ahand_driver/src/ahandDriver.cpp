#include "ahand_driver/ahandDriver.h"

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>

bool AhandDriver::openCAN(){
#if defined(PEAKCAN)
  CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
  CAN_Ch = 1;
#elif defined(SOFTINGCAN)
  CAN_Ch = 1;
#else
  CAN_Ch = 1;
#endif
  CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
  printf(">CAN(%d): open\n", CAN_Ch);
  int ret = command_can_open(CAN_Ch);
  if(ret < 0){
    printf("ERROR command_canopen !!! \n");
    return false;
  }

  /* initialize condition variable */
  int ioThread_error = pthread_create(&hThread, NULL, ioThreadProc, 0);
  printf(">CAN: starts listening CAN frames\n");

  printf(">CAN: query system id\n");
  ret = command_can_query_id(CAN_Ch);
  if(ret < 0)
    {
      printf("ERROR command_can_query_id !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: AHRS set\n");
  ret = command_can_AHRS_set(CAN_Ch, AHRS_RATE_100Hz, AHRS_MASK_POSE | AHRS_MASK_ACC);
  if(ret < 0)
    {
      printf("ERROR command_can_AHRS_set !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: system init\n");
  ret = command_can_sys_init(CAN_Ch, 3/*msec*/);
  if(ret < 0)
    {
      printf("ERROR command_can_sys_init !!! \n");
      command_can_close(CAN_Ch);
      return false;
    }

  printf(">CAN: start periodic communication\n");
  ret = command_can_start(CAN_Ch);

  if(ret < 0)
    {
      printf("ERROR command_can_start !!! \n");
      command_can_stop(CAN_Ch);
      command_can_close(CAN_Ch);
      return false;
    }

  return true;
}


void AhandDriver::communcationCAN(){
    AllegroHand_DeviceMemory_t vars;
    char id_des;
    char id_cmd;
    char id_src;
    int len;
    unsigned char data[8];
    unsigned char data_return = 0;
    int i;

    while (ioThreadRun){
        /* wait for the event */
        while (0 == get_message(CAN_Ch, &id_cmd, &id_src, &id_des, &len, data, FALSE)){
            switch (id_cmd){
                 case ID_CMD_QUERY_ID:
                 {
                    printf(">CAN(%d): AllegroHand revision info: 0x%02x%02x\n", CAN_Ch, data[3], data[2]);
                    printf("                      firmware info: 0x%02x%02x\n", data[5], data[4]);
                    printf("                      hardware type: 0x%02x\n", data[7]);
                 }
                 break;
                 case ID_CMD_AHRS_POSE:
                 {
                    printf(">CAN(%d): AHRS Roll : 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
                    printf("               Pitch: 0x%02x%02x\n", data[2], data[3]);
                    printf("               Yaw  : 0x%02x%02x\n", data[4], data[5]);
                 }
                 break;
                 case ID_CMD_AHRS_ACC:
                 {
                    printf(">CAN(%d): AHRS Acc(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
                    printf("               Acc(y): 0x%02x%02x\n", data[2], data[3]);
                    printf("               Acc(z): 0x%02x%02x\n", data[4], data[5]);
                 }
                 break;
                 case ID_CMD_AHRS_GYRO:
                 {
                    printf(">CAN(%d): AHRS Angular Vel(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
                    printf("               Angular Vel(y): 0x%02x%02x\n", data[2], data[3]);
                    printf("               Angular Vel(z): 0x%02x%02x\n", data[4], data[5]);
                 }
                 break;
                 case ID_CMD_AHRS_MAG:
                 {
                    printf(">CAN(%d): AHRS Magnetic Field(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
                    printf("               Magnetic Field(y): 0x%02x%02x\n", data[2], data[3]);
                    printf("               Magnetic Field(z): 0x%02x%02x\n", data[4], data[5]);
                 }
                 break;
                 case ID_CMD_QUERY_CONTROL_DATA:
                 {
                     if (id_src >= ID_DEVICE_SUB_01 && id_src <= ID_DEVICE_SUB_04){
                        vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 0] = (int)(data[0] | (data[1] << 8));
                        vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 1] = (int)(data[2] | (data[3] << 8));
                        vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 2] = (int)(data[4] | (data[5] << 8));
                        vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 3] = (int)(data[6] | (data[7] << 8));
                        data_return |= (0x01 << (id_src-ID_DEVICE_SUB_01));
                        recvNum++;
                     }
                     if (data_return == (0x01 | 0x02 | 0x04 | 0x08)){
                        // convert encoder count to joint angle
                        for (i=0; i<MAX_DOF; i++){
                            q[i] = (double)(vars.enc_actual[i]*enc_dir[i]-32768-enc_offset[i])*(333.3/65536.0)*(3.141592/180.0);
                        }
                        // compute joint torque
                        ComputeTorque();
                        // convert desired torque to desired current and PWM count
                        for (i=0; i<MAX_DOF; i++){
                            cur_des[i] = tau_des[i] * motor_dir[i];
                            if (cur_des[i] > 1.0) cur_des[i] = 1.0;
                            else if (cur_des[i] < -1.0) cur_des[i] = -1.0;
                        }
                        // send torques
                        for (int i=0; i<4;i++){
                           // the index order for motors is different from that of encoders
                           switch (HAND_VERSION)
                             {
                             case 1:
                             case 2:
                               vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v2);
                               vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v2);
                               vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v2);
                               vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v2);
                               break;

                             case 3:
                             default:
                               vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v3);
                               vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v3);
                               vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v3);
                               vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v3);
                               break;
                             }
                           write_current(CAN_Ch, i, &vars.pwm_demand[4*i]);
                           usleep(5);
                        }

                        //sendNum++;
                        // curTime += delT;

                        data_return = 0;
                     }
                 }
                 break;
            }
       }
    }
}

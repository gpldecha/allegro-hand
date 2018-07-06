

/*======================*/
/*       Includes       */
/*======================*/
//system headers
#include <stdio.h>
#include <errno.h>
#ifndef _WIN32
#include <inttypes.h>
#include <pthread.h>
#include <syslog.h>
#include <unistd.h>
#else
#include <windows.h>
#endif
#include <malloc.h>
#include <assert.h>

typedef unsigned int DWORD;
typedef unsigned short WORD;
typedef char BYTE;
typedef void* LPSTR;

#include <PCANBasic.h>

#include "can_api/canDef.h"
#include "can_api/canAPI.h"


//CANAPI_BEGIN

/*=====================*/
/*       Defines       */
/*=====================*/
//macros
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )
#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#define BROADCAST    (GROUPID(0))
#define Border(Value,Min,Max)  (Value<Min)?Min:((Value>Max)?Max:Value)
//typedefs & structs
//typedef unsigned long DWORD;

/*=========================================*/
/*       Global file-scope variables       */
/*=========================================*/

TPCANHandle canDev[MAX_BUS] = {
	PCAN_NONEBUS, // Undefined/default value for a PCAN bus

	PCAN_ISABUS1, // PCAN-ISA interface, channel 1
	PCAN_ISABUS2, // PCAN-ISA interface, channel 2
	PCAN_ISABUS3, // PCAN-ISA interface, channel 3
	PCAN_ISABUS4, // PCAN-ISA interface, channel 4
	PCAN_ISABUS5, // PCAN-ISA interface, channel 5
	PCAN_ISABUS6, // PCAN-ISA interface, channel 6
	PCAN_ISABUS7, // PCAN-ISA interface, channel 7
	PCAN_ISABUS8, // PCAN-ISA interface, channel 8

	PCAN_DNGBUS1, // PCAN-Dongle/LPT interface, channel 1

	PCAN_PCIBUS1, // PCAN-PCI interface, channel 1
	PCAN_PCIBUS2, // PCAN-PCI interface, channel 2
	PCAN_PCIBUS3, // PCAN-PCI interface, channel 3
	PCAN_PCIBUS4, // PCAN-PCI interface, channel 4
	PCAN_PCIBUS5, // PCAN-PCI interface, channel 5
	PCAN_PCIBUS6, // PCAN-PCI interface, channel 6
	PCAN_PCIBUS7, // PCAN-PCI interface, channel 7
	PCAN_PCIBUS8, // PCAN-PCI interface, channel 8

	PCAN_USBBUS1, // PCAN-USB interface, channel 1
	PCAN_USBBUS2, // PCAN-USB interface, channel 2
	PCAN_USBBUS3, // PCAN-USB interface, channel 3
	PCAN_USBBUS4, // PCAN-USB interface, channel 4
	PCAN_USBBUS5, // PCAN-USB interface, channel 5
	PCAN_USBBUS6, // PCAN-USB interface, channel 6
	PCAN_USBBUS7, // PCAN-USB interface, channel 7
	PCAN_USBBUS8, // PCAN-USB interface, channel 8

	PCAN_PCCBUS1, // PCAN-PC Card interface, channel 1
	PCAN_PCCBUS2, // PCAN-PC Card interface, channel 2
};


/*==========================================*/
/*       Private functions prototypes       */
/*==========================================*/

/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int CANAPI::initCAN(int bus){
	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];
	TPCANBaudrate Baudrate = PCAN_BAUD_1M;
	TPCANType HwType = 0;
	DWORD IOPort = 0;
	WORD Interrupt = 0;

	Status = CAN_Initialize(canDev[bus], Baudrate, HwType, IOPort, Interrupt);
	if (Status != PCAN_ERROR_OK)
	{
		CAN_GetErrorText(Status, 0, strMsg);
        printf("initCAN(): CAN_Initialize() failed with error %ld\n",(long)Status);
		printf("%s\n", strMsg);
		return Status;
	}

	Status = CAN_Reset(canDev[bus]);
	if (Status != PCAN_ERROR_OK)
	{
		CAN_GetErrorText(Status, 0, strMsg);
        printf("initCAN(): CAN_Reset() failed with error %ld\n", (long)Status);
		printf("%s\n", strMsg);
		return Status;
	}

	return 0; // PCAN_ERROR_OK
}

int CANAPI::freeCAN(int bus){
	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];

	Status = CAN_Uninitialize(canDev[bus]);
	if (Status != PCAN_ERROR_OK)
	{
		CAN_GetErrorText(Status, 0, strMsg);
        printf("freeCAN(): CAN_Uninitialize() failed with error %ld\n", (long)Status);
		printf("%s\n", strMsg);
		return Status;
	}

	return 0; // PCAN_ERROR_OK
}

int CANAPI::canReadMsg(int bus, int *id, int *len, unsigned char *data, int blocking){
	TPCANMsg CANMsg;
	TPCANTimestamp CANTimeStamp;
	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];
	int i;

	// We execute the "Read" function of the PCANBasic
	Status = CAN_Read(canDev[bus], &CANMsg, &CANTimeStamp);
	if (Status != PCAN_ERROR_OK)
	{
		if (Status != PCAN_ERROR_QRCVEMPTY)
		{
			CAN_GetErrorText(Status, 0, strMsg);
            printf("canReadMsg(): CAN_Read() failed with error %ld\n", (long)Status);
			printf("%s\n", strMsg);
		}

		return Status;
	}

	*id = CANMsg.ID;
	*len = CANMsg.LEN;
	for(i = 0; i < CANMsg.LEN; i++)
		data[i] = CANMsg.DATA[i];

	return 0;
}

int CANAPI::canSendMsg(int bus, int id, char len, unsigned char *data, int blocking){
	TPCANMsg CANMsg;
	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];
	int i;

	CANMsg.ID = id;
	CANMsg.LEN = len & 0x0F;
	for(i = 0; i < len; i++)
        CANMsg.DATA[i] = data[i];
	CANMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
	Status = CAN_Write(canDev[bus], &CANMsg);
	if (Status != PCAN_ERROR_OK)
	{
		CAN_GetErrorText(Status, 0, strMsg);
        printf("canSendMsg(): CAN_Write() failed with error %ld\n", (long)Status);
		printf("%s\n", strMsg);
		return Status;
	}

	return 0; //PCAN_ERROR_OK;
}

/*========================================*/
/*       CAN API                          */
/*========================================*/
int CANAPI::command_can_open(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	DWORD ret;

	printf("<< CAN: Open Channel...\n");
	ret = initCAN(ch);
	if (ret != 0) return ret;
	printf("\t- Ch.%2d (OK)\n", ch);
	printf("\t- Done\n");

	return 0;
}

int CANAPI::command_can_open_ex(int ch, int type, int index)
{
	return command_can_open(ch);
}

int CANAPI::command_can_reset(int ch)
{
	return -1;
}

int CANAPI::command_can_close(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	TPCANStatus Status = PCAN_ERROR_OK;
	char strMsg[256];
	printf("<< CAN: Close...\n");

	Status = CAN_Uninitialize(canDev[ch]);
	if (Status != PCAN_ERROR_OK)
	{
		CAN_GetErrorText(Status, 0, strMsg);
        printf("freeCAN(): CAN_Uninitialize() failed with error %ld\n", (long)Status);
		printf("%s\n", strMsg);
		return Status;
	}

	printf("\t- Done\n");
	return 0; //PCAN_ERROR_OK;
}

int CANAPI::command_can_query_id(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	unsigned char data[8];

	long Txid = ((unsigned long)ID_CMD_QUERY_ID<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	int ret = canSendMsg(ch, Txid, 0, data, TRUE);

	return 0; //PCAN_ERROR_OK;
}

int CANAPI::command_can_sys_init(int ch, int period_msec)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ((unsigned long)ID_CMD_SET_PERIOD<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	data[0] = (unsigned char)period_msec;
	ret = canSendMsg(ch, Txid, 1, data, TRUE);

	usleep(10);

	Txid = ((unsigned long)ID_CMD_SET_MODE_TASK<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data, TRUE);

	usleep(10);

	Txid = ((unsigned long)ID_CMD_QUERY_STATE_DATA<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data, TRUE);

	return 0; //PCAN_ERROR_OK;
}

int CANAPI::command_can_start(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ((unsigned long)ID_CMD_QUERY_STATE_DATA<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data, TRUE);

	usleep(10);

	Txid = ((unsigned long)ID_CMD_SET_SYSTEM_ON<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data, TRUE);

	return 0; //PCAN_ERROR_OK;
}

int CANAPI::command_can_stop(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ((unsigned long)ID_CMD_SET_SYSTEM_OFF<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	ret = canSendMsg(ch, Txid, 0, data, TRUE);

	return 0; //PCAN_ERROR_OK;
}

int CANAPI::command_can_AHRS_set(int ch, unsigned char rate, unsigned char mask)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ((unsigned long)ID_CMD_AHRS_SET<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);
	data[0] = (unsigned char)rate;
	data[1] = (unsigned char)mask;
	ret = canSendMsg(ch, Txid, 2, data, TRUE);

	return 0; //PCAN_ERROR_OK;
}

int CANAPI::write_current(int ch, int findex, short* pwm)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	unsigned char data[8];
	int ret;

	if (findex >= 0 && findex < 4)
	{
		data[0] = (unsigned char)( (pwm[0] >> 8) & 0x00ff);
		data[1] = (unsigned char)(pwm[0] & 0x00ff);

		data[2] = (unsigned char)( (pwm[1] >> 8) & 0x00ff);
		data[3] = (unsigned char)(pwm[1] & 0x00ff);

		data[4] = (unsigned char)( (pwm[2] >> 8) & 0x00ff);
		data[5] = (unsigned char)(pwm[2] & 0x00ff);

		data[6] = (unsigned char)( (pwm[3] >> 8) & 0x00ff);
		data[7] = (unsigned char)(pwm[3] & 0x00ff);

		Txid = ((unsigned long)(ID_CMD_SET_TORQUE_1 + findex)<<6) | ((unsigned long)ID_COMMON <<3) | ((unsigned long)ID_DEVICE_MAIN);

		ret = canSendMsg(ch, Txid, 8, data, TRUE);
	}
	else
		return -1;

	return 0;
}

int CANAPI::get_message(int ch, char* cmd, char* src, char* des, int* len, unsigned char* data, int blocking)
{
	int err;
	unsigned long Rxid;

	err = canReadMsg(ch, (int*)&Rxid, len, data, blocking);
	if (!err)
	{
		*cmd = (char)( (Rxid >> 6) & 0x1f );
		*des = (char)( (Rxid >> 3) & 0x07 );
		*src = (char)( Rxid & 0x07);
	}
	else
	{
		return err;
	}
	return 0;
}



// CANAPI_END

#ifndef _ModbusProvider
#define _ModbusProvider
#include <stdlib.h>
#include <time.h>
#include <list>
#include <windows.h>
#include "CustomFunctionMapping.h"
#include "DeviceTypeEnum.h"
#include "timespec.h"

#define PORT_SOCKET_FAILURE     -0x0D
#define MSG_LENGTH_UNDEFINED -1
#define TIME_OUT_BEGIN_OF_TRAME 4
#define SELECT_FAILURE          -0x0E
#define CONNECTION_CLOSED       -0x12
#define MAX_MESSAGE_LENGTH     256
#define TOO_MANY_DATA           -0x0F
#define TIME_OUT_END_OF_TRAME   500000
#define INVALID_CRC             -0x10
#define COM_TIME_OUT           -0x0C
#define HEADER_LENGTH_RTU           0
#define CHECKSUM_LENGTH_RTU      2
#define COM_WAIT_TIMEOUT 500

using namespace std;

class ModbusProvider
	{
		private :

	    DeviceTypeEnum deviceType;
	    char * device;
       int baud;

		HANDLE hCom2;
		HANDLE g_syncReceive;

		CRITICAL_SECTION * syncReceive;

		CustomFunctionMapping ** mapping;
		int mappinglength;

		COMMTIMEOUTS CommTimeOuts;

	    public :
		ModbusProvider(DeviceTypeEnum deviceType, const char * device, CustomFunctionMapping ** mapping, int mappinglength);
	    ~ModbusProvider();
	    void Initialize(HANDLE hCom2);
	    void SendData(unsigned char * query, unsigned char querylength, unsigned char * response, unsigned char responseLength, unsigned char & realResponseLength);
	    void Close();
	    static short crc16(char rw, unsigned char * buffer, short buffer_length);
	    int modbus_send(unsigned char *query, int query_length);
	    int receive_msg(DWORD msg_length_computed, unsigned char *msg, unsigned char & p_msg_length);
	    static timespec * CreateTime(int aditionalInterval);
	    static timespec * CreateDelay(int delayInterval);
	    private :
	    unsigned char compute_query_length_header(int function);
	    int compute_query_length_data(unsigned char *msg);
	    static int check_crc16(unsigned char *msg, int length);
	    //int WaitData(struct timeval * tv);
	    int WaitData(struct timespec * ts);
	    void modbus_init_rtu(const char *device, int baud);
	    int modbus_connect_rtu(HANDLE hCom2);
	    void modbus_close_rtu();
		void VerifyCOMPorts();
		void OpenPort(HANDLE hCom2);
		void ModbusProvider_Port1_DataReceived();
	};
#endif
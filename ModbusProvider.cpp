#pragma once
#include "stdafx.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <Winsock2.h>

#include "MWTSoft2.h"
#include "ModbusProvider.h"

using namespace std;

#define BILLION 1000000000

ModbusProvider::ModbusProvider(DeviceTypeEnum deviceType, const char * device, CustomFunctionMapping ** mapping, int mappinglength)
{
	syncReceive = new CRITICAL_SECTION();
	InitializeCriticalSection(syncReceive);
	g_syncReceive = CreateEvent(NULL, TRUE, FALSE, NULL);	
	this->mapping = mapping;
	this->mappinglength = mappinglength;
	this->device = (char *)device;
}

ModbusProvider::~ModbusProvider()
{
	this->Close();        
}

void ModbusProvider::ModbusProvider_Port1_DataReceived()
{
	/*DWORD result = WaitForSingleObject(syncReceive, INFINITE);
	SetEvent(syncReceive);		*/
}

void ModbusProvider::Initialize(HANDLE hCom2)
{	
    this->modbus_init_rtu(device, 19200);
	this->VerifyCOMPorts();	
	this->modbus_connect_rtu(hCom2);
}

void ModbusProvider::SendData(unsigned char * query, unsigned char querylength, unsigned char * response, unsigned char responseLength, unsigned char & realResponseLength)
{
    this->modbus_send(query, querylength);
    this->receive_msg(responseLength, response, realResponseLength);
    if(realResponseLength == 0)
    {
        //cout << "Repeat!"<< endl;
        //timespec * ts = new timespec();
        //nanosleep(CreateDelay(COM_WAIT_TIMEOUT), NULL);
        //nanosleep(ts, NULL);
        this->receive_msg(responseLength, response, realResponseLength);
    }

    //cout << ts1->tv_sec << " " << ts1->tv_nsec << "!!!!!!!!!!!!!!!!!!Server received length" << (int)realResponseLength <<  (int)response[realResponseLength - 2] << " " << (int)response[realResponseLength - 1] << endl;    
}

void ModbusProvider::Close()
{
    this->modbus_close_rtu();
}

/* Sends a query/response over a serial or a TCP communication */
int ModbusProvider::modbus_send(unsigned char *query, int query_length)
{
	DWORD real_length;
   unsigned char symbol[1];
   symbol[0] = 255;
	WriteFile(hCom2,symbol,1,&real_length,NULL);
	WriteFile(hCom2,query,query_length,&real_length,NULL);
	return (int)real_length;
}


short ModbusProvider::crc16(char rw, unsigned char * buffer, short buffer_length)
// vozvrat: 0-vipolneno,
// rw=='w' - vichislenie CRC dla heredachi (CRC dobavlaetsa v konce posilki)
// rw=='r' - vichislenie CRC otveta (CRC dobavlaetsa posle prinjatogo CRC)
// Len - chislo byte CRC kotorih vichislaetsja
{
  unsigned char CRCbyteLOW=0xFF ,CRCbyteHIGH=0xFF;
  int i,index;

  for (i=0; i<buffer_length; i++)
         {
          index=CRCbyteHIGH^buffer[i];
          CRCbyteHIGH=CRCbyteLOW^crc_table[index];
          CRCbyteLOW=crc_table[index+256];
         }

  if (rw=='w')
      {
       	buffer[buffer_length+1]=CRCbyteLOW;
       	buffer[buffer_length]=CRCbyteHIGH;
         printf("\r\nsend crc : %i, %i\r\n", buffer[buffer_length+1], buffer[buffer_length]);
      }
   if (rw=='r')
      {
         return ((short)CRCbyteHIGH << 8) | CRCbyteLOW;
		}

  return 0;

}

/*
int ModbusProvider::WaitData(struct timeval * tv)
{
	int waitTime = new TimeSpan(tv->tv_sec * 1000000 + tv->tv_usec * 1000);
	if(!Monitor::Wait(this->syncReceive, *waitTime))
	{
		return COM_TIME_OUT;
	}
	else
	{
		return -1;
	}

	// Linux reqlisation

    fd_set * rfds = new fd_set();
    // Add a file descriptor to the set
    FD_ZERO(rfds);
    FD_SET(mb_param->fd, rfds);

    int select_ret;
    while ((select_ret = select(mb_param->fd+1, rfds, NULL, NULL, tv)) == -1) {
            if (errno == EINTR) {
                    printf("A non blocked signal was caught\n");
                    //Necessary after an error
                    FD_ZERO(rfds);
                    FD_SET(mb_param->fd, rfds);
            }
            else
            {
                break;
            }
    }

    delete rfds;

    if (select_ret == 0) {
            //Call to error_treat is done later to manage exceptions
            return COM_TIME_OUT;
    }
    else
    {
        //error_treat(mb_param, SELECT_FAILURE, "Select failure");
        return SELECT_FAILURE;
    }
}
*/

int ModbusProvider::WaitData(struct timespec * ts)
{
	int waitTime = ts->tv_sec * 1000 + ts->tv_nsec;	
	Sleep(200);
	//DWORD result = WaitForSingleObject(hCom2, waitTime); //Richter p.353
	//if(!result)
	//{
	//	//Console::WriteLine("COM_TIME_OUT");
	//	return COM_TIME_OUT;
	//}
	return 0;		
}

/* Waits a reply from a modbus slave or a query from a modbus master.
   This function blocks for timeout seconds if there is no reply.

   In
   - msg_length_computed must be set to MSG_LENGTH_UNDEFINED if undefined

   Out
   - msg is an array of unsigned char to receive the message
   - p_msg_length, the variable is assigned to the number of
     characters received. This value won't be greater than
     msg_length_computed.

   Returns 0 in success or a negative value if an error occured.
*/
int ModbusProvider::receive_msg(DWORD msg_length_computed, unsigned char *msg, unsigned char & p_msg_length)
{
	int select_ret;	
	struct timeval tv;
	int length_to_read;
	unsigned char *p_msg;
	enum { FUNCTION, BYTE, COMPLETE };
	int state;        

	if (msg_length_computed == MSG_LENGTH_UNDEFINED) {
		// Wait for a message
		tv.tv_sec = 0;
		tv.tv_usec = 4;

		// The message length is undefined (query receiving) so
		// we need to analyse the message step by step.
		// At the first step, we want to reach the function
		// code because all packets have that information.
		msg_length_computed = HEADER_LENGTH_RTU + 2;
		state = FUNCTION;
	} else {
		tv.tv_sec = 0;
		tv.tv_usec = TIME_OUT_BEGIN_OF_TRAME;
		state = COMPLETE;
	}

	length_to_read = msg_length_computed;

	select_ret = 1;
	if(WaitData(CreateDelay(COM_WAIT_TIMEOUT)) == COM_TIME_OUT)
	{
		return -1;
	}
	//time_t t = time(NULL);
	//cout<<ctime((const time_t *)(&t))<<endl;

	// Initialize the readin the message
	p_msg_length = 0;
	p_msg = msg;

	unsigned char bytes [256];

	//DWORD result = WaitForSingleObject(g_syncReceive, INFINITE);
	//SetEvent(g_syncReceive);
   int index = 0;
   unsigned long count = 0;
   do
   {
		ReadFile(hCom2,bytes+index,256,&count,NULL);
		index += (int)count;
      Sleep(200);
   }
   while(count > 0);
	//ResetEvent(g_syncReceive);
	/*
	while (select_ret) {


		//read_ret = read(mb_param->fd, p_msg, length_to_read);
		//cout << "result : " << read_ret << "length" << length_to_read << " " << msg_length_computed<< endl;

		//if (read_ret == 0) {
		//        printf("Connection closed\n");
		//        return CONNECTION_CLOSED;
		//} else
		if (read_ret < 0) {
			// The only negative possible value is -1
			//error_treat(mb_param, PORT_SOCKET_FAILURE, "Read port/socket failure");
			return PORT_SOCKET_FAILURE;
		}

		// Sums bytes received
		p_msg_length += read_ret;

		if (p_msg_length < msg_length_computed) {
			// Message incomplete
			length_to_read = msg_length_computed - p_msg_length;
		} else {
			switch (state) {
			case FUNCTION:
				// Function code position
				length_to_read = compute_query_length_header(msg[HEADER_LENGTH_RTU + 1]);
				msg_length_computed += length_to_read;
				// It's useless to check
				//   p_msg_length_computed value in this
				//   case (only defined values are used).
				state = BYTE;
				break;
			case BYTE:
				length_to_read = compute_query_length_data(msg);
				msg_length_computed += length_to_read;
				if (msg_length_computed > MAX_MESSAGE_LENGTH) {
					//error_treat(mb_param, TOO_MANY_DATA, "Too many data");
					return TOO_MANY_DATA;
				}
				state = COMPLETE;
				break;
			case COMPLETE:
				length_to_read = 0;
				break;
			}
		}

		// Moves the pointer to receive other data
		p_msg = &(p_msg[read_ret]);

		if (length_to_read > 0) {
			// If no character at the buffer wait
			//   TIME_OUT_END_OF_TRAME before to generate an error.
			tv.tv_sec = 0;
			tv.tv_usec = TIME_OUT_END_OF_TRAME;

			if(WaitData(CreateDelay(10000000)) == COM_TIME_OUT)
			{
				return -1;
			}

			//cout<<"!" <<gth_to_read<< ctime((const time_t *)(&t)) << endl;
		} else {
			// All chars are received
			cout << "All chars are received" << endl;
			select_ret = false;
		}
	}
	*/

	if(index > 2)
	{
		short crc = crc16('r', bytes, index - 2);
      unsigned char crclow = crc & 0xFF;
      unsigned char crchigh = crc >> 8;
		if( crchigh != bytes[index - 2] ||
			crclow != bytes[index - 1])
		{
			//printf("crc wrong %i %i %i\n", msg_length_computed, crc16(bytes, msg_length_computed - 2) & 0xFF, (crc16(bytes, msg_length_computed - 2) & 0xFF00) >> 8);
			printf("crc wrong %i %i %i (%i %i)\n", index, bytes[index - 2], bytes[index - 1], crchigh, crclow);
			return INVALID_CRC;
		}

		for(int i =0; i < index; i++)
		{
			msg[i] = bytes[i];
		}
      p_msg_length = index;
	}


	// OK
	return 0;
}


/* Computes the length of the header following the function code */
unsigned char ModbusProvider::compute_query_length_header(int function)
{
        int length = 0;

		for(int i =0; i < this->mappinglength; i++)
	    {
			if(this->mapping[i]->function == function)
            {
                    return this->mapping[i]->headerlength;
            }
		}

        /*
        if (function <= FC_FORCE_SINGLE_COIL ||
            function == FC_PRESET_SINGLE_REGISTER)
                // Read and single write
                length = 4;
        else if (function == FC_FORCE_MULTIPLE_COILS ||
                 function == FC_PRESET_MULTIPLE_REGISTERS)
                // Multiple write
                length = 5;
        else
                length = 0;
        */
        return length;
}


/* Computes the length of the data to write in the query */
int ModbusProvider::compute_query_length_data(unsigned char *msg)
{
        int function = msg[HEADER_LENGTH_RTU + 1];
        int length = 0;

		for(int i =0; i < this->mappinglength; i++)
		{
			if(this->mapping[i]->function == function)
            {
                    return this->mapping[i]->datalength + CHECKSUM_LENGTH_RTU;
            }
		}

        /*if (function == FC_FORCE_MULTIPLE_COILS ||
            function == FC_PRESET_MULTIPLE_REGISTERS)
                length = msg[HEADER_LENGTH_RTU + 6];
        else
                length = 0;

        length += mb_param->checksum_length;

       */
        return length;
}


/* If CRC is correct returns 0 else returns INVALID_CRC */
int ModbusProvider::check_crc16(unsigned char * msg, int length)
{
        int ret;
        short crc_calc;
        short crc_received;

        crc_calc = crc16('r', msg, length - 2);
        crc_received = (msg[length - 2] << 8) | msg[length - 1];

        /* Check CRC of msg */
        if (crc_calc == crc_received) {
                ret = 0;
        } else {
                char s_error[64];
                sprintf(s_error,
                        "invalid crc received %0X - crc_calc %0X",
                        crc_received, crc_calc);
                ret = INVALID_CRC;
                //error_treat(mb_param, ret, s_error);
        }

        return ret;
}

/* Initializes the modbus_param_t structure for RTU
   - device: "/dev/ttyS0"
   - baud:   9600, 19200, 57600, 115200, etc
   - parity: "even", "odd" or "none"
   - data_bits: 5, 6, 7, 8
   - stop_bits: 1, 2
*/

void ModbusProvider::modbus_init_rtu(const char *device, int baud)
{
	this->device = (char *)device;
   this->baud = baud;
}

void ModbusProvider::OpenPort(HANDLE hCom2)
{
	//WCHAR * str = new WCHAR [256];
	//for(int i=0; i < strlen(device); i++)
	//{
	//	str[i] = device[i];
	//}

	//str[strlen(device)] = 0;

	this->hCom2 = hCom2;

	unsigned char message_length = 0;
	unsigned char buffer[256];
	receive_msg(256, buffer, message_length);
}

int ModbusProvider::modbus_connect_rtu(HANDLE hCom2)
{
	ModbusProvider::OpenPort(hCom2);
	return 0;
}



/* Closes the file descriptor in RTU mode */
void ModbusProvider::modbus_close_rtu()
{
	CloseHandle(&hCom2);
}


timespec * ModbusProvider::CreateTime(int aditionalInterval)
{
    timespec * waittime = new timespec();

    waittime->tv_nsec += aditionalInterval;

    if(waittime->tv_nsec > BILLION)
    {
        waittime->tv_sec++;
        waittime->tv_nsec-=BILLION;
    }

    return waittime;
}


timespec * ModbusProvider::CreateDelay(int delayInterval)
{
    timespec * delay = new timespec();

    delay->tv_sec = 0;
    delay->tv_nsec = delayInterval;

    return delay;
}

void ModbusProvider::VerifyCOMPorts()
{
	/*
	if(this->port1->IsOpen)
		return;
	try
	{
		this->modbus_connect_rtu();
		portAvailable = true;
	}
	catch (IOException ^ ex)
	{
		portAvailable = false;
		return;
	}
	finally
	{
		this->modbus_close_rtu();
	}
	*/
}
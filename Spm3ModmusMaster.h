#pragma once
#include <stdlib.h>
#include "CMODEEnum.h"
#include "ReadBufferEnum.h"
#include "SpeedEnum.h"
#include "ModbusProvider.h"
#include "CommandEntity.h"
#include "CommandFactory.h"
#include "PrivateMessanger.h"

class Spm3ModbusMaster : public PrivateMessanger
	{
	    private :
	    unsigned char address;
	    ModbusProvider * provider;
		CommandFactory * commandFactory;
	    public :
	    Spm3ModbusMaster(unsigned char address,const char * device); //also provider
	    ~Spm3ModbusMaster();

	    void Open();
	    int CMODE(CMODEEnum mode);
	    int ReadBuffer(ReadBufferEnum channel);
	    int GetRegisters(short startAddress, unsigned char count);
	    int SetRegisters(short startAddress, unsigned char count, const unsigned char * registers, unsigned char registerslength);
	    int WriteEEPROM();    
	    int SetSpeed(SpeedEnum speed);
	    int SendId();
	    void Close();

	    static void CalculateStartAddressAndSizeRegisters(unsigned char * data, unsigned char startAddressHight, unsigned char startAddressLow, unsigned char count, short & startAddress, unsigned char & realBytesCount, unsigned char * & startData);
	    static void CalculateStartAddressAndSizeRegisters(unsigned char * data, short startAddress, unsigned char count, unsigned char & realBytesCount, unsigned char * & startData);
	    static void CalculateStartAddressAndSizeRegisters(short startAddress, unsigned char count, unsigned char & realBytesCount);

	};
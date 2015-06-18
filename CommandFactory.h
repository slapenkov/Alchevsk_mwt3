#ifndef _CommandFactory
#define _CommandFactory

#include <time.h>
#include <windows.h>
#include "CommandEntity.h"
#include "CMODEEnum.h"
#include "SpeedEnum.h"
#include "ReadBufferEnum.h"

class CommandFactory
	{
		public :

		CommandFactory();
		~CommandFactory();
	    CommandEntity * CreateCMODE(unsigned char address, CMODEEnum mode);
	    CommandEntity * CreateReadBuffer(unsigned char address, ReadBufferEnum channel);
	    CommandEntity * CreateGetRegisters(unsigned char address, short startAddress, unsigned char count);
	    CommandEntity * CreateSetRegisters(unsigned char address, short startAddress, unsigned char count, const unsigned char * registers, unsigned char registerslength);
	    CommandEntity * CreateWriteToEEPROM(unsigned char address);
	    CommandEntity * CreateSetSpeed(unsigned char address, SpeedEnum mode);
	    CommandEntity * CreateSendId(unsigned char address);
	    CommandEntity * CreateResponse(unsigned char * rawresponse, int count);
		
		CommandEntity * CreateScanController(unsigned char address);
		CommandEntity * CreateRegisterController(unsigned char address, unsigned char deviceType);
		CommandEntity * CreateDeviceWasHalted(unsigned char address);		
		
		CommandEntity * CreateRequiestData(unsigned char address, time_t startDate, int duration);
		CommandEntity * CreateDataPacketReceived(unsigned char address);
		CommandEntity * CreateDataPacketBrocken(unsigned char address);
		CommandEntity * CreateDataTransactionComplited(unsigned char address);
		CommandEntity * CreateBeginDataSend(unsigned char address, int duration);
		CommandEntity * CreateSendDataPacket(unsigned char address, int offset, unsigned char count, unsigned char * data, int datalength);
		CommandEntity * CreateEndDataSend(unsigned char address, bool hasData);
		CommandEntity * CreateNoMedium(unsigned char address);
		
		CommandEntity * CreateBeginRemoteControl(unsigned char address);
		CommandEntity * CreateRequestDeviceFile(unsigned char address, int fileCode);
		CommandEntity * CreateSaveDeviceFiles(unsigned char address, int fileMask);
		CommandEntity * CreateDeviceFileUpload(unsigned char address, int fileCode);
		CommandEntity * CreateCompliteRemoteControl(unsigned char address);
		CommandEntity * CreateSendDeviceFiles(unsigned char address, int fileMask);

		CommandEntity * CreateResponseTemplate(CommandEntity * request);

		CommandEntity * CreateDefaultResponse(CommandEntity * request);
	};
#endif
#pragma once
#include "stdafx.h"
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include "CommandFactory.h"
#include "CommandResponseFactory.h"
#include "CustomFunctionMapping.h"
#include "ControllerCommandsEnum.h"
#include "DataTransferCommandEnum.h"
#include "RemoteAccessCommandsEnum.h"
#include "DefaultCommandsEnum.h"

CommandFactory::CommandFactory()
{	
}

CommandFactory::~CommandFactory()
{	
}

CommandEntity * CommandFactory::CreateCMODE(unsigned char address, CMODEEnum mode)
{	
    return new CommandEntity(address, SMP3_CMODE, (unsigned char)mode, true, NULL, 0, SMP3_CMODE_RESPONSE_DATA_LENGTH);
}

CommandEntity * CommandFactory::CreateReadBuffer(unsigned char address, ReadBufferEnum channel)
{
    return new CommandEntity(address, SMP3_READ_BUFFER, (unsigned char)channel, true, NULL, 0, SMP3_READ_BUFFER_RESPONSE_DATA_LENGTH);
}

CommandEntity * CommandFactory::CreateGetRegisters(unsigned char address, short startAddress, unsigned char count)
{
	static unsigned char rawdata[3];
    rawdata[0] = startAddress >> 8;
    rawdata[1] = startAddress &0xFF;
    rawdata[2] = count;	
	return  new CommandEntity(address, SMP3_GET_REGISTERS, 0, false, rawdata, 3, SMP3_GET_REGISTERS_RESPONSE_DATA_LENGTH);
}

CommandEntity * CommandFactory::CreateSetRegisters(unsigned char address, short startAddress, unsigned char count, const unsigned char * registers, unsigned char registerslength)
{
	static unsigned char rawdata[3];
    //unsigned char realBytesCount = 0;
    //CommandResponseFactory::CalculateStartAddressAndSizeRegisters(startAddress, count, realBytesCount);
    rawdata[0] = startAddress >> 8;
    rawdata[1] = startAddress & 0xFF;
    rawdata[2] = count;
    memcpy(rawdata + 3, registers, registerslength);
	return new CommandEntity(address, SMP3_SET_REGISTERS, 0, false, rawdata, registerslength + 3, SMP3_SET_REGISTERS_RESPONSE_DATA_LENGTH);
}

CommandEntity * CommandFactory::CreateWriteToEEPROM(unsigned char address)
{
    return new CommandEntity(address, SMP3_WRITE_EEPROM, 0, false, NULL, 0, SMP3_WRITE_EEPROM_RESPONSE_DATA_LENGTH);
}

CommandEntity * CommandFactory::CreateSetSpeed(unsigned char address, SpeedEnum mode)
{
    return new CommandEntity(address, SMP3_SET_SPEED, (unsigned char)mode, true, NULL, 0, SMP3_SET_SPEED_RESPONSE_DATA_LENGTH);
}

CommandEntity * CommandFactory::CreateSendId(unsigned char address)
{
    return new CommandEntity(address, SMP3_SEND_ID, 0, false, NULL, 0, SMP3_SEND_ID_RESPONSE_DATA_LENGTH);
}

CommandEntity * CommandFactory::CreateResponse(unsigned char * rawresponse, int count)
{
	static unsigned char rawdata[256];
    unsigned char address = rawresponse[0];
    unsigned char function = rawresponse[1];
    unsigned char subfunction = 0;
    bool hassubfunction = false;
    unsigned char * data = NULL;
    unsigned char datacount = 0;

    switch(function)
    {
        case SMP3_CMODE :
            hassubfunction = false;
            break;
        case SMP3_READ_BUFFER :
            hassubfunction = false;
            datacount = count - 4;
            break;
        case SMP3_GET_REGISTERS :
            hassubfunction = false;
            datacount = count - 4;
            break;
        case SMP3_SET_REGISTERS :
            hassubfunction = false;
            break;
        case SMP3_WRITE_EEPROM :
            hassubfunction = false;
            break;
        case SMP3_SET_SPEED :
            hassubfunction = false;
            break;
        case SMP3_SEND_ID :
            hassubfunction = false;
            datacount = count - 4;
            break;
    }

    if(datacount > 0)
    {
        memcpy(rawdata, rawresponse + count - datacount - 2, datacount);
    }

    return new CommandEntity(address, function, subfunction, hassubfunction, rawdata, datacount, 0);
}

CommandEntity * CommandFactory::CreateScanController(unsigned char address)
{
	return new CommandEntity(address, SCANCONTROLLER_FUNCTION, 0, false, NULL, 0, SCANCONTROLLER_FUNCTION_RESPONSE_DATA_LENGTH, ScanController);
}

CommandEntity * CommandFactory::CreateRegisterController(unsigned char address, unsigned char deviceType)
{
	static unsigned char rawdata[1];
	rawdata[0] = 1;    // char * name = "TM3"; May me use crc to set value from name
	return new CommandEntity(address, REGISTERCONTROLLER_FUNCTION, 0, false, rawdata, 1, REGISTERCONTROLLER_FUNCTION_RESPONSE_DATA_LENGTH, RegisterController);

}

CommandEntity * CommandFactory::CreateDeviceWasHalted(unsigned char address)
{
	return new CommandEntity(address, DEVICEWASHALTED_FUNCTION, 0, false, NULL, 0, DEVICEWASHALTED_FUNCTION_RESPONSE_DATA_LENGTH, DeviceWasHalted);
}

CommandEntity * CommandFactory::CreateRequiestData(unsigned char address, time_t startDate, int duration)
{
	static unsigned char rawdata[10];

	rawdata[0] = (startDate >> 56) & 0xFF;
	rawdata[1] = (startDate >> 48) & 0xFF;
	rawdata[2] = (startDate >> 40) & 0xFF;
	rawdata[3] = (startDate >> 32) & 0xFF;
	rawdata[4] = (startDate >> 24) & 0xFF;
	rawdata[5] = (startDate >> 16) & 0xFF;
	rawdata[6] = (startDate >> 8) & 0xFF;
	rawdata[7] = startDate & 0xFF;
	rawdata[8] = duration / 60;
	rawdata[9] = duration % 60;
	return new CommandEntity(address, REQUESTDATA_FUNCTION, 0, false, rawdata, 10, REQUESTDATA_FUNCTION_RESPONSE_DATA_LENGTH, RequiestData);

}

CommandEntity * CommandFactory::CreateDataPacketReceived(unsigned char address)
{
	return new CommandEntity(address, DATAPACKETRECEIVED_FUNCTION, 1, true, NULL, 0, DATAPACKETRECEIVED_FUNCTION_RESPONSE_DATA_LENGTH, DataPacketReceived);
}

CommandEntity * CommandFactory::CreateDataPacketBrocken(unsigned char address)
{
	return new CommandEntity(address, DATAPACKETRECEIVED_FUNCTION, 0, true, NULL, 0, DATAPACKETRECEIVED_FUNCTION_RESPONSE_DATA_LENGTH, DataPacketBrocken);
}

CommandEntity * CommandFactory::CreateDataTransactionComplited(unsigned char address)
{
	return new CommandEntity(address, DATATRANSACTIONCOMPLITED_FUNCTION, 0, false, NULL, 0, DATATRANSACTIONCOMPLITED_FUNCTION_RESPONSE_DATA_LENGTH, DataTransactionComplited);
}

CommandEntity * CommandFactory::CreateBeginDataSend(unsigned char address, int duration)
{
	static unsigned char rawdata[2];
	rawdata[0] = duration / 60;
	rawdata[1] = duration % 60;

	int identifier = BeginDataSend;

	if(duration == 0)
	{
		int identifier = NoMedium;
	}

	return new CommandEntity(address, REQUESTDATA_FUNCTION, 0, false, rawdata, 2, BEGINDATASEND_FUNCTION_RESPONSE_DATA_LENGTH, identifier);
	
}

CommandEntity * CommandFactory::CreateSendDataPacket(unsigned char address, int offset, unsigned char count, unsigned char * data, int datalength)
{
	return new CommandEntity(address, SENDDATAPACKET_FUNCTION, 0, false, data, datalength, SENDDATAPACKET_FUNCTION_RESPONSE_DATA_LENGTH, SendDataPacket);
}

CommandEntity * CommandFactory::CreateEndDataSend(unsigned char address, bool hasData)
{
	return new CommandEntity(address, DATATRANSACTIONCOMPLITED_FUNCTION, hasData, true, NULL, 0, ENDDATASEND_FUNCTION_RESPONSE_DATA_LENGTH, EndDataSend);
}

CommandEntity * CommandFactory::CreateNoMedium(unsigned char address)
{
	return CommandFactory::CreateBeginDataSend(address, 0);	
}

		
CommandEntity * CommandFactory::CreateBeginRemoteControl(unsigned char address)
{
	return new CommandEntity(address, BEGINREMOTECONTROL_FUNCTION, 0, false, NULL, 0, BEGINREMOTECONTROL_FUNCTION_RESPONSE_DATA_LENGTH, BeginRemoteControl);
}

CommandEntity * CommandFactory::CreateRequestDeviceFile(unsigned char address, int fileCode)
{
	static unsigned char rawdata[4];

	rawdata[0] = fileCode >> 24;
	rawdata[1] = (fileCode >> 16) & 0xFF;
	rawdata[2] = (fileCode >> 8) & 0xFF;
	rawdata[3] = fileCode & 0xFF;	

	return new CommandEntity(address, REQUESTDEVICEFILE_FUNCTION, 0, false, rawdata, 4, REQUESTDEVICEFILE_FUNCTION_RESPONSE_DATA_LENGTH, RequestDeviceFile);
	

}

CommandEntity * CommandFactory::CreateSaveDeviceFiles(unsigned char address, int fileMask)
{
	static unsigned char rawdata[4];
	
	rawdata[0] = fileMask >> 24;
	rawdata[1] = (fileMask >> 16) & 0xFF;
	rawdata[2] = (fileMask >> 8) & 0xFF;
	rawdata[3] = fileMask & 0xFF;	

	return new CommandEntity(address, SAVEDEVICEFILES_FUNCTION, 0, false, rawdata, 4, SAVEDEVICEFILES_FUNCTION_RESPONSE_DATA_LENGTH, SaveDeviceFiles);
	
}

CommandEntity * CommandFactory::CreateDeviceFileUpload(unsigned char address, int fileCode)
{
	static unsigned char rawdata[4];
	
	rawdata[0] = fileCode >> 24;
	rawdata[1] = (fileCode >> 16) & 0xFF;
	rawdata[2] = (fileCode >> 8) & 0xFF;
	rawdata[3] = fileCode & 0xFF;		

	return new CommandEntity(address, DEVICEFILEUPLOAD_FUNCTION, 0, false, rawdata, 4, DEVICEFILEUPLOAD_FUNCTION_RESPONSE_DATA_LENGTH, DeviceFileUpload);
	
}

CommandEntity * CommandFactory::CreateCompliteRemoteControl(unsigned char address)
{
	return new CommandEntity(address, COMPLITEREMOTECONTROL_FUNCTION, 0, false, NULL, 0, COMPLITEREMOTECONTROL_FUNCTION_RESPONSE_DATA_LENGTH, CompliteRemoteControl);
}

CommandEntity * CommandFactory::CreateSendDeviceFiles(unsigned char address, int fileMask)
{
	static unsigned char rawdata[4];

	rawdata[0] = fileMask >> 24;
	rawdata[1] = (fileMask >> 16) & 0xFF;
	rawdata[2] = (fileMask >> 8) & 0xFF;
	rawdata[3] = fileMask & 0xFF;	
	
	return new CommandEntity(address, SENDDEVICEFILES_FUNCTION, 0, false, rawdata, 4, SENDDEVICEFILES_FUNCTION_RESPONSE_DATA_LENGTH, SendDeviceFiles);
	
}

CommandEntity * CommandFactory::CreateResponseTemplate(CommandEntity * request)
{
	switch(request->identifier)
	{		
		case ScanController :
			return CreateRegisterController(request->GetAddress(), 0);
			break;
		case RegisterController :
			return CreateDefaultResponse(request);
			break;
		case DeviceWasHalted :
			return CreateDefaultResponse(request);
			break;
		case DataPacketReceived :
			return CreateDefaultResponse(request);
			break;
		case DataPacketBrocken :
			return CreateDefaultResponse(request);
			break;
		case DataTransactionComplited :
			return CreateEndDataSend(request->GetAddress(), false);
			break;
		case BeginDataSend :
			return CreateDefaultResponse(request);
			break;
		case SendDataPacket :
			return CreateDefaultResponse(request);
			break;
		case EndDataSend :
			return CreateDefaultResponse(request);
			break;
		case NoMedium :
			return CreateDefaultResponse(request);
			break;
		case BeginRemoteControl :
			return CreateDefaultResponse(request);
			break;
		case RequestDeviceFile :
			return CreateDefaultResponse(request);
			break;
		case SaveDeviceFiles :
			return CreateDefaultResponse(request);
			break;
		case DeviceFileUpload :
			return CreateDefaultResponse(request);
			break;
		case CompliteRemoteControl :
			return CreateDefaultResponse(request);
			break;
		case SendDeviceFiles :
			return CreateDefaultResponse(request);
			break;
		default:
			return CreateDefaultResponse(request);
			break;
	}	
}

CommandEntity * CommandFactory::CreateDefaultResponse(CommandEntity * request)
{
	return new CommandEntity(request->GetAddress(), request->GetFunction(), 0, false, NULL, 0, 0, DefaultResponse);
}
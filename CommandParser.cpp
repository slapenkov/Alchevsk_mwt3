#pragma once
#include "stdafx.h"
#include <stdlib.h>
#include <string.h>
#include "CustomFunctionMapping.h"
#include "CommandParser.h"
#include "ControllerCommandsEnum.h"
#include "DataTransferCommandEnum.h"
#include "RemoteAccessCommandsEnum.h"
#include "DefaultCommandsEnum.h"

CommandEntity * CommandParser::Parse(unsigned char * rawcommand, unsigned int rawcommandlength)
{
    unsigned char address = rawcommand[0];
    unsigned char function = rawcommand[1];
    unsigned char subfunction = 0;
    bool hassubfunction = false;
    static unsigned char data[256];
    unsigned char datacount = 0;
	int identifier = DefaultResponse;

    switch(function)
    {
        case SMP3_CMODE :
            hassubfunction = true;
            subfunction = rawcommand[2];
            break;
        case SMP3_READ_BUFFER :
            hassubfunction = true;
            subfunction = rawcommand[2];
            break;
        case SMP3_GET_REGISTERS :
            hassubfunction = false;
            datacount = rawcommandlength - 4;
            break;
        case SMP3_SET_REGISTERS :
            hassubfunction = false;
            datacount = rawcommandlength - 4;
            break;
        case SMP3_WRITE_EEPROM :
            hassubfunction = false;
            break;
        case SMP3_SET_SPEED :
            hassubfunction = true;
            subfunction = rawcommand[2];
            break;
        case SMP3_SEND_ID :
            hassubfunction = false;
            break;
			// datatransfer
		case SCANCONTROLLER_FUNCTION:
			hassubfunction = false;
			identifier = ScanController;
			break;
		case REGISTERCONTROLLER_FUNCTION:
			hassubfunction = false;
			datacount = 1;
			identifier = RegisterController;
			break;
		case DEVICEWASHALTED_FUNCTION:
			hassubfunction = false;
			identifier = DeviceWasHalted;
			break;
		case REQUESTDATA_FUNCTION:
			if(rawcommandlength == 14)
			{
				hassubfunction = false;
				datacount = 10;
				identifier = RequiestData;
			}
			else if(rawcommandlength == 6)
			{
				hassubfunction = false;
				datacount = 2;
				identifier = rawcommand[2] == 0 && rawcommand[3] == 0 ? NoMedium : BeginDataSend;
			}
			break;
		case SENDDATAPACKET_FUNCTION:
			hassubfunction = false;
			datacount = rawcommandlength - 4;
			identifier = SendDataPacket;
			break;
		case DATATRANSACTIONCOMPLITED_FUNCTION:
			if(rawcommandlength == 4)
			{
				hassubfunction = false;
				identifier = DataTransactionComplited;
			}
			else if(rawcommandlength == 5)
			{
				subfunction = rawcommand[2];
				identifier = EndDataSend;
			}
			break;
		case BEGINREMOTECONTROL_FUNCTION:
			hassubfunction = false;			
			identifier = BeginRemoteControl;
			break;
		case SENDDEVICEFILES_FUNCTION:
			hassubfunction = false;
			datacount = 4;
			identifier = SendDeviceFiles;
			break;
		case REQUESTDEVICEFILE_FUNCTION:
			hassubfunction = false;
			datacount = 4;
			identifier = RequestDeviceFile;
			break;
		case SAVEDEVICEFILES_FUNCTION:
			hassubfunction = false;
			datacount = 4;
			identifier = SaveDeviceFiles;
			break;
		case DEVICEFILEUPLOAD_FUNCTION:
			hassubfunction = false;
			datacount = 4;
			identifier = DeviceFileUpload;
			break;
		case COMPLITEREMOTECONTROL_FUNCTION:
			hassubfunction = false;
			identifier = CompliteRemoteControl;
			break;
	}

    if(datacount > 0)
    {        
        memcpy(data, rawcommand + rawcommandlength - datacount - 2, datacount);
    }

	if(identifier == DefaultResponse)
	{
		return new CommandEntity(address, function, subfunction, hassubfunction, data, datacount, 0);
	}
	else
	{
		return new CommandEntity(address, function, subfunction, hassubfunction, data, datacount, 0, identifier);
	}
}

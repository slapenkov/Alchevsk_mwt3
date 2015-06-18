#pragma once
#include "stdafx.h"
#include <iostream>
#include <string.h>
#include "CommandResponseFactory.h"
#include "CustomFunctionMapping.h"
#include "CMODEEnum.h"
#include "ReadBufferEnum.h"
#include "SpeedEnum.h"
#include "SMP3RegisterEnum.h"
#include "Spm3ModmusMaster.h"
#include "CommandFactory.h"
#include "ControllerTypeEnum.h"

using namespace std;

CommandResponseFactory::CommandResponseFactory(const char * filename)
{
	CreateInternals();	
}

CommandResponseFactory::CommandResponseFactory(int dataDuration)
{
	CreateInternals();	
    this->dataDuration = dataDuration;
	this->dataDurationSpan = dataDuration * 10;	
}

CommandResponseFactory::~CommandResponseFactory()
{    
}

CommandEntity * CommandResponseFactory::Build(CommandEntity * request)
{
    static CommandEntity * response = new CommandEntity();

    unsigned char data[256];
    int datalength = 0;
    switch(request->GetFunction())
    {
        case SMP3_CMODE :
            if(request->HasSubfunction() && request->datalength == 0)
            {
                switch(request->GetSubFunction())
                {
                    case normal :
                    case oneCycle:
                    case reset_mcu:						
                        response = new CommandEntity(request->GetAddress(), request->GetFunction(), 0, false, NULL, 0, 0);
                        break;
                    default:
                        response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_value, true, NULL, 0, 0);
                        break;
                }
            }
            else
            {
                response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_address, true, NULL, 0, 0);
            }
            break;
        case SMP3_READ_BUFFER :
            if(request->HasSubfunction() && request->datalength == 0)
            {
                switch(request->GetSubFunction())
                {
                    case channel0:
                    case channel1:
                        datalength = 128;                        
                        memset(data, 0, datalength);
                        response = new CommandEntity(request->GetAddress(), request->GetFunction(), 0, false, data, datalength, 0);
                        break;
                    default:
                        response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_value, true, NULL, 0, 0);
                        break;
                }
            }
            else
            {
                response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_address, true, NULL, 0, 0);
            }
            break;
        case SMP3_GET_REGISTERS :
            if(request->datalength == 3)
            {
                unsigned char startAddressHight = request->data[0];
                unsigned char startAddressLow = request->data[1];
                unsigned char count = request->data[2];
                short startAddress = 0;
                unsigned char realBytesCount = 0;
                unsigned char * startData = NULL;

                datalength = 34;                

                //Spm3ModbusMaster::CalculateStartAddressAndSizeRegisters(data, startAddressHight, startAddressLow, count, startAddress, realBytesCount, startData);

                if(startAddress + request->data[2] <= 30)
                {
					int * channeldata = new int[datalength - 4];
                    //float * channeldata = new float[datalength - 4];
					int count = 0;					
					
					if(count !=  datalength - 4)
					{
						for(int i=0; i<datalength - 4; i++)
						{
							channeldata[i] = 0;
						}						

						if(!this->noData)
						{
							this->noData = true;
							this->NoData();
						}
					}

					

                    int index = 0;
                    for(int i =0; i < datalength - 4; i++)
                    {
                        //cout << i << ":" << channeldata[i] << endl;
                        if(i < 23 || i > 26)
                        {
                            data[index * 2] = (((int)channeldata[i] >> 8) & 0xFF);
                            data[index * 2 + 1] = ((int)channeldata[i] & 0xFF);
                            //cout << index * 2 << " : " << (int)data[index * 2] << endl;
                            //cout << index * 2 + 1 << " : " << (int)data[index * 2 + 1] << endl;
                            index++;
                        }
                        else
                        {
                            data[index * 2] = ((int)channeldata[i] >> 24) & 0xFF;
                            data[index * 2 + 1] = ((int)channeldata[i] >> 16) & 0xFF;
                            data[index * 2 + 2] = ((int)channeldata[i] >> 8) & 0xFF;
                            data[index * 2 + 3] = (int)channeldata[i] & 0xFF;

                            //cout << index * 2 << " : " << (int)data[index * 2] << endl;
                            //cout << index * 2 + 1 << " : " << (int)data[index * 2 + 1] << endl;
                            //cout << index * 2 + 2 << " : " << (int)data[index * 2 + 2] << endl;
                            //cout << index * 2 + 3 << " : " << (int)data[index * 2 + 3] << endl;
                            index+=2;
                        }
                    }

                    response = new CommandEntity(request->GetAddress(), request->GetFunction(), 0, false, startData, realBytesCount, 0);
                }
                else
                {
                    response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_value, true, NULL, 0, 0);
                }
            }
            else
            {
                response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_address, true, NULL, 0, 0);
            }
            break;
        case SMP3_SET_REGISTERS :
            if(request->datalength >= 5 && !((request->datalength - 3) & 0x01))
            {
                unsigned char startAddressHight = request->data[0];
                unsigned char startAddressLow = request->data[1];
                unsigned char count = request->data[2];
                short startAddress = 0;
                unsigned char realBytesCount = 0;
                unsigned char * startData = NULL;

                //Spm3ModbusMaster::CalculateStartAddressAndSizeRegisters(request->data + 3, startAddressHight, startAddressLow, count, startAddress, realBytesCount, startData);

                if(request->data[0] *0xFF + request->data[1] + request->data[2] <= 30 && request->datalength - 3 == realBytesCount)
                {
                    response = new CommandEntity(request->GetAddress(), request->GetFunction(), 0, false, NULL, 0, 0);
                }
                else
                {
                    response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_value, true, NULL, 0, 0);
                }
            }
            else
            {
                response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_address, true, NULL, 0, 0);
            }
            break;
        case SMP3_WRITE_EEPROM :
            if(request->datalength == 0)
            {
                response = new CommandEntity(request->GetAddress(), request->GetFunction(), 0, false, NULL, 0, 0);
            }
            else
            {
                response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_address, true, NULL, 0, 0);
            }
            break;
        case SMP3_SET_SPEED :
            if(request->HasSubfunction() && request->datalength == 0)
            {
                switch(request->GetSubFunction())
                {
                    case Speed_1200:
                    case Speed_1800:
                    case Speed_2400:
                    case Speed_4800:
                    case Speed_7200:
                    case Speed_9600:
                    case Speed_14400:
                    case Speed_19200:
                        response = new CommandEntity(request->GetAddress(), request->GetFunction(), 0, false, NULL, 0, 0);
                        break;
                    default:
                        response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_value, true, NULL, 0, 0);
                        break;
                }
            }
            else
            {
                response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_address, true, NULL, 0, 0);
            }
            break;
        case SMP3_SEND_ID :
            if(request->datalength == 0)
            {
                datalength = 13;                
                memcpy(data, "MVT-SPM3v1.00", datalength);
                response = new CommandEntity(request->GetAddress(), request->GetFunction(), 0, false, data, datalength, 0);
            }
            else
            {
                response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_data_address, true, NULL, 0, 0);
            }
            break;
		case SCANCONTROLLER_FUNCTION :
			response = commandFactory->CreateRegisterController(request->GetAddress(), HumidityController);
			break;
		case REGISTERCONTROLLER_FUNCTION :
			response = commandFactory->CreateDefaultResponse(request);
			break;
		case DEVICEWASHALTED_FUNCTION :
			response = commandFactory->CreateDefaultResponse(request);
			break;
		case REQUESTDATA_FUNCTION :
			response = commandFactory->CreateBeginDataSend(request->GetAddress(), this->dataDurationSpan);
			break;
		case SENDDATAPACKET_FUNCTION :
			response = commandFactory->CreateSendDataPacket(request->GetAddress(),0, 0, NULL, 0 );
			break;
		case DATATRANSACTIONCOMPLITED_FUNCTION :
			response = commandFactory->CreateEndDataSend(request->GetAddress(), true);
			break;
		case BEGINREMOTECONTROL_FUNCTION :
			response = commandFactory->CreateDefaultResponse(request);
			break;
		case SENDDEVICEFILES_FUNCTION :
			response = commandFactory->CreateDefaultResponse(request);
			break;
		case REQUESTDEVICEFILE_FUNCTION :
			response = commandFactory->CreateDefaultResponse(request);
			break;
		case SAVEDEVICEFILES_FUNCTION :
			response = commandFactory->CreateDefaultResponse(request);
			break;
		case DEVICEFILEUPLOAD_FUNCTION :
			response = commandFactory->CreateDefaultResponse(request);
			break;
		case COMPLITEREMOTECONTROL_FUNCTION :
			response = commandFactory->CreateDefaultResponse(request);
			break;
        default:
            //error response illegal_function
            response = new CommandEntity(request->GetAddress(), ERROR_FUNCTION, illegal_function, true, NULL, 0, 0);
            break;
    }
    return response;
}

void CommandResponseFactory::CreateInternals()
{
	this->noData = true;
	commandFactory = new CommandFactory();	
	response = new CommandEntity();
}
														
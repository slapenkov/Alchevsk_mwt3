#pragma once
#include "stdafx.h"
#include <iostream>
#include <windows.h>
#include "DataTransferMaster.h"
#include "CommandFactory.h"
#include "CommandParser.h"
#include "CommandEntity.h"
#include "DefaultCommandsEnum.h"

using namespace std;

DataTransferMaster::DataTransferMaster(DeviceTypeEnum deviceType, const char * device, int listenPeriod)
{	
	responseFactoy = new CommandResponseFactory(listenPeriod);
	this->listen = false;
	CustomFunctionMapping * mapping[15];
    mapping[0] = new CustomFunctionMapping(SCANCONTROLLER_FUNCTION, 0, SCANCONTROLLER_FUNCTION_REQUEST_DATA_LENGTH);
    mapping[1] = new CustomFunctionMapping(REGISTERCONTROLLER_FUNCTION, 0, REGISTERCONTROLLER_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[2] = new CustomFunctionMapping(DEVICEWASHALTED_FUNCTION, 0, DEVICEWASHALTED_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[3] = new CustomFunctionMapping(REQUESTDATA_FUNCTION, 0, REQUESTDATA_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[4] = new CustomFunctionMapping(SENDDATAPACKET_FUNCTION, 0, SENDDATAPACKET_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[5] = new CustomFunctionMapping(DATAPACKETRECEIVED_FUNCTION, 0, DATAPACKETRECEIVED_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[6] = new CustomFunctionMapping(DATATRANSACTIONCOMPLITED_FUNCTION, 0, DATATRANSACTIONCOMPLITED_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[7] = new CustomFunctionMapping(BEGINREMOTECONTROL_FUNCTION, 0, BEGINREMOTECONTROL_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[8] = new CustomFunctionMapping(SENDDEVICEFILES_FUNCTION, 0, SENDDEVICEFILES_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[9] = new CustomFunctionMapping(REQUESTDEVICEFILE_FUNCTION, 0, REQUESTDEVICEFILE_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[10] = new CustomFunctionMapping(SAVEDEVICEFILES_FUNCTION, 0, SAVEDEVICEFILES_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[11] = new CustomFunctionMapping(DEVICEFILEUPLOAD_FUNCTION, 0, DEVICEFILEUPLOAD_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[12] = new CustomFunctionMapping(COMPLITEREMOTECONTROL_FUNCTION, 0, COMPLITEREMOTECONTROL_FUNCTION_REQUEST_DATA_LENGTH);	

	this->provider = new ModbusProvider(deviceType, device, mapping, 13);	
	commandFactory= new CommandFactory();
}

DataTransferMaster::DataTransferMaster(const char * device, int listenPeriod)
{
	responseFactoy = new CommandResponseFactory(listenPeriod);
	this->listen = false;
	CustomFunctionMapping * mapping [15];
    mapping[0] = new CustomFunctionMapping(SCANCONTROLLER_FUNCTION, 0, SCANCONTROLLER_FUNCTION_REQUEST_DATA_LENGTH);
    mapping[1] = new CustomFunctionMapping(REGISTERCONTROLLER_FUNCTION, 0, REGISTERCONTROLLER_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[2] = new CustomFunctionMapping(DEVICEWASHALTED_FUNCTION, 0, DEVICEWASHALTED_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[3] = new CustomFunctionMapping(REQUESTDATA_FUNCTION, 0, REQUESTDATA_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[4] = new CustomFunctionMapping(SENDDATAPACKET_FUNCTION, 0, SENDDATAPACKET_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[5] = new CustomFunctionMapping(DATAPACKETRECEIVED_FUNCTION, 0, DATAPACKETRECEIVED_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[6] = new CustomFunctionMapping(DATATRANSACTIONCOMPLITED_FUNCTION, 0, DATATRANSACTIONCOMPLITED_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[7] = new CustomFunctionMapping(BEGINREMOTECONTROL_FUNCTION, 0, BEGINREMOTECONTROL_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[8] = new CustomFunctionMapping(SENDDEVICEFILES_FUNCTION, 0, SENDDEVICEFILES_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[9] = new CustomFunctionMapping(REQUESTDEVICEFILE_FUNCTION, 0, REQUESTDEVICEFILE_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[10] = new CustomFunctionMapping(SAVEDEVICEFILES_FUNCTION, 0, SAVEDEVICEFILES_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[11] = new CustomFunctionMapping(DEVICEFILEUPLOAD_FUNCTION, 0, DEVICEFILEUPLOAD_FUNCTION_REQUEST_DATA_LENGTH);
	mapping[12] = new CustomFunctionMapping(COMPLITEREMOTECONTROL_FUNCTION, 0, COMPLITEREMOTECONTROL_FUNCTION_REQUEST_DATA_LENGTH);
	this->provider = new ModbusProvider(RS485, device, mapping, 13);	
	commandFactory = new CommandFactory();
}

DataTransferMaster::~DataTransferMaster()
{

}

void DataTransferMaster::Open(HANDLE hCom2)
{
	DWORD dwThreadId;
	this->provider->Initialize(hCom2);	
    this->listen = true;

	if(CreateThread(NULL, //Choose default security
		0, //Default stack size
		(LPTHREAD_START_ROUTINE)&WatchCycle,
		//Routine to execute
		this, //Thread parameter
		0, //Immediately run the thread
		&dwThreadId //Thread Id
		) == NULL)
	 {
		 printf("ReadKeyThreadProc, Error create thread ThreadProc");
	 }	
}



void DataTransferMaster::Close()
{
	if(this->listen)
    {
        this->listen = false;		     
        this->provider->Close();
    }
}

void DataTransferMaster::SendResponse(CommandEntity * responseCommand)
{
	unsigned char rawresponse[256];
    int rawresponselength = 0;
    responseCommand->Build(rawresponse, rawresponselength);
	this->provider->modbus_send(rawresponse, rawresponselength);
}


void DataTransferMaster::WatchCycle(void * arg)
{
	DataTransferMaster * datatransfermaster = (DataTransferMaster *)arg;

	unsigned char request[256];
    unsigned char length = 256;
    int index = 0;
    cout<< "Start listen"<< endl;
    short oldcrc = -1;
	while(datatransfermaster->listen)
    {
        length = 256;

        datatransfermaster->provider->receive_msg(256, request, length);
        if(length > 0)
        {
            /*
            // resend message if not received
            short crc_received = (request[length - 2] << 8) | request[length - 1];
            if(oldcrc == crc_received)
            {
                deviceSemulator->provider->modbus_send(request, length);
                timespec * ts = new timespec();
                nanosleep(ModbusProvider::CreateDelay(COM_WAIT_TIMEOUT), ts);
                nanosleep(ts, NULL);
                oldcrc = -1;
                continue;
            }
            else
            {
                oldcrc = crc_received;
            }
            */
            cout << "Address : " << (int)request[0] << " function :" << (int)request[1] << " length :" << (int)length << endl;

            for(int i =0; i<length; i++)
            {
                cout << (int)request[i] << " ";
            }

            cout<< endl;
            cout.flush();

            CommandEntity * requestCommand = CommandParser::Parse(request, length);
			//CommandEntity * responseTemplateCommand = CommandFactory::CreateDefaultResponse(requestCommand);
			CommandEntity * responseTemplateCommand = datatransfermaster->responseFactoy->Build(requestCommand);

			if(responseTemplateCommand->identifier == DefaultResponse)
			{
				datatransfermaster->SendResponse(responseTemplateCommand);
			}
			else
			{
				datatransfermaster->CompliteTemplate(responseTemplateCommand);
			}
        }
	}
}

int DataTransferMaster::RetriveResponse(CommandEntity * entity)
{	
	CommandEntity * response = this->SendWithResponse(entity, this->provider);
    unsigned char rawcommand[256];
    int rawcommandlength;
    response->Build(rawcommand, rawcommandlength);
	CompliteTemplate(response);

    return rawcommandlength;
}

int DataTransferMaster::ScanController(unsigned char address)
{
	CommandEntity * entity =  commandFactory->CreateScanController(address);
    return RetriveResponse(entity);
}

int DataTransferMaster::RegisterController(unsigned char address, unsigned char deviceType)
{
	CommandEntity * entity =  commandFactory->CreateRegisterController(address,deviceType);
    return RetriveResponse(entity);
}

int DataTransferMaster::DeviceWasHalted(unsigned char address)
{
	CommandEntity * entity =  commandFactory->CreateDeviceWasHalted(address);
    return RetriveResponse(entity);
}

int DataTransferMaster::RequestData(unsigned char address, time_t startDate, int duration)
{
	CommandEntity * entity =  commandFactory->CreateRequiestData(address, startDate, duration);
    return RetriveResponse(entity);
}

int DataTransferMaster::SendDataPacket(unsigned char address, int offset, unsigned char count, unsigned char * data, int datalength)
{
	CommandEntity * entity =  commandFactory->CreateSendDataPacket(address, offset, count, data, datalength);
    return RetriveResponse(entity);
}

int DataTransferMaster::DataTransactionComplited(unsigned char address)
{
	CommandEntity * entity =  commandFactory->CreateDataTransactionComplited(address);
    return RetriveResponse(entity);
}

int DataTransferMaster::BeginRemoteControl(unsigned char address)
{
	CommandEntity * entity =  commandFactory->CreateBeginRemoteControl(address);
    return RetriveResponse(entity);
}

int DataTransferMaster::SendDeviceFiles(unsigned char address, int fileMask)
{
	CommandEntity * entity =  commandFactory->CreateSendDeviceFiles(address, fileMask);
    return RetriveResponse(entity);
}

int DataTransferMaster::RequestDeviceFile(unsigned char address, int fileCode)
{
	CommandEntity * entity =  commandFactory->CreateRequestDeviceFile(address, fileCode);
    return RetriveResponse(entity);
}

int DataTransferMaster::SaveDeviceFiles(unsigned char address, int fileMask)
{
	CommandEntity * entity =  commandFactory->CreateSaveDeviceFiles(address, fileMask);
    return RetriveResponse(entity);
}

int DataTransferMaster::DeviceFileUpload(unsigned char address, int fileCode)
{
	CommandEntity * entity =  commandFactory->CreateDeviceFileUpload(address, fileCode);
    return RetriveResponse(entity);
}

int DataTransferMaster::CompliteRemoteControl(unsigned char address)
{
	CommandEntity * entity =  commandFactory->CreateCompliteRemoteControl(address);
    return RetriveResponse(entity);
}
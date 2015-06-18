#pragma once
#include "stdafx.h"
#include "Spm3ModmusMaster.h"
#include "CustomFunctionMapping.h"
#include "CommandFactory.h"

Spm3ModbusMaster::Spm3ModbusMaster(unsigned char address,const char * device) //also provider
{
    this->address = address;

	CustomFunctionMapping * mapping [7];
    mapping[0] = new CustomFunctionMapping(SMP3_CMODE, 0, SMP3_CMODE_RESPONSE_DATA_LENGTH);
    mapping[1] = new CustomFunctionMapping(SMP3_READ_BUFFER, 0, SMP3_READ_BUFFER_RESPONSE_DATA_LENGTH);
    mapping[2] = new CustomFunctionMapping(SMP3_GET_REGISTERS, 0, SMP3_GET_REGISTERS_RESPONSE_DATA_LENGTH);
    mapping[3] = new CustomFunctionMapping(SMP3_SET_REGISTERS, 0, SMP3_SET_REGISTERS_RESPONSE_DATA_LENGTH);
    mapping[4] = new CustomFunctionMapping(SMP3_WRITE_EEPROM, 0, SMP3_WRITE_EEPROM_RESPONSE_DATA_LENGTH);
    mapping[5] = new CustomFunctionMapping(SMP3_SET_SPEED, 0, SMP3_SET_SPEED_RESPONSE_DATA_LENGTH);
    mapping[6] = new CustomFunctionMapping(SMP3_SEND_ID, 0, SMP3_SEND_ID_RESPONSE_DATA_LENGTH);
	this->provider.SetProperties(PIPE, device, mapping, 7);	
}

Spm3ModbusMaster::~Spm3ModbusMaster()
{    
}

void Spm3ModbusMaster::Open()
{
    this->provider.Initialize();
}

int Spm3ModbusMaster::CMODE(CMODEEnum mode)
{
    CommandEntity * entity =  commandFactory.CreateCMODE(this->address, mode);
    CommandEntity * response = this->SendWithResponse(entity, &this->provider);
    unsigned char rawcommand [256];
    int rawcommandlength;
    response->Build(rawcommand, rawcommandlength);

    return rawcommandlength;
}

int Spm3ModbusMaster::ReadBuffer(ReadBufferEnum channel)
{
    CommandEntity * entity =  commandFactory.CreateReadBuffer(this->address, channel);
    CommandEntity * response = this->SendWithResponse(entity, &this->provider);

    static unsigned char rawcommand [256];
    int rawcommandlength;
    response->Build(rawcommand, rawcommandlength);

    return rawcommandlength;
}

int Spm3ModbusMaster::GetRegisters(short startAddress, unsigned char count)
{
    CommandEntity * entity =  commandFactory.CreateGetRegisters(this->address, startAddress, count);
    CommandEntity * response = this->SendWithResponse(entity, &this->provider);

    static unsigned char rawcommand [256];
    int rawcommandlength;
    response->Build(rawcommand, rawcommandlength);

    return rawcommandlength;
}

int Spm3ModbusMaster::SetRegisters(short startAddress, unsigned char count, const unsigned char * registers, unsigned char registerslength)
{
    CommandEntity * entity =  commandFactory.CreateSetRegisters(this->address, startAddress, count, registers, registerslength);
    CommandEntity * response = this->SendWithResponse(entity, &this->provider);

    static unsigned char rawcommand [256];
    int rawcommandlength;
    response->Build(rawcommand, rawcommandlength);

    return rawcommandlength;
}

int Spm3ModbusMaster::WriteEEPROM()
{
    CommandEntity * entity =  commandFactory.CreateWriteToEEPROM(this->address);
    CommandEntity * response = this->SendWithResponse(entity, &this->provider);

    static unsigned char rawcommand [256];
    int rawcommandlength;
    response->Build(rawcommand, rawcommandlength);

    return rawcommandlength;
}

int Spm3ModbusMaster::SetSpeed(SpeedEnum mode)
{
    CommandEntity * entity =  commandFactory.CreateSetSpeed(this->address, mode);
    CommandEntity * response = this->SendWithResponse(entity, &this->provider);

    static unsigned char rawcommand [256];
    int rawcommandlength;
    response->Build(rawcommand, rawcommandlength);

    return rawcommandlength;
}

int Spm3ModbusMaster::SendId()
{
    CommandEntity * entity =  commandFactory.CreateSendId(this->address);
    CommandEntity * response = this->SendWithResponse(entity, &this->provider);

    static unsigned char rawcommand [256];
    int rawcommandlength;
    response->Build(rawcommand, rawcommandlength);

    return rawcommandlength;
}

void Spm3ModbusMaster::Close()
{
    this->provider.Close();
}

void Spm3ModbusMaster::CalculateStartAddressAndSizeRegisters(unsigned char * data, short startAddress, unsigned char count, unsigned char & realBytesCount, unsigned char * & startData)
{
    if(startAddress < 23)
    {
        startData = data +  2 * startAddress;
    }
    else if(startAddress >= 23 && startAddress <= 26)
    {
        startData = data + 23 * 2 + (27 - startAddress) * 4;
    }
    else
    {
        startData = data + startAddress * 2 + 4 * 2;
    }

    CalculateStartAddressAndSizeRegisters( startAddress, count, realBytesCount);
}

void Spm3ModbusMaster::CalculateStartAddressAndSizeRegisters(unsigned char * data, unsigned char startAddressHight, unsigned char startAddressLow, unsigned char count, short & startAddress, unsigned char & realBytesCount, unsigned char * & startData)
{
    startAddress = startAddressHight *0xFF + startAddressLow;

    CalculateStartAddressAndSizeRegisters(data, startAddress , count, realBytesCount, startData);
}

void Spm3ModbusMaster::CalculateStartAddressAndSizeRegisters(short startAddress, unsigned char count, unsigned char & realBytesCount)
{
    realBytesCount = count * 2;

    if(startAddress + count >= 23 && startAddress <= 26)
    {
        realBytesCount += 8;
        if(startAddress >=23 && startAddress <= 26)
        {
            realBytesCount -= 2*(startAddress -23);
        }

        if(startAddress + count <= 26)
        {
            realBytesCount -= 2*(27 - startAddress - count);
        }
    }
    else if(startAddress + count > 26)
    {
        realBytesCount += 8;
    }
}

#pragma once
#include <stdio.h>
#include "stdafx.h"
#include "CommandEntity.h"
#include "ModbusProvider.h"

CommandEntity::CommandEntity() : CommandHeader()
{
}

CommandEntity::CommandEntity(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction, unsigned char * data, int datalength, int responselength) : CommandHeader(address, function, subfunction, hasSubfunction)
{
	this->CreateInternals(data,datalength,responselength);
    this->identifier = -1;
}

CommandEntity::CommandEntity(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction, unsigned char * data, int datalength, int responselength, int identifier)  : CommandHeader(address, function, subfunction, hasSubfunction)
{
	this->CreateInternals(data,datalength,responselength);
	this->identifier = identifier;
}

void CommandEntity::SetValues(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction, unsigned char * data, int datalength, int responselength)
{	
	this->data = data;
	this->datalength = datalength;
	this->responselength = responselength;
}

void CommandEntity::SetValues(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction, unsigned char * data, int datalength, int responselength, int identifier)
{
	SetValues( address, function, subfunction, hasSubfunction, data, datalength, responselength);
	this->identifier = identifier;
}

void CommandEntity::CreateInternals(unsigned char * data, int datalength, int responselength)
{
	this->data = data;
    this->datalength = datalength;
    this->responselength = responselength;	
}

CommandEntity::~CommandEntity()
{
    
}

void CommandEntity::Build(unsigned char * rawcommand, int & rawcommandlength)
{
	int commandlengthWithoutCRC = 1 + 1 + (this->HasSubfunction() ? 1 : 0) + this->datalength;
    rawcommandlength = commandlengthWithoutCRC + 2;    
    rawcommand[0] = this->GetAddress();
    rawcommand[1] = this->GetFunction();
    int index = 2;
    if(this->HasSubfunction())
    {
        rawcommand[2] = this->GetSubFunction();
        index = 3;
    }

    for(int i = 0; i < this->datalength; i++)
    {
        rawcommand[index++] = this->data[i];
    }

    ModbusProvider::crc16('w', rawcommand, commandlengthWithoutCRC);
    printf("\r\nCommand length Without CRC %i\r\n", commandlengthWithoutCRC);
}
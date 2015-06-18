#pragma once
#include "stdafx.h"
#include "CommandHeader.h"

CommandHeader::CommandHeader()
{
}

CommandHeader::CommandHeader(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction)
{
    this->address = address;
    this->function = function;
    this->subfunction = subfunction;
    this->hasSubfunction = hasSubfunction;
}

CommandHeader::~CommandHeader()
{
}

unsigned char CommandHeader::GetAddress()
{
    return this->address;
}

unsigned char CommandHeader::GetFunction()
{
    return this->function;
}

bool CommandHeader::HasSubfunction()
{
    return this->hasSubfunction;
}

unsigned char CommandHeader::GetSubFunction()
{
    if(this->hasSubfunction)
    {
        return this->subfunction;
    }

    return 0;
}

void CommandHeader::SetFunction(unsigned char function)
{
	this->function = function;
}

void CommandHeader::SethasSubfunction(bool hasSubfunction)
{
	this->hasSubfunction = hasSubfunction;
}

void CommandHeader::SetSubFunction(unsigned char subfunction)
{
	this->subfunction = subfunction;
}

void CommandHeader::SetValues(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction)
{
	this->address = address;
    this->function = function;
    this->subfunction = subfunction;
    this->hasSubfunction = hasSubfunction;
}
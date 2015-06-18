#pragma once
#include "stdafx.h"
#include "CustomFunctionMapping.h"

CustomFunctionMapping::CustomFunctionMapping(unsigned char function, unsigned char headerlength, unsigned char datalength)
{
    this->function = function;
    this->headerlength = headerlength;
    this->datalength = datalength;
}

CustomFunctionMapping::~CustomFunctionMapping()
{
}

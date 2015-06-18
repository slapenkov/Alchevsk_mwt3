#pragma once
#include "stdafx.h"
#include "CustomFunctionResponse.h"


CustomFunctionResponse::CustomFunctionResponse(unsigned char address, unsigned char function, unsigned char subfunction, unsigned char * data, unsigned char datalength)
{
    this->address = address;
    this->function = function;
    this->subfunction = subfunction;
    this->data = data;
    this->datalength = datalength;
}

CustomFunctionResponse::~CustomFunctionResponse()
{    
}

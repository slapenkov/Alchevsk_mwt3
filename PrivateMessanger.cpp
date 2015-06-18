#pragma once
#include "stdafx.h"
#include "PrivateMessanger.h"
#include "CommandFactory.h"

CommandEntity * PrivateMessanger::SendWithResponse(CommandEntity * queryentity, ModbusProvider * provider)
{
    static unsigned char query[256];
    int querylength = 0;
    queryentity->Build(query, querylength);
    unsigned char response [256];
    unsigned char responselength = queryentity->responselength;
    unsigned char realResponselength = 0;
    provider->SendData(query, querylength, response, responselength, realResponselength);
    CommandEntity * responseentity = commandFactory->CreateResponse(response, realResponselength);
    return responseentity;
}

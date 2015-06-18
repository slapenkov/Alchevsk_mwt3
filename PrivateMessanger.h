#ifndef _PrivateMessanger
#define _PrivateMessanger

#include "CommandEntity.h"
#include "CommandFactory.h"
#include "ModbusProvider.h"

class PrivateMessanger
	{
		protected :
		CommandFactory * commandFactory;
	    protected :
	    CommandEntity * SendWithResponse(CommandEntity * queryentity, ModbusProvider * provider);
	};

#endif
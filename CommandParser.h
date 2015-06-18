#ifndef _CommandParser
#define _CommandParser

#include "CommandEntity.h"

class CommandParser
	{
	    public :
	    static CommandEntity * Parse(unsigned char * rawcommand, unsigned int rawcommandlength);
	};
#endif
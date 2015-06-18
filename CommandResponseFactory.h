#ifndef _CommandResponseFactory
#define _CommandResponseFactory

#include <stdio.h>
#include "CommandEntity.h"
#include "CommandFactory.h"

class CommandResponseFactory
	{
	    private :
	    unsigned char * buffer;
		bool noData;
		int dataDuration;
		int dataDurationSpan;
		CommandFactory * commandFactory;
		CommandEntity * response;

		public :
	    CommandResponseFactory(const char * filename);
		CommandResponseFactory(int dataDuration);
	    ~CommandResponseFactory();

		void (*NoData)(void);

	    CommandEntity *  Build(CommandEntity * request);

		private :
		void CreateInternals();
	};
#endif
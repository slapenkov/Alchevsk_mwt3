#ifndef _CommandEntity
#define _CommandEntity

#include "CommandHeader.h"

class CommandEntity : public CommandHeader
	{
	    public :
	    unsigned char * data;
	    int datalength;
	    int responselength;
		int identifier;
		CommandEntity();
	    CommandEntity(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction, unsigned char * data, int datalength, int responselength);
		CommandEntity(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction, unsigned char * data, int datalength, int responselength, int identifier);
	    ~CommandEntity();
		void SetValues(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction, unsigned char * data, int datalength, int responselength);
		void SetValues(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction, unsigned char * data, int datalength, int responselength, int identifier);

	    void Build(unsigned char * rawcommand, int & rawcommandlength);
		private:
		void CreateInternals(unsigned char * data, int datalength, int responselength);
	};
#endif
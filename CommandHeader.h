#ifndef _CommandHeader
#define _CommandHeader

class CommandHeader
	{
	    private :
	    unsigned char address;
	    unsigned char function;
	    bool hasSubfunction;
	    unsigned char subfunction;
	    public :

		CommandHeader();
	    CommandHeader(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction);
	    ~CommandHeader();

	    unsigned char GetAddress();
	    unsigned char GetFunction();
		void SetFunction(unsigned char function);
	    bool HasSubfunction();
		void SethasSubfunction(bool hasSubfunction);
	    unsigned char GetSubFunction();
		void SetSubFunction(unsigned char subfunction);
		void SetValues(unsigned char address, unsigned char function, unsigned char subfunction, bool hasSubfunction);

	};
#endif
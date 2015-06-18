#ifndef _CustomFunctionResponse
#define _CustomFunctionResponse

class CustomFunctionResponse
	{
	    public :
	    unsigned char address;
	    unsigned char function;
	    unsigned char subfunction;
	    unsigned char * data;
	    unsigned char datalength;

	    public :
	    CustomFunctionResponse(unsigned char address, unsigned char function, unsigned char subfunction, unsigned char * data, unsigned char datalength);
	    ~CustomFunctionResponse();

	};
#endif
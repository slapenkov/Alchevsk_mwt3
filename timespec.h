#ifndef _timespec
#define _timespec

struct timespec
	{
	public:
		timespec();
		timespec(unsigned long tv_sec, unsigned long tv_nsec);
		unsigned long tv_sec;
		unsigned long tv_nsec;
	};

#endif
#pragma once
#include "stdafx.h"
#include "timespec.h"

timespec::timespec()
{
}

timespec::timespec(unsigned long tv_sec, unsigned long tv_nsec)
{
	this->tv_sec = tv_sec;
	this->tv_nsec = tv_nsec;
}

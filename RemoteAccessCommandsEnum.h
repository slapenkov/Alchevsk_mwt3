#ifndef _RemoteAccessCommandsEnum
#define _RemoteAccessCommandsEnum

enum RemoteAccessCommandsEnum
	{
		BeginRemoteControl = 0x800,
		RequestDeviceFile = 0x1000,
		SaveDeviceFiles = 0x2000,
		DeviceFileUpload = 0x4000,
		CompliteRemoteControl = 0x8000,
		SendDeviceFiles = 0x10000
	};

#endif
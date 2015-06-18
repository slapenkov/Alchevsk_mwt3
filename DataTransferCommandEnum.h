#ifndef _DataTransferCommandEnum
#define _DataTransferCommandEnum

enum DataTransferCommandEnum
	{
		RequiestData = 0x8,
		DataPacketReceived = 0x10,
		DataPacketBrocken = 0x20,
		DataTransactionComplited = 0x40,
		BeginDataSend = 0x80,
		SendDataPacket = 0x100,
		EndDataSend = 0x200,
		NoMedium = 0x400
	};
#endif
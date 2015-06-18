#ifndef _DataTransferMaster
#define _DataTransferMaster

#include <windows.h>
#include "PrivateMessanger.h"
#include "CommandEntity.h"
#include "CommandResponseFactory.h"

//public delegate void CompliteTemplateEventHandler (void * sender, TypedEventArgs<CommandEntity *> ^ e);
	class DataTransferMaster : public PrivateMessanger
	{
		private :
			ModbusProvider * provider;
			volatile bool listen;			
			CommandResponseFactory * responseFactoy;
			HANDLE g_syncReceive;
		public:						
			void (*CompliteTemplate)(CommandEntity *);			
		public:			
			DataTransferMaster(DeviceTypeEnum deviceType, const char * device, int listenPeriod);
			DataTransferMaster(const char * device, int listenPeriod);
			~DataTransferMaster();

			void Open(HANDLE hCom2);
			int ScanController(unsigned char address);
			int RegisterController(unsigned char address, unsigned char deviceType);
			int DeviceWasHalted(unsigned char address);
			int RequestData(unsigned char address, time_t startDate, int duration);
			int SendDataPacket(unsigned char address, int offset, unsigned char count, unsigned char * data, int datalength);
			int DataTransactionComplited(unsigned char address);
			int BeginRemoteControl(unsigned char address);
			int SendDeviceFiles(unsigned char address, int fileMask);
			int RequestDeviceFile(unsigned char address, int fileCode);
			int SaveDeviceFiles(unsigned char address, int fileMask);
			int DeviceFileUpload(unsigned char address, int fileCode);
			int CompliteRemoteControl(unsigned char address);
			void Close();

			void SendResponse(CommandEntity * responseCommand);
			int RetriveResponse(CommandEntity * entity);
		private :
			static void WatchCycle(void * arg);
	};
#endif
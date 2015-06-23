#include "stdafx.h"
#include <stdio.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <tchar.h>
#include <string.h>
#include <direct.h>
#include <conio.h>
#include "MWTSoft2.h"
#include <windows.h>
#include <math.h>
#include <time.h>
#include "DataTransferMaster.h"
#include "DeviceTypeEnum.h"
#include "CommandEntity.h"
#include "DataTransferCommandEnum.h"
#include "ControllerCommandsEnum.h"

char Calc(); // Vichislenie vlajnosti
char WriteUBI();
char Write2Master();
char Aout(); //
char Reader(bool restore = false); //ctenie faylov nastroek
char Rabota(); // Regim raboty
char Graduirovka(); // Regim graduirovki
char Kalibrovka(); // Regim kalibrovki
char Nastrojka(); // Regim nastrojki
char PeriphSetup(); // Peripheral equipment setting screen
char NGrad(); // Regim nastrojki
char NSlug(); // Regim nastrojki
char Porog(); // Regim nastrojki
char Rezonans(); // Poisk rezonansa
bool NewMedium(); // New medium write
char SchablonNSlug(); // Regim nastrojki
char SchablonPorog(); // Regim nastrojki
char ComCMode(char UstrAdr,char UstrFuncArgument); // Ustanjvit regim raboti
char ComReadIntReg(char UstrAdr,char StartAdr, char Length); // Chtenie registrov
char ReadDataSPMCurrent(); // Chtenie tekuchih dannih SPM
char ReadDataSPMSet(); // Chtenie nastroek SPM
char ComWriteIntReg(char UstrAdr,char StartAdr, char Length); // Zapis registrov
char WriteDataSPMSet(); // Zapis nastroek SPM
char ReadKey(); // Chtenie klaviatury immediately
char ReadKeyBuffered(); // Chtenie klaviatury
//extern char ReadKey(); // Chtenie klaviatury
char EEPROMSave(char UstrAdr); // Zapis v EEPROM
char ClrScr(); // Ochistka ekrana
char SetCursor(byte X,byte Y);
char PrintXY(byte X, byte Y, byte length); // Pechat stroki simvolov
char SchablonRabota(); // Schablon ekrana raboty
char SchablonGrad(); // Schablon ekrana graduirovki
char SchablonKalibr(); // Schablon ekrana kalibrovki
char SchablonNastrojka(); // Schablon ekrana
char SchablonNGrad(); // Schablon ekrana
void SchablonKeyboard(char * title, char * buffer); //Schablon ekrana New medium
char ShablonPeriphSetup();	//Periph setup screen 
char CRCcount(char rw,byte Ln); // Vichislenie CRC
char IniCom1(int BaudRate); //Inicializacia Com1
char IniCom2(int BaudRate); //Inicializacia Com2

void SetPathDir(char * applicationName);
char * CombinePath(const char * firstPart, const char * secondPart);
void T2A(char * dest, TCHAR * source, int length);
void TimedIncrementT_Gen_T_Sr0_T_Sr1_W(bool setAverage, int t_Gen = 0, int t_Sr0 = 0, int t_Sr1 = 0, float w = 0.0f);
DWORD ReadKeyThreadProc (LPVOID lpdwThreadParam = NULL);
unsigned long ClockToMilliseconds(clock_t value);
char WaitNoKey();
char WaitKey();
char WaitAnyKey();
void WaitNoKeyImmediately();
void WaitKeyImmediately();
bool ShutdownServer();
bool CheckFilling();
void SleepWithService(long sleeptime);
void CheckShutdown();
char VirtualKeyboard(char * buffer);
enum FileType {Grad, Nastr, Kalibr, Set};
char * GetNumberedFileName(FileType type);
char * GetFullRow(char * buffer);
char * SetMediumRow(int newIndex = -1);
char * GetCenteredRow(char * buffer);

char * GetTranslatedFileName(FileType type, char * mediumName);
char * GetTranslatedString(char * source);
char * Trim(char * source);
char ReaderOld();
void SetDefaultValues();
void SaveGrad();
void SaveNastr();
void SaveKalibr();
void SaveSet();
void SaveAll();
bool ReadDefaultMedium();
void ReadMedium();
void SaveMedium();
void SetMedium(char * newMediumName);
bool HasSomeDefaultFiles();
void CompliteAndSendData(CommandEntity * entity);
void InitialiseBuffer(unsigned char * const buffer, int length);
//extern char IniMWT(); //Inicializacia pribora

byte ComDataWrite[260];
byte ComDataRead[260];
char Text[255];
int RegistrSPM_int[26];
float RegistrSPM_float[4];
int RegistrTM_int [11];
int Gamma0 = 0, Gamma1 = 0;    //Period mejdu pikami                     *0.1 mkS
int Gamma0_k = 0, Gamma1_k = 0;//Period mejdu pikami pri kalibrovke      *0.1 mkS
int Haw0 = 0, Haw1 = 0;        //Shirina impulsa na polovine amplitudy   *0.1 mkS
int Am0 = 0, Am1 = 0;          //Amplituda impulsa                       *1000 V
int Haw0_k = 0, Haw1_k = 0;    //Shirina impulsa na polovine amplitudy   *0.1 mkS
//pri kalibrovke
int Am0_k = 0, Am1_k = 0;      //Amplituda impulsa pri kalibrovke        *1000 V
int T_Gen = 0, T_Gen_k = 0;    //Temperatura generatora                  *100  C
int T_Sr0 = 0, T_Sr1 = 0;      //Temperatura sredy                       *100  C
int T_Sr0_k = 0;           //Temperatura sredy pri kalibrovke        *100  C
int Ain0 = 0, Ain1 = 0;        //Tekuchee znachenie na analogovom vhode  *1000 V
int Saw_A0 = 0;            //Amplituda podporki                      *1000 V
int Saw_A1 = 0;            //Amplituda pily                          *1000 V
int Ku0 = 0, Ku1 = 0;          //Koefficient usilenija                   2^x
int Taq = 0;               //Period sbora dannih                     x*20mkS
int Fwdt = 0;              //Shirina okna filtra
int Pcount = 0;            //Kol-vo periodov usrednenija
int BaudRateSPM = 0;       //Skorost obmena SPM
int HostSPM = 0;           //Adres SPM
int ValidReqSPM = 0;       //Schetchik uspeshnih zaprosov SPM
int CRCReqSPM = 0;         //Schetchik error CRC SPM
int ExcReqSPM = 0;         //Schetchik error obrabotki SPM
int VO = 0, VZ = 0, VI = 0, AK = 0, Tak = 0;
unsigned int Aout_tip = 0;			//out type
unsigned int DacOffs=0;				//DAC offset
unsigned int DacScale=4095;			//DAC scale
unsigned int PotVal=0x40;			//Digital potentiometer value
float W = 0, AA = 0, BB = 0, a0 = 0, a1 = 0, a2 = 0, a3 = 0,a4 = 0,a5 = 0,a6 = 0,a7 = 0,a8 = 0,a9 = 0,k1 = 0,Wmin = 0,Wmax = 0;
float Va = 0, kgt = 0, kgv = 0, T = 0,tgn = 0,tsn = 0; //delta_gamma; tgn;
float delta_gamma = 0, delta_gamma2 = 0;
int Kol_dat = 0, Nom_dat1 = 0, Nom_dat2 = 0, Nom_ind = 0;
float N_Rez = 0, K_Rez = 0, Step_Rez = 0;
int UstRele = 0;
byte Rel = 0, Rele3 = 0, Rele3_old = 0;
int Nzp = 0;
float Poralfa = 0, Porbeta = 0, Porgamma = 0;
long int xtim = 0, xtim_n = 0;
byte Regim = 0;
HANDLE hCom1 = NULL,hCom2 = NULL;

//LPDWORD lpEvtMask;

char applicationPath[256];
volatile char keyBuffer = 0;
volatile long servicetime = 100;

CRITICAL_SECTION * g_LockData;

unsigned char controlleradr = 0x001;
const int MBSIZE = 4;
unsigned char masterbuffer[4];
bool dataSent = true;

#define MEAN_TIME 12000 // Mean time is 12 sec

#define PATH_SEPARATOR "\\"

#define READKEY_INTERVAL 100

#define NET_ZAPOLNENIYA_PING_PERIOD 60*1000

#ifndef SHTDN_REASON_MAJOR_OTHER
#define SHTDN_REASON_MAJOR_OTHER 0
#endif
static const char * abc_ru = {"абвгде жзийклмнопрстуфхцчшщъыьэюя1234567890"};
static const char * notation[] = {"a", "b", "v", "g", "d", "ye", "zh", "z", "i", "y", "k", "l", "m", "n", "o", "p", "r", "s", "t", "u", "f", "kh", "ts", "ch", "sh", "sch", "tz", "y", "mz", "e", "yu", "ya", "x", "yy", "iy", "yo"};
static const char * emptyrow = "                    ";
const int ROW_WIDTH = 21;
char mediumName[ROW_WIDTH];
const int MEDIUM_COUNT = 10;
char mediums[MEDIUM_COUNT][ROW_WIDTH];
int media_count = 0;
int NomGrad = 0;
char fullrow[ROW_WIDTH];
const bool IS_EXPERIMENT = true;
char trimStr[ROW_WIDTH];
char newMediumStr[ROW_WIDTH];

DataTransferMaster master(RS485, "COM2", 12 * 1000);

int _tmain(int argc, _TCHAR* argv[])
{
	for(int i =0; i<MEDIUM_COUNT; i++) mediums[i][0] = 0;
	memset(&trimStr[0], 0, sizeof(trimStr));
	memset(&fullrow[0], 0, sizeof(fullrow));
	memset(&masterbuffer[0], 0, sizeof(masterbuffer));
	memset(&RegistrTM_int[0], 0, sizeof(RegistrTM_int));
	memset(&RegistrSPM_float[0], 0, sizeof(RegistrSPM_float));
	memset(&RegistrSPM_int[0], 0, sizeof(RegistrSPM_int));
	memset(&Text[0], 0, sizeof(Text));
	memset(&ComDataRead[0], 0, sizeof(ComDataRead));
	memset(&ComDataWrite[0], 0, sizeof(ComDataWrite));
	memset(&newMediumStr[0], 0, sizeof(newMediumStr));
	memset(&mediumName[0], 0, sizeof(mediumName));



	InitialiseBuffer(masterbuffer, MBSIZE);	
	strcpy(mediumName, emptyrow);
	mediumName[strlen(emptyrow)] = 0;



	g_LockData = new CRITICAL_SECTION();
	InitializeCriticalSection(g_LockData);

	int osize = 256;
	TCHAR obuf[256];
	osize = GetModuleFileName(NULL, obuf, osize);
	char applicationName[256];
	T2A(applicationName, obuf, osize);
	SetPathDir(applicationName);

	//char * binarypath = new char[256];
	//char * messagepath = new char[256];

	char readBuffer = 'Z';

	DWORD dwThreadId;

	if(CreateThread(NULL, //Choose default security
		0, //Default stack size
		(LPTHREAD_START_ROUTINE)&ReadKeyThreadProc,
		//Routine to execute
		NULL, //Thread parameter
		0, //Immediately run the thread
		&dwThreadId //Thread Id
		) == NULL)
	{
		//printf("ReadKeyThreadProc, Error create thread ThreadProc");
	}




	char tempchar;
	int tempcount=0;
	//Inicializacia pribora
	//NomUBI=10;
lab1:
	tempchar=IniCom1(19200);
	if (tempchar!=0) {tempcount++;if (tempcount<5) goto lab1;};

	tempchar=IniCom2(19200);
	if (tempchar!=0) {tempcount++;if (tempcount<5) goto lab1;};
	master.Open(hCom2);
	master.CompliteTemplate = &CompliteAndSendData;
	SleepWithService(1000);
	tempchar=ComCMode(0x2,0x2);
	SleepWithService(1000);
	tempchar=ComCMode(0x1,0x0);
	SleepWithService(1000);
	//AA=0;
	//BB=1;
	//a0=1234.5678;
	//a1=1;
	//a2=1;
	//a3=1;
	//a4=4; a5=5; a6=6; a7=7; a8=8; a9=9; k1=1; Wmin=10; Wmax=80;

	//Write2Master();

	//Am0_k=20000; Haw0_k=30000; Gamma0_k=40000;
	//T_Gen_k=2100; T_Sr0_k=2300;
	//UstRele=123;
	//VO=5;VZ=12345;VI=5; AK=0; Tak=100;
	//Aout_tip=2;  NomGrad=1;
	//Va=1234.567;kgt=1234.567;kgv=1234.567;T=1234.567;
	//N_Rez=3.0; K_Rez=6.5; Step_Rez=0.1;

	//WriteDataSPMSet();
	////printf("Test1"); //printf(" \n");

	if(!ReadDefaultMedium())
	{
		Reader(true);
		NewMedium();
		if(!ReadDefaultMedium())
		{
			SetMedium("по умолчанию");
			if(!HasSomeDefaultFiles())
			{
				SetDefaultValues();
			}
			SaveAll();
		}
	}
	else
	{
		Reader();
	}

	Va=Saw_A1;
	//printf("Saw_A0 =");printf("%u",Saw_A0);printf(" \n");
	////printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
	//SleepWithService(10000);

	////printf("Test1"); printf(" \n");
	//Ku0=2;Ku1=2;
	//Saw_A0=3000;Saw_A1=2000;
	WriteDataSPMSet();SleepWithService(500);  //////////////////////////
	////printf("Test1"); printf(" \n");
	ReadDataSPMSet();SleepWithService(500);

	//AA=60; Aout_tip=1;
	//printf("Ku0 =");printf("%u",Ku0);printf(" \n");
	//printf("Ku1=");printf("%u",Ku1);printf(" \n");
	//printf("Saw_A0 =");printf("%u",Saw_A0);printf(" \n");
	//printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
	//SleepWithService(10000);

	Regim='1';
	do {
Reg:
		//printf("Regim=");printf("%c",Regim);printf(" \n");
		if (Regim=='1') {ClrScr();SleepWithService(1000);SchablonRabota();
		Regim=Rabota();
		goto Reg;};
		if (Regim=='3') {ClrScr();SleepWithService(1000);SchablonGrad();Regim=Graduirovka();goto Reg;};
		if (Regim=='5') {ClrScr();SleepWithService(1000);SchablonKalibr();Regim=Kalibrovka();goto Reg;};
		if (Regim=='7') {ClrScr();SleepWithService(1000);
		SchablonNastrojka();
		Regim=Nastrojka();
		goto Reg;};

		//   tempchar=ReadKey();
		//printf("%c",tempchar);printf("  ");
		SleepWithService(50);
		Regim=	ReadKeyBuffered();
	} while (true);

	/*
	InterlockedExchangeAdd(readkeyStarted, -1);
	WaitForSingleObject(&dwThreadId, INFINITE);
	CloseHandle(&g_CalculationFinishedEvent);
	CloseHandle(&g_ReadKeyEvent);
	CloseHandle(&dwThreadId);
	master.Close();

	DeleteCriticalSection(g_WandT_Calc);
	DeleteCriticalSection(g_ReadKey);
	*/
}

char ReadKeyBuffered()
{
	char currentKey = keyBuffer;
	if(currentKey == 'Z' || currentKey == 0)
	{
		keyBuffer = currentKey = ReadKey();

		CheckShutdown();	
	}
	keyBuffer = 'Z';	
	return currentKey;
}

////////////////////////////
char ReadKey() // Chtenie klaviatury
{
	clock_t starttime = clock();	 	 
	int Keyint = 0;
	byte Keychar = 0;
	ComReadIntReg(0x2,0x30,0x1);
	Keyint=ComDataRead[2]*256+ComDataRead[3];
	ComDataRead[2]=0;ComDataRead[3]=0;
	switch(Keyint) {
	case 16:Keychar='0';break;
	case 2:Keychar='1';break;
	case 32:Keychar='2';break;
	case 512:Keychar='3';break;
	case 4:Keychar='4';break;
	case 64:Keychar='5';break;
	case 1024:Keychar='6';break;
	case 8:Keychar='7';break;
	case 128:Keychar='8';break;
	case 2048:Keychar='9';break;
	case 1:Keychar='S';break;
	case 256:Keychar='C';break;
	case 32768:Keychar='*';break;
	case 16384:Keychar='-';break;
	case 8192:Keychar='+';break;
	case 4096:Keychar='=';break;
	case 32785:Keychar='H';break;
	case 0:Keychar='Z';break;
	};	 

	clock_t finishtime = clock();
	servicetime = finishtime - starttime;
	if(servicetime == 0)
	{
		servicetime = 100;
	}

	//char * buffer = new char[256];
	//WriteLog(itoa(finishtime - starttime,buffer, 10));
	//delete [] buffer;
	return Keychar;
}
/////////////////////////////


////////////////////////////
char Calc() // Vichislenie vlajnosti

{
	float difftime0;
	tgn=a8;tsn=a9;
	////printf("kgv=");printf("%f",kgv);printf(" \n");
	//printf("kgt=");printf("%f",kgt);printf(" \n");
	//SleepWithService(5000);
	if (kgv==0) kgv=1;if (Va==0) Va=1;
	delta_gamma =(kgt/kgv)*(T/Va)*(T_Gen-T_Gen_k);
	delta_gamma2=k1*(T_Sr0-tsn);
	difftime0=(Gamma0-Gamma0_k);
	//difftime0=Gamma0/1000;
	difftime0=difftime0+delta_gamma+delta_gamma2;
	//printf("Am0=");printf("%u",Am0);printf(" \n");
	W=AA+BB*(a0+a1*(Am0-Am0_k)*10+a2*(Haw0-Haw0_k)/1000+a3*(Gamma0-Gamma0_k)/1000+a4*(T_Gen-T_Gen_k)/100+a5*(T_Sr0-T_Sr0_k)/100);
	printf("W=");printf("%f",W);printf(" \n");

	//SleepWithService(6000);
	return 0;
}
/////////////////////////////

////////////////////////////
char Kalibrovka() // Regim kalibrovki

{


	int i;
	//char len;
	//char pos=0;
	byte Key;
	byte tempchar;
	float y,z;

	clock_t mean_time = clock();

Kalibr:
	byte PosX=0xA;
	byte PosY=0x10;

	SetCursor(0xA,0x10);
	Key=0x0;
	if (VO!=0)
	{//SetCursor(0xA,0xF);
		ComReadIntReg(0x2,0x31,0x1);
		Rel=ComDataRead[3];
		Rel=Rel ^  0x01;
		RegistrTM_int[1]=Rel;
		ComWriteIntReg(0x2,0x31,0x1);
		strcpy(Text,"Очистка   "); tempchar=PrintXY(0,3,10);
		xtim_n=clock();
		do {xtim=clock();
		tempchar=ReadKeyBuffered();
		if (tempchar=='1'||tempchar=='3'||tempchar=='5'||
			tempchar=='7'||tempchar=='9') {Key=tempchar;};
		if (IS_EXPERIMENT && Key=='1'||Key=='3'||Key=='7') {goto Exk;};

		} while ((xtim/1000-xtim_n/1000) <VO);
		Rel=Rel ^ 0x01;
		RegistrTM_int[1]=Rel;
		ComWriteIntReg(0x2,0x31,0x1);
		strcpy(Text,"          "); tempchar=PrintXY(0,3,10);
		//SetCursor(0xA,0x10);
	};
	SleepWithService(1000);
	strcpy(Text,"Измерение "); tempchar=PrintXY(0,3,10);
	SleepWithService(50);
	tempchar=ReadDataSPMCurrent();
	Calc();
	WriteUBI();

	dataSent = true;   

	Aout();
	if (W>(((float)UstRele)/10)){Rele3=1;} else {Rele3=0;};
	if (Rele3 !=Rele3_old){Rel=Rel^0x04;};
	RegistrTM_int[1]=Rel;
	ComWriteIntReg(0x2,0x31,0x1);
	Rele3_old=Rele3;

	printf("W=");printf("%f",W);printf(" \n");
	printf("Tgen=");printf("%u",T_Gen);printf(" \n");
	printf("Tsr=");printf("%u",T_Sr0);printf(" \n");
	printf("T3=");printf("%u",T_Sr1);printf(" \n");
	printf("Alfa0=");printf("%u",Am0);printf(" \n");
	printf("Beta0=");printf("%u",Haw0);printf(" \n");
	printf("Gamma0=");printf("%u",Gamma0);printf(" \n");

	//y=W; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	//Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	//tempchar=PrintXY(14,4,5);
	y=Am0_k*10; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
	// Text[0]=Text[1];Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(7,5,5);
	y=Haw0_k; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	tempchar=PrintXY(7,6,5);
	y=Gamma0_k; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	tempchar=PrintXY(7,7,5);
	//
	y=Am0*10; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
	//Text[0]=Text[1];Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(7,9,5);
	y=Haw0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	tempchar=PrintXY(7,10,5);
	y=Gamma0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	tempchar=PrintXY(7,11,5);
	strcpy(Text,"          "); tempchar=PrintXY(0,3,10);

	if (T_Gen_k<0) {Text[0]='-';} else {Text[0]='+';};
	y=T_Gen_k; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(15,5,5);


	if (T_Sr0_k<0) {Text[0]='-';} else {Text[0]='+';};
	y=T_Sr0_k; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(15,6,5);


	//
	if (T_Gen<0) {Text[0]='-';} else {Text[0]='+';};
	y=T_Gen; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(15,9,5);


	if (T_Sr0<0) {Text[0]='-';} else {Text[0]='+';};
	y=T_Sr0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(15,10,5);

	//printf("%c", Key);printf(" \n");SleepWithService(5000);
	do {

		if (Key=='9'||Key=='3'||Key=='5'||Key=='7'||Key=='1')
		{	  
			if (Key=='5') {Key=ReadKeyBuffered();goto Kalibr;};
			if (Key=='9') {
				SaveKalibr();				   
				SleepWithService(100); Reader();Va=Saw_A1;Key=WaitNoKey();
				strcpy(Text,"Save Ok"); tempchar=PrintXY(11,1,7);SleepWithService(1000);
				strcpy(Text,"       "); tempchar=PrintXY(11,1,7);SetCursor(PosX,PosY);
			}; //Save
			if (Key=='1'||Key=='3'||Key=='7') {goto Exk;};
		};
		Key=WaitKey();
		////printf("%c", Key);printf(" \n");
	} while (Key=='1'||Key=='3'||Key=='5'||Key=='7'||Key=='9');

Exk:
	return Key;
}
/////////////////////////////


////////////////////////////
char Rabota() // Regim raboty

{
	int i;
	//char len;
	//char pos=0;
	byte  Key = 0;
	char tempchar;
	float y,z;

	clock_t mean_time = clock();

	SetCursor(0xA,0x10);
Rab:
	if (VO!=0)
	{//SetCursor(0xA,0xF);
		ComReadIntReg(0x2,0x31,0x1);
		Rel=ComDataRead[3];
		Rel=Rel ^  0x01;
		RegistrTM_int[1]=Rel;
		ComWriteIntReg(0x2,0x31,0x1);
		strcpy(Text,"Очистка   "); tempchar=PrintXY(0,3,10);
		xtim_n=clock();
		do {xtim=clock();
		tempchar=ReadKeyBuffered();
		if (tempchar=='3'||tempchar=='5'||
			tempchar=='7') {Key=tempchar;};
		if (IS_EXPERIMENT && Key=='3'||Key=='5'||Key=='7') {goto Exr;};
		} while ((xtim/1000-xtim_n/1000) <VO);
		Rel=Rel ^ 0x01;
		RegistrTM_int[1]=Rel;
		ComWriteIntReg(0x2,0x31,0x1);
		strcpy(Text,"          "); tempchar=PrintXY(0,3,10);
		//SetCursor(0xA,0x10);
	};
	SleepWithService(1000);
	if (VZ!=0)
	{//SetCursor(0xA,0xF);
		ComReadIntReg(0x2,0x31,0x1);
		Rel=ComDataRead[3];
		Rel=Rel ^  0x02;
		//SleepWithService(2000);
		RegistrTM_int[1]=Rel;
		tempchar=ComWriteIntReg(0x2,0x31,0x1);
		////printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
		strcpy(Text,"Загрузка  "); tempchar=PrintXY(0,3,10);
		////printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
		xtim_n=clock();
		do {xtim=clock();
		tempchar=ReadKeyBuffered();
		if (tempchar=='3'||tempchar=='5'||
			tempchar=='7') {Key=tempchar;};
		if (IS_EXPERIMENT && Key=='3'||Key=='5'||Key=='7') {goto Exr;};

		} while ((xtim/1000-xtim_n/1000) <VZ);
		Rel=Rel ^ 0x02;
		RegistrTM_int[1]=Rel;
		ComWriteIntReg(0x2,0x31,0x1);
		////printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
		strcpy(Text,"          "); tempchar=PrintXY(0,3,10);
		////printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
		//SetCursor(0xA,0x10);
	};


	xtim_n=clock();
	do
	{
		xtim=clock();
		tempchar=ReadKeyBuffered();
		if (tempchar=='3'||tempchar=='5'||tempchar=='7')
		{
			Key=tempchar;
		};
		if (IS_EXPERIMENT && Key=='3'||Key=='5'||Key=='7') {goto Exr;};


		strcpy(Text,"Измерение "); tempchar=PrintXY(0,3,10);
		SleepWithService(servicetime);

		tempchar=ReadDataSPMCurrent();
		//printf("Test1"); printf(" \n");
		Calc();
		//Write2Master();
		TimedIncrementT_Gen_T_Sr0_T_Sr1_W(false, T_Gen, T_Sr0, T_Sr1, W);
		//printf("Test2"); printf(" \n");

		WriteUBI();
		//printf("Test3"); printf(" \n");printf("Test4"); printf(" \n");
		//printf("W=");printf("%f",W);printf(" \n");
		//printf("Wmin=");printf("%f",Wmin);printf(" \n");
		//printf("Wmax=");printf("%f",Wmax);printf(" \n");
		//printf("T3=");printf("%u",T_Sr1);printf(" \n");
		//printf("Alfa0=");printf("%u",Am0);printf(" \n");
		//printf("Beta0=");printf("%u",Haw0);printf(" \n");
		//printf("Gamma0=");printf("%u",Gamma0);printf(" \n");
		//SleepWithService(5000);

		if (CheckFilling() &&W>=Wmin && W<=Wmax)
		{
			y=W*1000; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
			Text[0]=Text[1];Text[1]=Text[2];
			tempchar=PrintXY(14,5,2);
			Text[0]='.';
			Text[1]=Text[3];
			Text[2]=Text[4];
			tempchar=PrintXY(16,5,3);

		}
		else if (W<Wmin) {strcpy(Text,"<Wmin     ");  tempchar=PrintXY(11,5,10);}
		else if (W>Wmax) {strcpy(Text,">Wmax     ");  tempchar=PrintXY(11,5,10);};

		if (T_Sr0<0) {Text[0]='-';} else {Text[0]='+';};
		y=T_Sr0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
		Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
		tempchar=PrintXY(13,8,6);
		strcpy(Text,"          "); tempchar=PrintXY(0,3,10);

		strcpy(Text,"              ");
		tempchar=PrintXY(3,4,14);
		if(!CheckFilling())
		{
			strcpy(Text,"           "); tempchar=PrintXY(14,5,5);
			strcpy(Text,"Нет заполнения");
			tempchar=PrintXY(3,4,14);
		}


	} while ((xtim/1000-xtim_n/1000) <VI);

	TimedIncrementT_Gen_T_Sr0_T_Sr1_W(true);
	if(!CheckFilling())
	{
		W = 0.0f;
	}


	Aout();

	if (W>(((float)UstRele)/10)){Rele3=1;} else {Rele3=0;};
	//printf("UstRele=%f \n",((float)UstRele)/10);
	if (Rele3 !=Rele3_old){Rel=Rel^0x04;};
	RegistrTM_int[1]=Rel;
	ComWriteIntReg(0x2,0x31,0x1);
	Rele3_old=Rele3;

	//printf("%c", Key);printf(" \n");SleepWithService(5000);
	tempchar=ReadKeyBuffered();
	if (tempchar=='3'||tempchar=='5'||tempchar=='7') {Key=tempchar;};
	if (Key=='3'||Key=='5'||Key=='7') {goto Exr;};

	goto Rab;

Exr:
	return Key;
}
/////////////////////////////


////////////////////////////
char Graduirovka() // Regim graduirovki
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC
	//          13-err adr komandy


{
	int i;
	//char len;
	//char pos=0;
	byte Key=0;
	char tempchar;
	float y,z;

	clock_t mean_time = clock();

Grad:
	if (VO!=0)
	{SetCursor(0xA,0xF);
	ComReadIntReg(0x2,0x31,0x1);
	Rel=ComDataRead[3];
	Rel=Rel ^  0x01;
	RegistrTM_int[1]=Rel;
	ComWriteIntReg(0x2,0x31,0x1);
	strcpy(Text,"Очистка   "); tempchar=PrintXY(0,3,10);
	xtim_n=clock();
	do {xtim=clock();
	tempchar=ReadKeyBuffered();
	if (tempchar=='1'||tempchar=='5'||
		tempchar=='7'||tempchar=='9') {Key=tempchar;};
	if (IS_EXPERIMENT && Key=='1'||Key=='5'||Key=='7') {goto Exg;};

	} while ((xtim/1000-xtim_n/1000) <VO);
	Rel=Rel ^ 0x01;
	RegistrTM_int[1]=Rel;
	ComWriteIntReg(0x2,0x31,0x1);
	strcpy(Text,"          "); tempchar=PrintXY(0,3,10);
	SetCursor(0xA,0x10);
	};
	SleepWithService(1000);
	if (VZ!=0)
	{SetCursor(0xA,0xF);
	ComReadIntReg(0x2,0x31,0x1);
	Rel=ComDataRead[3];
	Rel=Rel ^  0x02;
	//SleepWithService(2000);
	RegistrTM_int[1]=Rel;
	tempchar=ComWriteIntReg(0x2,0x31,0x1);
	////printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
	strcpy(Text,"Загрузка  "); tempchar=PrintXY(0,3,10);
	////printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
	xtim_n=clock();
	do {xtim=clock();
	tempchar=ReadKeyBuffered();
	if (tempchar=='1'||tempchar=='5'||
		tempchar=='7'||tempchar=='9') {Key=tempchar;};
	if (IS_EXPERIMENT && Key=='1'||Key=='5'||Key=='7') {goto Exg;};

	} while ((xtim/1000-xtim_n/1000) <VZ);
	Rel=Rel ^ 0x02;
	RegistrTM_int[1]=Rel;
	ComWriteIntReg(0x2,0x31,0x1);
	////printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
	strcpy(Text,"          "); tempchar=PrintXY(0,3,10);
	////printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
	SetCursor(0xA,0x10);
	};


	//SleepWithService(1000);
	xtim_n=clock();
	do {xtim=clock();
	tempchar=ReadKeyBuffered();
	if (tempchar=='1'||tempchar=='5'||
		tempchar=='7'||tempchar=='9') {Key=tempchar;};
	if (IS_EXPERIMENT && Key=='1'||Key=='5'||Key=='7') {goto Exg;};

	strcpy(Text,"Измерение "); tempchar=PrintXY(0,3,10);
	SleepWithService(50);
	tempchar=ReadDataSPMCurrent();
	Calc();
	WriteUBI();
	TimedIncrementT_Gen_T_Sr0_T_Sr1_W(false, T_Gen, T_Sr0, T_Sr1, W);

	//printf("W=");printf("%f",W);printf(" \n");
	//printf("Tgen=");printf("%u",T_Gen);printf(" \n");
	//printf("Tsr=");printf("%u",T_Sr0);printf(" \n");
	//printf("T3=");printf("%u",T_Sr1);printf(" \n");
	//printf("Alfa0=");printf("%u",Am0);printf(" \n");
	printf("Beta0=");printf("%u",Haw0);printf(" \n");
	printf("Gamma0=");printf("%u",Gamma0);printf(" \n");

	y=W*1000; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	tempchar=PrintXY(14,4,5);

	if (T_Gen<0) {Text[0]='-';} else {Text[0]='+';};
	y=T_Gen; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(13,6,6);

	if (T_Sr0<0) {Text[0]='-';} else {Text[0]='+';};
	y=T_Sr0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(13,7,6);

	if (T_Sr1<0) {Text[0]='-';} else {Text[0]='+';};
	y=T_Sr1; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[1]=Text[2];Text[2]=Text[3];Text[3]='.';
	tempchar=PrintXY(13,8,6);


	//
	y=Am0*10; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
	//Text[0]=Text[1];Text[1]=Text[2];Text[2]=Text[3];
	//Text[3]='.';
	tempchar=PrintXY(14,10,5);
	y=Haw0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	tempchar=PrintXY(14,11,5);
	y=Gamma0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	tempchar=PrintXY(14,12,5);
	strcpy(Text,"          "); tempchar=PrintXY(0,3,10);
	} while ((xtim/1000-xtim_n/1000) <VI);

	TimedIncrementT_Gen_T_Sr0_T_Sr1_W(true);
	//Write2Master();

	Aout();

	if (W>(((float)UstRele)/10)){Rele3=1;} else {Rele3=0;};
	if (Rele3 !=Rele3_old){Rel=Rel^0x04;};
	RegistrTM_int[1]=Rel;
	ComWriteIntReg(0x2,0x31,0x1);
	Rele3_old=Rele3;

	tempchar=ReadKeyBuffered();
	if (tempchar=='9'||tempchar=='5'||tempchar=='7'||tempchar=='1') {Key=tempchar;};

	if (Key=='9') {printf("Save");printf(" \n");Key='Z'; SleepWithService(2000);}; //Save
	if (Key=='1'||Key=='5'||Key=='7') {goto Exg;};

	goto Grad;

Exg:
	return Key;
}
/////////////////////////////

////////////////////////////
char Nastrojka() // Regim nastrojki


{
	int i, j;
	byte PosY,PosX, Key;
	byte temp[7][3];
	char tempchar;
	float y,z;

	PosY=0;
	//y=NomGrad; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
	PosY++;
	PosY++;
	y=VZ; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
	y=VO; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
	y=VI; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
	y=Tak; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
	y=UstRele; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;

Nastr:
	strcpy(Text, SetMediumRow());
	PrintXY(8, 0, strlen(SetMediumRow()) < 13 ? strlen(SetMediumRow()) : 12 );

	for (j=0;j<7;j++)
	{
		if (j<=1) continue;
		if(j==6)
		{
			strcpy(Text, emptyrow);
			for (i=0;i<2;i++)
			{
				Text[i]=temp[j][i];
			};
			Text[2]='.';
			for (i=0;i<1;i++)
			{
				Text[3+i]=temp[j][2+i];
			};
			tempchar=PrintXY(15,j,4);
			continue;
		}
		strcpy(Text, emptyrow);
		for (i=0;i<3;i++)
		{
			Text[i]=temp[j][i];		 
		};
		tempchar=PrintXY(16,j,3);		 
	};

	if (AK==0) {Text[2]='ч';Text[1]='у';Text[0]='р';} else {Text[2]='т';Text[1]='в';Text[0]='а';};
	tempchar=PrintXY(14,7,3);
	//if(Aout_tip == 0) Aout_tip = 1;	
	/*if (Aout_tip==1) {Text[3]=' ';Text[2]='5';Text[1]='-';Text[0]='0';};
	if (Aout_tip==2) {Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='0';};
	if (Aout_tip==3) {Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='4';};*/
	//tempchar=PrintXY(14,8,4); TODO Change selection to new window creation

	PosX=0; PosY=0; SetCursor(PosX, PosY);

	do {
		tempchar=ReadKeyBuffered();

		if (tempchar=='9'||tempchar=='5'||tempchar=='3'||tempchar=='1'
			||tempchar=='2'||tempchar=='8'||tempchar=='S') {Key=tempchar;};

		if (Key=='2') {PosY++; if (PosY>12) PosY=0; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='8') {PosY--; if (PosY>20) PosY=12; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='S') {// printf("%x",Key);printf(" \n"); Sleep(2000);
			if (PosY==7) {AK++; if (AK>1) AK=0;
			if (AK==0) {Text[2]='ч';Text[1]='у';Text[0]='р';} else {Text[2]='т';Text[1]='в';Text[0]='а';};
			tempchar=PrintXY(14,7,3);do {Key=ReadKeyBuffered();} while (Key!='Z');};
			//Aout settings
			if (PosY==8)
			{
				/*Aout_tip++; if (Aout_tip>3) Aout_tip=1;
			if (Aout_tip==1) {Text[3]=' ';Text[2]='5';Text[1]='-';Text[0]='0';};
			if (Aout_tip==2) {Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='0';};
			if (Aout_tip==3) {Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='4';};
			tempchar=PrintXY(14,8,4);do {Key=ReadKeyBuffered();} while (Key!='Z');
			*/
				ClrScr();
				Sleep(500);
				ShablonPeriphSetup();
				PeriphSetup();
				//Sleep(5000);
				ClrScr();
				Sleep(500);
				SchablonNastrojka();
				do{
					Key=ReadKeyBuffered();
				}while(Key!='Z');
				goto Nastr;
			};


			if (PosY==9) {
				ClrScr();Sleep(500);SchablonNSlug();NSlug();
			ClrScr();Sleep(500);SchablonNastrojka();
			do {Key=ReadKeyBuffered();} while (Key!='Z');goto Nastr;
			};

			if (PosY==11) {ClrScr();Sleep(500);SchablonPorog();Porog();
			ClrScr();Sleep(500);SchablonNastrojka();
			do {Key=ReadKeyBuffered();} while (Key!='Z');goto Nastr;};
			if (PosY==0)
			{
				if(media_count > 1)
				{
					SetMediumRow(1);
					//strcpy(Text, GetFullRow(mediums[NomGrad]));
					//PrintXY(12, 0, strlen(SetMediumRow()) < 13 ? strlen(SetMediumRow()) : 12 );
					SetMediumRow();
					strcpy(mediumName, fullrow);
					mediumName[strlen(fullrow)] = 0;
					Reader();
					return '7';
				}
			}
			if (PosY==2||PosY==3||PosY==4||PosY==5)
			{PosX=16; Key='Z';
			for (i=PosX;i<PosX+3;i++)
			{//Sleep(1000);
				Key='Z';
				SetCursor(i, PosY);//Sleep(1000);
				//printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
				//printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				Text[0]=Key;
				PrintXY(i,PosY,1);
				temp[PosY][i-16]=Text[0];
				Sleep(1000);
				do {Key=ReadKeyBuffered();} while (Key!='Z');
			};
			PosX=0;SetCursor(PosX, PosY);};
			if(PosY==6)
			{
				PosX=16; Key='Z';
				int index = 0;
				for (i=PosX-1;i<PosX+1;i++)
				{//Sleep(1000);
					Key='Z';
					SetCursor(i, PosY);//Sleep(1000);
					//printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
					do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
					//printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
					Text[0]=Key;
					PrintXY(i,PosY,1);
					temp[PosY][index++]=Text[0];
					Sleep(1000);
					do {Key=ReadKeyBuffered();} while (Key!='Z');
				};

				Key='Z';
				SetCursor(PosX+2, PosY);//Sleep(1000);
				//printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
				//printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				Text[0]=Key;
				PrintXY(PosX+2,PosY,1);
				temp[PosY][index]=Text[0];
				Sleep(1000);
				do {Key=ReadKeyBuffered();} while (Key!='Z');
				PosX=0;SetCursor(PosX, PosY);
			}
			if (PosY==10){ClrScr();Sleep(500);SchablonNGrad();NGrad();
			ClrScr();Sleep(500);SchablonNastrojka();
			do {Key=ReadKeyBuffered();} while (Key!='Z');goto Nastr;};
			if(PosY==12 && NewMedium())
			{					   
				return '7';
			}
		};
		if (Key=='9') {PosY=0;
		//NomGrad=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;PosY++;
		PosY++;PosY++;
		VZ=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
		VO=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
		VI=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
		Tak=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
		UstRele=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
		////printf("%u",NomGrad);printf(" \n");Sleep(10000);
		SaveNastr();
		Sleep(100); Reader();do {Key=ReadKeyBuffered();} while (Key!='Z');
		strcpy(Text,"Save Ok"); tempchar=PrintXY(11,1,7);Sleep(1000);
		strcpy(Text,"       "); tempchar=PrintXY(11,1,7);SetCursor(PosX,PosY);
		}; //Save
		if (Key=='1'||Key=='3'||Key=='5') {goto Exn;};
	} while (true);

	goto Nastr;


Exn:
	return Key;
}
/////////////////////////////

/*
Periph setup
*/
char PeriphSetup(void){
	int i,j;
	byte PosX,PosY, Key;
	byte temp[3][4]; //char array for digits - first - rows; second - columns
	char tempchar;
	float y,z;

	if(Aout_tip>2) Aout_tip=1;

	PosY=0;

	/* DAC offset */
	y = DacOffs;
	for (i=4; i!=0; i--){
		z=(byte)fmod(y,10);
		y=floor(y/10);
		temp[PosY][i-1]=z+48;
	};
	PosY++;

	/* DAC scale */
	y = DacScale;
	for (i=4; i!=0; i--){
		z=(byte)fmod(y,10);
		y=floor(y/10);
		temp[PosY][i-1]=z+48;
	};
	PosY++;

	/* Digital pot*/
	y = PotVal;
	for (i=4; i!=0; i--){
		z=(byte)fmod(y,10);
		y=floor(y/10);
		temp[PosY][i-1]=z+48;
	};
	PosY++;

	//print
	if (Aout_tip==1) {
		Text[3]=' ';Text[2]='5';Text[1]='-';Text[0]='0';
	};
	if (Aout_tip==2) {
		Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='0';
	};
	tempchar = PrintXY(14,2,4);

	for(j=0; j<3; j++)
	{
		strcpy(Text, emptyrow);
		for (i=0; i<4; i++)
		{
			Text[i]=temp[j][i];
		};
		tempchar=PrintXY(14,j+3,4);
	};

	//controler
	PosX=0;
	PosY=2;
	SetCursor(PosX, PosY);

	do{
		tempchar=ReadKeyBuffered();
		if (tempchar=='9'||tempchar=='5'||tempchar=='3'||tempchar=='1'||tempchar=='2'||tempchar=='8'||tempchar=='S')
		{
			Key = tempchar;
		};
		//move up & down
		if (Key=='2') 
		{
			PosY++;
			if (PosY>6) PosY=6;
			SetCursor(PosX, PosY);
			do 
			{
				Key=ReadKeyBuffered();
			} while (Key!='Z');
		};

		if (Key=='8') 
		{
			PosY--;
			if (PosY<2) PosY=2;
			SetCursor(PosX, PosY);
			do 
			{
				Key=ReadKeyBuffered();
			} while (Key!='Z');
		};

		//edit key
		if (Key=='S'){
			if (PosY==2){
				Aout_tip++;
				if(Aout_tip>2) Aout_tip=1;
				if (Aout_tip==1) {
					Text[3]=' ';Text[2]='5';Text[1]='-';Text[0]='0';
					PotVal=0xff;
				};
				if (Aout_tip==2) {
					Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='0';
					PotVal=0x40;
				};
				tempchar = PrintXY(14,PosY,4);
				//update pot val
				y = PotVal;
				for (i=4; i!=0; i--){
					z=(byte)fmod(y,10);
					y=floor(y/10);
					Text[i-1]=z+48;
				};
				tempchar = PrintXY(14,5,4);
				do {
					Key=ReadKeyBuffered();
				} while (Key=='Z');
			}else
			{
				if (PosY==6){
					RegistrTM_int[2]=DacScale; //set scale dac value
					RegistrTM_int[3]=0;
					RegistrTM_int[4]=PotVal; //set potentiometer value
					RegistrTM_int[5]=0x40;
					ComWriteIntReg(0x02,0x32,4); //setup DAC registers
					Sleep(200);
					do {
						Key=ReadKeyBuffered();
					} while (Key=='Z');
				}else
				{
					PosX = 14;
					for (i=PosX; i<PosX+4; i++){
						Key='Z';
						SetCursor(i,PosY);
						do {
							Key=ReadKeyBuffered();
						} while (Key=='Z' || Key=='S');
						Text[0]=Key;
						PrintXY(i,PosY,1);
						temp[PosY-3][i-14]=Text[0];
						Sleep(200);
						do {
							Key=ReadKeyBuffered();
						} while (Key=='Z');
					};
					PosX=0;
					SetCursor(PosX,PosY);
				};
			};
		};

		//save
		if(Key=='9'){
			PosY=0;
			DacOffs=1000*(temp[PosY][0]-48)+100*(temp[PosY][1]-48)+10*(temp[PosY][2]-48)+(temp[PosY][3]-48);
			PosY++;
			DacScale=1000*(temp[PosY][0]-48)+100*(temp[PosY][1]-48)+10*(temp[PosY][2]-48)+(temp[PosY][3]-48);
			PosY++;
			PotVal=1000*(temp[PosY][0]-48)+100*(temp[PosY][1]-48)+10*(temp[PosY][2]-48)+(temp[PosY][3]-48);

			SaveNastr(); //TODO Add new variables to save list
			Sleep(100);
			Reader(); //TODO ADD new variables to load list
			strcpy(Text,"Save Ok"); tempchar=PrintXY(11,1,7);Sleep(1000);
			strcpy(Text,"       "); tempchar=PrintXY(11,1,7);SetCursor(PosX,PosY);
			do {
					Key=ReadKeyBuffered();
				} while (Key=='Z');
		};
		
		//exit
		if(Key=='C'){
			Key='Z';
			do {
				Key=ReadKeyBuffered();
			} while (Key=='Z');
			goto ps_ex;
		};

	}while(true);

ps_ex:
	return Key;
}

////////////////////////////
char Porog() // Regim slugebnojn nastrojki
{  int i,j;
byte PosY,PosX, Key;
byte temp[15][8];
char tempchar;
float y,z;

PosY=3;
y=Poralfa*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';
PosY++;
y=Porbeta*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';
PosY++;
y=Porgamma*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
PosY++;
for (j=3;j<6;j++)
{ 
	strcpy(Text, emptyrow);
	for (i=0;i<8;i++)
	{
		Text[i]=temp[j][i];  
	};
	tempchar=PrintXY(12,j,8);
};
if (Nzp==0) {strcpy(Text,"Не использ."); tempchar=PrintXY(9,7,11);Sleep(100);};
if (Nzp==1) {strcpy(Text,"alfa > por "); tempchar=PrintXY(9,7,11);Sleep(100);};
if (Nzp==2) {strcpy(Text,"alfa < por "); tempchar=PrintXY(9,7,11);Sleep(100);};
if (Nzp==3) {strcpy(Text,"beta > por "); tempchar=PrintXY(9,7,11);Sleep(100);};
if (Nzp==4) {strcpy(Text,"beta < por "); tempchar=PrintXY(9,7,11);Sleep(100);};
if (Nzp==5) {strcpy(Text,"gamma > por"); tempchar=PrintXY(9,7,11);Sleep(100);};
if (Nzp==6) {strcpy(Text,"gamma < por"); tempchar=PrintXY(9,7,11);Sleep(100);};
PosX=0; PosY=3; SetCursor(PosX, PosY);
do {
	tempchar=ReadKeyBuffered();
	if (tempchar=='5'||tempchar=='9'||tempchar=='C'||tempchar=='2'||tempchar=='8'||tempchar=='S') {Key=tempchar;};

	if (Key=='2') {PosY++; if (PosY>7) PosY=3; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
	if (Key=='8') {PosY--; if (PosY<3) PosY=7; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
	if (Key=='S') { //printf("%x",Key);printf(" \n");
		if (PosY==3||PosY==4||PosY==5)
		{PosX=12; Key='Z';
		for (i=PosX;i<PosX+8;i++)
		{//Sleep(1000);
			Key='Z';
			SetCursor(i, PosY);//Sleep(1000);
			if (i==16) goto npl1;
			////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
			do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
			////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
			Text[0]=Key;
			PrintXY(i,PosY,1);
			temp[PosY][i-12]=Text[0];
			Sleep(200);
			do {Key=ReadKeyBuffered();} while (Key!='Z');

npl1:;
		};
		};
		if (PosY==7)//66666666666


		{
			Nzp++; if (Nzp>6) {Nzp=0;};
			if (Nzp==0) {strcpy(Text,"Не использ."); tempchar=PrintXY(9,7,11);Sleep(100);};
			if (Nzp==1) {strcpy(Text,"alfa > por "); tempchar=PrintXY(9,7,11);Sleep(100);};
			if (Nzp==2) {strcpy(Text,"alfa < por "); tempchar=PrintXY(9,7,11);Sleep(100);};
			if (Nzp==3) {strcpy(Text,"beta > por "); tempchar=PrintXY(9,7,11);Sleep(100);};
			if (Nzp==4) {strcpy(Text,"beta < por "); tempchar=PrintXY(9,7,11);Sleep(100);};
			if (Nzp==5) {strcpy(Text,"gamma > por"); tempchar=PrintXY(9,7,11);Sleep(100);};
			if (Nzp==6) {strcpy(Text,"gamma < por"); tempchar=PrintXY(9,7,11);Sleep(100);};
			// do {Key=ReadKey();} while (Key=='Z'||Key=='S');
			////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
			// Text[0]=Key;
			//  PrintXY(i,PosY,1);
			//  temp[PosY][i-12]=Text[0];
			//   Sleep(200);
			do {Key=ReadKeyBuffered();} while (Key!='Z');

			//  nsl1:
			//};
		};
		PosX=0;SetCursor(PosX, PosY);
	};
	if (Key=='9') { PosY=3;

	Poralfa=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
		(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
	Porbeta=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
		(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
	Porgamma=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
		(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;

	////printf("%u",Ku0);printf(" \n");printf("%u",Ku0);printf(" \n");  Sleep(10000);
	////printf("Save ");printf("%f",a0);printf(" \n");
	SaveSet();
	Sleep(100); Reader();
	strcpy(Text,"SaveOk"); tempchar=PrintXY(13,2,6);Sleep(1000);
	strcpy(Text,"      "); tempchar=PrintXY(13,2,6);SetCursor(PosX,PosY);  //////////////////////////
	do {Key=ReadKeyBuffered();} while (Key!='Z'); }; //Save
	if (Key=='7') { PosY=0;
	Saw_A0=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
		(temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;

	//printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
	Saw_A1=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
		(temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;
	//printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
	Sleep(5000);
	WriteDataSPMSet();
	Sleep(500);
	EEPROMSave(0x01);
	Sleep(500);
	do {Key=ReadKeyBuffered();} while (Key!='Z');};
	if (Key=='5') {ClrScr(); Sleep(500); Rezonans();do {Key=ReadKeyBuffered();} while (Key!='Z'); }; //Rezonans

	if (Key=='C') {Key='Z';do {Key=ReadKeyBuffered();} while (Key!='Z'); goto Exnsl;};

	//   };
	//goto NastrSlug;
} while (true);

Exnsl:



//Sleep(10000);
return 0;
}
/////////////////////////////

////////////////////////////
char NSlug() // Regim slugebnojn nastrojki


{
	int i,j;
	byte PosY,PosX, Key;
	byte temp[15][8];
	char tempchar;
	float y,z;
	PosX=0; PosY=0; SetCursor(PosX, PosY);

	y=Saw_A0; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	y=Saw_A1; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;PosY++;

	y=Va*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;

	if (kgt<0) {y=kgt*(-1000);} else {y=kgt*1000;};
	for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (kgt<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};
	temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;

	if (kgv<0) {y=kgv*(-1000);} else {y=kgv*1000;};
	for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (kgv<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};
	temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;

	y=T*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;PosY++;
	PosY++;
	PosY++;
	PosY++;
	y=controlleradr; for (i=2;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	PosY++;
	y=Ku0; for (i=2;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
NastrSlug:
	////printf("NGrad");printf(" \n");Sleep(30000);
	for (j=0;j<7;j++)
	{ 
		strcpy(Text, emptyrow);
		for (i=0;i<8;i++)
		{
			Text[i]=temp[j][i];        
		};
		if(j != 2)
		{
			tempchar=PrintXY(12,j,8);
		}
	};
	j=11;
	strcpy(Text, emptyrow);
	for (i=0;i<2;i++)
	{
		Text[i]=temp[j][i];        
	};
	tempchar=PrintXY(18,j,2);
	j=12;
	strcpy(Text, emptyrow);
	for (i=0;i<2;i++)
	{
		Text[i]=temp[j][i];        
	};
	tempchar=PrintXY(18,j,2);
	PosX=0; PosY=0;

	do {
		tempchar=ReadKeyBuffered();
		if (tempchar=='5'||tempchar=='9'||tempchar=='C'||tempchar=='2'||tempchar=='8'||tempchar=='S') {Key=tempchar;};

		if (Key=='2') {PosY++; if (PosY>12) PosY=0; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='8') {PosY--; if (PosY>20) PosY=12; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='S') { //printf("%x",Key);printf(" \n");
			if (PosY==11||PosY==12)
			{PosX=18; Key='Z';
			for (i=PosX;i<PosX+2;i++)
			{//Sleep(1000);
				Key='Z';
				SetCursor(i, PosY);//Sleep(1000);
				////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
				////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				Text[0]=Key;
				PrintXY(i,PosY,1);
				temp[PosY][i-18]=Text[0];
				Sleep(200);
				do {Key=ReadKeyBuffered();} while (Key!='Z');


			};
			} else
			{
				PosX=12;  Key='Z';
				for (i=PosX;i<PosX+8;i++)
				{//Sleep(1000);
					if (i==16) goto nsl1;
					Key='Z';
					SetCursor(i, PosY);//Sleep(1000);
					////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
					do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
					////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
					Text[0]=Key;
					PrintXY(i,PosY,1);
					temp[PosY][i-12]=Text[0];
					Sleep(200);
					do {Key=ReadKeyBuffered();} while (Key!='Z');

nsl1:;
				};
			};
			PosX=0;SetCursor(PosX, PosY);
		};
		if (Key=='9') { PosY=0;
		Saw_A0=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
			(temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;
		Saw_A1=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
			(temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;PosY++;
		Va=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
		kgt=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;

		kgv=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;

		T=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
		controlleradr=(temp[11][0]-48)*10+temp[11][1]-48;
		Ku0=(temp[12][0]-48)*10+temp[12][1]-48;
		Ku1=(temp[12][0]-48)*10+temp[12][1]-48;

		////printf("%u",Ku0);printf(" \n");printf("%u",Ku0);printf(" \n");  Sleep(10000);
		////printf("Save ");printf("%f",a0);printf(" \n");
		SaveSet();
		Sleep(100); Reader();Va=Saw_A1;WriteDataSPMSet();Sleep(500);
		strcpy(Text,"SaveOk"); tempchar=PrintXY(13,2,6);Sleep(1000);
		strcpy(Text,"      "); tempchar=PrintXY(13,2,6);SetCursor(PosX,PosY);  //////////////////////////
		do {Key=ReadKeyBuffered();} while (Key!='Z'); }; //Save
		if (Key=='7') { PosY=0;
		Saw_A0=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
			(temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;

		//printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
		Saw_A1=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
			(temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;
		//printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
		Sleep(5000);
		WriteDataSPMSet();
		Sleep(500);
		EEPROMSave(0x01);
		Sleep(500);
		do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='5') {ClrScr(); Sleep(500); Rezonans();do {Key=ReadKeyBuffered();} while (Key!='Z'); }; //Rezonans

		if (Key=='C') {Key='Z';do {Key=ReadKeyBuffered();} while (Key!='Z'); goto Exnsl;};

		//   };
		//goto NastrSlug;
	} while (true);

Exnsl:
	return Key;
}
/////////////////////////////

char Rezonans() // Poisk rezonansa


{
	int i,j;
	byte PosY,PosX, Key;
	byte temp[15][8];
	char tempchar;
	float y,z;

	strcpy(Text,"Поиск резонанса     ");  tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Подпорка            ");  tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Альфа               ");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Бета                ");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Гамма               ");  tempchar=PrintXY(0,7,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Нач. поиска        В");  tempchar=PrintXY(0,9,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Кон. поиска        В");  tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Шаг поиска         В");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F3 -Поиск           ");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Sh-Ред F5-Сохр С-вых");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");

	PosY=9;
	y=N_Rez*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	y=K_Rez*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	y=Step_Rez*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	for (j=9;j<12;j++)
	{ 
		strcpy(Text, emptyrow);
		for (i=0;i<8;i++)
		{
			Text[i]=temp[j][i];
		};
		tempchar=PrintXY(12,j,8);
	};
	PosX=0; PosY=9;SetCursor(PosX,PosY);

	do {
		tempchar=ReadKeyBuffered();
		if (tempchar=='9'||tempchar=='5'||tempchar=='C'||tempchar=='2'||tempchar=='8'||tempchar=='S') {Key=tempchar;};
		//5 (F3) - Poisk rezonansa 9 (F5) - save nastr C - vihod  S - redaktirovanie 2 - vniz 8 - vverh
		if (Key=='2') {PosY++; if (PosY>11) PosY=9; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='8') {PosY--; if (PosY<9) PosY=11; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='C') {Key='Z';do {Key=ReadKeyBuffered();} while (Key!='Z'); goto Exrez;};
		if (Key=='S') { //printf("%x",Key);printf(" \n");
			PosX=12; Key='Z';
			for (i=PosX;i<PosX+8;i++)
			{//Sleep(1000);
				if (i==16) goto nr1;
				Key='Z';
				SetCursor(i, PosY);//Sleep(1000);
				////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
				////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				Text[0]=Key;
				PrintXY(i,PosY,1);
				temp[PosY][i-12]=Text[0];
				Sleep(200);
				do {Key=ReadKeyBuffered();} while (Key!='Z');

nr1:;
			};
			PosX=0;SetCursor(PosX, PosY);
		};
		if (Key=='9') {PosY=9;
		//     //printf("%f",N_Rez);printf("  "); printf("%f",K_Rez);printf("  ");printf("%f",Step_Rez);printf("\n"); Sleep(5000);
		N_Rez=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
		K_Rez=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
		Step_Rez=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
		//  //printf("%f",N_Rez);printf("  "); printf("%f",K_Rez);printf("  ");printf("%f",Step_Rez);printf("\n"); Sleep(5000);
		SaveNastr();
		Sleep(100); Reader();//do {Key=ReadKey();} while (Key!='Z');
		do {Key=ReadKeyBuffered();} while (Key!='Z');
		strcpy(Text,"Save Ok"); tempchar=PrintXY(11,1,7);Sleep(1000);
		strcpy(Text,"       "); tempchar=PrintXY(11,1,7);SetCursor(PosX,PosY);
		PosX=0;PosY=0;SetCursor(PosX, PosY);
		}; //Save

		if (Key=='5') { 
			PosY=0;
			Saw_A0=N_Rez*1000;
			//ComCMode(0x01,0x01);
			Sleep(1000);
			do {
				PosY=3;
				y=Saw_A0; 
				for (i=8;i!=0;i--) {
					z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;
				};
				temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
				temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';
				for (i=0;i<8;i++){
					Text[i]=temp[PosY][i];
					tempchar=PrintXY(12,PosY,8);
				};
				Sleep(1000);
				WriteDataSPMSet();
				Sleep(1000);
				//ComCMode(0x01,0x00);
				Sleep(2000);
				ReadDataSPMCurrent();
				//printf("%u",Saw_A0);//printf(" ");printf("%u",Am0);printf(" ");printf("%u",Haw0);printf(" ");printf("%u",Gamma0);printf("\n");
				Sleep(1000);
				PosY=5;
				y=Am0*1000; 
				for (i=8;i!=0;i--) {
					z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;
				};
				temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
				temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
				y=Haw0; 
				for (i=8;i!=0;i--) {
					z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;
				};
				temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
				temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
				y=Gamma0; 
				for (i=8;i!=0;i--) {
					z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;
				};
				temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
				temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
				for (j=5;j<8;j++){ 
					for (i=0;i<8;i++){
						Text[i]=temp[j][i];
						tempchar=PrintXY(12,j,8);
					};
				};  //
				Saw_A0=Saw_A0+Step_Rez*1000;
			} while (Saw_A0<K_Rez*1000); //Cicl Rezonans
			do {
				Key=ReadKeyBuffered();
			} while (Key!='Z');//}
		};// Key 5

		//goto NastrSlug;
	} while (true);
Exrez:
	return Key;
}


bool NewMedium()
{
	newMediumStr[0] = 0;
	ClrScr();

	char tempchar = VirtualKeyboard(newMediumStr);
	if(tempchar == '9')
	{ 
		strcpy(mediumName, newMediumStr);
		mediumName[strlen(newMediumStr)] = 0;
		for(int i = 0; i < MEDIUM_COUNT; i++)
		{
			if(mediums[i][0] == 0)
			{
				strcpy(mediums[i], mediumName);
				mediums[i][strlen(mediumName)] = 0;

				media_count++;
				SetMediumRow(2);
				break;
			};
		}

		SetDefaultValues();
		SaveAll();
		return true;
	}

	return false;
}

char VirtualKeyboard(char * buffer)
{
	ClrScr();
	SchablonKeyboard(" Новая градуировка: ", buffer);

	char tempchar = 0;
	int charindex = strlen(buffer);
	int index = 0;

	SetCursor(0,9);

	do
	{
		tempchar = WaitAnyKey();
		switch(tempchar)
		{
		case '4' :
			if(charindex > 0) charindex--;
			Sleep(500);
			break;
		case '6' :
			if(charindex < strlen(abc_ru) - 1) charindex++;
			Sleep(500);
			break;
		case '8' :
			if(charindex > 19) charindex -= 20;
			Sleep(500);
			break;
		case '2' :
			if(charindex <= 22) charindex += 20;
			Sleep(500);
			break;
		case 'S' :
			if(index < 19)
			{
				buffer[index++] = abc_ru[charindex];
				buffer[index] = 0;
			};
			break;
		case 'C' :
			if(index == 0) buffer[0] = 0;
			if(index > 0) buffer[--index] = 0;
			break;
		}

		if(tempchar == 'S' || tempchar == 'C')
		{
			char * fullrow = GetFullRow(buffer);
			strcpy(Text, fullrow);
			PrintXY(0, 2, 20);
			Sleep(1000);
		}

		if(tempchar == '4' || tempchar == '6' || tempchar == '8' || tempchar == '2')
		{
			SetCursor(charindex % 20, charindex / 20 + 9);
		}
		if(tempchar == '9' && strlen(buffer) > 0)
		{
			return tempchar;
		}
	}
	while(tempchar != '1' && tempchar != '3' && tempchar != '7');
	return tempchar;
}

////////////////////////////
char NGrad() // Regim nastrojki


{
	int i,j;
	byte PosY,PosX, Key;
	byte temp[15][9];
	char tempchar;
	float y,z;
	PosX=0; PosY=0; SetCursor(PosX, PosY);
	PosY=0;
	if (a0<0) {y=a0*(-10000);} else {y=a0*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a0<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a1<0) {y=a1*(-10000);} else {y=a1*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a1<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a2<0) {y=a2*(-10000);} else {y=a2*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a2<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a3<0) {y=a3*(-10000);} else {y=a3*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a3<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a4<0) {y=a4*(-10000);} else {y=a4*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a4<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a5<0) {y=a5*(-10000);} else {y=a5*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a5<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a6<0) {y=a6*(-10000);} else {y=a6*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a6<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a7<0) {y=a7*(-10000);} else {y=a7*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a7<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a8<0) {y=a8*(-10000);} else {y=a8*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a8<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (a9<0) {y=a9*(-10000);} else {y=a9*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (a9<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (AA<0) {y=AA*(-10000);} else {y=AA*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (AA<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (BB<0) {y=BB*(-10000);} else {y=BB*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (BB<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (k1<0) {y=k1*(-10000);} else {y=k1*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (k1<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (Wmax<0) {y=Wmax*(-10000);} else {y=Wmax*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (Wmax<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
	if (Wmin<0) {y=Wmin*(-10000);} else {y=Wmin*10000;};
	for (i=9;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
	if (Wmin<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
	temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;

NastrGrad:
	////printf("NGrad");printf(" \n");Sleep(30000);
	for (j=0;j<15;j++)
	{ 
		strcpy(Text, emptyrow);
		for (i=0;i<9;i++)
		{
			Text[i]=temp[j][i];        
		};
		tempchar=PrintXY(6,j,9);
	};
	PosX=0; PosY=0;

	do {
		tempchar=ReadKeyBuffered();
		if (tempchar=='9'||tempchar=='C'||tempchar=='2'||tempchar=='8'||tempchar=='S') {Key=tempchar;};

		if (Key=='2') {PosY++; if (PosY>14) PosY=0; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='8') {PosY--; if (PosY>20) PosY=14; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
		if (Key=='S') { ////printf("%x",Key);printf(" \n");
			PosX=6; Key='Z';
			for (i=PosX;i<PosX+9;i++)
			{//Sleep(1000);
				if (i==10) goto ng1;
				Key='Z';
				SetCursor(i, PosY);//Sleep(1000);
				////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
				do {
ng3:
					Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
					////printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
					if (i==6) {if (Key=='+'||Key=='-'){goto ng2;} else {goto ng3;};};
ng2:
					Text[0]=Key;
					PrintXY(i,PosY,1);
					temp[PosY][i-6]=Text[0];
					Sleep(200);
					do {Key=ReadKeyBuffered();} while (Key!='Z');
ng1:;
			};
			PosX=0;SetCursor(PosX, PosY);
		};
		if (Key=='9') { PosY=0;
		a0=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001 +(temp[PosY][8]-48)*0.0001);PosY++;
		a1=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		a2=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		a3=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		a4=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		a5=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		a6=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		a7=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		a8=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		a9=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		AA=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		BB=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		k1=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		Wmax=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		Wmin=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
			(temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001+(temp[PosY][8]-48)*0.0001);PosY++;
		////printf("Save ");printf("%f",a0);printf(" \n");
		SaveGrad();
		Sleep(100); Reader();do {Key=ReadKeyBuffered();} while (Key!='Z');
		strcpy(Text,"SaveOk"); tempchar=PrintXY(14,1,6);Sleep(1000);
		strcpy(Text,"      "); tempchar=PrintXY(14,1,6);SetCursor(PosX,PosY); };//Save
		if (Key=='C') {Key='Z';do {Key=ReadKeyBuffered();} while (Key!='Z'); goto Exng;};
	} while (true);

	goto NastrGrad;


Exng:
	return Key;
}

////////////////////////////

/////////////////////////////
char SaverGrad()
{
	return 0;
}
////////////////////////////

////////////////////////////
char SchablonNSlug() // Schablon ekrana

{
	char tempchar;
	////printf("Schablon");printf(" \n");SleepWithService(3000);

	strcpy(Text,"Подпорка   =        ");  tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Амплитуда  =        ");  tempchar=PrintXY(0,1,20);printf("%x",tempchar);printf(" \n");
	//strcpy(Text,"                    ");  tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Va  =               ");  tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"kgt =               ");  tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"kgv =               ");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"T   =               ");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");
	// strcpy(Text,"                    ");  tempchar=PrintXY(0,12,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Кол-во датчиков     ");  tempchar=PrintXY(0,8,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Ном. датчика 1      ");  tempchar=PrintXY(0,9,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Ном. датчика 2      ");  tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Ном. прибора        ");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"К-т усиления        ");  tempchar=PrintXY(0,12,20);printf("%x",tempchar);printf(" \n");
	// strcpy(Text,"                    ");  tempchar=PrintXY(0,13,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F3 -Поиск резонанса ");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Sh-Ред F5-Сохр С-вых");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");
	return 0;
}
/////////////////////////////

////////////////////////////
char SchablonPorog() // Schablon ekrana

{
	char tempchar;
	//printf("Schablon");printf(" \n");SleepWithService(3000);

	strcpy(Text,"    Выбор порога    ");  tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Порог alfa          ");  tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Порог beta          ");  tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Порог gamma         ");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Условие             ");  tempchar=PrintXY(0,7,20);printf("%x",tempchar);printf(" \n");

	strcpy(Text,"Sh-Ред F5-Сохр С-вых");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");

	// SleepWithService(10000);
	return 0;
}
/////////////////////////////

char SchablonNGrad() // Schablon ekrana

{
	char tempchar;

	strcpy(Text,"a0  =               ");  tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"a1  =               ");  tempchar=PrintXY(0,1,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"a2  =               ");  tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"a3  =               ");  tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"a4  =               ");  tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"a5  =               ");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"a6  =               ");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"a7  =               ");  tempchar=PrintXY(0,7,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Tg n=               ");  tempchar=PrintXY(0,8,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Ts n=               ");  tempchar=PrintXY(0,9,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"AA  =               ");  tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"BB  =               ");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"k1  =               ");  tempchar=PrintXY(0,12,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Wmax=               ");  tempchar=PrintXY(0,13,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Wmin=               ");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Sh-Ред F5-Сохр С-вых");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");
	return 0;
}
/////////////////////////////
////////////////////////////
char SchablonNastrojka() // Schablon ekrana

{
	//int i;
	//char len;
	//char pos=0;
	char tempchar;

	strcpy(Text,"Среда :             "); tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"Время заполн.      с");  tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Время очист.       с");  tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Время измер.       с");  tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Период калибр.     с");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Уставка реле     . %");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Тип калибр.         ");  tempchar=PrintXY(0,7,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Настройка ток. вых. ");  tempchar=PrintXY(0,8,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Служебн. настр.     ");  tempchar=PrintXY(0,9,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Градуир. коэфф.     ");  tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Выбор порога        ");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n"); 
	strcpy(Text,"Новая среда         ");  tempchar=PrintXY(0,12,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"Sh-Редактировать    ");  tempchar=PrintXY(0,13,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F1-Работа F2-Град.  ");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F3-Калибр F5-Сохран.");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");
	return 0;
}
/////////////////////////////

////////////////////////////
char SchablonRabota() // Schablon ekrana

{
	//int i;
	//char len;
	//char pos=0;
	char tempchar;

	strcpy(Text,GetCenteredRow(mediumName)); 
	tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"Режим: Работа       "); tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");

	strcpy(Text,"ВЛАЖНОСТЬ:         %");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");

	strcpy(Text,"Т среды:           С");  tempchar=PrintXY(0,8,20);printf("%x",tempchar);printf(" \n");

	strcpy(Text,"F2-Град.  F3-Калибр.");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F4-Настройка        ");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");
	return 0;
}
/////////////////////////////

////////////////////////////
char SchablonGrad() // Schablon ekrana

{
	//int i;
	//char len;
	//char pos=0;
	char tempchar;

	strcpy(Text,GetCenteredRow(mediumName));
	tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"Режим: Градуировка  "); tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");
	//strcpy(Text,"Операция:  Ошибка:  "); tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
	//strcpy(Text,"                    "); tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Влажность:         %");  tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Т датчика:         С");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Т среды:           С");  tempchar=PrintXY(0,7,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Т датчика2:        С");  tempchar=PrintXY(0,8,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"Альфа:              ");  tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Бета :              ");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Гамма:              ");  tempchar=PrintXY(0,12,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"F1-Работа F3-Калибр.");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F4-Настр. F5-Сохран.");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");
	return 0;
}
/////////////////////////////

////////////////////////////
char SchablonKalibr() // Schablon ekrana

{
	//int i;
	//char len;
	//char pos=0;
	char tempchar;

	strcpy(Text,GetCenteredRow(mediumName));
	tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"Режим: Калибровка   "); tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");
	//strcpy(Text,"Операция:  Ошибка:  "); tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
	//strcpy(Text,"                    "); tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Альфа0:     Тг:     ");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Бета0 :     Тс:     ");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Гамма0:             ");  tempchar=PrintXY(0,7,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"Альфа :     Тг:     ");  tempchar=PrintXY(0,9,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Бета  :     Тс:     ");  tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Гамма :             ");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
	//
	strcpy(Text,"F3-Калибровка       ");  tempchar=PrintXY(0,13,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F1-Работа F2-Град.  ");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F4-Настр. F5-Сохран.");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");
	return 0;
}

void SchablonKeyboard(char * title, char * buffer)
{
	char tempchar;
	strcpy(Text,title); tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	if(buffer != NULL) { strcpy(Text,GetFullRow(buffer)); tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n"); };
	strcpy(Text,"Up-Вверх Dn-Вниз    "); tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Lf-Влево Rg-Вправо  "); tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");

	strcpy(Text,"абвгде жзийклмнопрст"); tempchar=PrintXY(0,9,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"уфхцчшщъыьэюя1234567"); tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"890                 "); tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Sh-Вставка C-Стереть");  tempchar=PrintXY(0,12,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F5-Готово           ");  tempchar=PrintXY(0,13,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F1-Работа F2-Град.  ");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"F3-Калибр. F4-Настр.");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");
}

/////////////////////////////


/*
Periph setup screen template
*/

char ShablonPeriphSetup(){

	char tempchar;
	//printf("Schablon");printf(" \n");SleepWithService(3000);

	strcpy(Text,"Настройка ток. вых. ");  tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Тип выхода:       мА");  tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Смещение ЦАП:       ");  tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Масштаб ЦАП:        ");  tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Знач.потенц.:       ");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
	strcpy(Text,"Тест ЦАП            ");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");

	strcpy(Text,"Sh-Ред F5-Сохр С-вых");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");

	// SleepWithService(10000);
	return 0;
} 


////////////////////////////
char PrintXY(byte X, byte Y, byte length) // Pechat stroki simvolov
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC
	//          13-err adr komandy

	//-----Komanda-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =2-terminal;
	//1-UstrCodeFunc - Kod funkcii (1 byte)- =0x68;
	//2-koordinata X (0-19) gorizont
	//3-koordinata Y (0-15) vertikal
	//4- 3+N - stroka simvolov
	//3+N-CRC
	//4+N-CRC
	//-----Otvet-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =UstrAdr v komande;
	//1-UstrCodeFunc - Vozvrat koda funkcii (1 byte)- =0x68 - esli OK;
	//=0xE8 - esli error
	//2 - esli error - kod oshibki    0x01 - err koda func
	//0x02 - err adr reg
	//0x03 - err data
	//2 - esli OK - CRC
	//3 - CRC
	//4 - esli error - CRC
	//4 - esli OK - net
{
	EnterCriticalSection(g_LockData);
	int i;
	char len;
	char pos=0;

	// Podgotovka komandnoj stroki             //Write:
	ComDataWrite[pos]=0x02;pos++;              //byte 0
	ComDataWrite[pos]=0x68; pos++;             //byte 1
	ComDataWrite[pos]=X; pos++;                //byte 2
	ComDataWrite[pos]=Y; pos++;                //byte 3
	for (i=0;i<length;i++)
	{ComDataWrite[pos]=Text[i]; pos++;};
	//pos=pos+length;
	CRCcount('w',pos);                         //


	//Peredacha komandnoj stroki i priem otveta
	len=pos+2;
	for (i=0;i<len;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
	//printf("\n");

	DWORD ret = 0;
	WriteFile(hCom1,ComDataWrite,len,&ret,NULL);

	//Read:
	//OK:    Error:
	//adres //byte 0 byte 0
	//kod funkcii //byte 1 byte 1(modificirovan)
	//kod error   //       byte 2
	//CRC         //byte 2 byte 3
	// CRC        //byte 3 byte 4
	len=4; //SleepWithService(100);

	ReadFile(hCom1,ComDataRead,len,&ret,NULL);
	for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
	printf("   ");
	//SleepWithService(1000);
	//Rasshifrovka otveta
	if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_LockData); return 10;}
	if (ComDataWrite[1]!=ComDataRead[1])
	{if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_LockData);return 11;} else
	{LeaveCriticalSection(g_LockData);return ComDataRead[2];};}
	else {CRCcount('r',len);

	};            // vichislenie CRC

	//proverka CRC
	//if (ComDataRead[len]!=ComDataRead[len+2] ||
	//  ComDataRead[len+1]!=ComDataRead[len+1+2]) return 12;
	LeaveCriticalSection(g_LockData);
	return 0;
}
/////////////////////////////


////////////////////////////
char SetCursor(byte X, byte Y) // Ustanovka kursora
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC
	//          13-err adr komandy

	//-----Komanda-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =2-terminal;
	//1-UstrCodeFunc - Kod funkcii (1 byte)- =0x67;
	//2-koordinata X (0-19) gorizont
	//3-koordinata Y (0-15) vertikal
	//4-CRC
	//5-CRC
	//-----Otvet-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =UstrAdr v komande;
	//1-UstrCodeFunc - Vozvrat koda funkcii (1 byte)- =0x67 - esli OK;
	//=0xE7 - esli error
	//2 - esli error - kod oshibki    0x01 - err koda func
	//0x02 - err adr reg
	//0x03 - err data
	//2 - esli OK - CRC
	//3 - CRC
	//4 - esli error - CRC
	//4 - esli OK - net
{
	EnterCriticalSection(g_LockData);
	int i;
	char len;
	char pos=0;

	// Podgotovka komandnoj stroki             //Write:
	ComDataWrite[pos]=0x02;pos++;              //byte 0
	ComDataWrite[pos]=0x67; pos++;             //byte 1
	ComDataWrite[pos]=X; pos++;                //byte 2
	ComDataWrite[pos]=Y; pos++;                //byte 3
	CRCcount('w',pos);                         //


	//Peredacha komandnoj stroki i priem otveta
	len=pos+2;
	for (i=0;i<len;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
	//printf("\n");
	DWORD ret = 0;
	WriteFile(hCom1,ComDataWrite,len,&ret,NULL);

	//Read:
	//OK:    Error:
	//adres //byte 0 byte 0
	//kod funkcii //byte 1 byte 1(modificirovan)
	//kod error   //       byte 2
	//CRC         //byte 2 byte 3
	// CRC        //byte 3 byte 4
	//len=4;	  

	ReadFile(hCom1,ComDataRead,len,&ret,NULL);
	for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
	//printf("\n");

	//Rasshifrovka otveta
	if (ComDataWrite[0]!=ComDataRead[0])
	{
		LeaveCriticalSection(g_LockData);
		return 10;
	}
	if (ComDataWrite[1]!=ComDataRead[1])
	{if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_LockData);return 11;} else
	{LeaveCriticalSection(g_LockData);return ComDataRead[2];};}
	else {CRCcount('r',len);

	};            // vichislenie CRC

	//proverka CRC
	if (ComDataRead[len]!=ComDataRead[len+2] ||
		ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_LockData); return 12;}
	LeaveCriticalSection(g_LockData);
	return 0;
}
/////////////////////////////



////////////////////////////
char ClrScr() // Ochistka ekrana
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC
	//          13-err adr komandy

	//-----Komanda-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =2-terminal;
	//1-UstrCodeFunc - Kod funkcii (1 byte)- =0x66;
	//2-CRC
	//3-CRC
	//-----Otvet-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =UstrAdr v komande;
	//1-UstrCodeFunc - Vozvrat koda funkcii (1 byte)- =0x66 - esli OK;
	//=0xE6 - esli error
	//2 - esli error - kod oshibki    0x01 - err koda func
	//0x02 - err adr reg
	//0x03 - err data
	//2 - esli OK - CRC
	//3 - CRC
	//4 - esli error - CRC
	//4 - esli OK - net
{
	EnterCriticalSection(g_LockData);
	int i;
	char len;
	char pos=0;

	// Podgotovka komandnoj stroki             //Write:
	ComDataWrite[pos]=0x02;pos++;           //byte 0
	ComDataWrite[pos]=0x66; pos++;             //byte 1

	CRCcount('w',pos);                         //


	//Peredacha komandnoj stroki i priem otveta
	len=pos+2;
	for (i=0;i<len;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
	//printf("\n");
	DWORD ret = 0;
	WriteFile(hCom1,ComDataWrite,len,&ret,NULL);

	//Read:
	//OK:    Error:
	//adres //byte 0 byte 0
	//kod funkcii //byte 1 byte 1(modificirovan)
	//kod error   //       byte 2
	//CRC         //byte 2 byte 3
	// CRC        //byte 3 byte 4
	len=4;

	ReadFile(hCom1,ComDataRead,len,&ret,NULL);
	for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
	//printf("\n");

	//Rasshifrovka otveta
	if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_LockData); return 10;}
	if (ComDataWrite[1]!=ComDataRead[1])
	{if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_LockData); return 11;} else
	{LeaveCriticalSection(g_LockData);return ComDataRead[2];};}
	else {CRCcount('r',len);

	};            // vichislenie CRC

	//proverka CRC
	if (ComDataRead[len]!=ComDataRead[len+2] ||
		ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_LockData); return 12;}
	LeaveCriticalSection(g_LockData);
	return 0;
}
/////////////////////////////

////////////////////////////
char EEPROMSave(char UstrAdr) // Zapis v EEPROM
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC
	//          13-err adr komandy

	//-----Komanda-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =1-dathik; =2-terminal;
	//1-UstrCodeFunc - Kod funkcii (1 byte)- =0x65;
	//2-CRC
	//3-CRC
	//-----Otvet-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =UstrAdr v komande;
	//1-UstrCodeFunc - Vozvrat koda funkcii (1 byte)- =0x65 - esli OK;
	//=oxE5 - esli error
	//2 - esli error - kod oshibki    0x01 - err koda func
	//0x02 - err adr reg
	//0x03 - err data
	//2 - esli OK - CRC
	//3 - CRC
	//4 - esli error - CRC
	//4 - esli OK - net
{
	EnterCriticalSection(g_LockData);
	char len;
	char pos=0;

	// Podgotovka komandnoj stroki             //Write:
	ComDataWrite[pos]=UstrAdr;pos++;           //byte 0
	ComDataWrite[pos]=0x65; pos++;             //byte 1

	CRCcount('w',pos);                         //


	//Peredacha komandnoj stroki i priem otveta
	len=pos+2;
	//for (i=0;i<len;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
	//printf("\n");
	DWORD ret = 0;
	WriteFile(hCom1,ComDataWrite,len,&ret,NULL);

	//Read:
	//OK:    Error:
	//adres //byte 0 byte 0
	//kod funkcii //byte 1 byte 1(modificirovan)
	//kod error   //       byte 2
	//CRC         //byte 2 byte 3
	// CRC        //byte 3 byte 4
	len=4;

	ReadFile(hCom1,ComDataRead,len,&ret,NULL);

	//Rasshifrovka otveta
	if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_LockData); return 10;}
	if (ComDataWrite[1]!=ComDataRead[1])
	{if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_LockData); return 11;} else
	{LeaveCriticalSection(g_LockData); return ComDataRead[2];};}
	else {CRCcount('r',len);

	};            // vichislenie CRC

	//proverka CRC
	if (ComDataRead[len]!=ComDataRead[len+2] ||
		ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_LockData); return 12;}
	LeaveCriticalSection(g_LockData);
	return 0;
}
/////////////////////////////


////////////////////////////
char WriteDataSPMSet() // Zapis nastroek SPM
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC


{
	byte UstrAdr=1;
	byte StartAdr=0x0B;
	byte Length=4;//<<< KOD PEREDACHI Dannih v DATCHIK (dolgen bit=2)
	byte temp;
	byte i;

	i=StartAdr;
	RegistrSPM_int[i]=Saw_A0;
	//printf("%u", RegistrSPM_int[i]); printf("\n");
	i++;
	RegistrSPM_int[i]=Saw_A1;i++;
	RegistrSPM_int[i]=Ku0;i++;
	RegistrSPM_int[i]=Ku1;i++;
	RegistrSPM_int[i]=Taq;i++;
	RegistrSPM_int[i]=Fwdt;i++;
	RegistrSPM_int[i]=Pcount;i++;
	//BaudRateSPM=RegistrSPM_int[i];i++;
	//RegistrSPM_int[i]=HostSPM;i++;
	//RegistrSPM_int[i]=ValidReqSPM;i++;
	//RegistrSPM_int[i]=CRCReqSPM;i++;
	//RegistrSPM_int[i]=ExcReqSPM;i++;

	temp=ComWriteIntReg(UstrAdr,StartAdr,Length);
	if (temp !=0) return temp;

	return 0;
}
////////////////////////////

////////////////////////////
char ComWriteIntReg(char UstrAdr,char StartAdr, char Length) // Zapis registrov
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC
	//          13-err adr komandy

	//-----Komanda-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =1-dathik; =2-terminal;
	//1-UstrCodeFunc - Kod funkcii (1 byte)- =0x45;
	//2-0        - start byte zapisi - (2 byte)
	//3-StartAdr - start byte zapisi
	//4...(3+Length*2) - data zapisivaemie
	//(4+Length*2)...(5+Length*2)-CRC ;
	//-----Otvet-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =UstrAdr v komande;
	//1-UstrCodeFunc - Vozvrat koda funkcii (1 byte)- =0x45 - esli OK;
	//=oxC5 - esli error
	//2 - esli error - kod oshibki    0x01 - err koda func
	//0x02 - err adr reg
	//0x03 - err data
	//2 - esli OK - CRC
	//3 - CRC
	//4 - esli error - CRC
	//4 - esli OK - net
{
	EnterCriticalSection(g_LockData);
	if (UstrAdr<1 || UstrAdr>2) {LeaveCriticalSection(g_LockData);return 10;};
	if (UstrAdr==1){if (StartAdr>0x16 || StartAdr+Length>0x17) {LeaveCriticalSection(g_LockData);return 13;};};
	if (UstrAdr==2){if (StartAdr<0x30) {LeaveCriticalSection(g_LockData);return 13;};
	if (StartAdr>0x35 || StartAdr+Length>0x36) {LeaveCriticalSection(g_LockData);return 13;};};
	int i;
	char len;
	char pos=0;

	// Podgotovka komandnoj stroki             //Write:
	ComDataWrite[pos]=UstrAdr;pos++;           //byte 0
	ComDataWrite[pos]=0x45; pos++;             //byte 1
	ComDataWrite[pos]=0x00;pos++;              //byte 2
	ComDataWrite[pos]=StartAdr;pos++;          //byte 3

	if (UstrAdr==1){for (i=0; i<Length; i++)
	{ComDataWrite[pos]=floor((float)RegistrSPM_int[StartAdr+i]/256);pos++;
	ComDataWrite[pos]=fmod((float)RegistrSPM_int[StartAdr+i],256);pos++;
	};
	};
	if (UstrAdr==2){for (i=0; i<Length; i++)
	{ComDataWrite[pos]=floor((float)RegistrTM_int[StartAdr-0x30+i]/256);pos++;
	ComDataWrite[pos]=fmod((float)RegistrTM_int[StartAdr-0x30+i],256);pos++;
	};
	};

	CRCcount('w',pos);                         //


	//Peredacha komandnoj stroki i priem otveta
	len=pos+2;
	//printf("%x",length); printf("\n");
	for (i=0;i<len;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
	//printf("\n");
	DWORD ret = 0;
	WriteFile(hCom1,ComDataWrite,len,&ret,NULL);

	//Read:
	//OK:    Error:
	//adres //byte 0 byte 0
	//kod funkcii //byte 1 byte 1(modificirovan)
	//kod error   //       byte 2
	//CRC         //byte 2 byte 3
	// CRC        //byte 3 byte 4
	len=4;
	ReadFile(hCom1,ComDataRead,len,&ret,NULL);
	for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
	//printf("\n");

	//Rasshifrovka otveta
	if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_LockData); return 10;}
	if (ComDataWrite[1]!=ComDataRead[1])
	{if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_LockData); return 11;} else
	{LeaveCriticalSection(g_LockData); return ComDataRead[2];};}
	else {CRCcount('r',len);

	};            // vichislenie CRC

	//proverka CRC
	if (ComDataRead[len]!=ComDataRead[len+2] ||
		ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_LockData); return 12;}
	LeaveCriticalSection(g_LockData);
	return 0;
}
/////////////////////////////


////////////////////////////
char ReadDataSPMSet() // Chtenie nastroek SPM
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC


{
	byte UstrAdr=1;
	byte StartAdr=0x0B;
	byte Length=12;
	byte temp;
	byte i;

	i=StartAdr;
	temp=ComReadIntReg(UstrAdr,StartAdr,Length);
	if (temp !=0) return temp;
	//Saw_A0=RegistrSPM_int[i];i++;
	//Saw_A1=RegistrSPM_int[i];i++;
	Ku0=RegistrSPM_int[i];i++;
	Ku1=RegistrSPM_int[i];i++;
	Taq=RegistrSPM_int[i];i++;
	Fwdt=RegistrSPM_int[i];i++;
	Pcount=RegistrSPM_int[i];i++;
	BaudRateSPM=RegistrSPM_int[i];i++;
	HostSPM=RegistrSPM_int[i];i++;
	ValidReqSPM=RegistrSPM_int[i];i++;
	CRCReqSPM=RegistrSPM_int[i];i++;
	ExcReqSPM=RegistrSPM_int[i];i++;
	return 0;
}
////////////////////////////
////////////////////////////
char ReadDataSPMCurrent() // Chtenie tekuchih dannih SPM
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC


{
	byte UstrAdr=1;
	byte StartAdr=0;
	byte Length=11;
	byte temp;
	byte i;

	i=StartAdr;
	temp=ComReadIntReg(UstrAdr,StartAdr,Length);
	if (temp !=0) return temp;
	Gamma0=RegistrSPM_int[i]*10;i++; //mks
	Gamma1=RegistrSPM_int[i]*10;i++;
	Haw0=RegistrSPM_int[i]*10;i++;
	Haw1=RegistrSPM_int[i]*10;i++;
	Am0=RegistrSPM_int[i];i++; //mV
	Am1=RegistrSPM_int[i];i++;
	T_Gen=RegistrSPM_int[i];i++; //C *100
	T_Sr0=RegistrSPM_int[i];i++;
	T_Sr1=RegistrSPM_int[i];i++;
	Ain0=RegistrSPM_int[i];i++;  //mV
	Ain1=RegistrSPM_int[i];i++;

	return 0;
}
////////////////////////////

////////////////////////////
char ComReadIntReg(char UstrAdr,char StartAdr, char Length) // Chtenie registrov
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC
	//          13-err adr komandy

	//-----Komanda-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =1-dathik; =2-terminal;
	//1-UstrCodeFunc - Kod funkcii (1 byte)- =0x44;
	//2-0        - start byte chtenija - (2 byte)
	//3-StartAdr - start byte chtenija
	//4-Length - dlina (byte) bloka chtenija
	//5-6-CRC ;
	//-----Otvet-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =UstrAdr v komande;
	//1-UstrCodeFunc - Vozvrat koda funkcii (1 byte)- =0x44 - esli OK;
	//=oxC4 - esli error
	//2- esli error - kod oshibki  0x01 - err koda func
	//0x02 - err adr reg
	//0x03 - err data
	//2...(Length*2)+1  - esli OK - data
	//3-4-esli error - CRC
	//(Length*2)+2... (Length*2)+3 - esli OK - CRC
{
	EnterCriticalSection(g_LockData);
	if (UstrAdr<1 || UstrAdr>2) {LeaveCriticalSection(g_LockData);return 10;};
	if (UstrAdr==1){if (StartAdr>0x16 || StartAdr+Length>0x17) {LeaveCriticalSection(g_LockData);return 13;};};
	if (UstrAdr==2){if (StartAdr<0x30) {LeaveCriticalSection(g_LockData);return 13;};
	if (StartAdr>0x35 || StartAdr+Length>0x36) {LeaveCriticalSection(g_LockData);return 13;};};
	int i;
	byte lenr;
	byte pos=0;

	// Podgotovka komandnoj stroki             //Write:
	ComDataWrite[pos]=UstrAdr;pos++;           //byte 0
	ComDataWrite[pos]=0x44; pos++;             //byte 1
	ComDataWrite[pos]=0x00;pos++;              //byte 2
	ComDataWrite[pos]=StartAdr;pos++;          //byte 3
	ComDataWrite[pos]=Length;pos++;            //byte 4
	CRCcount('w',pos);                         //byte 5-6
	lenr=pos+2;


	//Peredacha komandnoj stroki i priem otveta
	DWORD ret = 0;
	WriteFile(hCom1,ComDataWrite,lenr,&ret,NULL);
	//printf("%x",length);
	printf("\nTX:->");
	for (i=0;i<lenr;i++) {
		printf("%x ",ComDataWrite[i]);
	}
	//Read:
	//OK:    Error:
	//adres //byte 0 byte 0
	//kod funkcii //byte 1 byte 1(modificirovan)
	//kod error   //       byte 2
	//CRC         //byte 2 byte 3
	// CRC        //byte 3 byte 4
	lenr=(Length*2)+4;
	ReadFile(hCom1,ComDataRead,lenr,&ret,NULL);
	printf("\nRX:<-");
	for (i=0;i<lenr;i++) {
		printf("%x ",ComDataRead[i]);
	}
	//printf("\n");printf("\n");

	//Rasshifrovka otveta
	if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_LockData); return 10;}
	if (ComDataWrite[1]!=ComDataRead[1])
	{if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_LockData);return 11;} else
	{LeaveCriticalSection(g_LockData); return ComDataRead[2];};}
	else {
		CRCcount('r',(Length*2+2));
		//for (i=0;i<(Length*2)+6;i++) {printf("%x",ComDataRead[i]);printf(" ");};
		//printf("\n");
	};            // vichislenie CRC

	//proverka CRC
	if (ComDataRead[(Length*2)+2]!=ComDataRead[(Length*2)+2+2] ||
		ComDataRead[(Length*2)+3]!=ComDataRead[(Length*2)+3+2]){LeaveCriticalSection(g_LockData); return 12;}
	if (UstrAdr==1){for (i=0; i<Length; i++)
	{RegistrSPM_int[StartAdr+i]=ComDataRead[i*2+2]*256+ComDataRead[i*2+3];
	// //printf("%u",StartAdr+i);printf(" = ");
	// printf("%u",RegistrSPM_int[StartAdr+i]);printf("\n");
	};
	};
	if (UstrAdr==2){for (i=0; i<Length; i++)
	{RegistrTM_int[StartAdr+i]=ComDataRead[i*2+2]*256+ComDataRead[i*2+3];
	// printf("%u",StartAdr+i);printf(" = ");
	// printf("%u",RegistrTM_int[StartAdr+i]);printf("\n");
	};
	};
	LeaveCriticalSection(g_LockData);
	return 0;
}
/////////////////////////////



////////////////////////////
char ComCMode(char UstrAdr,char UstrFuncArgument) // Ustanovka regima raboti
	// vozvrat: 0-OK,
	//          1-err koda func, 2- err adr reg, 3-err data;
	//          10-err adr ustr, 11-err koda func, 12-err CRC


	//-----Komanda-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =1-dathik; =2-terminal; =3-PC Com2;
	//1-UstrCodeFunc - Kod funkcii (1 byte)- =0x42;
	//2-UstrMode - regim raboty (1 byte)- =0- Ini i ciclicheskoe izmerenie (SPM);
	// =1- Ini i odnokratniy prohod piloy (SPM);
	// =2- Reset (SPM & TM);
	//3-4-UstrCRC - (2 byte);
	//-----Otvet-----
	//0-UstrAdr - adres ustrojstva (1 byte)- =UstrAdr v komande;
	//1-UstrCodeFunc - Vozvrat koda funkcii (1 byte)- =0x42 - esli OK;
	//=oxC2 - esli error
	//2- esli error dalee kod oshibki  0x01 - err koda func
	//0x02 - err adr reg
	//0x03 - err data
	//2-3 (ili 3-4)-UstrCRC - (2 byte);
{
	EnterCriticalSection(g_LockData);
	int i;
	char pos=0;
	byte length;
	// Podgotovka komandnoj stroki             //Write:
	ComDataWrite[pos]=UstrAdr;pos++;           //byte 0
	ComDataWrite[pos]=0x42; pos++;             //byte 1
	ComDataWrite[pos]=UstrFuncArgument;pos++;  //byte 2
	CRCcount('w',pos);                         //byte 3-4
	length=pos+2;


	//Peredacha komandnoj stroki i priem otveta
	if (UstrAdr==0x1 || UstrAdr==0x2)
	{
		DWORD ret = 0;
		WriteFile(hCom1,ComDataWrite,length,&ret,NULL);
		//printf("%x",length); printf("\n");
		for (i=0;i<length;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
		//printf("       ");
		ReadFile(hCom1,ComDataRead,length-1,&ret,NULL);
	};
	if (UstrAdr==3)
	{
		//WriteFile(hCom2,&ComDataWrite,length,&ret,NULL);
		//ReadFile(hCom2,&ComDataRead,length-1,&ret,NULL);
	};
	//Read:
	//OK:    Error:
	//adres //byte 0 byte 0
	//kod funkcii //byte 1 byte 1(modificirovan)
	//kod error   //       byte 2
	//CRC         //byte 2 byte 3
	// CRC        //byte 3 byte 4
	//Rasshifrovka otveta
	//  if (ComDataWrite[0]!=ComDataRead[0]) return 10;
	//  if (ComDataWrite[1]!=ComDataRead[1])
	//      {if (ComDataRead[1]!= ComDataWrite[1]+0x80) {return 11;} else
	//           {return ComDataRead[2];};}
	//                else {CRCcount('r',2);};            // vichislenie CRC
	//  for (i=0;i<length+2;i++) {printf("%x",ComDataRead[i]);printf(" ");};
	//   printf("\n");
	//proverka CRC
	// if (ComDataRead[2]!=ComDataRead[2+2] || ComDataRead[3]!=ComDataRead[3+2]) return 12;
	LeaveCriticalSection(g_LockData);
	return 0;
}
/////////////////////////////




////////////////////////////
char CRCcount(char rw,byte Ln) // Vichislenie CRC
	// vozvrat: 0-vipolneno,
	// rw=='w' - vichislenie CRC dla heredachi (CRC dobavlaetsa v konce posilki)
	// rw=='r' - vichislenie CRC otveta (CRC dobavlaetsa posle prinjatogo CRC)
	// Len - chislo byte CRC kotorih vichislaetsja
{
	unsigned char CRCbyteLOW=0xFF ,CRCbyteHIGH=0xFF;
	int i,index;

	if (rw=='w')
	{
		for (i=0; i<Ln; i++)
		{
			index=CRCbyteHIGH^ComDataWrite[i];
			CRCbyteHIGH=CRCbyteLOW^crc_table[index];
			CRCbyteLOW=crc_table[index+256];
		}
		ComDataWrite[Ln+1]=CRCbyteLOW;
		ComDataWrite[Ln]=CRCbyteHIGH;
	}
	if (rw=='r')
	{
		for (i=0; i<Ln; i++)
		{
			index=CRCbyteHIGH^ComDataRead[i];
			CRCbyteHIGH=CRCbyteLOW^crc_table[index];
			CRCbyteLOW=crc_table[index+256];
		}
		ComDataRead[Ln+3]=CRCbyteLOW;
		ComDataRead[Ln+2]=CRCbyteHIGH;
	}

	return 0;

}
/////////////////////////////
//////////////////////////////////////

char IniCom1(int BaudRate)   //initializacija COM1
	//vozvrat: 0- norm; 1-error1; 2-error2;
{
	hCom1 = CreateFile( TEXT("COM1"), GENERIC_READ | GENERIC_WRITE,
		0,               // comm devices must be opened w/exclusive-access
		NULL,            // no security attrs
		OPEN_EXISTING,   // comm devices must use OPEN_EXISTING
		0,               // not overlapped I/O
		NULL             // hTemplate must be NULL for comm devices
		);
	DCB dcb;

	GetCommState(hCom1,&dcb);
	dcb.BaudRate =BaudRate;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_ENABLE;
	bool success = SetCommState( hCom1, &dcb);
	if(!success) {printf(" fail Com1\n");return 1;};     // fail
	COMMTIMEOUTS CommTimeOuts;
	GetCommTimeouts (hCom1,&CommTimeOuts);
	CommTimeOuts.ReadIntervalTimeout=100;
	CommTimeOuts.ReadTotalTimeoutMultiplier=100;
	CommTimeOuts.ReadTotalTimeoutConstant=100;
	// CommTimeOuts.WriteTotalTimeoutMultiplier=5;
	// CommTimeOuts.WriteTotalTimeoutConstant=5;
	success = SetCommTimeouts (hCom1,&CommTimeOuts);
	//printf("%i",CommTimeOuts.WriteTotalTimeoutMultiplier);
	printf("  ");
	//printf("%i",CommTimeOuts.WriteTotalTimeoutConstant);
	//printf("\n");
	if(!success) {printf(" fail Com1\n");return 2;};     // fail
	ReadKey();
	//printf("Com1 complit\n");
	//printf("BaudRate="); printf("%i",BaudRate);
	//printf("\n");
	return 0;
};

//////////////////////////////////////
//////////////////////////////////////

char IniCom2(int BaudRate)   //initializacija COM2
	//vozvrat: 0- norm; 1-error1; 2-error2;
{
	hCom2 = CreateFile( TEXT("COM2"), GENERIC_READ | GENERIC_WRITE,
		0,               // comm devices must be opened w/exclusive-access
		NULL,            // no security attrs
		OPEN_EXISTING,   // comm devices must use OPEN_EXISTING
		0,               // not overlapped I/O
		NULL             // hTemplate must be NULL for comm devices
		);
	DCB dcb;
	GetCommState(hCom2,&dcb);
	dcb.BaudRate =BaudRate;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_ENABLE;
	bool success = SetCommState( hCom2, &dcb);
	if(!success) {printf(" fail Com2\n");return 1;};     // fail
	COMMTIMEOUTS CommTimeOuts;
	GetCommTimeouts (hCom2,&CommTimeOuts);
	CommTimeOuts.ReadIntervalTimeout=10;
	CommTimeOuts.ReadTotalTimeoutMultiplier=10;
	CommTimeOuts.ReadTotalTimeoutConstant=10;
	success = SetCommTimeouts (hCom2,&CommTimeOuts);
	if(!success) {printf(" fail Com2\n");return 2;};     // fail
	//printf("Com2 complit\n");
	return 0;
};

//////////////////////////////////////

///////////////////////////////////////

char Reader(bool restore) //ctenie faylov nastroek
{	 
	bool exists = false;
	ReadMedium();
	char gradPath[256];
	strcpy(gradPath, CombinePath(applicationPath, restore ? "Grad.dat" : GetNumberedFileName(Grad)));
	char nastrPath[256];
	strcpy(nastrPath, CombinePath(applicationPath, restore ? "Nastr.dat" : GetNumberedFileName(Nastr)));
	char kalibrPath[256];
	strcpy(kalibrPath, CombinePath(applicationPath, restore ? "Kalibr.dat" : GetNumberedFileName(Kalibr)));
	char setPath[256];
	strcpy(setPath, CombinePath(applicationPath, restore ? "Set.dat" : GetNumberedFileName(Set)));
	//printf("Read");printf(" \n");
	FILE *fpg;
	fpg=fopen(gradPath, "r+b");
	if(fpg != NULL)
	{
		fread(&a0, sizeof(float), 1, fpg);
		fread(&a1, sizeof(float), 1, fpg);
		fread(&a2, sizeof(float), 1, fpg);
		fread(&a3, sizeof(float), 1, fpg);
		fread(&a4, sizeof(float), 1, fpg);
		fread(&a5, sizeof(float), 1, fpg);
		fread(&a6, sizeof(float), 1, fpg);
		fread(&a7, sizeof(float), 1, fpg);
		fread(&a8, sizeof(float), 1, fpg);
		fread(&a9, sizeof(float), 1, fpg);
		fread(&AA, sizeof(float), 1, fpg);
		fread(&BB, sizeof(float), 1, fpg);
		fread(&k1, sizeof(float), 1, fpg);
		fread(&Wmax, sizeof(float), 1, fpg);
		fread(&Wmin, sizeof(float), 1, fpg);
		fclose(fpg);
		exists = true;
	}

	FILE *fpn; fpn=fopen(nastrPath, "r+b");
	int fake;
	if(fpn != NULL)
	{
		//fread(&NomGrad, sizeof(int), 1, fpn);
		fread(&fake, sizeof(int), 1, fpn);
		fread(&VZ, sizeof(int), 1, fpn);
		fread(&VO, sizeof(int), 1, fpn);
		fread(&VI, sizeof(int), 1, fpn);
		fread(&Tak, sizeof(int), 1, fpn);
		fread(&UstRele, sizeof(int), 1, fpn);
		fread(&AK, sizeof(int), 1, fpn);
		fread(&Aout_tip, sizeof(unsigned int), 1, fpn);
		fread(&DacOffs, sizeof(unsigned int),1,fpn); //DAC offset
		fread(&DacScale, sizeof(unsigned int),1,fpn); //DAC scale
		fread(&PotVal, sizeof(unsigned int),1,fpn); //potentiometer value
		fread(&N_Rez, sizeof(float), 1, fpn);
		fread(&K_Rez, sizeof(float), 1, fpn);
		fread(&Step_Rez, sizeof(float), 1, fpn);
		fclose(fpn);
		exists = true;
	}

	FILE *fpk; fpk=fopen(kalibrPath, "r+b");
	if(fpk != NULL)
	{
		fread(&Am0_k, sizeof(int), 1, fpk);
		fread(&Haw0_k, sizeof(int), 1, fpk);
		fread(&Gamma0_k, sizeof(int), 1, fpk);
		fread(&T_Gen_k, sizeof(int), 1, fpk);
		fread(&T_Sr0_k, sizeof(int), 1, fpk);
		fclose(fpk);
		exists = true;
	}

	bool rewrite = false;
	FILE *fps; fps=fopen(setPath, "r+b");
	if(fps != NULL)
	{
		fread(&Va, sizeof(float), 1, fps);
		fread(&kgt, sizeof(float), 1, fps);
		fread(&kgv, sizeof(float), 1, fps);
		fread(&T, sizeof(float), 1, fps);
		fread(&Kol_dat, sizeof(int), 1, fps);
		fread(&Nom_dat1, sizeof(int), 1, fps);
		fread(&Nom_dat2, sizeof(int), 1, fps);
		fread(&Nom_ind, sizeof(int), 1, fps);
		fread(&Saw_A0, sizeof(int), 1, fps);
		fread(&Saw_A1, sizeof(int), 1, fps);
		fread(&controlleradr, sizeof(int), 1, fps);
		fread(&Ku0, sizeof(int), 1, fps);
		fread(&Ku1, sizeof(int), 1, fps);
		fread(&Nzp, sizeof(int), 1, fps);
		fread(&Poralfa, sizeof(float), 1, fps);
		fread(&Porgamma, sizeof(float), 1, fps);
		if(!feof(fps))
		{
			fread(&Porbeta, sizeof(float), 1, fps);
		}
		else
		{
			Porbeta = 0;
			rewrite = true;
		}
		fclose(fps);
		if(rewrite)
		{
			SaveSet();
		}
		exists = true;
	}
	//printf("Read OK");printf(" \n");

	//if(!exists) ReaderOld();
	return 0;
}

//////////////////////////////////////
//modified by SL 18.06.09
//modified by SL 23.12.2009 add data output throught COM2
char Write2Master() //write to master host
{
	/*
	GetCommState(hCom2,&dcb2);
	dcb2.BaudRate =9600;
	dcb2.ByteSize = 8;
	dcb2.Parity = NOPARITY;
	dcb2.StopBits = ONESTOPBIT;
	dcb2.fDtrControl = DTR_CONTROL_DISABLE;
	dcb2.fRtsControl = RTS_CONTROL_ENABLE;

	fSuccess = SetCommState( hCom2, &dcb2);

	// printf("%x",temp[23]);

	temp[0]='A';temp[1]='T';temp[2]=' ';
	//temp[3]='1';temp[4]='4';
	temp[5]=' ';
	// y=2 -  NAZNACHENIE NOMERA PRIBORA DLA UDALENNOGO KOMPUTERA (2);

	y=1; for (i=2;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[i+2]=z+48;};
	temp[6]='1';temp[7]=' ';temp[8]='1';temp[9]=' ';
	y=W*100; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[i+15]=z+48;};
	temp[16]=temp[17];temp[17]=temp[18];temp[18]='.';
	temp[21]=' ';
	y=T_Sr0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[i+9]=z+48;};
	temp[14]=temp[13];temp[13]='.';
	temp[15]=' ';
	temp[22]='$';
	CRC=0xA5;for (i=0;i<23;i++) {CRC=CRC+temp[i];};
	temp[23]=CRC;
	len=24;
	for (i=0;i<24;i++) {printf("%c",temp[i]);};
	//printf("%x",temp[23]);
	//SleepWithService(10000);
	*/

	printf("\nhumidity %f %i", W, T_Sr0);
	if(CheckFilling())
	{

		unsigned char wHigth = (unsigned char)W;
		unsigned char wLow = (unsigned char)(((W - (unsigned char)W)) * 100);

		unsigned char tHigth = (unsigned char)(T_Sr0 / 100);
		unsigned char tLow = (unsigned char)(T_Sr0 -(T_Sr0/100) * 100);

		masterbuffer[0] = wHigth;
		masterbuffer[1] = wLow;
		masterbuffer[2] = tHigth;
		masterbuffer[3] = tLow;
		dataSent = false;
	}
	else
	{
		masterbuffer[0] = 0;
		masterbuffer[1] = 0;
		masterbuffer[2] = 0;
		masterbuffer[3] = 0;

		dataSent = false;
	}
	//WriteFile(hCom2,&temp,len,&ret,NULL);
	//SleepWithService(3000);
	return 0;
}
//write to remote indicator
char WriteUBI() //display temperature and humidity on remote indicator
{
	EnterCriticalSection(g_LockData);
	int i;
	char len;
	char pos=0;
	float prt1 = 0, prt2;
	byte h4,h3,h2,h1;

	// Podgotovka komandnoj stroki             //Write:
	ComDataWrite[pos]=0x03;pos++; 	          //select indicator
	ComDataWrite[pos]=0x10;pos++;   	          //write multiple registers
	ComDataWrite[pos]=0x00;pos++;              //
	ComDataWrite[pos]=0x00;pos++;          	 //starting adress
	ComDataWrite[pos]=0x00;pos++;              //
	ComDataWrite[pos]=0x02;pos++;          	 //quantity
	ComDataWrite[pos]=0x04;pos++;          	 //byte count


	//humidity
	if (W<100)
	{
	byte numSet[5];
	float y,z;

	y=W*1000;
	for(int i=5; i!=0; i--){
		z=fmod(y,10);
		y=floor(y/10);
		numSet[i]=z;
	};
	//
	h4=numSet[1];
	h3=numSet[2];
	h2=numSet[3];
	h1=numSet[4];

	ComDataWrite[pos]=(byte)(16*h2+h1);
	pos++;
	ComDataWrite[pos]=(byte)(16*h4+h3);
	pos++;}
	else
	{
		ComDataWrite[pos]=0xDD;
		pos++;
		ComDataWrite[pos]=0xAC;
		pos++;};
		//temperature
		prt1=(float)(T_Sr0)/10000;
		prt2=floor(prt1*10);
		h4=(byte)(prt2);
		prt1=prt1*10-prt2;
		prt2=floor(prt1*10);
		h3=(byte)(prt2);
		prt1=prt1*10-prt2;
		prt2=floor(prt1*10);
		h2=(byte)(prt2);
		prt1=prt1*10-prt2;
		prt2=floor(prt1*10);
		h1=(byte)(prt2);
		prt1=prt1*10-prt2;
		ComDataWrite[pos]=(byte)(16*h2+h1);
		pos++;
		ComDataWrite[pos]=(byte)(16*h4+h3);
		pos++;
		//add CRC
		CRCcount('w',pos);                         //


		//Peredacha komandnoj stroki i priem otveta
		len=(char)(pos+2);
		printf("Remote indicator message \n");
		printf("W= %f\n",W);
		//printf("Tsr0= %i\n",T_Sr0);
		for (i=0;i<len;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
		//printf("\n");
		DWORD ret = 0;
		WriteFile(hCom2,ComDataWrite,len,&ret,NULL);

		//Read:
		//OK:    Error:
		//adres //byte 0 byte 0
		//kod funkcii //byte 1 byte 1(modificirovan)
		//kod error   //       byte 2
		//CRC         //byte 2 byte 3
		// CRC        //byte 3 byte 4
		len=8;
		ReadFile(hCom2,ComDataRead,len,&ret,NULL);
		for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
		printf("\n");
		//      SleepWithService(3000);

		//Rasshifrovka otveta
		if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_LockData); return 10;}
		if (ComDataWrite[1]!=ComDataRead[1])
		{if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_LockData); return 11;} else
		{LeaveCriticalSection(g_LockData);return ComDataRead[2];};}
		else {CRCcount('r',len);

		};            // vichislenie CRC

		//proverka CRC
		if (ComDataRead[len]!=ComDataRead[len+2] ||
			ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_LockData); return 12;}
		LeaveCriticalSection(g_LockData);
		return 0;
}



//////////////////////////////////////

char Aout() //
{
	if(Aout_tip <1 || Aout_tip> 3) return -1;
	//char temp[50];
	byte CRC;
	int ACout0,ACout1;
	int DacData;

	float Kout;
	Kout=(DacScale-DacOffs)/(Wmax-Wmin != 0 ? Wmax-Wmin : 1);
	DacData=(int)(floor((W-Wmin)*Kout)+DacOffs); //output scaling
	/*if (DacData>1220){
		DacData=60;
	};
	if (DacData<60){
		DacData=60;
	};*/

	RegistrTM_int[2]=DacData; //set current dac value
	RegistrTM_int[3]=0;
	RegistrTM_int[4]=PotVal; //set potentiometer value
	RegistrTM_int[5]=0x40;
	ComWriteIntReg(0x02,0x32,4); //setup DAC registers
	Sleep(200);

	//printf("%u",ACout0); printf("    ");printf("%u",DacData);printf("\n");
	//SleepWithService(2000);
	return 0;
}

//////////////////////////////////////

char combinePathStr[255];
char * CombinePath(const char * firstPart, const char * secondPart)
{
	strcpy(combinePathStr, firstPart);
	strcat(combinePathStr, PATH_SEPARATOR);
	strcat(combinePathStr, secondPart);
	return combinePathStr;
}

void SetPathDir(char * applicationName)
{
	int applicationNameSize = strlen(strrchr(applicationName,'\\'));
	int pathLength = strlen(applicationName) - applicationNameSize + 1;
	strncpy(applicationPath, applicationName, pathLength - 1);
	applicationPath[pathLength] = 0;
}


void T2A(char * dest, TCHAR * source, int length)
{
	for (int i =0; i < length; i++)
	{
		dest[i] = (char)source[i];
	}
	dest[length] = 0;
}

void TimedIncrementT_Gen_T_Sr0_T_Sr1_W(bool setAverage, int t_Gen, int t_Sr0, int t_Sr1, float w)
{
	static int means_count = 0;
	static int T_Gen_sum = 0;
	static int T_Sr0_sum = 0, T_Sr1_sum = 0;
	static float W_sum = 0;   

	if(!setAverage)
	{	   
		means_count++;
		T_Gen_sum += t_Gen;
		T_Sr0_sum += t_Sr0;
		T_Sr1_sum += t_Sr1;
		W_sum += w;
	}
	else   
	{
		if(means_count > 0)
		{
			T_Gen = (int)floor((float)T_Gen_sum / means_count);	   
			T_Sr0 = (int)floor((float)T_Sr0_sum / means_count);
			T_Sr1 = (int)floor((float)T_Sr1_sum / means_count);
			W = W_sum / means_count;
		}
		means_count = 0;
		T_Gen_sum = T_Sr0_sum = T_Sr1_sum = 0;
		W_sum = 0;
	}
}

DWORD ReadKeyThreadProc (LPVOID lpdwThreadParam)
{
	lpdwThreadParam;

	while(true)
	{
		Sleep(400);

		char tempchar = ReadKeyBuffered();
		if(tempchar != 'Z')
		{
			keyBuffer = tempchar;
		}
		CheckShutdown();
	}

	return 0;
}

unsigned long ClockToMilliseconds(clock_t value)
{
	return value * 1000 / CLOCKS_PER_SEC;
}

char WaitNoKey()
{
	do {
		Sleep(50);
		char key = ReadKeyBuffered();
		if(key == 'Z')
			return key;
	}
	while (true);
}

char WaitKey()
{
	do 
	{
		Sleep(50);
		char key = ReadKeyBuffered();
		if(key!='Z'&&key!='S')
			return key;
	}
	while (true);
}

char WaitAnyKey()
{		
	char key = 0;
	do
	{
		Sleep(50);
		char key = ReadKeyBuffered();
		if(key!='Z')
			return key;
	}
	while (key=='Z');

	return 'Z';
}

bool CheckFilling()
{
	printf("Haw0 = %i",Haw0);
	bool filling = Nzp == 0 ||
		!(Nzp==1 && Am0>Poralfa) &&
		!(Nzp==2 && Am0<Poralfa) &&
		!(Nzp==3 && (Haw0 / 1000.0)>Porbeta) &&
		!(Nzp==4 && (Haw0 / 1000.0)<Porbeta) &&
		!(Nzp==5 && (Gamma0 / 1000.0)>Porgamma) &&
		!(Nzp==6 && (Gamma0 / 1000.0)<Porgamma);
	return filling;
}


bool ShutdownServer()
{
	HANDLE hdlProcessHandle;
	HANDLE hdlTokenHandle;
	LUID tmpLuid;
	TOKEN_PRIVILEGES tkp;
	TOKEN_PRIVILEGES tkpNewButIgnored;
	DWORD lBufferNeeded = sizeof(TOKEN_PRIVILEGES);

	hdlProcessHandle = GetCurrentProcess();
	OpenProcessToken(hdlProcessHandle, TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY, &hdlTokenHandle);


	LookupPrivilegeValue(NULL, SE_SHUTDOWN_NAME, &tmpLuid);
	tkp.PrivilegeCount = 1;
	tkp.Privileges[0].Luid = tmpLuid;
	tkp.Privileges[0].Attributes = 0;
	tkp.Privileges[0].Attributes |= (SE_PRIVILEGE_ENABLED);

	AdjustTokenPrivileges(hdlTokenHandle, false, &tkp,sizeof(TOKEN_PRIVILEGES), &tkpNewButIgnored, &lBufferNeeded);
	DWORD error = GetLastError();
	if (error != ERROR_SUCCESS)
	{
		//printf("Cannot poweroff computer.");
		return false;
	}

	ExitWindowsEx(EWX_POWEROFF, SHTDN_REASON_MAJOR_OTHER);			
	error = GetLastError();
	if (error != ERROR_SUCCESS)
	{
		//printf("Cannot poweroff computer.");
		return false;
	}
	return true;
}

void SleepWithService(long sleeptime)
{	
	while(sleeptime >= servicetime && servicetime > 0)
	{
		sleeptime -= servicetime;
		Sleep(servicetime);
		if(sleeptime >= READKEY_INTERVAL)
		{
			sleeptime -= READKEY_INTERVAL;
			Sleep(READKEY_INTERVAL);
		}
		//char * buffer = new char[256];
		//WriteLog(itoa(servicetime,buffer, 10));
		//delete [] buffer;
	}

	if(sleeptime > 0)
	{
		Sleep(sleeptime);
	}
}

void CheckShutdown()
{
	if(keyBuffer == 'H')
	{
		ClrScr();
		if(ShutdownServer())
		{
			exit(0);
		}
	}
}


char translated[255];

char * GetFileRootName(FileType type)
{
	switch(type)
	{
	case Grad :
		return "Grad";
	case Nastr :
		return "Nastr";
	case Kalibr :
		return "Kalibr";
	case Set :
		return "Set";
	}
	return "Default";
}

char translatedFileName[255];
char * GetNumberedFileName(FileType type)
{
	char * rootstr = GetFileRootName(type);
	char mediumstr[2];
	mediumstr[0] = NomGrad + '0';
	mediumstr[1] = 0;

	strcpy(translatedFileName, rootstr);
	if(strlen(mediumstr) > 0)
	{
		strcat(translatedFileName, "_");
	}
	strcat(translatedFileName, mediumstr);
	strcat(translatedFileName, ".dat");

	return translatedFileName;
}


char * GetFullRow(char * buffer)
{
	int bufferlength = strlen(buffer) < ROW_WIDTH - 1 ? strlen(buffer) : ROW_WIDTH - 1;
	strcpy(fullrow, emptyrow);
	strncpy(fullrow, buffer, bufferlength);
	fullrow[bufferlength] = 0;
	strncat(fullrow, emptyrow, ROW_WIDTH - strlen(buffer) - 1);
	fullrow[ROW_WIDTH - 1] = 0;

	return fullrow;
}

char centeredStr[21];
char * GetCenteredRow(char * buffer)
{
	int rigntPadding = strlen(buffer);
	for(rigntPadding = strlen(buffer); rigntPadding > 0; rigntPadding--) { if(buffer[rigntPadding - 1] != ' ') { break;} };

	int leftPadding = (ROW_WIDTH - 1 - rigntPadding) / 2;
	char * leftPaddingStr = new char[rigntPadding + leftPadding  + 1];

	strncpy(leftPaddingStr, emptyrow, leftPadding );
	leftPaddingStr[leftPadding ] = 0;
	strncat(leftPaddingStr, buffer, rigntPadding);
	leftPaddingStr[rigntPadding + leftPadding] = 0;


	char * fullrow = GetFullRow(leftPaddingStr);
	delete [] leftPaddingStr;
	strcpy(centeredStr, fullrow);
	centeredStr[strlen(fullrow)] = 0;
	return centeredStr;
}

bool HasSomeDefaultFiles()
{
	struct stat fileinfo;
	return !stat(CombinePath(applicationPath, "Grad.dat"),(struct stat *)&fileinfo) || !stat( CombinePath(applicationPath, "Nastr.dat"),(struct stat *)&fileinfo) || !stat(CombinePath(applicationPath, "Kalibr.dat"),(struct stat *)&fileinfo) || !stat(CombinePath(applicationPath, "Set.dat"),(struct stat *)&fileinfo);
}

void SetDefaultValues()
{
	a0 = 0;
	a1 = 0;
	a2 = 0;
	a3 = 0;
	a4 = 0;
	a5 = 0;
	a6 = 0;
	a7 = 0;
	a8 = 0;
	a9 = 0;
	AA = 0;
	BB = 0;
	k1 = 0;
	Wmin = 0;
	Wmax = 0;

	//NomGrad = 0;
	VZ = 0;
	VO = 0;
	VI = 0;
	Tak = 0;
	UstRele = 0;
	AK = 0;
	Aout_tip = 1;
	N_Rez = 0;
	K_Rez = 0;
	Step_Rez = 0;

	Am0 = 0;
	Haw0 = 0;
	Gamma0 = 0;
	T_Gen = 0;
	T_Sr0 = 0;

	Am0_k = 0;
	Haw0_k = 0;
	Gamma0_k = 0;
	T_Gen_k = 0;
	T_Sr0_k = 0;

	Va = 0;
	kgt = 0;
	kgv = 0;
	T = 0;
	Kol_dat =0;
	Nom_dat1 = 0;
	Nom_dat2 = 0;
	Nom_ind = 0;
	Saw_A0 = 0;
	Saw_A1 = 0;
	controlleradr = 0;
	Ku0 = 0;
	Ku1 = 0;
	Nzp = 0;
	Poralfa = 0;
	Porbeta = 0;
	Porgamma = 0;
}

void SaveGrad()
{
	char * gradPath = CombinePath(applicationPath, GetNumberedFileName(Grad));
	FILE *fp; fp=fopen(gradPath, "w+b");

	if(fp == NULL) return;
	fwrite(&a0, sizeof(float), 1, fp);
	fwrite(&a1, sizeof(float), 1, fp);
	fwrite(&a2, sizeof(float), 1, fp);
	fwrite(&a3, sizeof(float), 1, fp);
	fwrite(&a4, sizeof(float), 1, fp);
	fwrite(&a5, sizeof(float), 1, fp);
	fwrite(&a6, sizeof(float), 1, fp);
	fwrite(&a7, sizeof(float), 1, fp);
	fwrite(&a8, sizeof(float), 1, fp);
	fwrite(&a9, sizeof(float), 1, fp);
	fwrite(&AA, sizeof(float), 1, fp);
	fwrite(&BB, sizeof(float), 1, fp);
	fwrite(&k1, sizeof(float), 1, fp);
	fwrite(&Wmax, sizeof(float), 1, fp);
	fwrite(&Wmin, sizeof(float), 1, fp);
	fclose(fp);
}

void SaveNastr()
{
	char * nastrPath = CombinePath(applicationPath, GetNumberedFileName(Nastr));
	FILE *fp; fp=fopen(nastrPath, "w+b");

	int fake = 0;
	if(fp == NULL) return;
	//fwrite(&NomGrad, sizeof(int), 1, fp);
	fwrite(&fake, sizeof(int), 1, fp);
	fwrite(&VZ, sizeof(int), 1, fp);
	fwrite(&VO, sizeof(int), 1, fp);
	fwrite(&VI, sizeof(int), 1, fp);
	fwrite(&Tak, sizeof(int), 1, fp);
	fwrite(&UstRele, sizeof(int), 1, fp);
	fwrite(&AK, sizeof(int), 1, fp);
	fwrite(&Aout_tip, sizeof(unsigned int), 1, fp);
	fwrite(&DacOffs, sizeof(unsigned int),1,fp); //DAC offset
	fwrite(&DacScale, sizeof(unsigned int),1,fp); //DAC scale
	fwrite(&PotVal, sizeof(unsigned int),1,fp); //potentiometer value
	fwrite(&N_Rez, sizeof(float), 1, fp);
	fwrite(&K_Rez, sizeof(float), 1, fp);
	fwrite(&Step_Rez, sizeof(float), 1, fp);
	fclose(fp);
}

void SaveKalibr()
{
	char * kalibrPath = CombinePath(applicationPath, GetNumberedFileName(Kalibr));
	FILE *fp; fp=fopen(kalibrPath, "w+b");

	if(fp == NULL) return;
	fwrite(&Am0, sizeof(int), 1, fp);
	fwrite(&Haw0, sizeof(int), 1, fp);
	fwrite(&Gamma0, sizeof(int), 1, fp);
	fwrite(&T_Gen, sizeof(int), 1, fp);
	fwrite(&T_Sr0, sizeof(int), 1, fp);
	fclose(fp);
}



void CopyKalibr()
{
	char * kalibrPath = CombinePath(applicationPath, GetNumberedFileName(Kalibr));
	FILE *fp; fp=fopen(kalibrPath, "w+b");

	if(fp == NULL) return;
	fwrite(&Am0_k, sizeof(int), 1, fp);
	fwrite(&Haw0_k, sizeof(int), 1, fp);
	fwrite(&Gamma0_k, sizeof(int), 1, fp);
	fwrite(&T_Gen_k, sizeof(int), 1, fp);
	fwrite(&T_Sr0_k, sizeof(int), 1, fp);
	fclose(fp);
}

void SaveSet()
{
	char * setPath = CombinePath(applicationPath, GetNumberedFileName(Set));
	FILE *fp; fp=fopen(setPath, "w+b");

	if(fp == NULL) return;
	fwrite(&Va, sizeof(float), 1, fp);
	fwrite(&kgt, sizeof(float), 1, fp);
	fwrite(&kgv, sizeof(float), 1, fp);
	fwrite(&T, sizeof(float), 1, fp);
	fwrite(&Kol_dat, sizeof(int), 1, fp);
	fwrite(&Nom_dat1, sizeof(int), 1, fp);
	fwrite(&Nom_dat2, sizeof(int), 1, fp);
	fwrite(&Nom_ind, sizeof(int), 1, fp);
	fwrite(&Saw_A0, sizeof(int), 1, fp);
	fwrite(&Saw_A1, sizeof(int), 1, fp);
	fwrite(&controlleradr, sizeof(int), 1, fp);
	fwrite(&Ku0, sizeof(int), 1, fp);
	fwrite(&Ku1, sizeof(int), 1, fp);
	fwrite(&Nzp, sizeof(int), 1, fp);
	fwrite(&Poralfa, sizeof(float), 1, fp);
	fwrite(&Porgamma, sizeof(float), 1, fp);
	fwrite(&Porbeta, sizeof(float), 1, fp);
	fclose(fp);
}

void SaveMedium()
{
	char * mediumPath = CombinePath(applicationPath, "Media.dat");
	FILE * file = fopen(mediumPath, "w");
	if(file == NULL) return;

	for(int i = 0; i < MEDIUM_COUNT; i++)
	{
		if(mediums[i][0] != 0) 
		{
			fwrite(mediums[i], sizeof(char), ROW_WIDTH, file);
		}
		else
		{
			break;
		}
	}

	fclose(file);
}

void ReadMedium()
{
	for(int i =0; i < MEDIUM_COUNT; i++)
	{
		mediums[i][0] = 0;
	}

	char * mediumPath = CombinePath(applicationPath, "Media.dat");
	FILE * file = fopen(mediumPath, "r");
	if(file == NULL) return;

	int index = 0;


	while(!feof(file))
	{	  
		if(index == 10) break;

		int reallength = fread(mediums[index], sizeof(char), ROW_WIDTH, file);
		mediums[index][reallength] = 0;
		if(reallength > 0)
		{
			index++;
		}      
	}

	media_count = index;

	fclose(file);
}

void SetMedium(char * newMediumName)
{	
	GetFullRow(newMediumName);
	strcpy(mediumName, fullrow);
	mediumName[strlen(fullrow)] = 0;	

	for(int i =0; i < MEDIUM_COUNT; i++)
	{
		if(mediums[i][0] == 0) 
		{
			strcpy(mediums[i], mediumName);
			mediums[i][strlen(mediumName)] = 0;
			break; 
		};
	}
}

bool ReadDefaultMedium()
{
	char * mediumPath = CombinePath(applicationPath, "Media.dat");
	FILE * file = fopen(mediumPath, "r");

	if(file == NULL)
	{		
		return false;
	}
	int reallength = fread(mediumName, sizeof(char), ROW_WIDTH - 1, file);
	mediumName[ROW_WIDTH - 1] = 0;

	fclose(file);
	if(reallength)
	{
		mediumName[reallength] = 0;
		SetMedium(mediumName);
		return true;
	}
	return false;
}

void SaveAll()
{
	SaveMedium();
	SaveGrad();
	SaveNastr();
	SaveKalibr();
	SaveSet();
}

char * SetMediumRow(int newIndex)
{
	if(newIndex == 1)
	{
		NomGrad++;
		NomGrad%=media_count;
	}
	if(newIndex == 2)
	{
		NomGrad=media_count - 1;
	}

	return GetFullRow(mediums[NomGrad]);
}



void CompliteAndSendData(CommandEntity * entity)
{
	//printf("\r\nentity address %i\r\n",entity->GetAddress());
	if(controlleradr == entity->GetAddress())
	{
		switch(entity->identifier)
		{
		case BeginDataSend :
			master.SendResponse(entity);
			break;
		case SendDataPacket :
			if(!dataSent)
			{
				master.SendDataPacket(
					controlleradr,
					0,
					4,
					masterbuffer,
					4);			
				dataSent = true;
			}
			else
			{
				master.DataTransactionComplited(controlleradr);
			}
		case DataPacketReceived:
			dataSent = false;
			break;
		case RegisterController :
			master.RegisterController(controlleradr, RS485);
			break;
		}
	}
}

void InitialiseBuffer(unsigned char * const buffer, int length)
{	
	for(int i=0; i < length; i++)
	{
		buffer[i] = 0;
	}
}




///////////////////////////////////////
char ReaderOld() //ctenie faylov nastroek
{	 
	bool exists = false;
	ReadMedium();
	char gradPath[256];
	strcpy(gradPath, CombinePath(applicationPath, GetTranslatedFileName(Grad, Trim(mediumName))));
	char nastrPath[256];
	strcpy(nastrPath, CombinePath(applicationPath, GetTranslatedFileName(Nastr, Trim(mediumName))));
	char kalibrPath[256];
	strcpy(kalibrPath, CombinePath(applicationPath, GetTranslatedFileName(Kalibr, Trim(mediumName))));
	char setPath[256];
	strcpy(setPath, CombinePath(applicationPath, GetTranslatedFileName(Set, Trim(mediumName))));
	FILE *fpg;
	fpg=fopen(gradPath, "r+b");
	if(fpg != NULL)
	{
		fread(&a0, sizeof(float), 1, fpg);
		fread(&a1, sizeof(float), 1, fpg);
		fread(&a2, sizeof(float), 1, fpg);
		fread(&a3, sizeof(float), 1, fpg);
		fread(&a4, sizeof(float), 1, fpg);
		fread(&a5, sizeof(float), 1, fpg);
		fread(&a6, sizeof(float), 1, fpg);
		fread(&a7, sizeof(float), 1, fpg);
		fread(&a8, sizeof(float), 1, fpg);
		fread(&a9, sizeof(float), 1, fpg);
		fread(&AA, sizeof(float), 1, fpg);
		fread(&BB, sizeof(float), 1, fpg);
		fread(&k1, sizeof(float), 1, fpg);
		fread(&Wmax, sizeof(float), 1, fpg);
		fread(&Wmin, sizeof(float), 1, fpg);
		fclose(fpg);
		exists = true;
	}

	FILE *fpn; fpn=fopen(nastrPath, "r+b");
	int fake;
	if(fpn != NULL)
	{
		//fread(&NomGrad, sizeof(int), 1, fpn);
		fread(&fake, sizeof(int), 1, fpn);
		fread(&VZ, sizeof(int), 1, fpn);
		fread(&VO, sizeof(int), 1, fpn);
		fread(&VI, sizeof(int), 1, fpn);
		fread(&Tak, sizeof(int), 1, fpn);
		fread(&UstRele, sizeof(int), 1, fpn);
		fread(&AK, sizeof(int), 1, fpn);
		fread(&Aout_tip, sizeof(int), 1, fpn);
		fread(&N_Rez, sizeof(float), 1, fpn);
		fread(&K_Rez, sizeof(float), 1, fpn);
		fread(&Step_Rez, sizeof(float), 1, fpn);
		fclose(fpn);
		exists = true;
	}

	FILE *fpk; fpk=fopen(kalibrPath, "r+b");
	if(fpk != NULL)
	{
		fread(&Am0_k, sizeof(int), 1, fpk);
		fread(&Haw0_k, sizeof(int), 1, fpk);
		fread(&Gamma0_k, sizeof(int), 1, fpk);
		fread(&T_Gen_k, sizeof(int), 1, fpk);
		fread(&T_Sr0_k, sizeof(int), 1, fpk);
		fclose(fpk);
		exists = true;
	}

	bool rewrite = false;
	FILE *fps; fps=fopen(setPath, "r+b");
	if(fps != NULL)
	{
		fread(&Va, sizeof(float), 1, fps);
		fread(&kgt, sizeof(float), 1, fps);
		fread(&kgv, sizeof(float), 1, fps);
		fread(&T, sizeof(float), 1, fps);
		fread(&Kol_dat, sizeof(int), 1, fps);
		fread(&Nom_dat1, sizeof(int), 1, fps);
		fread(&Nom_dat2, sizeof(int), 1, fps);
		fread(&Nom_ind, sizeof(int), 1, fps);
		fread(&Saw_A0, sizeof(int), 1, fps);
		fread(&Saw_A1, sizeof(int), 1, fps);
		fread(&controlleradr, sizeof(int), 1, fps);
		fread(&Ku0, sizeof(int), 1, fps);
		fread(&Ku1, sizeof(int), 1, fps);
		fread(&Nzp, sizeof(int), 1, fps);
		fread(&Poralfa, sizeof(float), 1, fps);
		fread(&Porgamma, sizeof(float), 1, fps);
		if(!feof(fps))
		{
			fread(&Porbeta, sizeof(float), 1, fps);
		}
		else
		{
			Porbeta = 0;
			rewrite = true;
		}
		fclose(fps);
		if(rewrite)
		{
			SaveSet();
		}
		exists = true;
	}

	if(exists) 
	{
		for(int i = 2; i < MEDIUM_COUNT;i++)
		{
			strcpy(mediums[i - 1], mediums[i]);
			mediums[i - 1][strlen(mediums[i])] = 0;
		}
		mediums[MEDIUM_COUNT - 1][0] = 0;
		media_count--;

		CopyKalibr();
		SaveMedium();
		SaveGrad();
		SaveNastr();
		SaveSet();
	}
	return 0;
}




char * GetTranslatedFileName(FileType type, char * mediumName)
{
	char * rootstr = GetFileRootName(type);
	char * mediumstr = GetTranslatedString(mediumName);
	int length = strlen(rootstr) + strlen(mediumstr) + strlen(mediumstr) > 0 ? strlen("_.dat") : strlen(".dat");

	strcpy(translatedFileName, rootstr);
	if(strlen(mediumstr) > 0)
	{
		strcat(translatedFileName, "_");
	}
	strcat(translatedFileName, mediumstr);
	strcat(translatedFileName, ".dat");

	return translatedFileName;
}




char * GetTranslatedString(char * source)
{
	if(source == NULL)
		return "";

	int initlength = strlen(source);

	translated[0] = 0;

	for(int i=0;i<initlength;i++)
	{
		if(i < initlength - 1 && source[i] == 'к' && source[i+1] == 'с')
		{
			strcat(translated, notation[32]);
			i++;
			continue;
		}

		if(i < initlength - 1 && source[i] == 'ы' && source[i+1] == 'й')
		{
			strcat(translated, notation[33]);
			i++;
			continue;
		}

		if(i < initlength - 1 && source[i] == 'и' && source[i+1] == 'й')
		{
			strcat(translated, notation[34]);
			i++;
			continue;
		}

		if(source[i] == '_')
		{
			strcat(translated, notation[35]);
			continue;
		}

		if(source[i] >= 'а' && source[i] <= 'я')
		{
			strcat(translated, notation[(int)source[i] - (int)'а']);
		}
		else
		{
			int currentLength = strlen(translated);
			translated[currentLength] = source[i];
			translated[currentLength + 1] = 0;
		}
	}
	return translated;
}



char * Trim(char * source)
{
	trimStr[0] = 0;
	int i = 0;
	int index = 0;
	while(source[i] != 0 && source[i] == ' ' )
	{
		i++;
	}

	while(source[i] != 0 && source[i] != ' ' )
	{
		trimStr[index++] = source[i++];
	}

	return trimStr;
}
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include <string.h>
#include <direct.h>
#include <conio.h>
#include "MWTSoft2.h"
#include <windows.h>
#include <math.h>
#include <time.h>

extern char Calc(); // Vichislenie vlajnosti
extern char WriteUBI();
extern char Write2Master();
extern char Aout(); //
extern char Reader(); //ctenie faylov nastroek
extern char Rabota(); // Regim raboty
extern char Graduirovka(); // Regim graduirovki
extern char Kalibrovka(); // Regim kalibrovki
extern char Nastrojka(); // Regim nastrojki
extern char NGrad(); // Regim nastrojki
extern char NSlug(); // Regim nastrojki
extern char Porog(); // Regim nastrojki
extern char Rezonans(); // Poisk rezonansa
extern char SchablonNSlug(); // Regim nastrojki
extern char SchablonPorog(); // Regim nastrojki
extern char ComCMode(char UstrAdr,char UstrFuncArgument); // Ustanjvit regim raboti
extern char ComReadIntReg(char UstrAdr,char StartAdr, char Length); // Chtenie registrov
extern char ReadDataSPMCurrent(); // Chtenie tekuchih dannih SPM
extern char ReadDataSPMSet(); // Chtenie nastroek SPM
extern char ComWriteIntReg(char UstrAdr,char StartAdr, char Length); // Zapis registrov
extern char WriteDataSPMSet(); // Zapis nastroek SPM
extern char ReadKey(); // Chtenie klaviatury immediately
extern char ReadKeyBuffered(); // Chtenie klaviatury
//extern char ReadKey(); // Chtenie klaviatury
extern char EEPROMSave(char UstrAdr); // Zapis v EEPROM
extern char ClrScr(); // Ochistka ekrana
extern char SetCursor(byte X,byte Y);
extern char PrintXY(byte X, byte Y, byte length); // Pechat stroki simvolov
extern char SchablonRabota(); // Schablon ekrana raboty
extern char SchablonGrad(); // Schablon ekrana graduirovki
extern char SchablonKalibr(); // Schablon ekrana kalibrovki
extern char SchablonNastrojka(); // Schablon ekrana
extern char SchablonNGrad(); // Schablon ekrana
extern char CRCcount(char rw,byte Ln); // Vichislenie CRC
extern char IniCom1(int BaudRate); //Inicializacia Com1
extern char IniCom2(int BaudRate); //Inicializacia Com2
extern char IniCom2(int BaudRate); //Inicializacia Com2
extern char * GetPathDir(char * applicationPath);
extern char * CombinePath(const char * firstPart, const char * secondPart);
extern void T2A(char * dest, TCHAR * source, int length);
extern void TimedIncrementT_Gen_T_Sr0_T_Sr1_W(bool setAverage, int t_Gen = 0, int t_Sr0 = 0, int t_Sr1 = 0, float w = 0.0f);
extern DWORD ReadKeyThreadProc (LPVOID lpdwThreadParam );
extern unsigned long ClockToMilliseconds(clock_t value);
extern char WaitNoKey();
extern char WaitKey();
extern void WaitNoKeyImmediately();
extern void WaitKeyImmediately();
extern bool ShutdownServer();
extern bool CheckFilling();
extern void SleepWithService(long sleeptime);
extern void CheckShutdown();
//extern char IniMWT(); //Inicializacia pribora

byte ComDataWrite[260];
byte ComDataRead [260];
char Text[20];
int RegistrSPM_int [26];
float RegistrSPM_float[4];
int RegistrTM_int [11];
int Gamma0, Gamma1;    //Period mejdu pikami                     *0.1 mkS
int Gamma0_k, Gamma1_k;//Period mejdu pikami pri kalibrovke      *0.1 mkS
int Haw0, Haw1;        //Shirina impulsa na polovine amplitudy   *0.1 mkS
int Am0, Am1;          //Amplituda impulsa                       *1000 V
int Haw0_k, Haw1_k;    //Shirina impulsa na polovine amplitudy   *0.1 mkS
                       //pri kalibrovke
int Am0_k, Am1_k;      //Amplituda impulsa pri kalibrovke        *1000 V
int T_Gen, T_Gen_k;    //Temperatura generatora                  *100  C
int T_Sr0, T_Sr1;      //Temperatura sredy                       *100  C
int T_Sr0_k;           //Temperatura sredy pri kalibrovke        *100  C
int Ain0, Ain1;        //Tekuchee znachenie na analogovom vhode  *1000 V
int Saw_A0;            //Amplituda podporki                      *1000 V
int Saw_A1;            //Amplituda pily                          *1000 V
int Ku0, Ku1;          //Koefficient usilenija                   2^x
int Taq;               //Period sbora dannih                     x*20mkS
int Fwdt;              //Shirina okna filtra
int Pcount;            //Kol-vo periodov usrednenija
int BaudRateSPM;       //Skorost obmena SPM
int HostSPM;           //Adres SPM
int ValidReqSPM;       //Schetchik uspeshnih zaprosov SPM
int CRCReqSPM;         //Schetchik error CRC SPM
int ExcReqSPM;         //Schetchik error obrabotki SPM
int VO, VZ, VI, AK, Tak, Aout_tip, NomGrad;
float W, AA, BB, a0, a1, a2, a3,a4,a5,a6,a7,a8,a9,k1,Wmin,Wmax;
float Va, kgt, kgv, T,tgn,tsn; //delta_gamma; tgn;
float delta_gamma, delta_gamma2;
int Kol_dat, Nom_dat1, Nom_dat2, Nom_ind;
float N_Rez, K_Rez, Step_Rez;
int UstRele, NomUBI;
byte Rel, Rele3, Rele3_old;
int Nzp;
float Poralfa, Porgamma;
long int xtim, xtim_n;
byte Regim;
HANDLE hCom1,hCom2;
DWORD ret;

//LPDWORD lpEvtMask;
DCB dcb, dcb2;
COMMTIMEOUTS CommTimeOuts;
BOOL fSuccess;

char * applicationPath;
volatile char keyBuffer = 0;
volatile long servicetime = 100;

CRITICAL_SECTION * g_ReadKey;
LONG * readkeyStarted;
HANDLE g_ReadKeyEvent;

#define MEAN_TIME 12000 // Mean time is 12 sec

#define PATH_SEPARATOR "\\"

#define READKEY_INTERVAL 100

#define NET_ZAPOLNENIYA_PING_PERIOD 60*1000

#ifndef SHTDN_REASON_MAJOR_OTHER
#define SHTDN_REASON_MAJOR_OTHER 0
#endif

void main()

{
	g_ReadKey = new CRITICAL_SECTION();
	InitializeCriticalSection(g_ReadKey);	
	g_ReadKeyEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	int osize = 256;
	TCHAR * obuf  = new TCHAR[256];
	osize = GetModuleFileName(NULL, obuf, osize);
	char * applicationName = new char [256];
	T2A(applicationName, obuf, osize);
	applicationPath = GetPathDir(applicationName);
	delete [] applicationName;
	delete [] obuf;
	
	char * binarypath = new char[256];
	char * messagepath = new char[256];	

	char readBuffer = 'Z';

	readkeyStarted = new LONG(0);
	InterlockedExchangeAdd(readkeyStarted, 1);

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
		 printf("ReadKeyThreadProc, Error create thread ThreadProc");
	 }

	char tempchar;
 int tempcount=0; 
//Inicializacia pribora
//NomUBI=10;
lab1:
 tempchar=IniCom1(19200);
 if (tempchar!=0) {tempcount++;if (tempcount<5) goto lab1;};

 tempchar=IniCom2(9600);
 if (tempchar!=0) {tempcount++;if (tempcount<5) goto lab1;};
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

 Write2Master();

 //Am0_k=20000; Haw0_k=30000; Gamma0_k=40000;
 //T_Gen_k=2100; T_Sr0_k=2300;
 //UstRele=123;
 //VO=5;VZ=12345;VI=5; AK=0; Tak=100;
 //Aout_tip=2;  NomGrad=1;
 //Va=1234.567;kgt=1234.567;kgv=1234.567;T=1234.567;
 //N_Rez=3.0; K_Rez=6.5; Step_Rez=0.1;

 //WriteDataSPMSet();
 //printf("Test1"); printf(" \n");
 
 
 Reader();
 Va=Saw_A1;
 //printf("Saw_A0 =");printf("%u",Saw_A0);printf(" \n");
 //printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
 //SleepWithService(10000);

 //printf("Test1"); printf(" \n");
 //Ku0=2;Ku1=2;
 //Saw_A0=3000;Saw_A1=2000;
 WriteDataSPMSet();SleepWithService(500);  //////////////////////////
 //printf("Test1"); printf(" \n");
 ReadDataSPMSet();SleepWithService(500);

 //AA=60; Aout_tip=1;
 printf("Ku0 =");printf("%u",Ku0);printf(" \n");
 printf("Ku1=");printf("%u",Ku1);printf(" \n");
 printf("Saw_A0 =");printf("%u",Saw_A0);printf(" \n");
 printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
 //SleepWithService(10000);


 Regim='1';
 do {
 Reg:
 printf("Regim=");printf("%c",Regim);printf(" \n");
 if (Regim=='1') {ClrScr();SleepWithService(1000);SchablonRabota();Regim=Rabota();goto Reg;};
 if (Regim=='3') {ClrScr();SleepWithService(1000);SchablonGrad();Regim=Graduirovka();goto Reg;};
 if (Regim=='5') {ClrScr();SleepWithService(1000);SchablonKalibr();Regim=Kalibrovka();goto Reg;};
 if (Regim=='7') {ClrScr();SleepWithService(1000);SchablonNastrojka();Regim=Nastrojka();goto Reg;};

 //   tempchar=ReadKey();
    printf("%c",tempchar);printf("  ");
	SleepWithService(50);
    Regim=	ReadKeyBuffered();
    } while (true);

 /*
	InterlockedExchangeAdd(readkeyStarted, -1);
	WaitForSingleObject(&dwThreadId, INFINITE);
	CloseHandle(&g_CalculationFinishedEvent);
	CloseHandle(&g_ReadKeyEvent);
	CloseHandle(&dwThreadId);

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
 //printf("kgv=");printf("%f",kgv);printf(" \n");
 //printf("kgt=");printf("%f",kgt);printf(" \n");
 //SleepWithService(5000);
 if (kgv==0) kgv=1;if (Va==0) Va=1;
 delta_gamma =(kgt/kgv)*(T/Va)*(T_Gen-T_Gen_k);
 delta_gamma2=k1*(T_Sr0-tsn);
 difftime0=(Gamma0-Gamma0_k);
 //difftime0=Gamma0/1000;
 difftime0=difftime0+delta_gamma+delta_gamma2;
 //printf("Am0=");printf("%u",Am0);printf(" \n");
 W=AA+BB*(a0+a1*(Am0-Am0_k)*10+a2*(Haw0-Haw0_k)/1000+a3*difftime0/1000+a4*(T_Gen-T_Gen_k)/100+a5*(T_Sr0-T_Sr0_k)/100);
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
				if (Key=='3'||Key=='5'||Key=='7') {goto Exk;};
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

   if(!CheckFilling())
   {
	   W = 0.0f;
   }
   Write2Master();

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

   y=T_Gen_k; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
   tempchar=PrintXY(16,5,4);
   y=T_Sr0_k; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
   tempchar=PrintXY(16,6,4);
    //
   y=T_Gen; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
   tempchar=PrintXY(16,9,4);
   y=T_Sr0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
   tempchar=PrintXY(16,10,4);

   //printf("%c", Key);printf(" \n");SleepWithService(5000);
  do {
   
   if (Key=='9'||Key=='3'||Key=='5'||Key=='7'||Key=='1')
   {	  
    if (Key=='5') {Key=ReadKeyBuffered();goto Kalibr;};
    if (Key=='9') {FILE *fp; fp=fopen("Kalibr.dat", "w+b");
                   fwrite(&Am0, sizeof(int), 1, fp);
                   fwrite(&Haw0, sizeof(int), 1, fp);
                   fwrite(&Gamma0, sizeof(int), 1, fp);
                   fwrite(&T_Gen, sizeof(int), 1, fp);
                   fwrite(&T_Sr0, sizeof(int), 1, fp);
                   fclose(fp);
                   SleepWithService(100); Reader();Va=Saw_A1;Key=WaitNoKey();
                   strcpy(Text,"Save Ok"); tempchar=PrintXY(11,1,7);SleepWithService(1000);
                   strcpy(Text,"       "); tempchar=PrintXY(11,1,7);SetCursor(PosX,PosY);
                   }; //Save
    if (Key=='1'||Key=='3'||Key=='7') {goto Exk;};
   };
   Key=WaitKey();
   //printf("%c", Key);printf(" \n");
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
				 if (Key=='3'||Key=='5'||Key=='7') {goto Exr;};
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
                    //printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
             strcpy(Text,"Загрузка  "); tempchar=PrintXY(0,3,10);
                    //printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
             xtim_n=clock();
             do {xtim=clock();
                 tempchar=ReadKeyBuffered();
                 if (tempchar=='3'||tempchar=='5'||
                 tempchar=='7') {Key=tempchar;};
				 if (Key=='3'||Key=='5'||Key=='7') {goto Exr;};
                } while ((xtim/1000-xtim_n/1000) <VZ);
             Rel=Rel ^ 0x02;
             RegistrTM_int[1]=Rel;
             ComWriteIntReg(0x2,0x31,0x1);
                     //printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
             strcpy(Text,"          "); tempchar=PrintXY(0,3,10);
                     //printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
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
	   if (Key=='3'||Key=='5'||Key=='7') {goto Exr;};
   	   strcpy(Text,"Измерение "); tempchar=PrintXY(0,3,10);
       SleepWithService(servicetime);

	   tempchar=ReadDataSPMCurrent();
	   printf("Test1"); printf(" \n");
	   Calc();
	   TimedIncrementT_Gen_T_Sr0_T_Sr1_W(false, T_Gen, T_Sr0, T_Sr1, W);
	   printf("Test2"); printf(" \n");   

	   WriteUBI();   
	   printf("Test3"); printf(" \n");printf("Test4"); printf(" \n");
	   printf("W=");printf("%f",W);printf(" \n");
	   printf("Wmin=");printf("%f",Wmin);printf(" \n");
	   printf("Wmax=");printf("%f",Wmax);printf(" \n");
	   printf("T3=");printf("%u",T_Sr1);printf(" \n");
	   printf("Alfa0=");printf("%u",Am0);printf(" \n");
	   printf("Beta0=");printf("%u",Haw0);printf(" \n");
	   printf("Gamma0=");printf("%u",Gamma0);printf(" \n");
	   //SleepWithService(5000);

	   y=W*1000; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
	   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	   if(CheckFilling())
	   {
	   	tempchar=PrintXY(14,5,5);
	   }
	   y=T_Sr0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
	   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
	   if(CheckFilling())
	   {
	   	tempchar=PrintXY(14,8,5);
	   }
	   strcpy(Text,"          "); tempchar=PrintXY(0,3,10);

	   if (W<Wmin) {strcpy(Text,"<Wmin     ");  tempchar=PrintXY(11,5,10);};
	   if (W>Wmax) {strcpy(Text,">Wmax     ");  tempchar=PrintXY(11,5,10);};
	  // Am0=10; Gamma0=20;
	   
	   if(!CheckFilling())
	   {
		   strcpy(Text,"Нет заполнения");
	       tempchar=PrintXY(3,4,14);
	   }   
	   

   } while ((xtim/1000-xtim_n/1000) <VI);
   
   TimedIncrementT_Gen_T_Sr0_T_Sr1_W(true);
   if(!CheckFilling())
   {
	   W = 0.0f;
   }
   Write2Master();

   Aout();

   if (W>(((float)UstRele)/10)){Rele3=1;} else {Rele3=0;};
   printf("UstRele=%f \n",((float)UstRele)/10);
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
				 if (Key=='3'||Key=='5'||Key=='7') {goto Exg;};
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
                    //printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
             strcpy(Text,"Загрузка  "); tempchar=PrintXY(0,3,10);
                    //printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
             xtim_n=clock();
             do {xtim=clock();
                 tempchar=ReadKeyBuffered();
                 if (tempchar=='1'||tempchar=='5'||
                 tempchar=='7'||tempchar=='9') {Key=tempchar;};
				 if (Key=='3'||Key=='5'||Key=='7') {goto Exg;};
                } while ((xtim/1000-xtim_n/1000) <VZ);
             Rel=Rel ^ 0x02;
             RegistrTM_int[1]=Rel;
             ComWriteIntReg(0x2,0x31,0x1);
                     //printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
             strcpy(Text,"          "); tempchar=PrintXY(0,3,10);
                     //printf("Err=");printf("%x",tempchar);printf(" \n");SleepWithService(2000);
             SetCursor(0xA,0x10);        
            };


   //SleepWithService(1000);
   xtim_n=clock();
   do {xtim=clock();
       tempchar=ReadKeyBuffered();
       if (tempchar=='1'||tempchar=='5'||
       tempchar=='7'||tempchar=='9') {Key=tempchar;};
	   if (Key=='3'||Key=='5'||Key=='7') {goto Exg;};
   strcpy(Text,"Измерение "); tempchar=PrintXY(0,3,10);
   SleepWithService(50);
   tempchar=ReadDataSPMCurrent();
   Calc();
   WriteUBI();
   TimedIncrementT_Gen_T_Sr0_T_Sr1_W(false, T_Gen, T_Sr0, T_Sr1, W);
   
   printf("W=");printf("%f",W);printf(" \n");
   printf("Tgen=");printf("%u",T_Gen);printf(" \n");
   printf("Tsr=");printf("%u",T_Sr0);printf(" \n");
   printf("T3=");printf("%u",T_Sr1);printf(" \n");
   printf("Alfa0=");printf("%u",Am0);printf(" \n");
   printf("Beta0=");printf("%u",Haw0);printf(" \n");
   printf("Gamma0=");printf("%u",Gamma0);printf(" \n");

   y=W*1000; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i]=z+48;};
   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
   tempchar=PrintXY(14,4,5);
   y=T_Gen; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
   tempchar=PrintXY(14,6,5);
   y=T_Sr0; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
   tempchar=PrintXY(14,7,5);
   y=T_Sr1; for (i=5;i!=0;i--) {z=fmod(y,10); y=floor(y/10); Text[i-1]=z+48;};
   Text[0]=Text[1];Text[1]=Text[2];Text[2]='.';
   tempchar=PrintXY(14,8,5);
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
   Write2Master();

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
 y=NomGrad; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
 PosY++;
 y=VZ; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
 y=VO; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
 y=VI; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
 y=Tak; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;
 y=UstRele; for (i=3;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};PosY++;

 Nastr:

 for (j=0;j<7;j++)
 { 
	 if (j==1) continue;
	 if(j==6)
	 {
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
	 for (i=0;i<3;i++)
	 {
		 Text[i]=temp[j][i];
		 tempchar=PrintXY(16,j,3);
     };    
   };

   if (AK==0) {Text[2]='ч';Text[1]='у';Text[0]='р';} else {Text[2]='т';Text[1]='в';Text[0]='а';};
   tempchar=PrintXY(14,7,3);
   if (Aout_tip==1) {Text[3]=' ';Text[2]='5';Text[1]='-';Text[0]='0';};
   if (Aout_tip==2) {Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='0';};
   if (Aout_tip==3) {Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='4';};
   tempchar=PrintXY(14,8,4);

   PosX=0; PosY=0; SetCursor(PosX, PosY);

   do {
   tempchar=ReadKeyBuffered();
   if (tempchar=='9'||tempchar=='5'||tempchar=='3'||tempchar=='1'
       ||tempchar=='2'||tempchar=='8'||tempchar=='S') {Key=tempchar;};

    if (Key=='2') {PosY++; if (PosY>11) PosY=0; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
    if (Key=='8') {PosY--; if (PosY>20) PosY=11; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
    if (Key=='S') {// printf("%x",Key);printf(" \n"); Sleep(2000);
                   if (PosY==7) {AK++; if (AK>1) AK=0;
                      if (AK==0) {Text[2]='ч';Text[1]='у';Text[0]='р';} else {Text[2]='т';Text[1]='в';Text[0]='а';};
                        tempchar=PrintXY(14,7,3);do {Key=ReadKeyBuffered();} while (Key!='Z');};
                   if (PosY==8) {Aout_tip++; if (Aout_tip>3) Aout_tip=1;
                      if (Aout_tip==1) {Text[3]=' ';Text[2]='5';Text[1]='-';Text[0]='0';};
                      if (Aout_tip==2) {Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='0';};
                      if (Aout_tip==3) {Text[3]='0';Text[2]='2';Text[1]='-';Text[0]='4';};
                      tempchar=PrintXY(14,8,4);do {Key=ReadKeyBuffered();} while (Key!='Z');};
                   if (PosY==9) {ClrScr();Sleep(500);SchablonNSlug();NSlug();
                                 ClrScr();Sleep(500);SchablonNastrojka();
                                 do {Key=ReadKeyBuffered();} while (Key!='Z');goto Nastr;};
                   if (PosY==11) {ClrScr();Sleep(500);SchablonPorog();Porog();
                                 ClrScr();Sleep(500);SchablonNastrojka();
                                 do {Key=ReadKeyBuffered();} while (Key!='Z');goto Nastr;};
                   if (PosY==0||PosY==2||PosY==3||PosY==4||PosY==5)
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
                    };
    if (Key=='9') {PosY=0;
      NomGrad=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;PosY++;
      VZ=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
      VO=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
      VI=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
      Tak=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
      UstRele=(temp[PosY][0]-48)*100+(temp[PosY][1]-48)*10+temp[PosY][2]-48;PosY++;
      //printf("%u",NomGrad);printf(" \n");Sleep(10000);
       FILE *fp; fp=fopen("Nastr.dat", "w+b");
                   fwrite(&NomGrad, sizeof(int), 1, fp);
                   fwrite(&VZ, sizeof(int), 1, fp);
                   fwrite(&VO, sizeof(int), 1, fp);
                   fwrite(&VI, sizeof(int), 1, fp);
                   fwrite(&Tak, sizeof(int), 1, fp);
                   fwrite(&UstRele, sizeof(int), 1, fp);
                   fwrite(&AK, sizeof(int), 1, fp);
                   fwrite(&Aout_tip, sizeof(int), 1, fp);
                   fwrite(&N_Rez, sizeof(float), 1, fp);
                   fwrite(&K_Rez, sizeof(float), 1, fp);
                   fwrite(&Step_Rez, sizeof(float), 1, fp);
                   fclose(fp);
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
  y=Porgamma*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
  for (j=3;j<5;j++)
  { for (i=0;i<8;i++)
  {Text[i]=temp[j][i];
  tempchar=PrintXY(12,j,8);
  };
  };
if (Nzp==0) {strcpy(Text,"Не использ."); tempchar=PrintXY(9,6,11);Sleep(100);};
if (Nzp==1) {strcpy(Text,"alfa > por "); tempchar=PrintXY(9,6,11);Sleep(100);};
if (Nzp==2) {strcpy(Text,"alfa < por "); tempchar=PrintXY(9,6,11);Sleep(100);};
if (Nzp==3) {strcpy(Text,"gamma > por"); tempchar=PrintXY(9,6,11);Sleep(100);};
if (Nzp==4) {strcpy(Text,"gamma < por"); tempchar=PrintXY(9,6,11);Sleep(100);};
  PosX=0; PosY=3; SetCursor(PosX, PosY);
   do {
   tempchar=ReadKeyBuffered();
   if (tempchar=='5'||tempchar=='9'||tempchar=='C'||tempchar=='2'||tempchar=='8'||tempchar=='S') {Key=tempchar;};

    if (Key=='2') {PosY++; if (PosY>6) PosY=3; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
    if (Key=='8') {PosY--; if (PosY<3) PosY=6; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
    if (Key=='S') { //printf("%x",Key);printf(" \n");
                   if (PosY==3||PosY==4)
                   {PosX=12; Key='Z';
                   for (i=PosX;i<PosX+8;i++)
                     {//Sleep(1000);
                     Key='Z';
                     SetCursor(i, PosY);//Sleep(1000);
                     if (i==16) goto npl1;
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
                     do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
                      Text[0]=Key;
                      PrintXY(i,PosY,1);
                      temp[PosY][i-12]=Text[0];
                      Sleep(200);
                      do {Key=ReadKeyBuffered();} while (Key!='Z');

                      npl1:;
                      };
                     };
                   if (PosY==6)//66666666666


                   {
                   Nzp++; if (Nzp>4) {Nzp=0;};
if (Nzp==0) {strcpy(Text,"Не использ."); tempchar=PrintXY(9,6,11);Sleep(100);};
if (Nzp==1) {strcpy(Text,"alfa > por "); tempchar=PrintXY(9,6,11);Sleep(100);};
if (Nzp==2) {strcpy(Text,"alfa < por "); tempchar=PrintXY(9,6,11);Sleep(100);};
if (Nzp==3) {strcpy(Text,"gamma > por"); tempchar=PrintXY(9,6,11);Sleep(100);};
if (Nzp==4) {strcpy(Text,"gamma < por"); tempchar=PrintXY(9,6,11);Sleep(100);};
                    // do {Key=ReadKey();} while (Key=='Z'||Key=='S');
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
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

      Porgamma=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;

      //printf("%u",Ku0);printf(" \n");printf("%u",Ku0);printf(" \n");  Sleep(10000);
                    //printf("Save ");printf("%f",a0);printf(" \n");
                   FILE *fp; fp=fopen("Set.dat", "w+b");
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
                   fwrite(&NomUBI, sizeof(int), 1, fp);
                   fwrite(&Ku0, sizeof(int), 1, fp);
                   fwrite(&Ku1, sizeof(int), 1, fp);
                   fwrite(&Nzp, sizeof(int), 1, fp);
                   fwrite(&Poralfa, sizeof(float), 1, fp);
                   fwrite(&Porgamma, sizeof(float), 1, fp);
                   fclose(fp);
                   Sleep(100); Reader();
                   strcpy(Text,"SaveOk"); tempchar=PrintXY(13,2,6);Sleep(1000);
                   strcpy(Text,"      "); tempchar=PrintXY(13,2,6);SetCursor(PosX,PosY);  //////////////////////////
                      do {Key=ReadKeyBuffered();} while (Key!='Z'); }; //Save
    if (Key=='7') { PosY=0;
      Saw_A0=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
         (temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;

      printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
      Saw_A1=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
         (temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;
         printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
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
 y=NomUBI; for (i=2;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  PosY++;
 y=Ku0; for (i=2;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
 NastrSlug:
 //printf("NGrad");printf(" \n");Sleep(30000);
 for (j=0;j<7;j++)
   { for (i=0;i<8;i++)
       {Text[i]=temp[j][i];
        tempchar=PrintXY(12,j,8);
       };
   };
   j=11;
   for (i=0;i<2;i++)
       {Text[i]=temp[j][i];
        tempchar=PrintXY(18,j,2);
       };
   j=12;
   for (i=0;i<2;i++)
       {Text[i]=temp[j][i];
        tempchar=PrintXY(18,j,2);
       };
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
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
                     do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
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
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
                     do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
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
      NomUBI=(temp[11][0]-48)*10+temp[11][1]-48;
      Ku0=(temp[12][0]-48)*10+temp[12][1]-48;
      Ku1=(temp[12][0]-48)*10+temp[12][1]-48;

      //printf("%u",Ku0);printf(" \n");printf("%u",Ku0);printf(" \n");  Sleep(10000);
                    //printf("Save ");printf("%f",a0);printf(" \n");
                   FILE *fp; fp=fopen("Set.dat", "w+b");
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
                   fwrite(&NomUBI, sizeof(int), 1, fp);
                   fwrite(&Ku0, sizeof(int), 1, fp);
                   fwrite(&Ku1, sizeof(int), 1, fp);
                   fwrite(&Nzp, sizeof(int), 1, fp);
                   fwrite(&Poralfa, sizeof(float), 1, fp);
                   fwrite(&Porgamma, sizeof(float), 1, fp);
                   fclose(fp);
                   Sleep(100); Reader();Va=Saw_A1;WriteDataSPMSet();Sleep(500);
                   strcpy(Text,"SaveOk"); tempchar=PrintXY(13,2,6);Sleep(1000);
                   strcpy(Text,"      "); tempchar=PrintXY(13,2,6);SetCursor(PosX,PosY);  //////////////////////////
                      do {Key=ReadKeyBuffered();} while (Key!='Z'); }; //Save
    if (Key=='7') { PosY=0;
      Saw_A0=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
         (temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;

      printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
      Saw_A1=(temp[PosY][0]-48)*1000000+(temp[PosY][1]-48)*100000+(temp[PosY][2]-48)*10000+(temp[PosY][3]-48)*1000+
         (temp[PosY][5]-48)*100+(temp[PosY][6]-48)*10+(temp[PosY][7]-48)*1;PosY++;
         printf("Saw_A1=");printf("%u",Saw_A1);printf(" \n");
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
     { for (i=0;i<8;i++)
           {Text[i]=temp[j][i];
            tempchar=PrintXY(12,j,8);};
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
                               //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
                               do {Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
                               //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
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
   //     printf("%f",N_Rez);printf("  "); printf("%f",K_Rez);printf("  ");printf("%f",Step_Rez);printf("\n"); Sleep(5000);
      N_Rez=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
      K_Rez=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
      Step_Rez=(temp[PosY][0]-48)*1000+(temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001;PosY++;
    //  printf("%f",N_Rez);printf("  "); printf("%f",K_Rez);printf("  ");printf("%f",Step_Rez);printf("\n"); Sleep(5000);
       FILE *fp; fp=fopen("Nastr.dat", "w+b");
                   fwrite(&NomGrad, sizeof(int), 1, fp);
                   fwrite(&VZ, sizeof(int), 1, fp);
                   fwrite(&VO, sizeof(int), 1, fp);
                   fwrite(&VI, sizeof(int), 1, fp);
                   fwrite(&Tak, sizeof(int), 1, fp);
                   fwrite(&UstRele, sizeof(int), 1, fp);
                   fwrite(&AK, sizeof(int), 1, fp);
                  fwrite(&Aout_tip, sizeof(int), 1, fp);
                   fwrite(&N_Rez, sizeof(float), 1, fp);
                   fwrite(&K_Rez, sizeof(float), 1, fp);
                   fwrite(&Step_Rez, sizeof(float), 1, fp);
                   fclose(fp);
                   Sleep(100); Reader();//do {Key=ReadKey();} while (Key!='Z');
      do {Key=ReadKeyBuffered();} while (Key!='Z');
      strcpy(Text,"Save Ok"); tempchar=PrintXY(11,1,7);Sleep(1000);
                   strcpy(Text,"       "); tempchar=PrintXY(11,1,7);SetCursor(PosX,PosY);
      PosX=0;PosY=0;SetCursor(PosX, PosY);
      }; //Save
        if (Key=='5') { PosY=0;
                        Saw_A0=N_Rez*1000;
                        ComCMode(0x01,0x01);Sleep(1000);
                         do {PosY=3;
                             y=Saw_A0; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
                             temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
                             temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';
                             for (i=0;i<8;i++)
                                  {Text[i]=temp[PosY][i];
                                   tempchar=PrintXY(12,PosY,8);
                                  };
                             Sleep(1000);
                             WriteDataSPMSet();
                             Sleep(1000);
                             ComCMode(0x01,0x00);
                             Sleep(2000);
                             ReadDataSPMCurrent();
                             printf("%u",Saw_A0);printf(" ");printf("%u",Am0);printf(" ");printf("%u",Haw0);printf(" ");printf("%u",Gamma0);printf("\n");
                             Sleep(1000);
                             PosY=5;
                             y=Am0*1000; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
                             temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
                             temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
                             y=Haw0; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
                             temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
                             temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
                             y=Gamma0; for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
                             temp[PosY][0]=temp[PosY][1];temp[PosY][1]=temp[PosY][2];
                             temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
                             for (j=5;j<8;j++)
                                 { for (i=0;i<8;i++)
                                      {Text[i]=temp[j][i];
                                       tempchar=PrintXY(12,j,8);
                                      };
                                 };  //
                             Saw_A0=Saw_A0+Step_Rez*1000;
                         } while (Saw_A0<K_Rez*1000); //Cicl Rezonans
                        do {Key=ReadKeyBuffered();} while (Key!='Z');//}
                      };// Key 5

 //goto NastrSlug;
    } while (true);
 Exrez:
 return Key;
 }

////////////////////////////
char NGrad() // Regim nastrojki


 {
 int i,j;
 byte PosY,PosX, Key;
 byte temp[15][8];
 char tempchar;
 float y,z;
 PosX=0; PosY=0; SetCursor(PosX, PosY);
 PosY=0;
 if (a0<0) {y=a0*(-1000);} else {y=a0*1000;};
   for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a0<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (a1<0) {y=a1*(-1000);} else {y=a1*1000;};
   for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a1<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (a2<0) {y=a2*(-1000);} else {y=a2*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a2<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (a3<0) {y=a3*(-1000);} else {y=a3*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a3<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (a4<0) {y=a4*(-1000);} else {y=a4*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a4<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (a5<0) {y=a5*(-1000);} else {y=a5*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a5<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (a6<0) {y=a6*(-1000);} else {y=a6*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a6<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (a7<0) {y=a7*(-1000);} else {y=a7*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a7<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (a8<0) {y=a8*(-1000);} else {y=a8*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a8<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
if (a9<0) {y=a9*(-1000);} else {y=a9*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (a9<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (AA<0) {y=AA*(-1000);} else {y=AA*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (AA<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (BB<0) {y=BB*(-1000);} else {y=BB*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (BB<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (k1<0) {y=k1*(-1000);} else {y=k1*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (k1<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (Wmax<0) {y=Wmax*(-1000);} else {y=Wmax*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (Wmax<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;
 if (Wmin<0) {y=Wmin*(-1000);} else {y=Wmin*1000;};
  for (i=8;i!=0;i--) {z=fmod(y,10); y=floor(y/10); temp[PosY][i-1]=z+48;};
  if (Wmin<0) {temp[PosY][0]='-';} else {temp[PosY][0]='+';};temp[PosY][1]=temp[PosY][2];
  temp[PosY][2]=temp[PosY][3];temp[PosY][3]=temp[PosY][4];temp[PosY][4]='.';PosY++;

 NastrGrad:
 //printf("NGrad");printf(" \n");Sleep(30000);
 for (j=0;j<15;j++)
   { for (i=0;i<8;i++)
       {Text[i]=temp[j][i];
        tempchar=PrintXY(6,j,8);
       };
   };
   PosX=0; PosY=0;

   do {
   tempchar=ReadKeyBuffered();
   if (tempchar=='9'||tempchar=='C'||tempchar=='2'||tempchar=='8'||tempchar=='S') {Key=tempchar;};

    if (Key=='2') {PosY++; if (PosY>14) PosY=0; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
    if (Key=='8') {PosY--; if (PosY>20) PosY=14; SetCursor(PosX, PosY);do {Key=ReadKeyBuffered();} while (Key!='Z');};
    if (Key=='S') { //printf("%x",Key);printf(" \n");
                   PosX=6; Key='Z';
                   for (i=PosX;i<PosX+8;i++)
                     {//Sleep(1000);
                     if (i==10) goto ng1;
                     Key='Z';
                     SetCursor(i, PosY);//Sleep(1000);
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
                     do {
                      ng3:
                      Key=ReadKeyBuffered();} while (Key=='Z'||Key=='S');
                     //printf("%x",Key);printf("  ");printf("%u",i);printf(" \n");
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
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a1=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a2=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a3=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a4=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a5=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a6=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a7=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a8=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      a9=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      AA=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      BB=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      k1=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      Wmax=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
      Wmin=(44-temp[PosY][0])*((temp[PosY][1]-48)*100+(temp[PosY][2]-48)*10+(temp[PosY][3]-48)+
         (temp[PosY][5]-48)*0.1+(temp[PosY][6]-48)*0.01+(temp[PosY][7]-48)*0.001);PosY++;
                    //printf("Save ");printf("%f",a0);printf(" \n");
                   FILE *fp; fp=fopen("Grad.dat", "w+b");
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
 //printf("Schablon");printf(" \n");SleepWithService(3000);

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
 strcpy(Text,"Ном. индикатора     ");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
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
 strcpy(Text,"Порог gamma         ");  tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Условие             ");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");

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

 strcpy(Text,"Ном. градуир.       "); tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
 //
 strcpy(Text,"Время заполн.      с");  tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Время очист.       с");  tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Время измер.       с");  tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Период калибр.     с");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Уставка реле     . %");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Тип калибр.         ");  tempchar=PrintXY(0,7,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Тип ток. вых.     мА");  tempchar=PrintXY(0,8,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Служебн. настр.     ");  tempchar=PrintXY(0,9,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Градуир. коэфф.     ");  tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Выбор порога        ");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
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

 strcpy(Text,"       КРУПА        "); tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
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

 strcpy(Text,"       КРУПА        "); tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
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

 strcpy(Text,"       КРУПА        "); tempchar=PrintXY(0,0,20);printf("%x",tempchar);printf(" \n");
 //
 strcpy(Text,"Режим: Калибровка   "); tempchar=PrintXY(0,2,20);printf("%x",tempchar);printf(" \n");
 //strcpy(Text,"Операция:  Ошибка:  "); tempchar=PrintXY(0,3,20);printf("%x",tempchar);printf(" \n");
 //strcpy(Text,"                    "); tempchar=PrintXY(0,4,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Альфа0:      Тг:    ");  tempchar=PrintXY(0,5,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Бета0 :      Тс:    ");  tempchar=PrintXY(0,6,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Гамма0:             ");  tempchar=PrintXY(0,7,20);printf("%x",tempchar);printf(" \n");
  //
 strcpy(Text,"Альфа :      Тг:    ");  tempchar=PrintXY(0,9,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Бета  :      Тс:    ");  tempchar=PrintXY(0,10,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"Гамма :             ");  tempchar=PrintXY(0,11,20);printf("%x",tempchar);printf(" \n");
 //
 strcpy(Text,"F3-Калибровка       ");  tempchar=PrintXY(0,13,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"F1-Работа F2-Град.  ");  tempchar=PrintXY(0,14,20);printf("%x",tempchar);printf(" \n");
 strcpy(Text,"F4-Настр. F5-Сохран.");  tempchar=PrintXY(0,15,20);printf("%x",tempchar);printf(" \n");
     return 0;
}
/////////////////////////////

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
	 EnterCriticalSection(g_ReadKey);
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
      printf("\n");

      WriteFile(hCom1,&ComDataWrite,len,&ret,NULL);

                                                 //Read:
                                                 //OK:    Error:
                                         //adres //byte 0 byte 0
                                   //kod funkcii //byte 1 byte 1(modificirovan)
                                   //kod error   //       byte 2
                                   //CRC         //byte 2 byte 3
                                   // CRC        //byte 3 byte 4
      len=4; //SleepWithService(100);
      ReadFile(hCom1,&ComDataRead,len,&ret,NULL);
      for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
      printf("   ");
      //SleepWithService(1000);
      //Rasshifrovka otveta
	  if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_ReadKey); return 10;}
     if (ComDataWrite[1]!=ComDataRead[1])
         {if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_ReadKey);return 11;} else
              {LeaveCriticalSection(g_ReadKey);return ComDataRead[2];};}
                   else {CRCcount('r',len);

                   };            // vichislenie CRC

                                                       //proverka CRC
     //if (ComDataRead[len]!=ComDataRead[len+2] ||
     //  ComDataRead[len+1]!=ComDataRead[len+1+2]) return 12;
	 LeaveCriticalSection(g_ReadKey);
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
	 EnterCriticalSection(g_ReadKey);
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
      printf("\n");

      WriteFile(hCom1,&ComDataWrite,len,&ret,NULL);

                                                 //Read:
                                                 //OK:    Error:
                                         //adres //byte 0 byte 0
                                   //kod funkcii //byte 1 byte 1(modificirovan)
                                   //kod error   //       byte 2
                                   //CRC         //byte 2 byte 3
                                   // CRC        //byte 3 byte 4
      //len=4;	  
      ReadFile(hCom1,&ComDataRead,len,&ret,NULL);
      for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
      printf("\n");

      //Rasshifrovka otveta
     if (ComDataWrite[0]!=ComDataRead[0])
	 {
		 LeaveCriticalSection(g_ReadKey);
		 return 10;
	 }
     if (ComDataWrite[1]!=ComDataRead[1])
         {if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_ReadKey);return 11;} else
              {LeaveCriticalSection(g_ReadKey);return ComDataRead[2];};}
                   else {CRCcount('r',len);

                   };            // vichislenie CRC

                                                       //proverka CRC
     if (ComDataRead[len]!=ComDataRead[len+2] ||
		 ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_ReadKey); return 12;}
	 LeaveCriticalSection(g_ReadKey);
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
	 EnterCriticalSection(g_ReadKey);
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
      printf("\n");

      WriteFile(hCom1,&ComDataWrite,len,&ret,NULL);

                                                 //Read:
                                                 //OK:    Error:
                                         //adres //byte 0 byte 0
                                   //kod funkcii //byte 1 byte 1(modificirovan)
                                   //kod error   //       byte 2
                                   //CRC         //byte 2 byte 3
                                   // CRC        //byte 3 byte 4
      len=4;
      ReadFile(hCom1,&ComDataRead,len,&ret,NULL);
      for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
      printf("\n");

      //Rasshifrovka otveta
	  if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_ReadKey); return 10;}
     if (ComDataWrite[1]!=ComDataRead[1])
         {if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_ReadKey); return 11;} else
              {LeaveCriticalSection(g_ReadKey);return ComDataRead[2];};}
                   else {CRCcount('r',len);

                   };            // vichislenie CRC

                                                       //proverka CRC
     if (ComDataRead[len]!=ComDataRead[len+2] ||
		 ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_ReadKey); return 12;}
	 LeaveCriticalSection(g_ReadKey);
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
	 EnterCriticalSection(g_ReadKey);
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

      WriteFile(hCom1,&ComDataWrite,len,&ret,NULL);

                                                 //Read:
                                                 //OK:    Error:
                                         //adres //byte 0 byte 0
                                   //kod funkcii //byte 1 byte 1(modificirovan)
                                   //kod error   //       byte 2
                                   //CRC         //byte 2 byte 3
                                   // CRC        //byte 3 byte 4
      len=4;
      ReadFile(hCom1,&ComDataRead,len,&ret,NULL);

      //Rasshifrovka otveta
	  if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_ReadKey); return 10;}
     if (ComDataWrite[1]!=ComDataRead[1])
         {if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_ReadKey); return 11;} else
              {LeaveCriticalSection(g_ReadKey); return ComDataRead[2];};}
                   else {CRCcount('r',len);
                     
                   };            // vichislenie CRC

                                                       //proverka CRC
     if (ComDataRead[len]!=ComDataRead[len+2] ||
		 ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_ReadKey); return 12;}
	 LeaveCriticalSection(g_ReadKey);
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
byte Length=2;//<<< KOD PEREDACHI Dannih v DATCHIK (dolgen bit=2)
byte temp;
byte i;

i=StartAdr;
RegistrSPM_int[i]=Saw_A0;
printf("%u", RegistrSPM_int[i]); printf("\n");
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
	 EnterCriticalSection(g_ReadKey);
 if (UstrAdr<1 || UstrAdr>2) {LeaveCriticalSection(g_ReadKey);return 10;};
 if (UstrAdr==1){if (StartAdr>0x16 || StartAdr+Length>0x17) {LeaveCriticalSection(g_ReadKey);return 13;};};
 if (UstrAdr==2){if (StartAdr<0x30) {LeaveCriticalSection(g_ReadKey);return 13;};
                 if (StartAdr>0x35 || StartAdr+Length>0x36) {LeaveCriticalSection(g_ReadKey);return 13;};};
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
      printf("\n");

      WriteFile(hCom1,&ComDataWrite,len,&ret,NULL);

                                                 //Read:
                                                 //OK:    Error:
                                         //adres //byte 0 byte 0
                                   //kod funkcii //byte 1 byte 1(modificirovan)
                                   //kod error   //       byte 2
                                   //CRC         //byte 2 byte 3
                                   // CRC        //byte 3 byte 4
      len=4;
      ReadFile(hCom1,&ComDataRead,len,&ret,NULL);
      for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
      printf("\n");

      //Rasshifrovka otveta
	  if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_ReadKey); return 10;}
     if (ComDataWrite[1]!=ComDataRead[1])
         {if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_ReadKey); return 11;} else
              {LeaveCriticalSection(g_ReadKey); return ComDataRead[2];};}
                   else {CRCcount('r',len);

                   };            // vichislenie CRC

                                                       //proverka CRC
     if (ComDataRead[len]!=ComDataRead[len+2] ||
		 ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_ReadKey); return 12;}
	 LeaveCriticalSection(g_ReadKey);
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
	 EnterCriticalSection(g_ReadKey);
 if (UstrAdr<1 || UstrAdr>2) {LeaveCriticalSection(g_ReadKey);return 10;};
 if (UstrAdr==1){if (StartAdr>0x16 || StartAdr+Length>0x17) {LeaveCriticalSection(g_ReadKey);return 13;};};
 if (UstrAdr==2){if (StartAdr<0x30) {LeaveCriticalSection(g_ReadKey);return 13;};
                 if (StartAdr>0x35 || StartAdr+Length>0x36) {LeaveCriticalSection(g_ReadKey);return 13;};};
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

      WriteFile(hCom1,&ComDataWrite,lenr,&ret,NULL);
      //printf("%x",length);
      printf("\n");  printf("\n");
      for (i=0;i<lenr;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
      printf("\n");
                                                 //Read:
                                                 //OK:    Error:
                                         //adres //byte 0 byte 0
                                   //kod funkcii //byte 1 byte 1(modificirovan)
                                   //kod error   //       byte 2
                                   //CRC         //byte 2 byte 3
                                   // CRC        //byte 3 byte 4
      lenr=(Length*2)+4;
      ReadFile(hCom1,&ComDataRead,lenr,&ret,NULL);
       for (i=0;i<lenr;i++) {printf("%x",ComDataRead[i]);printf(" ");};
      printf("\n");printf("\n");

      //Rasshifrovka otveta
	  if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_ReadKey); return 10;}
     if (ComDataWrite[1]!=ComDataRead[1])
         {if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_ReadKey);return 11;} else
              {LeaveCriticalSection(g_ReadKey); return ComDataRead[2];};}
                   else {CRCcount('r',(Length*2+2));
                     //for (i=0;i<(Length*2)+6;i++) {printf("%x",ComDataRead[i]);printf(" ");};
                     //printf("\n");
                   };            // vichislenie CRC

                                                       //proverka CRC
     if (ComDataRead[(Length*2)+2]!=ComDataRead[(Length*2)+2+2] ||
		 ComDataRead[(Length*2)+3]!=ComDataRead[(Length*2)+3+2]){LeaveCriticalSection(g_ReadKey); return 12;}
     if (UstrAdr==1){for (i=0; i<Length; i++)
                             {RegistrSPM_int[StartAdr+i]=ComDataRead[i*2+2]*256+ComDataRead[i*2+3];
                             // printf("%u",StartAdr+i);printf(" = ");
                             // printf("%u",RegistrSPM_int[StartAdr+i]);printf("\n");
                             };
                     };
     if (UstrAdr==2){for (i=0; i<Length; i++)
                             {RegistrTM_int[StartAdr+i]=ComDataRead[i*2+2]*256+ComDataRead[i*2+3];
                             // printf("%u",StartAdr+i);printf(" = ");
                             // printf("%u",RegistrTM_int[StartAdr+i]);printf("\n");
                             };
                     };
	 LeaveCriticalSection(g_ReadKey);
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
	 EnterCriticalSection(g_ReadKey);
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
      WriteFile(hCom1,&ComDataWrite,length,&ret,NULL);
      //printf("%x",length); printf("\n");
      for (i=0;i<length;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
      printf("       ");
      ReadFile(hCom1,&ComDataRead,length-1,&ret,NULL);
     };
      if (UstrAdr==3)
     {
      WriteFile(hCom2,&ComDataWrite,length,&ret,NULL);
      ReadFile(hCom2,&ComDataRead,length-1,&ret,NULL);
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
	LeaveCriticalSection(g_ReadKey);
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

   GetCommState(hCom1,&dcb);
      dcb.BaudRate =BaudRate;
      dcb.ByteSize = 8;
      dcb.Parity = NOPARITY;
      dcb.StopBits = ONESTOPBIT;
      dcb.fDtrControl = DTR_CONTROL_DISABLE;
      dcb.fRtsControl = RTS_CONTROL_DISABLE;
   fSuccess = SetCommState( hCom1, &dcb);
   if(!fSuccess) {printf(" fail Com1\n");return 1;};     // fail

   GetCommTimeouts (hCom1,&CommTimeOuts);
      CommTimeOuts.ReadIntervalTimeout=100;
      CommTimeOuts.ReadTotalTimeoutMultiplier=100;
      CommTimeOuts.ReadTotalTimeoutConstant=100;
     // CommTimeOuts.WriteTotalTimeoutMultiplier=5;
     // CommTimeOuts.WriteTotalTimeoutConstant=5;
   fSuccess = SetCommTimeouts (hCom1,&CommTimeOuts);
   printf("%i",CommTimeOuts.WriteTotalTimeoutMultiplier);
   printf("  ");
   printf("%i",CommTimeOuts.WriteTotalTimeoutConstant);
   printf("\n");
   if(!fSuccess) {printf(" fail Com1\n");return 2;};     // fail
   printf("Com1 complit\n");
   printf("BaudRate="); printf("%i",BaudRate);
   printf("\n");
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

   GetCommState(hCom2,&dcb);
      dcb.BaudRate =BaudRate;
      dcb.ByteSize = 8;
      dcb.Parity = NOPARITY;
      dcb.StopBits = ONESTOPBIT;
      dcb.fDtrControl = DTR_CONTROL_DISABLE;
      dcb.fRtsControl = RTS_CONTROL_ENABLE;
   fSuccess = SetCommState( hCom2, &dcb);
   if(!fSuccess) {printf(" fail Com2\n");return 1;};     // fail

   GetCommTimeouts (hCom2,&CommTimeOuts);
      CommTimeOuts.ReadIntervalTimeout=10;
      CommTimeOuts.ReadTotalTimeoutMultiplier=10;
      CommTimeOuts.ReadTotalTimeoutConstant=10;
   fSuccess = SetCommTimeouts (hCom2,&CommTimeOuts);
   if(!fSuccess) {printf(" fail Com2\n");return 2;};     // fail
   printf("Com2 complit\n");
   return 0;
  };

  //////////////////////////////////////

///////////////////////////////////////
 
  char Reader() //ctenie faylov nastroek
  {  	  
	  char * gradPath = CombinePath(applicationPath, "Grad.dat");
	  char * nastrPath = CombinePath(applicationPath, "Nastr.dat");
	  char * kalibrPath = CombinePath(applicationPath, "Kalibr.dat");
	  char * setPath = CombinePath(applicationPath, "Set.dat");
  		    //printf("Read");printf(" \n");
            FILE *fp;
            fp=fopen(gradPath, "r+b");	
            fread(&a0, sizeof(float), 1, fp);
            fread(&a1, sizeof(float), 1, fp);
            fread(&a2, sizeof(float), 1, fp);
            fread(&a3, sizeof(float), 1, fp);
            fread(&a4, sizeof(float), 1, fp);
            fread(&a5, sizeof(float), 1, fp);
            fread(&a6, sizeof(float), 1, fp);
            fread(&a7, sizeof(float), 1, fp);
            fread(&a8, sizeof(float), 1, fp);
            fread(&a9, sizeof(float), 1, fp);
            fread(&AA, sizeof(float), 1, fp);
            fread(&BB, sizeof(float), 1, fp);
            fread(&k1, sizeof(float), 1, fp);
            fread(&Wmax, sizeof(float), 1, fp);
            fread(&Wmin, sizeof(float), 1, fp);
            fclose(fp);

            FILE *fpn; fpn=fopen(nastrPath, "r+b");
                   fread(&NomGrad, sizeof(int), 1, fpn);
                   fread(&VZ, sizeof(int), 1, fpn);
                   fread(&VO, sizeof(int), 1, fpn);
                   fread(&VI, sizeof(int), 1, fpn);
                   fread(&Tak, sizeof(int), 1, fpn);
                   fread(&UstRele, sizeof(int), 1, fpn);
                   fread(&AK, sizeof(int), 1, fpn);
                   fread(&Aout_tip, sizeof(int), 1, fpn);
                   fread(&N_Rez, sizeof(float), 1, fp);
                   fread(&K_Rez, sizeof(float), 1, fp);
                   fread(&Step_Rez, sizeof(float), 1, fp);
                   fclose(fpn);

            FILE *fpk; fpk=fopen(kalibrPath, "r+b");
                   fread(&Am0_k, sizeof(int), 1, fpk);
                   fread(&Haw0_k, sizeof(int), 1, fpk);
                   fread(&Gamma0_k, sizeof(int), 1, fpk);
                   fread(&T_Gen_k, sizeof(int), 1, fpk);
                   fread(&T_Sr0_k, sizeof(int), 1, fpk);
                   fclose(fpk);

             FILE *fps; fps=fopen(setPath, "r+b");
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
                   fread(&NomUBI, sizeof(int), 1, fps);
                   fread(&Ku0, sizeof(int), 1, fps);
                   fread(&Ku1, sizeof(int), 1, fps);
                   fread(&Nzp, sizeof(int), 1, fp);
                   fread(&Poralfa, sizeof(float), 1, fp);
                   fread(&Porgamma, sizeof(float), 1, fp);
                   fclose(fps);
            //printf("Read OK");printf(" \n");

			delete [] gradPath;
			delete [] nastrPath;
			delete [] kalibrPath;
			delete [] setPath;

            return 0;
  }

  //////////////////////////////////////
  //modified by SL 18.06.09
  //modified by SL 23.12.2009 add data output throught COM2
  char Write2Master() //write to master host
  {  	 
	 char temp[50];
     byte CRC, len;
     int i;
     float y,z;

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
   printf("%x",temp[23]);
   //SleepWithService(10000);
   WriteFile(hCom2,&temp,len,&ret,NULL);
   //SleepWithService(3000);   
  return 0;
  }
  //write to remote indicator
   char WriteUBI() //display temperature and humidity on remote indicator
  {
	  EnterCriticalSection(g_ReadKey);
 int i;
 char len;
 char pos=0;
 float prt1,prt2;
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
      {prt1=W/100;
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
      len=pos+2;
      printf("Remote indicator message \n");
      printf("W= %f\n",W);
      printf("Tsr0= %i\n",T_Sr0);
      for (i=0;i<len;i++) {printf("%x",ComDataWrite[i]);printf(" ");};
      printf("\n");

      WriteFile(hCom1,&ComDataWrite,len,&ret,NULL);

                                                 //Read:
                                                 //OK:    Error:
                                         //adres //byte 0 byte 0
                                   //kod funkcii //byte 1 byte 1(modificirovan)
                                   //kod error   //       byte 2
                                   //CRC         //byte 2 byte 3
                                   // CRC        //byte 3 byte 4
      len=8;
      ReadFile(hCom1,&ComDataRead,len,&ret,NULL);
      for (i=0;i<len;i++) {printf("%x",ComDataRead[i]);printf(" ");};
      printf("\n");
//      SleepWithService(3000);

      //Rasshifrovka otveta
	  if (ComDataWrite[0]!=ComDataRead[0]){LeaveCriticalSection(g_ReadKey); return 10;}
     if (ComDataWrite[1]!=ComDataRead[1])
         {if (ComDataRead[1]!=ComDataWrite[1]+0x80) {LeaveCriticalSection(g_ReadKey); return 11;} else
              {LeaveCriticalSection(g_ReadKey);return ComDataRead[2];};}
                   else {CRCcount('r',len);

                   };            // vichislenie CRC

                                                       //proverka CRC
     if (ComDataRead[len]!=ComDataRead[len+2] ||
		 ComDataRead[len+1]!=ComDataRead[len+1+2]){LeaveCriticalSection(g_ReadKey); return 12;}
	LeaveCriticalSection(g_ReadKey);
  return 0;
  }



  //////////////////////////////////////

  char Aout() //
  {  char temp[50];
     byte CRC, len;
     int ACout0,ACout1;
	 int DacData;
     int i;
     float y,z,Kout;
 if (Aout_tip==1) {ACout0=0x3D; ACout1=0x3E; Kout=1025/(Wmax-Wmin != 0 ? Wmax-Wmin : 1);DacData=(int)(floor((W-Wmin)*Kout));
                    if (DacData>1025){DacData=1025;};
                    if (DacData<0){DacData=0;};
                    };//0-5
	 if (Aout_tip==2) {ACout0=65; ACout1=55; Kout=3025/(Wmax-Wmin != 0 ? Wmax-Wmin : 1);DacData=(int)(floor((W-Wmin)*Kout));
                    if (DacData>3025){DacData=3025;};
                    if (DacData<0){DacData=0;};
                   }; //0-20
	 if (Aout_tip==3) {ACout0=65; ACout1=55; Kout=2800/(Wmax-Wmin != 0 ? Wmax-Wmin : 1);DacData=(int)(floor(((W-Wmin)*Kout)/1+500));
                    if (DacData>3010){DacData=3010;};
                    if (DacData<500){DacData=500;};
                   }; //4-20
  Sleep(200);
  RegistrTM_int[4]=ACout0;
  //ComWriteIntReg(0x2,0x34,0x1);
  Sleep(200);
  RegistrTM_int[2]=DacData;
  ComWriteIntReg(0x2,0x32,0x1);
  Sleep(200);
  RegistrTM_int[4]=ACout1;
  //ComWriteIntReg(0x2,0x35,0x1);
  Sleep(200);
  RegistrTM_int[2]=DacData;
  ComWriteIntReg(0x2,0x33,0x1);
  Sleep(200);
  printf("%u",ACout0); printf("    ");printf("%u",DacData);printf("\n");
  //SleepWithService(2000);
  return 0;
  }

  //////////////////////////////////////


char * CombinePath(const char * firstPart, const char * secondPart)
{
    char * dest = new char[strlen(firstPart) + strlen(PATH_SEPARATOR) + strlen(secondPart) + 1];
    strcpy(dest, firstPart);
    strcat(dest, PATH_SEPARATOR);
    strcat(dest, secondPart);
    return dest;
}

char * GetPathDir(char * applicationPath)
{
    int applicationNameSize = strlen(strrchr(applicationPath,'\\'));
    int pathLength = strlen(applicationPath) - applicationNameSize + 1;
    char * pathDir = new char[pathLength];
    strncpy(pathDir, applicationPath, pathLength - 1);
    pathDir[pathLength - 1] = 0;

    char * relativeFounded = strchr(pathDir, '.');
    if(relativeFounded != NULL)
    {
        char * pwd = new char[256];
		pwd = getcwd(pwd, 256);
        int fullPathLength = strlen(pwd) + strlen(relativeFounded);
        char * fullPath = new char [fullPathLength];
        strcpy(fullPath, pwd);
        strcat(fullPath, ++relativeFounded);
		delete [] pwd;
		delete [] pathDir;
        return fullPath;
    }
    return pathDir;
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

extern DWORD ReadKeyThreadProc (LPVOID lpdwThreadParam )
{
#ifdef _VC
	InterlockedCompareExchange(readkeyStarted, 1, 1);
#else
	long value = 1;
	InterlockedCompareExchange((void **)(&readkeyStarted), &value, &value);
#endif
	
	while(*readkeyStarted)
	{		
		DWORD result = WaitForSingleObject(g_ReadKeyEvent, INFINITE);	
		
		char tempchar = ReadKeyBuffered();
		if(tempchar != 'Z')
		{
			keyBuffer = tempchar;
		}
		CheckShutdown();
		
		ResetEvent(g_ReadKeyEvent);
#ifdef _VC
		InterlockedCompareExchange(readkeyStarted, 1, 1);
#else
		long value = 1;
		InterlockedCompareExchange((void **)(&readkeyStarted), &value, &value);
#endif
	}

	return 0;
}

unsigned long ClockToMilliseconds(clock_t value)
{
	return value * 1000 / CLOCKS_PER_SEC;
}

char WaitNoKey()
{	
	char key = 0;
	do {
		Sleep(50);
		key=ReadKeyBuffered(); 		
	} 
	while (key!='Z');
	return key;
}

char WaitKey()
{		
	char key = 0;
	do 
	{
		Sleep(50);
		key=ReadKeyBuffered();		
	}
	while (key=='Z'||key=='S');

	return key;
}

bool CheckFilling()
{
   if (Nzp==1) if (Am0>Poralfa) return false;                
   if (Nzp==2) if (Am0<Poralfa) return false;                
   if (Nzp==3) if (Gamma0>Porgamma) return false;               
   if (Nzp==4) if (Gamma0<Porgamma) return false;
   return true;
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
		printf("Cannot poweroff computer.");
		return false;
	}

	ExitWindowsEx(EWX_POWEROFF, SHTDN_REASON_MAJOR_OTHER);			
	error = GetLastError();
	if (error != ERROR_SUCCESS)
	{
		printf("Cannot poweroff computer.");
		return false;
	}
	return true;
}

void SleepWithService(long sleeptime)
{	
	while(sleeptime >= servicetime && servicetime > 0)
	{
		SetEvent(g_ReadKeyEvent);
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

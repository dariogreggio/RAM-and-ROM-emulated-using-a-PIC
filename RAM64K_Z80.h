//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------


/* check if build is for a real debug tool */
#if defined(__DEBUG) && !defined(__MPLAB_ICD2_) && !defined(__MPLAB_ICD3_) && \
   !defined(__MPLAB_PICKIT2__) && !defined(__MPLAB_PICKIT3__) && \
   !defined(__MPLAB_REALICE__) && \
   !defined(__MPLAB_DEBUGGER_REAL_ICE) && \
   !defined(__MPLAB_DEBUGGER_ICD3) && \
   !defined(__MPLAB_DEBUGGER_PK3) && \
   !defined(__MPLAB_DEBUGGER_PICKIT2) && \
   !defined(__MPLAB_DEBUGGER_PIC32MXSK)
    #warning Debug with broken MPLAB simulator
    #define USING_SIMULATOR
#endif


//#define REAL_SIZE    


#define FCY 205000000ul    //Oscillator frequency; ricontrollato con baud rate, pare giusto così!

#define CPU_CLOCK_HZ             (FCY)    // CPU Clock Speed in Hz
#define CPU_CT_HZ            (CPU_CLOCK_HZ/2)    // CPU CoreTimer   in Hz
#define PERIPHERAL_CLOCK_HZ      (FCY/2 /*100000000UL*/)    // Peripheral Bus  in Hz
#define GetSystemClock()         (FCY)    // CPU Clock Speed in Hz
#define GetPeripheralClock()     (PERIPHERAL_CLOCK_HZ)    // Peripheral Bus  in Hz

#define US_TO_CT_TICKS  (CPU_CT_HZ/1000000UL)    // uS to CoreTimer Ticks
    
#define VERNUML 9
#define VERNUMH 0


typedef char BOOL;
typedef unsigned char UINT8;
typedef unsigned char BYTE;
typedef signed char INT8;
typedef unsigned short int WORD;
typedef unsigned short int SWORD;
typedef unsigned long UINT32;
typedef unsigned long DWORD;
typedef signed long INT32;
typedef unsigned short int UINT16;
typedef signed int INT16;

typedef DWORD COLORREF;

#define RGB(r,g,b)      ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))


#define TRUE 1
#define FALSE 0


#define _TFTWIDTH  		160     //the REAL W resolution of the TFT
#define _TFTHEIGHT 		128     //the REAL H resolution of the TFT

typedef signed char GRAPH_COORD_T;
typedef unsigned char UGRAPH_COORD_T;

#ifdef __PIC32
void mySYSTEMConfigPerformance(void);
void myINTEnableSystemMultiVectoredInt(void);

#define ReadCoreTimer()                  _CP0_GET_COUNT()           // Read the MIPS Core Timer

void ShortDelay(DWORD DelayCount);
#define __delay_ms(n) ShortDelay(n*100000UL)

#define ClrWdt() { WDTCONbits.WDTCLRKEY=0x5743; }
#else

#endif




void Timer_Init(void);
void PWM_Init(void);
void UART_Init(DWORD);
void putsUART1(unsigned int *buffer);

int decodeKBD(int, long, BOOL);

BYTE GetValue(DWORD t);
WORD GetValue16(DWORD t);
DWORD GetValue32(DWORD t);
DWORD GetPipe(DWORD t);
void PutValue(DWORD t,BYTE t1);
void PutValue16(DWORD t,WORD t1);
void PutValue32(DWORD t,DWORD t1);
DWORD GetPipe(DWORD t);
BYTE checkCond(BYTE);


int Emulate(int);

int UpdateScreen(BYTE);


#ifdef __PIC32
#define LED1 LATEbits.LATE2
#define LED2 LATEbits.LATE3
#define LED3 LATEbits.LATE4
#define SW1  PORTDbits.RD2
#define SW2  PORTDbits.RD3
#else
#define LED1 LATBbits.LATB3
#define LED2 LATBbits.LATB12
#define SW1  PORTAbits.RA0
#endif



// pcb SDRradio 2019
#define	SPISDITris 0		// niente qua
#define	SPISDOTris TRISGbits.TRISG8				// SDO
#define	SPISCKTris TRISGbits.TRISG6				// SCK
#define	SPICSTris  TRISGbits.TRISG7				// CS
#define	LCDDCTris  TRISEbits.TRISE7				// DC che su questo LCD è "A0" per motivi ignoti
//#define	LCDRSTTris TRISBbits.TRISB7
	
#define	m_SPISCKBit LATGbits.LATG6		// pin 
#define	m_SPISDOBit LATGbits.LATG8		// pin 
#define	m_SPISDIBit 0
#define	m_SPICSBit  LATGbits.LATG7		// pin 
#define	m_LCDDCBit  LATEbits.LATE7 		// pin 
//#define	m_LCDRSTBit LATBbits.LATB7 //FARE
//#define	m_LCDBLBit  LATBbits.LATB12


#define MAKEWORD(a, b)   ((WORD) (((BYTE) (a)) | ((WORD) ((BYTE) (b))) << 8)) 
#define MAKELONG(a, b)   ((unsigned long) (((WORD) (a)) | ((DWORD) ((WORD) (b))) << 16)) 
#define HIBYTE(w)   ((BYTE) ((((WORD) (w)) >> 8) /* & 0xFF*/)) 
//#define HIBYTE(w)   ((BYTE) (*((char *)&w+1)))		// molto meglio :)
#define HIWORD(l)   ((WORD) (((DWORD) (l) >> 16) & 0xFFFF)) 
#define LOBYTE(w)   ((BYTE) (w)) 
#define LOWORD(l)   ((WORD) (l)) 


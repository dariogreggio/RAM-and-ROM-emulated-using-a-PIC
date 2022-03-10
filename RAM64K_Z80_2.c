// Local Header Files
#include <stdlib.h>
#include <string.h>
#include <xc.h>
#include "RAM64K_Z80.h"

#include <sys/attribs.h>
#include <sys/kmem.h>


// Un'emulatore di RAM statica per Z80, come da
//  https://www.facebook.com/groups/1985662191717774/2625341117749875/?comment_id=2626265664324087&reply_comment_id=2627742284176425&notif_id=1588160670765846
// PMD0..7 è su RE0..7
// uso RB0..15 come bus indirizzi
// RD9 e RD11 sono PMPCS1..2
// RD4=PMPWR(EN); RD5=PMPRD(WR)


// PIC32MZ1024EFE064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
/* Default SYSCLK = 200 MHz (8MHz FRC / FPLLIDIV * FPLLMUL / FPLLODIV) */
//#pragma config FPLLIDIV = DIV_1, FPLLMULT = MUL_50, FPLLODIV = DIV_2
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (8x Divider)
#pragma config FPLLRNG = RANGE_34_68_MHZ// System PLL Input Range (34-68 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_50       // System PLL Multiplier (PLL Multiply by 128)
#pragma config FPLLODIV = DIV_2        // System PLL Output Clock Divider (32x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = FRCDIV           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS16384          // Watchdog Timer Postscaler (1:16384)
  // circa 6-7 secondi, 24.7.19
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = ON             // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3

// DEVADC0

// DEVADC1

// DEVADC2

// DEVADC3

// DEVADC4

// DEVADC7





const char CopyrightString[]= {'6','4','K','/','5','1','2','K',' ','S','t','a','t','i','c',' ','R','A','M',' ','v',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0',' ','-',' ', '0','8','/','0','9','/','2','1', 0 };

const char Copyr1[]="(C) Dario's Automation 2020 - G.Dar\xd\xa\x0";



// Global Variables:
BOOL fExit,debug;
BYTE DoIRQ,DoNMI,DoHalt,DoReset,ColdReset;
BYTE ram_seg[65536];

volatile BYTE CIA1IRQ,CIA2IRQ,VICIRQ;



int main(void) {

  // disable JTAG port
//  DDPCONbits.JTAGEN = 0;
  

  CFGCONbits.IOLOCK = 0;      // PPS Unlock
//  RPB15Rbits.RPB15R = 4;        // Assign RPB15 as U6TX, pin 30
//  U6RXRbits.U6RXR = 2;      // Assign RPB14 as U6RX, pin 29 

  CFGCONbits.IOLOCK = 1;      // PPS Lock

//  PPSOutput(4,RPC4,OC1);   //buzzer 4KHz , qua rimappabile 
  

  
#ifdef DEBUG_TESTREFCLK
// test REFCLK
  PPSOutput(4,RPC4,REFCLKO2);   // RefClk su pin 1 (RG15, buzzer)
	REFOCONbits.ROSSLP=1;
	REFOCONbits.ROSEL=1;
	REFOCONbits.RODIV=0;
	REFOCONbits.ROON=1;
	TRISFbits.TRISF3=1;
#endif

//	PPSLock;

   // Disable all Interrupts
  __builtin_disable_interrupts();
  
  


//  SPLLCONbits.PLLMULT=10;
  
  OSCTUN=0;
  OSCCONbits.FRCDIV=0;
  
  // Switch to FRCDIV, SYSCLK=8MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x00; // FRC
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
    // At this point, SYSCLK is ~8MHz derived directly from FRC
 //http://www.microchip.com/forums/m840347.aspx
  // Switch back to FRCPLL, SYSCLK=200MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x01; // SPLL
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
  // At this point, SYSCLK is ~200MHz derived from FRC+PLL
//***
  mySYSTEMConfigPerformance();
  //myINTEnableSystemMultiVectoredInt(();
  


    
	TRISB=0b0000000000110000;			// AN4,5 (rb4..5)

	TRISC=0b0000000000000000;
	TRISD=0b0000000000001100;			// 2 pulsanti
	TRISE=0b0000000000000000;			// 3 led
	TRISF=0b0000000000000000;			// 
	TRISG=0b0000000000000000;			// SPI2 (rg6..8)



  ANSELE=0;
  ANSELG=0;



  CNPUDbits.CNPUD2=1;   // switch/pulsanti
  CNPUDbits.CNPUD3=1;
  CNPUGbits.CNPUG6=1;   // I2C tanto per
  CNPUGbits.CNPUG8=1;  

      
  /* Example configuration for Addressable Slave mode */
  IEC1CLR = 0x0004; // Disable PMP interrupt in case it is already enabled
  PMCON = 0x0000; // Stop and configure PMCON register for Address mode, all active low, address/data separati
  PMCONbits.CSF=0b10;     // both CS
  PMMODE = 0x0000; // Configure PMMODE register: legacy Slave, 8bit, no increment
  PMMODEbits.IRQM=0b01;
  // wait states tutto minimo = 1TPbClk
  PMAENbits.PTEN=0b11;      // CS attivi
  
  
  IPC7SET = 0x001C; // Set priority level = 7 and
  IPC7SET = 0x0003; // Set subpriority level = 3
  // Could have also done this in single operation by assigning IPC7SET = 0x001F
// scegliere...
  
  // Configure the PMP interrupts
  IPC7SET = 0x0014; // Set priority level = 5
  IPC7SET = 0x0003; // Set subpriority level = 3
  // Could have also done this in single operation by assigning IPC7SET = 0x0017
  IFS1CLR = 0x0004; // Clear the PMP interrupt status flag ?
  IEC1SET = 0x0004; // Enable PMP interrupts?
  PMCONSET = 0x8000; // Enable the PMP module
  

//  Timer_Init();

 // UART_Init(115200L);


  myINTEnableSystemMultiVectoredInt();
#ifndef __MPLAB_DEBUGGER_SIMULATOR
  ShortDelay(50000); 
#endif



  
	ColdReset=0;

  for(;;) {
    ClrWdt();
    }
  }



void mySYSTEMConfigPerformance(void) {
  unsigned PLLIDIV;
  unsigned PLLMUL;
  unsigned PLLODIV;
  float CLK2USEC;
  unsigned SYSCLK;
  static char PLLODIVVAL[]={
    2,2,4,8,16,32,32,32
    };

  PLLIDIV=SPLLCONbits.PLLIDIV+1;
  PLLMUL=SPLLCONbits.PLLMULT+1;
  PLLODIV=PLLODIVVAL[SPLLCONbits.PLLODIV];

  SYSCLK=(FCY*PLLMUL)/(PLLIDIV*PLLODIV);
  CLK2USEC=SYSCLK/1000000.0f;

  SYSKEY = 0x0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;

  if(SYSCLK<=60000000)
    PRECONbits.PFMWS=0;
  else if(SYSCLK<=120000000)
    PRECONbits.PFMWS=1;
  else if(SYSCLK<=200000000)
    PRECONbits.PFMWS=2;
  else if(SYSCLK<=252000000)
    PRECONbits.PFMWS=4;
  else
    PRECONbits.PFMWS=7;

  PRECONbits.PFMSECEN=0;    // non c'è nella versione "2019" ...
  PRECONbits.PREFEN=0x1;

  SYSKEY = 0x0;
  }

void myINTEnableSystemMultiVectoredInt(void) {

  PRISS = 0x76543210;
  INTCONSET = _INTCON_MVEC_MASK /*0x1000*/;    //MVEC
  asm volatile ("ei");
  //__builtin_enable_interrupts();
  }

/* CP0.Count counts at half the CPU rate */
#define TICK_HZ (CPU_HZ / 2)


void xdelay_us(uint32_t us) {
  
  if(us == 0) {
    return;
    }
  unsigned long start_count = ReadCoreTimer /*_CP0_GET_COUNT*/();
  unsigned long now_count;
  long cycles = ((GetSystemClock() + 1000000U) / 2000000U) * us;
  do {
    now_count = ReadCoreTimer /*_CP0_GET_COUNT*/();
    } while ((unsigned long)(now_count-start_count) < cycles);
  }

void __attribute__((used)) DelayUs(unsigned int usec) {
  unsigned int tWait, tStart;

  tWait=(GetSystemClock()/2000000)*usec;
  tStart=_mfc0(9,0);
  while((_mfc0(9,0)-tStart)<tWait)
    ClrWdt();        // wait for the time to pass
  }

void __attribute__((used)) DelayMs(unsigned int ms) {
  
  for(;ms;ms--)
    DelayUs(1000);
  }

// ===========================================================================
// ShortDelay - Delays (blocking) for a very short period (in CoreTimer Ticks)
// ---------------------------------------------------------------------------
// The DelayCount is specified in Core-Timer Ticks.
// This function uses the CoreTimer to determine the length of the delay.
// The CoreTimer runs at half the system clock. 100MHz
// If CPU_CLOCK_HZ is defined as 80000000UL, 80MHz/2 = 40MHz or 1LSB = 25nS).
// Use US_TO_CT_TICKS to convert from uS to CoreTimer Ticks.
// ---------------------------------------------------------------------------

void ShortDelay(                       // Short Delay
  DWORD DelayCount)                   // Delay Time (CoreTimer Ticks)
{
  DWORD StartTime;                    // Start Time
  StartTime = ReadCoreTimer();         // Get CoreTimer value for StartTime
  while( (DWORD )(ReadCoreTimer() - StartTime) < DelayCount ) 
    ClrWdt();
  }

 

void Timer_Init(void) {


  T2CON=0;
  T2CONbits.TCS = 0;                  // clock from peripheral clock
  T2CONbits.TCKPS = 7;                // 1:256 prescaler (pwm clock=390625Hz)
  T2CONbits.T32 = 0;                  // 16bit
//  PR2 = 2000;                         // rollover every n clocks; 2000 = 50KHz
  PR2 = 65535;                         // per ora faccio solo onda quadra, v. SID
  T2CONbits.TON = 1;                  // start timer per PWM
  
  // TIMER 3 INITIALIZATION (TIMER IS USED AS A TRIGGER SOURCE FOR ALL CHANNELS).
  T3CON=0;
  T3CONbits.TCS = 0;                  // clock from peripheral clock
  T3CONbits.TCKPS = 4;                // 1:16 prescaler
  PR3 = 3906;                         // rollover every n clocks; 
  T3CONbits.TON = 1;                  // start timer 

  IPC3bits.T3IP=4;            // set IPL 4, sub-priority 2??
  IPC3bits.T3IS=0;
  IEC0bits.T3IE=1;             // enable Timer 3 interrupt se si vuole
  
  
	}




void __attribute__((no_fpu)) __ISR(_TIMER_3_VECTOR,ipl4AUTO) TMR_ISR(void) {
// https://www.microchip.com/forums/m842396.aspx per IRQ priority ecc
  static BYTE divider;


//  LED2 ^= 1;      // check timing: 1600Hz, 9/11/19 (fuck berlin day))
  
  divider++;
  if(divider>=32) {   // 50 Hz per TOD
    divider=0;
//    CIA1IRQ=1;
    }


  IFS0CLR = _IFS0_T3IF_MASK;
  }


/* This code example demonstrates a simple Interrupt Service Routine for PMP
interrupts. The user?s code at this vector should perform any application specific
operations and must clear the PMP interrupt status flag before exiting. */
void __attribute__((no_fpu)) __ISR(_PMP_VECTOR, ipl5AUTO) PMP_HANDLER(void) {
  //... perform application specific operations in response to the interrupt
  WORD n;
  
//  PMADDRbits.CS1;
  // gestire altri pin di abilitazione, tipo WAIT, REFRESH o altri... nel caso

/*A27020 Series PRELIMINARY      (August, 2002, Version 0.0)2AMIC Technology, Inc.
Operating Modes and Truth Table
Mode									CE	OE	PGM				A0		A1		A9	VPP			VCC			I/O7-I/O0
Read									VIL VIL X					X			X			X		VCC			VCC			Data Out
Output Disable				VIL VIH X					X			X			X		VCC			VCC			Hi-Z
Standby								VIH X		X					X			X			X		VCC			VCC			Hi-Z
Program								VIL VIH VIL Pulse	X			X			X		12.75V	6.25V		Data In 
Program Verify				VIL VIL VIH				X			X			X		12.75V	6.25V		Data Out
Program Inhibit				VIH X		X					X			X			X		12.75V	6.25V		Hi-Z
Manufacturer Code(3)	VIL VIL VIH				VIL		VIL		VID VCC			VCC			37H
Device Code(3)				VIL VIL VIH				VIH		VIL		VID VCC			VCC			64H
Continuation Code(3)	VIL VIL VIH				VIL		VIH		VID VCC			VCC			7FH
	*/


  n=PORTB;
  if(PMSTATbits.IB0F)
    ram_seg[n]=PMDIN;
  else    // if PMSTATbits.OB0E
    PMDOUT=ram_seg[n];
  
  IFS1CLR = 0x0004; // Be sure to clear the PMP interrupt status
// flag before exiting the service routine.?
  }


// ---------------------------------------------------------------------------------------
// declared static in case exception condition would prevent
// auto variable being created
static enum {
	EXCEP_IRQ = 0,			// interrupt
	EXCEP_AdEL = 4,			// address error exception (load or ifetch)
	EXCEP_AdES,				// address error exception (store)
	EXCEP_IBE,				// bus error (ifetch)
	EXCEP_DBE,				// bus error (load/store)
	EXCEP_Sys,				// syscall
	EXCEP_Bp,				// breakpoint
	EXCEP_RI,				// reserved instruction
	EXCEP_CpU,				// coprocessor unusable
	EXCEP_Overflow,			// arithmetic overflow
	EXCEP_Trap,				// trap (possible divide by zero)
	EXCEP_IS1 = 16,			// implementation specfic 1
	EXCEP_CEU,				// CorExtend Unuseable
	EXCEP_C2E				// coprocessor 2
  } _excep_code;

static unsigned int _epc_code;
static unsigned int _excep_addr;

void __attribute__((weak)) _general_exception_handler(uint32_t __attribute__((unused)) code, uint32_t __attribute__((unused)) address) {
  }

void __attribute__((nomips16,used)) _general_exception_handler_entry(void) {
  
	asm volatile("mfc0 %0,$13" : "=r" (_epc_code));
	asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

	_excep_code = (_epc_code & 0x0000007C) >> 2;

  _general_exception_handler(_excep_code, _excep_addr);

	while (1)	{
		// Examine _excep_code to identify the type of exception
		// Examine _excep_addr to find the address that caused the exception
    }
  }



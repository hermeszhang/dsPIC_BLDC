 /**********************************************************************
 *                                                                     *
 *                        Software License Agreement                   *
 *                                                                     *
 *    The software supplied herewith by Microchip Technology           *
 *    Incorporated (the "Company") for its dsPIC controller            *
 *    is intended and supplied to you, the Company's customer,         *
 *    for use solely and exclusively on Microchip dsPIC                *
 *    products. The software is owned by the Company and/or its        *
 *    supplier, and is protected under applicable copyright laws. All  *
 *    rights are reserved. Any use in violation of the foregoing       *
 *    restrictions may subject the user to criminal sanctions under    *
 *    applicable laws, as well as to civil liability for the breach of *
 *    the terms and conditions of this license.                        *
 *                                                                     *
 *    THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
 *    WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
 *    BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
 *    FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
 *    COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
 *    INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 *    Filename:       PMSM.c                                           *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ32MC204.gld                                 *
 *                                                                     *
 ***********************************************************************
 *      Code Description                                               *
 *                                                                     *
 *  This file demonstrates Vector Control of a 3 phase PMSM using the  *
 *  dsPIC. SVM is used as the modulation strategy. Currents are        *
 *  measured to estimate position and speed of PMSM Motors             *
 *                                                                     *
fehlt: ? warum Nist ohne Motor konstant statt 0
 Timer + ADC eventuell höhere Priorität stellen?
 standard Drehrichtung?
Versionsnummer korrigieren
Regelparameter optimieren?
**********************************************************************/

/************** GLOBAL DEFINITIONS ***********/
#define VERSION 3 //Softwareversion zum auslesen Byte 4
	//TE 15.12.10 V2 mit Timeout, solldrehzahl statt istdrehzahl, istdrehzahl mit 0 korrigiert hinten angehängt
	//mode bit 3 = motor run auslesen
	//timeout 3sec ergänzt
	//V3 12.1.11: Timeout auf 2 sec reduziert, kleinere von 2 Drehzahen hintereinander wird verwendet
#define INITIALIZE
#include "general.h"
#include "Parms.h"
#include "SVGen.h"
#include "ReadADC.h"
#include "MeasCurr.h"
#include "Control.h"
#include "PI.h"
#include "Park.h"
#include "UserParms.h"
#include "smcpos.h"
#include <p33FJ32MC202.h>

#define MOTENABLE 0x0010	//#define MOTENABLE LATAbits.LATA4
#define I2C
#define IMAXR 6950	//Rshunt 2*1R 0,125W -> Umax=0,35V -> ADCWert max Nennwert 0,106, VZ entfernt -> 6950 Maximalwert
unsigned int Imaxcnt,ADCmax;
volatile unsigned int timeout;
#define TIMEOUT 20	//in 0,1sec V2 16.12.10, V3 auf 2 sec reduziert
volatile unsigned int drehzahl=93*0; //93*255;  //5000 entspricht 70Hz = 1,4U/sec = 84U/min
										//nmax = 400U/min gew?cht -> 6,66U/sec -> drehzahlmax=23810
volatile unsigned int drehzahlneu,drehzahlalt;	//V3
#define DREHZAHL_SCALE 93	//93*255 = 23715
#define DREHZAHL_MIN 800
//#define DREHZAHL_MAX 65535
#define I2C_ANZ	10                        //maximale Byteanzahl
#define SLAVEADDRESS 0x70
unsigned char i2cdata[I2C_ANZ],i2canz=0;

bool DREHRICHTUNG,OPENLOOPMODE;

/******************************************************************************/
/* Configuration bits                                                         */
_FOSCSEL(FNOSC_FRC);			
/* Start with FRC will switch to Primary (XT, HS, EC) Oscillator with PLL */
_FOSC(FCKSM_CSECMD & POSCMD_XT & IOL1WAY_OFF);	
/* Clock Switching Enabled and Fail Safe Clock Monitor is disable   Primary Oscillator Mode: XT Crystal */
//_FBS (BSS_NO_FLASH & BWRP_WRPROTECT_OFF);
/* no Boot sector and   write protection disabled */
_FBS (BSS_NO_FLASH & BWRP_WRPROTECT_ON);	//TE schreibgeschützt
//_FWDT (FWDTEN_OFF);
/* Turn off Watchdog Timer */
_FWDT (FWDTEN_ON & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS16)//TE wdt enable, windows off, 
	//prescaler 128, postscaler 16 -> 31us*128*16=64ms
//_FGS (GSS_OFF & GCP_OFF & GWRP_OFF);
/* Set Code Protection Off for the General Segment */
_FGS (GSS_STD & GCP_ON & GWRP_ON);	//TE code protection on
//_FPOR (PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR128);
_FPOR (PWMPIN_ON & HPOL_OFF & LPOL_OFF & FPWRT_PWR128); //TE Ausgang invertiert
/* PWM mode is Port registers PWM high & low active high alternate I2C mapped to SDA1/SCL1 FPOR power on reset 128ms */
_FICD (ICS_PGD3 & JTAGEN_OFF & COE_ON);
/* Use PGC3/PGD3 for programming and debugging */
/************** END OF GLOBAL DEFINITIONS ***********/

SMC smc1 = SMC_DEFAULTS;
unsigned long Startup_Ramp = 0;	/* Start up ramp in open loop. This variable
								is incremented in CalculateParkAngle()
								subroutine, and it is assigned to 
								ParkParm.qAngle as follows:
								ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

unsigned int Startup_Lock = 0;	/* This is a counter that is incremented in
								CalculateParkAngle() every time it is called. 
								Once this counter has a value of LOCK_TIME, 
								then theta will start increasing moving the 
								motor in open loop. */

union   {
        struct
            {
            unsigned OpenLoop:1;	// Indicates if motor is running in open or closed loop
            unsigned RunMotor:1;	// If motor is running, or stopped.
            unsigned ChangeMode:1;	// This flag indicates that a transition from open to closed
									// loop, or closed to open loop has happened. This
									// causes DoControl subroutine to initialize some variables
									// before executing open or closed loop for the first time
            unsigned ChangeSpeed:1;	// This flag indicates a step command in speed reference.
									// This is mainly used to analyze step response
            unsigned    :12;
            }bit;
        WORD Word;
        } uGF;

tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;	// Structure definition for Speed, or Omega

tReadADCParm ReadADCParm;	// Struct used to read ADC values.

// Speed Calculation Variables
SFRAC16 PrevTheta = 0;	// Previous theta which is then substracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.
SFRAC16 AccumTheta = 0;	// Accumulates delta theta over a number of times
WORD AccumThetaCnt = 0;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is 
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.
SFRAC16 qVdSquared = 0;	// This variable is used to know what is left from the VqVd vector
						// in order to have maximum output PWM without saturation. This is
						// done before executing Iq control loop at the end of DoControl()
SFRAC16 Theta_error = 0;// This value is used to transition from open loop to closed looop. 
						// At the end of open loop ramp, there is a difference between 
						// forced angle and estimated angle. This difference is stored in 
						// Theta_error, and added to estimated theta (smc1.Theta) so the 
						// effective angle used for commutating the motor is the same at 
						// the end of open loop, and at the begining of closed loop. 
						// This Theta_error is then substracted from estimated theta 
						// gradually in increments of 0.05 degrees until the error is less
						// than 0.05 degrees.

SFRAC16 SEVTCMP_First = 0;
SFRAC16 SEVTCMP_Second = 0;
SFRAC16 Sector = 0;
SFRAC16 Ia = 0;
SFRAC16 Ib = 0;
SFRAC16 Ic = 0;

long openspeed;
unsigned int interrupt_cnt;

void I2C_init(void){
static int data;
int dummy;
  I2CADD=SLAVEADDRESS; 
  //I2CCON=0x8200; //I2CEN, clockstretching, no slew rate control
  I2CCON=0x9000;	//I2CEN, kein clockstretching, slew rate control, general call disabled, disable receive clock stretching
  _SI2CIF=0;
  _SI2CIE=1;	//enable slave I2C Interrupt
}

void __attribute__((__interrupt__)) _SI2C1Interrupt(void){	
static unsigned char data,mode,buffer;
int dummy;
  _SI2C1IF=0;
  if (_RBF){
    if(_D_A){
      data=I2C1RCV;
      if (i2canz<I2C_ANZ){ 
		i2cdata[i2canz++]=data;
//		if(i2canz==1){if (i2cdata[0]>0) {PORTA|=MOTENABLE; timeout=0;} else PORTA&=~MOTENABLE; drehzahl=DREHZAHL_SCALE*i2cdata[0];}  //V2 timeout reset
		if(i2canz==1){if (i2cdata[0]>0) {PORTA|=MOTENABLE; timeout=0;} else PORTA&=~MOTENABLE;  //V3 timeout reset + drehzahl2
          drehzahlalt=drehzahlneu;
		  drehzahlneu=DREHZAHL_SCALE*i2cdata[0];
		  if (drehzahlalt>drehzahlneu) drehzahl=drehzahlneu; else drehzahl=drehzahlalt;
		}  
		if(i2canz==2){
		  mode=i2cdata[1];
		  if (mode&0x01) DREHRICHTUNG=1; else DREHRICHTUNG=0;
		  if (mode&0x02) OPENLOOPMODE=1; else OPENLOOPMODE=0;
		  if (mode&0x04) asm("RESET");
		}
	  }
    }else{
	  i2canz=0;
      dummy=I2C1RCV; //Adresse, mu?aber gelesen werden
    }
  }  
  if (_R_W) {
    i2canz++;
/*    switch(i2canz){	//V1
  	  case 1:dummy=smc1.OmegaFltred/DREHZAHL_SCALE; if (dummy>255) dummy=255; I2CTRN=dummy; break;	//nist, funktioniert nicht bei abgestecktem Motor
	  case 2:I2CTRN=mode; break;
	  case 3:dummy=ADCmax/32; if (dummy>255) dummy=255; I2CTRN=dummy; ADCmax=0; break;	//entspricht Moment
	  case 4: I2CTRN=VERSION; break;
      default: I2CTRN=i2canz;			//dummywerte
	}
*/
    switch(i2canz){		//V2 16.12.10
  	  case 1: dummy=drehzahl/DREHZAHL_SCALE; if (dummy>255) dummy=255; I2CTRN=dummy; break;	//nsoll Ausgabe statt nist
	  case 2: if (PORTA&MOTENABLE) mode|=(1<<3); else mode&=~(1<<3); I2CTRN=mode; break;		//MOTENABLE in Statusabfrag ergänzt
	  case 3: dummy=ADCmax/32; if (dummy>255) dummy=255; I2CTRN=dummy; ADCmax=0; break;	//entspricht Moment
	  case 4: I2CTRN=VERSION; break;
	  case 5: dummy=smc1.OmegaFltred/DREHZAHL_SCALE; if (dummy>255) dummy=255; if (ADCmax<32) dummy=0; I2CTRN=dummy; break;	//nist, funktioniert nicht bei abgestecktem Motor, Byte 5 statt Byte 1
      default: I2CTRN=i2canz;			//dummywerte
	}

    _SCLREL=1;
  }
  _SI2CIF=0;
}


void watch(void){
  if (interrupt_cnt>1000){	//ca. 0,05sec: Wenn Strom > IMAX -> Reset
	if (Imaxcnt>10) asm("RESET");	//10*50u=500us
	if (Imaxcnt>0) Imaxcnt--;
  }
  if (OPENLOOPMODE) openspeed = drehzahl * (POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 /(60.0*DREHZAHL_SCALE));
  ClrWdt();
  if (timeout>=TIMEOUT){     //V2  Verriegelung nicht notwendig da das kein normaler Betrieb ist
    drehzahl=0; drehzahlalt=0; drehzahlneu=0; //V3 alt+neu ergänzt
	PORTA&=~MOTENABLE;
  }		
}

/************* START OF MAIN FUNCTION ***************/
int main ( void )
{
main_swr:
	PLLFBD = (int)(DPLL * 4 - 2); // dPLL derived in UserParms.h
	CLKDIVbits.PLLPOST = 0;		// N1=2
	CLKDIVbits.PLLPRE = 0;		// N2=2

	__builtin_write_OSCCONH(0x01);		//FRC statt Quarz
	__builtin_write_OSCCONL(0x01);		//Funktion??????????


	while(OSCCONbits.COSC != 0b001);
	// Wait for PLL to lock
	while(OSCCONbits.LOCK != 1);

	SMCInit(&smc1);
    SetupPorts();
   	SetupControlParameters(); 
    uGF.Word = 0;                   // clear flags

	TRISA&=~MOTENABLE;	
//	PORTA|=MOTENABLE;	//wird nur bei Bedarf eingeschaltet
	I2C_init();	

    while(1)
    {
        uGF.bit.ChangeSpeed = 0;
        // init Mode
        uGF.bit.OpenLoop = 1;           // start in openloop
		
        IEC0bits.AD1IE = 0;				// Make sure ADC does not generate interrupts while parameters are being initialized
        
        // init user specified parms and stop on error
        if( SetupParm() )
        {
            // Error
            uGF.bit.RunMotor=0;
            return;
        }
        
        // zero out i sums 
        PIParmD.qdSum = 0;
        PIParmQ.qdSum = 0;
        PIParmW.qdSum = 0;
     
        // Enable ADC interrupt and begin main loop timing
        IFS0bits.AD1IF = 0; 
        IEC0bits.AD1IE = 1;
		

        if(!uGF.bit.RunMotor)
        {	            
	   		while(drehzahl<DREHZAHL_MIN) watch();
			SetupParm();
            uGF.bit.RunMotor = 1;               //then start motor
        }

        // Run the motor
        uGF.bit.ChangeMode = 1;	// Ensure variable initialization when open loop is executed for the first time
		
		//Run Motor loop	
        while(1)
        {
			watch();
			if(drehzahl<DREHZAHL_MIN){  
	            uGF.bit.RunMotor = 0;
	            break;
			}
			if (uGF.bit.RunMotor==0) break; //TE Schleife verlassen falls Drehzahl zu klein und Motor daher gestoppt
           	uGF.bit.ChangeSpeed = !uGF.bit.ChangeSpeed; //???????????????ß

        }   // End of Run Motor loop
    } // End of Main loop
    while(1){}
}

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

void DoControl( void )
{
	ReadSignedADC0( &ReadADCParm );

    if(uGF.bit.OpenLoop)
        {
        // OPENLOOP:	force rotating angle, and control Iq and Id Also limits Vs vector to ensure maximum PWM duty
		//				cycle and no saturation

		// This If statement is executed only the first time we enter open loop, everytime we run the motor
        if( uGF.bit.ChangeMode )
        {
            // just changed to openloop
            uGF.bit.ChangeMode = 0;
            // synchronize angles

            // VqRef & VdRef not used
            CtrlParm.qVqRef = 0;
            CtrlParm.qVdRef = 0;
			CtrlParm.qVelRef = 0;
			Startup_Lock = 0;
			Startup_Ramp = 0;
			// Initialize SMC
			smc1.Valpha = 0;
			smc1.Ealpha = 0;
			smc1.EalphaFinal = 0;
			smc1.Zalpha = 0;
			smc1.EstIalpha = 0;
			smc1.Vbeta = 0;
			smc1.Ebeta = 0;
			smc1.EbetaFinal = 0;
			smc1.Zbeta = 0;
			smc1.EstIbeta = 0;
			smc1.Ialpha = 0;
			smc1.IalphaError = 0;
			smc1.Ibeta = 0;
			smc1.IbetaError = 0;
			smc1.Theta = 0;
			smc1.Omega = 0;
        }

		CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);

        if(AccumThetaCnt == 0)
	    {
            PIParmW.qInMeas = smc1.Omega;
		}

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);
        ParkParm.qVd    = PIParmD.qOut;

		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%.
		// The 5% left is needed to be able to measure current through
		// shunt resistors.
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)
		qVdSquared = FracMpy(PIParmD.qOut, PIParmD.qOut);
       	PIParmQ.qOutMax = _Q15sqrt(Q15(0.95*0.95) - qVdSquared);
		PIParmQ.qOutMin = -PIParmQ.qOutMax;

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);
        ParkParm.qVq    = PIParmQ.qOut;

    }
    else
    // Closed Loop Vector Control
    {
		// When it first transition from open to closed loop, this If statement is
		// executed
        if( uGF.bit.ChangeMode )
        {
            // just changed from openloop
            uGF.bit.ChangeMode = 0;
			// An initial value is set for the speed controller accumulation.
			// The first time the speed controller is executed, we want the output to be the same as it was the last time open loop was executed. So,
			// last time open loop was executed, torque refefernce was constant, and set to CtrlParm.qVqRef.
			// First time in closed loop, CtrlParm.qVqRef = PIParmW.qdSum >> 16
			// assuming the error is zero at time zero. This is why we set PIParmW.qdSum = (long)CtrlParm.qVqRef << 16.
			PIParmW.qdSum = (long)CtrlParm.qVqRef << 16;
			Startup_Lock = 0;
			Startup_Ramp = 0;
	    }               

		CtrlParm.qVelRef=drehzahl;

        // Check to see if new velocity information is available by comparing the number of interrupts per velocity calculation against the
        // number of velocity count samples taken.  If new velocity info is available, calculate the new velocity value and execute the speed control loop.
        if(AccumThetaCnt == 0)
        {
        	// Execute the velocity control loop
			PIParmW.qInMeas = smc1.Omega;
        	PIParmW.qInRef  = CtrlParm.qVelRef;
        	CalcPI(&PIParmW);
        	CtrlParm.qVqRef = PIParmW.qOut;
        }
       
		CtrlParm.qVdRef = 0; //keine Feldschwächung

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);

		ParkParm.qVd = PIParmD.qOut;

		// Vector limitation /Vd is not limited/ Vq is limited so the vector Vs is less than a maximum of 95%. 
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95// Vq = SQRT(0.95^2 - Vd^2)
		qVdSquared = FracMpy(ParkParm.qVd, ParkParm.qVd);
       	PIParmQ.qOutMax = _Q15sqrt(Q15(0.95*0.95) - qVdSquared);
		PIParmQ.qOutMin = -PIParmQ.qOutMax;

		if (CtrlParm.qVqRef>imax) CtrlParm.qVqRef=imax; //TE Mmax im Regelbetrieb
        if (CtrlParm.qVqRef<-imax) CtrlParm.qVqRef=-imax; //TE Mmax im Regelbetrieb

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);

		// If voltage ripple compensation flag is set, adjust the output of the Q controller depending on measured DC Bus voltage
       	ParkParm.qVq = PIParmQ.qOut;

		// Limit, if motor is stalled, stop motor commutation
		if (smc1.OmegaFltred < 0)
		{
			uGF.bit.RunMotor = 0;
        }
	}
}


void __attribute__((__interrupt__,no_auto_psv)) _MPWM1Interrupt(void)
{
static unsigned int time_32u;
	IFS3bits.PWM1IF = 0;
	if (uGF.bit.RunMotor)
	{
		if (PTMRbits.PTDIR == 0)
		{
			AD1CHS0bits.CH0SA = 0;	
			SEVTCMP = SEVTCMP_First;
			IFS0bits.AD1IF = 0;
			IEC0bits.AD1IE = 1;
			
			CalcRefVec();	// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
			CalcSVGen_ss();	// Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
		}
		else
		{
			// Calculate currents based on buffered measurements
			switch(Sector)
			{
				case 1:
				case 6:
					Ia = -Ic - Ib;
					break;
				case 3:
				case 4:
					Ib = -Ia - Ic;
					break;
				case 2:
				case 5:
					Ic = -Ia - Ib;				
					break;
			}				

			IFS0bits.AD1IF = 0;	// Clear any isr flags
			IEC0bits.AD1IE = 0;	// No interrupts to measure POT
			AD1CHS0bits.CH0SA = 3;	// Channel 0 positive input is AN3 for POT
			SEVTCMP = 0x8000 | (unsigned int)(2*LOOPINTCY/5); // 1 us after

		    interrupt_cnt++;
			
			MeasCompCurr_ss();	// Store Ia and Ib in Park, with offset comp.
			CalculateParkAngle(); // Estimate angle using SMC
			ClarkePark();// Calculate qId,qIq from qSin,qCos,qIa,qIb
	        DoControl();// Calculate control values
	        SinCos();// Calculate qSin,qCos from qAngle
	        InvPark();// Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq    
	        CalcRefVec();// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
	        CalcSVGen_ss();// Calculate and set PWM duty cycles from Vr1,Vr2,Vr3

			IFS0bits.AD1IF = 0;
		}	
	}
	time_32u++;
	if (time_32u>3125){
	  time_32u=0;
	  timeout++; //in 0,1s
	}
	return;
}

//---------------------------------------------------------------------
// The ADC ISR does speed calculation and executes the vector update loop.
// The ADC sample and conversion is triggered by the PWM period.
// The speed calculation assumes a fixed time interval between calculations.
//---------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void)
{
unsigned int ADCwert;
    	IFS0bits.AD1IF = 0;        
		ADCwert=ADCBUF0^0x8000; //VZ entfernt
		if (ADCwert>ADCmax){ADCmax=ADCwert;}
		if (ADCwert>IMAXR){Imaxcnt++;}	//TE für Überlastabschaltung bei 0,7A
		if (uGF.bit.RunMotor)
		{
			if (SEVTCMP == SEVTCMP_First) 
			{
				SEVTCMP = SEVTCMP_Second;
				switch(Sector)
				{
					case 1: 
					case 3:
						Ic = (signed int)(-ADCBUF0) + MeasCurrParm.Offseta;
						break;
					case 2:
					case 6:
						Ib = (signed int)(-ADCBUF0) + MeasCurrParm.Offseta;
						break;
					case 4:
					case 5:
						Ia = (signed int)(-ADCBUF0) + MeasCurrParm.Offseta;
						break;
				}
			}		
			else
			{
				switch(Sector)
				{
					case 1:
					case 5:
						Ib = (signed int)(ADCBUF0) - MeasCurrParm.Offseta;
						break;
					case 3:
					case 2:
						Ia = (signed int)(ADCBUF0) - MeasCurrParm.Offseta;
						break;
					case 6:
					case 4:
						Ic = (signed int)(ADCBUF0) - MeasCurrParm.Offseta;
						break;
				}	
			}
		}
	return;
}

//---------------------------------------------------------------------
bool SetupParm(void)
{
    CORCONbits.SATA  = 0;	// Turn saturation on to insure that overflows will be handled smoothly.

// ============= Open Loop ======================
	// MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	// Then, * 65536 which is a right shift done in "void CalculateParkAngle(void)"
	// ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;

    MeasCurrParm.qKa    = DQKA;    

    SVGenParm.iPWMPeriod = LOOPINTCY;          // Set PWM period to Loop Time 

// ============= Motor PWM ======================
    PDC1 = 0;
    PDC2 = 0;
    PDC3 = 0;

    // Center aligned PWM.
    // Note: The PWM period is set to dLoopInTcy/2 but since it counts up and 
    // and then down => the interrupt flag is set to 1 at zero => actual 
    // interrupt period is dLoopInTcy

    PTPER = LOOPINTCY/2;   // Setup PWM period to Loop Time defined in parms.h 

    PWMCON1 = 0x0077;       // Enable PWM 1,2,3 pairs for complementary mode
    DTCON1 = (0x40 | (DDEADTIME/2));     // Dead time
    DTCON2 = 0;
    FLTACON = 0;            // PWM fault pins not used

	IFS3bits.PWM1IF = 0;

	IEC3bits.PWM1IE = 1;
	PTCON = 0x8003;	// Enable PWM for center aligned with double updates
	// Phase of ADC capture set relative to PWM cycle: 0 offset and counting up
	SEVTCMP = PTPER / 2;	// This compare value is only used while motor is
								// not running to measure current offset

// ============= ADC - Measure Current & Pot ======================
// Sampling triggered by PWM and stored in signed fractional form.

    AD1CON1 = 0;
    // Signed fractional (DOUT = sddd dddd dd00 0000)
	AD1CON1bits.FORM = 3;
        //AD1CON1bits.FORM = 2;	//TE
    // Motor Control PWM interval ends sampling and starts conversion
    AD1CON1bits.SSRC = 3;  
    // Simultaneous Sample Select bit (only applicable when CHPS = 01 or 1x)
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    // Samples CH0 and CH1 simultaneously (when CHPS = 01)
//    AD1CON1bits.SIMSAM = 1;  
	AD1CON1bits.SIMSAM = 0;  //sequentiell
    // Sampling begins immediately after last conversion completes. 
    // SAMP bit is auto set.
    AD1CON1bits.ASAM = 1;  
	AD1CON1bits.AD12B=1; //12bit

    AD1CON2 = 0;

    AD1CHS0 = 0;    // ADCHS: ADC Input Channel Select Register 

    // Samples CH0
    AD1CON2bits.CHPS = 0;

	AD1CON3bits.SAMC = 3;	// 75 ns of sampling at TAD = 75 ns
	AD1CON3bits.ADRC = 0;	// Use system clock
	AD1CON3bits.ADCS = 2;	// TAD = (2 + 1) * TCY = 75 ns

	AD1CHS0bits.CH0SA = 0; 

    IFS0bits.AD1IF = 0; 
    IEC0bits.AD1IE = 1;

    // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
    AD1CHS123bits.CH123SA = 0;

    /* ADPCFG: ADC Port Configuration Register */
	AD1PCFGL = 0xFFFF;	    // Set all ports digital
    AD1PCFGLbits.PCFG0 = 0;   // AN0 analog - IA
//    AD1PCFGLbits.PCFG1 = 0;   // AN1 analog - IB
	AD1PCFGLbits.PCFG2 = 0;   // AN2 analog - VBUS
//    AD1PCFGLbits.PCFG3 = 0;   // AN2 analog - VBUS	
	//##########TE AD1PCFGLbits.PCFG5 = 0;   // AN5 analog - IBUS
//    AD1PCFGLbits.PCFG8 = 0;   // AN8 analog - POT

    /* ADCSSL: ADC Input Scan Select Register */
    AD1CSSL = 0;
	AD1CON2bits.SMPI = 0;		//Interrupt after every conversion (DMA takes all four conversions)

    // Turn on A/D module
    AD1CON1bits.ADON = 1;

	// Wait until first conversion takes place to measure offsets.
	DebounceDelay();

	// Initial Current offsets
	InitMeasCompCurr_ss( ADC1BUF0 );
	
    return False;
}

void CalculateParkAngle(void)
{
 	smc1.Ialpha = ParkParm.qIalpha;
  	smc1.Ibeta = ParkParm.qIbeta;
    smc1.Valpha = ParkParm.qValpha;
    smc1.Vbeta = ParkParm.qVbeta;

	SMC_Position_Estimation(&smc1);

	if(uGF.bit.OpenLoop)	
	{
        if(!OPENLOOPMODE){
		  if (Startup_Ramp < MotorParm.EndSpeed)
			// Ramp starts, and increases linearly until EndSpeed is reached.
			// After ramp, estimated theta is used to commutate motor.
			Startup_Ramp += DELTA_STARTUP_RAMP;
		  else
		  {
			// This section enables closed loop, right after open loop ramp.
            uGF.bit.ChangeMode = 1;
               uGF.bit.OpenLoop = 0;

			// Difference between force angle and estimated theta is saved,
			// so a soft transition is made when entering closed loop.
			Theta_error = ParkParm.qAngle - smc1.Theta;
		  }
		}else{		//openloopmode: 
		  if (Startup_Ramp < openspeed)
			// Ramp starts, and increases linearly until EndSpeed is reached.
			// After ramp, estimated theta is used to commutate motor.
			Startup_Ramp += DELTA_STARTUP_RAMP;
		  else 
 		  if(Startup_Ramp > openspeed)
			Startup_Ramp -= DELTA_STARTUP_RAMP;
		}
		ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	}
	else
	{
		// This value is used to transition from open loop to closed looop. 
		// At the end of open loop ramp, there is a difference between 
		// forced angle and estimated angle. This difference is stored in 
		// Theta_error, and added to estimated theta (smc1.Theta) so the 
		// effective angle used for commutating the motor is the same at 
		// the end of open loop, and at the begining of closed loop. 
		// This Theta_error is then substracted from estimated theta 
		// gradually in increments of 0.05 degrees until the error is less
		// than 0.05 degrees.
		ParkParm.qAngle = smc1.Theta + Theta_error;
		if (_Q15abs(Theta_error) > _0_05DEG)
		{
			if (Theta_error < 0)
				Theta_error += _0_05DEG;
			else
				Theta_error -= _0_05DEG;
		}
	}
	return;
}

void SetupControlParameters(void)
{

// ============= PI D Term ===============      
    PIParmD.qKp = DKP;       
    PIParmD.qKi = DKI;              
    PIParmD.qKc = DKC;       
    PIParmD.qOutMax = DOUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

// ============= PI Q Term ===============
    PIParmQ.qKp = QKP;    
    PIParmQ.qKi = QKI;
    PIParmQ.qKc = QKC;
    PIParmQ.qOutMax = QOUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

// ============= PI W Term ===============
    PIParmW.qKp = WKP;       
    PIParmW.qKi = WKI;       
    PIParmW.qKc = WKC;       
    PIParmW.qOutMax = WOUTMAX;   
    PIParmW.qOutMin = -PIParmW.qOutMax;

    InitPI(&PIParmW);
	return;
}

void DebounceDelay(void)
{
	long i;
	for (i = 0;i < 100000;i++)
		;
	return;
}


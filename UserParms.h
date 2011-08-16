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
 *    Filename:       UserParms.h                                      *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ32MC204.gld                                 *
 *                                                                     *
 **********************************************************************/
#ifndef UserParms_H
#define UserParms_H

//************** Start-Up Parameters **************

#define LOCKTIMEINSEC  0.1	//#####0.25	// Initial rotor lock time in seconds			//0.1
								// Make sure LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC)
								// is less than 65535.
#define OPENLOOPTIMEINSEC 0.2 // Open loop time in seconds. This is the time that	
								// will take from stand still to closed loop.
#define INITIALTORQUE	0.35 // Imax=(3,3/2)/(0.5*1.0)=3,3A	Meßbereich, Motor max 1,5A
								// Initial Torque demand in Amps.
								// Enter initial torque demand in Amps using REFINAMPS() 
								// macro. Maximum Value for reference is defined by 
								// shunt resistor value and differential amplifier gain.
								// Use this equation to calculate maximum torque in 
								// Amperes:
								// 
								// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
								//
								// For example:
								//
								// RSHUNT = 0.005
								// VDD = 3.3
								// DIFFAMPGAIN = 75
								//
								// Maximum torque reference in Amps is:
								//
								// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
#define ENDSPEEDOPENLOOP MINSPEEDINRPM+10

//************** Motor Parameters **************
// 0,3..1,3U/min nach Getriebe 375/1,3 -> 86..375U/min -> 1,44..6,25U/sec -> 72..312Hz
#define POLEPAIRS      	50       // Number of pole pairs
#define PHASERES		((float)3.0) //((float)6.0) //((float)2.67)	// Phase resistance in Ohms.
#define PHASEIND		((float)0.0051) //((float)0.0102) //((float)0.00192)// Phase inductance in Henrys.
#define NOMINALSPEEDINRPM 400	// Make sure NOMINALSPEEDINRPM generates a MAXOMEGA < 1.0
#define FIELDWEAKSPEEDRPM NOMINALSPEEDINRPM 
								// Use this formula:
								// MAXOMEGA = NOMINALSPEEDINRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce NOMINALSPEEDINRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// NOMINALSPEEDINRPM.
#define MINSPEEDINRPM	20		// Minimum speed in RPM. Closed loop will operate at this
								// speed. Open loop will transition to closed loop at
								// this minimum speed. Minimum POT position (CCW) will set
								// a speed reference of MINSPEEDINRPM

//************** Oscillator Parameters **************

#define PLLIN		7300000		// External Crystal or Clock Frequency (Hz)
#define DESIREDMIPS	40000000	// Enter desired MIPS. 

//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY	16000		// PWM Frequency in Hertz
#define DEADTIMESEC		0.000000	// Deadtime in seconds
//#define	BUTPOLLOOPTIME	0.100		// Button polling loop period in sec
#define SPEEDLOOPFREQ	1000		// Speed loop Frequency in Hertz. This value must  //1000
									// be an integer to avoid pre-compiler error

//************** Slide Mode Controller Parameters **********

#define SMCGAIN			0.4 //0.4		// Slide Mode Controller Gain (0.0 to 0.9999)	
#define MAXLINEARSMC    0.003 //0.003		// If measured current - estimated current  
								// is less than MAXLINEARSMC, the slide mode
								// Controller will have a linear behavior
								// instead of ON/OFF. Value from (0.0 to 0.9999)
#define FILTERDELAY		90 //90		// Phase delay of two low pass filters for			
								// theta estimation. Value in Degrees from
								// from 0 to 359.

//************** Hardware Parameters ****************

#define RSHUNT			0.5 // Value in Ohms of shunt resistors used.
#define DIFFAMPGAIN		1.0  	// Gain of differential amplifier.
#define VDD				3.3		// VDD voltage, only used to convert torque
								// reference from Amps to internal variables

//*************** Optional Modes **************
//#define OPENLOOPONLY

//************** PI Coefficients **************

//******** D Control Loop Coefficients *******
#define     DKP        Q15(0.01)	//0.01
#define     DKI        Q15(0.005)	//0.005
#define     DKC        Q15(0.99999)	//0.99999
#define     DOUTMAX    Q15(0.95) 	//0.95

//******** Q Control Loop Coefficients *******
#define     QKP        Q15(0.01)	//0.01
#define     QKI        Q15(0.005)	//0.005
#define     QKC        Q15(0.99999) //0.99999
#define     QOUTMAX    Q15(0.5) 	//0.5
#define imax REFINAMPS(INITIALTORQUE)	//TE

//*** Velocity Control Loop Coefficients *****
#define     WKP        Q15(0.05)	//0.05
#define     WKI        Q15(0.005)	//0.005
#define     WKC        Q15(0.99999) //0.99999
#define     WOUTMAX    Q15(0.2) 	//0.2

//************** ADC Scaling **************
#define     DQKA       Q15(0.5) //(0.5)	// Current feedback software gain

//************** Derived Parameters ****************
#define DPLL		(unsigned int)(2.0*DESIREDMIPS/PLLIN)	// PLL ratio
#define FOSC		(PLLIN*DPLL)	// Clock frequency (Hz)
#define DFCY        (FOSC/2)		// Instruction cycle frequency (Hz)
#define DTCY        (1.0/DFCY)		// Instruction cycle period (sec)
#define DDEADTIME   (unsigned int)(DEADTIMESEC*DFCY)	// Dead time in dTcys
#define LOOPTIMEINSEC (1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (unsigned int)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOOPINTCY	(LOOPTIMEINSEC/DTCY)   // Basic loop period in units of Tcy
//#define LOCKTIME	(unsigned int)(LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC))
// Time it takes to ramp from zero to MINSPEEDINRPM. Time represented in seconds
#define DELTA_STARTUP_RAMP	(unsigned int)(MINSPEEDINRPM*POLEPAIRS*LOOPTIMEINSEC* \
							LOOPTIMEINSEC*65536*65536/(60*OPENLOOPTIMEINSEC))
// Number of control loops that must execute before the button routine is executed.
//#define	BUTPOLLOOPCNT	(unsigned int)(BUTPOLLOOPTIME/LOOPTIMEINSEC)

#endif

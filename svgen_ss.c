//*******************************************************************
// SVGen
//  
//Description:  Calculate and load SVGen PWM values.
//
//
//
//Functional prototype:
// 
// void CalcSVGen( void )
//
//On Entry:   SVGenParm structure must contain qVr1, qVr2, qVr3
//
//On Exit:    PWM registers loaded
//
//Parameters: 
// Input arguments: None
//
// Return:
//   Void
//
// SFR Settings required:
//         CORCON.SATA  = 0
//         CORCON.IF    = 0
//
// Support routines required: None
//
// Local Stack usage: 0
//
// Registers modified: w0,w2,w3,w4,w5,w6,AccA
//
// Timing: 34 instruction cycles
//
//*******************************************************************
// C-Version of code
//

#include "general.h"
#include "svgen.h"
#include "park.h"

#define SQ3OV2 0x6ED9
volatile int TCRIT = 420;
//volatile int DEADTIME = 120;
volatile int DEADTIME = 0;	//da bereits im Treiber TE
volatile short T1, T2, Ta, Tb, Tc, __Ta, __Tb, __Tc;
volatile int T1Comp = 0, T2Comp = 0;
extern volatile int Sector;
extern volatile int SEVTCMP_First;
extern volatile int SEVTCMP_Second;
extern unsigned char DREHRICHTUNG;
#define PDC1x PDC1
#define PDC2x PDC2
#define PDC3x PDC3
void CalcSVGen_ss( void )
{ 
    if( SVGenParm.qVr1 >= 0 )
    {       
        // (xx1)
        if( SVGenParm.qVr2 >= 0 )
        { // PDC1 > PDC2 > PDC3
            // (x11)
            // Must be Sector 3 since Sector 7 not allowed
            // Sector 3: (0,1,1)  0-60 degrees
			Sector = 3;
            T2 = SVGenParm.qVr2;
            T1 = SVGenParm.qVr1;
            CalcTimes_ss();
            PDC1x = Ta;
            if(DREHRICHTUNG){PDC2x = Tb; PDC3x = Tc;}
			else{PDC3x = Tb; PDC2x = Tc;}
			if (PTMRbits.PTDIR == 1)
			{
				SEVTCMP_First = (Tb + Tc) / 4 + DEADTIME; //(PDC2x + PDC3x) / 4 + DEADTIME;
				SEVTCMP_Second = (Ta + Tb) / 4 + DEADTIME; //(PDC1x + PDC2x) / 4 + DEADTIME;
			}
        }
        else
        {            
            // (x01)
            if( SVGenParm.qVr3 >= 0 )
            { // PDC2 > PDC3 > PDC1
                // Sector 5: (1,0,1)  120-180 degrees
				Sector = 5;
                T2 = SVGenParm.qVr1;
                T1 = SVGenParm.qVr3;
                CalcTimes_ss();
                PDC1x = Tc;
                if(DREHRICHTUNG){PDC2x = Ta; PDC3x = Tb;}
				else{PDC3x = Ta; PDC2x = Tb;}
				if (PTMRbits.PTDIR == 1)
				{
					SEVTCMP_First = (Tc + Tb) / 4 + DEADTIME;			//(PDC1x + PDC3x) / 4 + DEADTIME;			
					SEVTCMP_Second = (Ta + Tb) / 4 + DEADTIME;		//(PDC2x + PDC3x) / 4 + DEADTIME;
				}
            }
            else
            { // PDC2 > PDC1 > PDC3
                // Sector 1: (0,0,1)  60-120 degrees
				Sector = 1;
                T2 = -SVGenParm.qVr2;
                T1 = -SVGenParm.qVr3;
                CalcTimes_ss();
                PDC1x = Tb;
                if(DREHRICHTUNG){PDC2x = Ta; PDC3x = Tc;}
				else{PDC3x = Ta; PDC2x = Tc;}
				if (PTMRbits.PTDIR == 1)
				{
					SEVTCMP_First = (Tb + Tc) / 4 + DEADTIME;		//(PDC1x + PDC3x) / 4 + DEADTIME;
					SEVTCMP_Second = (Tb + Ta) / 4 + DEADTIME;	//(PDC1x + PDC2x) / 4 + DEADTIME;
				}
            }
        }
    }
    else
        {
        // (xx0)
        if( SVGenParm.qVr2 >= 0 )
        {
            // (x10)
            if( SVGenParm.qVr3 >= 0 )
            { // PDC3 > PDC1 > PDC2
                // Sector 6: (1,1,0)  240-300 degrees
				Sector = 6;
		        T2 = SVGenParm.qVr3;
                T1 = SVGenParm.qVr2;
                CalcTimes_ss();
                PDC1x = Tb;
                if(DREHRICHTUNG){PDC2x = Tc; PDC3x = Ta;}
				else{PDC3x = Tc; PDC2x = Ta;}
				if (PTMRbits.PTDIR == 1)
				{
					SEVTCMP_First = (Tb + Tc) / 4 + DEADTIME;		//(PDC1x + PDC2x) / 4 + DEADTIME;
					SEVTCMP_Second = (Tb + Ta) / 4 + DEADTIME;	//(PDC1x + PDC3x) / 4 + DEADTIME;
        		}
            }
            else
            { // PDC1 > PDC3 > PDC2
                // Sector 2: (0,1,0)  300-0 degrees
				Sector = 2;
                T2 = -SVGenParm.qVr3;
                T1 = -SVGenParm.qVr1;
                CalcTimes_ss();
                PDC1x = Ta;
                if(DREHRICHTUNG){PDC2x = Tc; PDC3x = Tb;}
				else{PDC3x = Tc; PDC2x = Tb;}
				if (PTMRbits.PTDIR == 1)
				{
					SEVTCMP_First = (Tc + Tb) / 4 + DEADTIME;		//(PDC2x + PDC3x) / 4 + DEADTIME;
					SEVTCMP_Second = (Ta + Tb) / 4 + DEADTIME;	//(PDC1x + PDC3x) / 4 + DEADTIME;
				}
            }
        }
        else
        { // PDC3 > PDC2 > PDC1            
            // (x00)
            // Must be Sector 4 since Sector 0 not allowed
            // Sector 4: (1,0,0)  180-240 degrees
            Sector = 4;
			T2 = -SVGenParm.qVr1;
            T1 = -SVGenParm.qVr2;
            CalcTimes_ss();
            PDC1x = Tc;
            if(DREHRICHTUNG){PDC2x = Tb; PDC3x = Ta;}
			else{PDC3x = Tb; PDC2x = Ta;}
			if (PTMRbits.PTDIR == 1)
			{
				SEVTCMP_First = (Tc + Tb) / 4 + DEADTIME;	//(PDC1x + PDC2x) / 4 + DEADTIME;
				SEVTCMP_Second = (Tb + Ta) / 4 + DEADTIME; //(PDC2x + PDC3x) / 4 + DEADTIME;
			}
		}
    }
	return;
}


void CalcTimes_ss(void)
{
    T1 = ((long)SVGenParm.iPWMPeriod*(long)T1) >> 15;
    T2 = ((long)SVGenParm.iPWMPeriod*(long)T2) >> 15;
    Tc = (SVGenParm.iPWMPeriod-T1-T2)>>1;
	if (PTMRbits.PTDIR == 1)
	{
		if (T1 > TCRIT)
		{
			Tb = Tc + T1;
		}
		else
		{
			Tb = Tc + TCRIT;
			T1Comp = 1;
		}
		if (T2 > TCRIT)
		{
		    Ta = Tb + T2;
		}
		else
		{
		    Ta = Tb + TCRIT;
			T2Comp = 1;
		}
	}
	else
	{
		if (T1Comp == 1)
		{
			T1Comp = 0;
			Tb = Tc + T1 + T1 - TCRIT;
		}
		else
		{
			Tb = Tc + T1;
		}
		if (T2Comp == 1)
		{
			T2Comp = 0;
			Ta = Tb + T2 + T2 - TCRIT;
		}
		else
		{
			Ta = Tb + T2;
		}
		
	}
		
	return;
}        


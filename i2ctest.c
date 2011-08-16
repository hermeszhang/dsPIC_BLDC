/*****************************************************
This program was produced by the
CodeWizardAVR V1.25.7 Professional
Automatic Program Generator
© Copyright 1998-2007 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 20.01.2009
Author  : F4CG                            
Company : F4CG                            
Comments: 


Chip type           : ATmega128
Program type        : Application
Clock frequency     : 8,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024
*****************************************************/

#include <mega128.h> 
#include <delay.h>
char ia,ib,ic; //globale I2C Lesevariablen
#define SLAVEADR 0x70
char n;
//######################################################## I2C Pallhuber, modifiziert ANFANG    
//delays für slow I2C ergänzt
//#define I2C_BUS_TIMEOUT			500
#define SDA         PORTD.1     //Ver 2.6 für Wasserplatine D.1
#define SCL         PORTD.0     //Ver 2.6 für Wasserplatne D.0 
#define PIN_SDA     PIND.1      //D.1
#define PIN_SCL     PIND.0      //D.0
#define SDA_OUT     (DDRD.1 = 1)
#define SDA_IN      (DDRD.1 = 0) 
#define SCL_OUT     (DDRD.0 = 1)
#define SCL_IN      (DDRD.0 = 0) 
#define HIGH        1
#define LOW         0   
#define MSB         0x80    
#define RETURN_FALSE            0xFF
#define RETURN_TRUE             0x01
#define RETURN_NO_ERROR         0x00 

#undef CLOCK_STRETCH_TIME       
void i2cInit(void)
{
    SDA_IN;        
    SCL_IN;      
    #asm("WDR");
}

void i2cStart(void)
{ 
  SDA = LOW;     // könnte immer 0 bleiben
  SDA_OUT;
  delay_us(5);
  SCL = LOW;        // könnte immer 0 bleiben
  SCL_OUT;    
  delay_us(5);  
}

unsigned char i2cStop(void)
{
  SDA = LOW;         // könnte immer 0 bleiben
  SDA_OUT;    
  delay_us(5);
  SCL_IN; 
  delay_us(5);
#ifdef CLOCK_STRETCH_TIME	
	Interbuswait_ms = 0;
    do  {
        if ( Interbuswait_ms >= I2C_BUS_TIMEOUT )
            return RETURN_TIMEOUT;
    }while ( !PIN_SCL );	
#else	
	while ( !PIN_SCL ); 
#endif	
	SDA_IN;
    return RETURN_NO_ERROR;    
}
#pragma used+
unsigned char i2cRestart(void)
{
  SDA_IN;    
  delay_us(5);
  SCL_IN;
  delay_us(5);  
#ifdef CLOCK_STRETCH_TIME	
	Interbuswait_ms = 0;
    do  {
        if ( Interbuswait_ms >= I2C_BUS_TIMEOUT )
            return RETURN_TIMEOUT;
    }while ( !PIN_SCL );	
#else	
	while ( !PIN_SCL ); 
#endif        
  SDA = LOW;        //könnte immer 0 bleiben
  SDA_OUT;    
  delay_us(5);
  SCL = LOW;        //könnte immer 0 bleiben
  SCL_OUT;
  return RETURN_NO_ERROR;              
}
#pragma used-

unsigned char i2cSendAck(void)
{
  SDA = LOW;           //könnte immer 0 bleiben
  SDA_OUT; 
  delay_us(1);
  SCL_IN;     
  delay_us(5);
#ifdef CLOCK_STRETCH_TIME	
	Interbuswait_ms = 0;
    do  {
        if ( Interbuswait_ms >= I2C_BUS_TIMEOUT )
            return RETURN_TIMEOUT;
    }while ( !PIN_SCL );	
#else	
	while ( !PIN_SCL ); 
#endif
  SCL = LOW;            //könnte immer 0 bleiben
  SCL_OUT; 
  delay_us(5);  
  return RETURN_NO_ERROR;         
}
unsigned char i2cSendNack(void)
{
  SDA_IN;  
  delay_us(5);
  SCL_IN;
  delay_us(5);
#ifdef CLOCK_STRETCH_TIME	
	Interbuswait_ms = 0;
    do  {
        if ( Interbuswait_ms >= I2C_BUS_TIMEOUT )
            return RETURN_TIMEOUT;
    }while ( !PIN_SCL );	
#else	
	while ( !PIN_SCL ); 
#endif
  SCL = LOW;              //könnte immer 0 bleiben
  SCL_OUT;    
  delay_us(5);
  return RETURN_NO_ERROR; 
}   

unsigned char i2cReceiveAck(void)
{
  SDA_IN;
  delay_us(5);
  SCL_IN;     
  delay_us(5);
#ifdef CLOCK_STRETCH_TIME	
	Interbuswait_ms = 0;
    do  {
        if ( Interbuswait_ms >= I2C_BUS_TIMEOUT )
            return RETURN_TIMEOUT;
    }while ( !PIN_SCL );	
#else	
	while ( !PIN_SCL ); 
#endif
  if ( !PIN_SDA ) {
    SCL = LOW;                 //könnte immer 0 bleiben
    SCL_OUT;  
    delay_us(5);
    return RETURN_TRUE;    
    } else {
        SCL = LOW;               //könnte immer 0 bleiben
        SCL_OUT;
        delay_us(5); 
        return RETURN_FALSE;     
    }    
    return RETURN_NO_ERROR;    
}
unsigned char i2cSendByte(unsigned char data)
{
    unsigned char i;
    for( i=0 ; i<8 ; i++ )  {
        if ( (data << i) & MSB )    {
          SDA_IN; 
          delay_us(5);
 	  SCL_IN;         
 	  delay_us(5);
#ifdef CLOCK_STRETCH_TIME	
        Interbuswait_ms = 0;
        do  {
            if ( Interbuswait_ms >= I2C_BUS_TIMEOUT )
                return RETURN_TIMEOUT;
        }while ( !PIN_SCL );	
#else	
	    while ( !PIN_SCL ); 
#endif
        } else {
          SDA = LOW;                    //könnte immer 0 bleiben
	  SDA_OUT; 
	  delay_us(5);
	  SCL_IN; 
	  delay_us(5);
#ifdef CLOCK_STRETCH_TIME	
	    Interbuswait_ms = 0;
        do  {
            if ( Interbuswait_ms >= I2C_BUS_TIMEOUT )
                return RETURN_TIMEOUT;
        }while ( !PIN_SCL );	
#else	
	    while ( !PIN_SCL ); 
#endif			       
        }
		SCL = LOW;           //könnte immer 0 bleiben
		SCL_OUT;   
		delay_us(5);
    }
    return RETURN_NO_ERROR;
}
unsigned char i2cReceiveByte(void) 
{
    unsigned char i, received;
    received = 0;
	SDA_IN;
	for( i=0 ; i<8 ; i++ )  {
 	  SCL_IN;
 	  delay_us(5);
#ifdef CLOCK_STRETCH_TIME	
	    Interbuswait_ms = 0;
        do  {
            if ( Interbuswait_ms >= I2C_BUS_TIMEOUT )
                return RETURN_TIMEOUT;
        }while ( !PIN_SCL );	
#else	
	    while ( !PIN_SCL ); 
#endif
        if ( PIN_SDA ) {
            received |= 1 << ( 7 - i ); 
        }
        SCL = LOW;                         //könnte immer 0 bleiben
        SCL_OUT;	
        delay_us(5);	
	}
    return received;
}
//######################################################## I2C Pallhuber, modifiziert ENDE 

#define slave_adress_uhr 0x51 

void rtc_init_i(char on){
char dummy;
  i2cStart();
  i2cSendByte(2*slave_adress_uhr);    dummy = i2cReceiveAck();
  i2cSendByte(0x0d);    dummy = i2cReceiveAck();      
  if (on==0){   //1024 Hz off
    i2cSendByte(0x00);    dummy = i2cReceiveAck();      //off Takt
  }  
  else{                                                             
    i2cSendByte(0x81);    dummy = i2cReceiveAck();      //1KHz Takt
  }  
  i2cStop();                                                                      
}            
   
#define NEW PORTE.2           
#define MOTADR 0x13     //ECO Print Klappenverstellung als Wasser Zubehör, gefädelt
/*void i2c_mot(void){
char dummy,a,b,c;
static char count=0;
  i2cStart();
  i2cSendByte(2*MOTADR);    dummy = i2cReceiveAck(); //motor
  i2cSendByte(0xf7);    dummy = i2cReceiveAck();    
  i2cSendByte(0x31);    dummy = i2cReceiveAck();      
  i2cSendByte(count++);    dummy = i2cReceiveAck();  
  i2cStop();                    //#############ACHTUNG dsPIC Fehler, kein RESTART!     
  NEW=0;
  delay_us(150);
  NEW=1;
  i2cStart();                   //erneuter Start   
  i2cSendByte(2*MOTADR+1);    dummy = i2cReceiveAck(); //motor  
  a=i2cReceiveByte(); dummy=i2cSendAck();
  b=i2cReceiveByte(); dummy=i2cSendAck();
  c=i2cReceiveByte(); dummy=i2cSendNack();
  i2cStop();                                                                      
}         
*/

void i2c_mot_write(char n, char bef, char dat1, char dat2){
char dummy;
  i2cStart();
  i2cSendByte(2*SLAVEADR);    dummy = i2cReceiveAck(); //motor
  i2cSendByte(bef);    dummy = i2cReceiveAck();    
  if (n>1){ i2cSendByte(dat1);    dummy = i2cReceiveAck();}      
  if (n>2){ i2cSendByte(dat2);    dummy = i2cReceiveAck();}  
  i2cStop();  
  delay_ms(5); //da derzeit Befehl im ECO nicht zwischengespeichert wird                   
}

void i2c_mot_read(void){
char dummy;
  i2cStart();            
  i2cSendByte(2*SLAVEADR+1); dummy = i2cReceiveAck(); //motor  
  ia=i2cReceiveByte(); dummy=i2cSendAck();
  ib=i2cReceiveByte(); dummy=i2cSendAck();
  ic=i2cReceiveByte(); dummy=i2cSendNack();
  i2cStop();                                                                      
}

void pause(char t){
char i;
  for (i=0; i<t; i++){
    i2c_mot_write(1,20,0,0);        //dummy Befehl ohne Funktion, aber RS232 Protokoll wird getriggert  
    delay_ms(500);
  }
}  

void main(void)
{
PORTA=0x00; DDRA=0x00;
PORTB=0x00; DDRB=0x00;
PORTC=0x00; DDRC=0x00;
PORTD=0x00; DDRD=0x00;
PORTE=0x80; DDRE=0x04;
PORTF=0x00; DDRF=0x00;
PORTG=0x00; DDRG=0x00;

// Analog Comparator: Off
ACSR=0x80; SFIOR=0x00;

n=255;
while(1){
  i2c_mot_write(1,200,0,0); //phisoll
  delay_ms(4000);
  i2c_mot_write(1,125,0,0); //phisoll
  delay_ms(2000);
  i2c_mot_write(1,60,0,0); //phisoll
  delay_ms(2000);
  i2c_mot_write(1,30,0,0); //phisoll
  delay_ms(1000);
  i2c_mot_write(1,15,0,0); //n soll
  delay_ms(1000);
  i2c_mot_write(1,100,0,0); //phisoll
  delay_ms(2000);
    
}


/* 
#define M_RES 0
#define M_INC 1
#define M_DEC 2
#define M_MAX 3
#define M_MIN 4
#define M_IST 5
#define M_SOLL 6
i2c_mot_write(3,5,29300/256,29300%256); //phiist   
i2c_mot_write(1,0,0,0); //Fehlerzähler löschen
while (1){
/*  rtc_init_i(1);        //1KHz ein    zum Test der I2C Adressierung
  delay_ms(300);
  rtc_init_i(0);        //1KHz aus
  delay_ms(300);
*/
/*  i2c_mot_write(2,1,40,0);  //+40 Schritte
  pause(10);
  i2c_mot_write(2,2,40,0);  //-40 Schritte
  pause(10);
*/

/* i2c_mot_write(1,3,0,0); //Anschlag rechts    
  pause(20);
  i2c_mot_write(1,4,0,0); //Anschlag links
  pause(20);    
*/
  
/*  i2c_mot_write(3,6,29500/256,29500%256); //phisoll
  pause(30);  
  i2c_mot_write(3,6,29700/256,29700%256);   
  pause(30);
  i2c_mot_read();
  if (ia>2){
    i2c_mot_write(3,5,31000/256,31000%256); //phiist Initialisierung (Anschlag)   
    i2c_mot_write(3,6,31000/256,31000%256); //phisoll 
    i2c_mot_write(1,0,0,0); //Fehlerzähler löschen
  }  
    
  
};
*/
}

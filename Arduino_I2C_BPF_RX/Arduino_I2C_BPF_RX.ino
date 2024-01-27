/*

I2C Slave with multiple addresses

It receives I2C commands to activate the BandPass and/or Receiver Relays.
It receives I2C commands as more than one I2C slave (multiple addresses)
    to replace the two PCF8574 in Arjan-5 (uSDX Pico FFT project)
Adding new I2C command to read the SWR
    it uses A0 and A1 analog inputs to read the SWR Forward and Reflected information.
  
Reference:
https://stackoverflow.com/questions/34691478/arduino-as-slave-with-multiple-i2c-addresses
https://github.com/alexisgaziello/TwoWireSimulator

I2C Wire library with multi address:
https://github.com/arduino/ArduinoCore-avr/pull/90/files#diff-e4603cea13a2a6370bdf819d929e8fb9b272c812bc1df9a9190b365875c47db3



Arduino Pro Mini 3V3   ATmega328P 8Mhz
Flash: 32KB
SRAM: 2KB
EEPROM: 1KB
14 digital pins, input or output
Serial: 0 (RX) and 1 (TX)
External Interrupts: 2 and 3
PWM: 3, 5, 6, 9, 10, and 11
PI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK)
LED: 13
8 analog inputs, 10 bits of resolution: A0 - A7
I2C: A4 (SDA) and A5 (SCL)


Created: May 2023
Author: Klaus Fensterseifer  PY2KLA
https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj

*/

#include "Wire_.h"  //Local Wire Library  (same as Wire but with addings)



/* BPF write */
#define I2C_BPF		(0x20)	//0x20 write
#define REL_LPF2_pin		2   //160MHz
#define REL_BPF6_pin		3   //80MHz
#define REL_BPF12_pin		4   //40MHz
#define REL_BPF24_pin		5   //20MHz
#define REL_BPF40_pin		6   //10Mhz

#define REL_BPF_val_num   5
#define REL_LPF2_val		0x01
#define REL_BPF6_val		0x02
#define REL_BPF12_val		0x04
#define REL_BPF24_val		0x08
#define REL_BPF40_val		0x10

/* RX write */
#define I2C_RX 		(0x21)	//0x21 write
#define REL_ATT_20_pin	9
#define REL_ATT_10_pin	8
#define REL_PRE_10_pin	7

#define REL_ATT_val_num   5
#define REL_ATT_30_val	0x03
#define REL_ATT_20_val	0x01
#define REL_ATT_10_val	0x02
#define REL_ATT_00_val	0x00
#define REL_PRE_10_val	0x04
#define REL_PRE_00_val	0x00

/* SWR read */
#define I2C_SWR 	    	0x20   // read (use the same I2C_BPF address to make easy to build the multi I2C mask)
#define I2C_SWR_FOR 		0x21   // read
#define I2C_SWR_REF 		0x22   // read
#define vForwardPin       A0    // select the input pin for the swr analog reading
#define vReflectedPin     A1    // select the input pin for the swr analog reading
#define VMIN              20    // min forward AD value for swr
#define SWR_BASE10        10    // numeric base for SWR decimal
#define SWR_MIN           (1 * SWR_BASE10)    // SWR = 1.0
#define SWR_MAX           ((15 * SWR_BASE10)+(SWR_BASE10-1))  // byte max value, SWR max = 15.9

uint8_t swr_8bits, swr_for, swr_ref;  //swr info to send to I2C master

#define I2C_ADDR (I2C_BPF | I2C_RX)
#define I2C_MASK ((I2C_BPF | I2C_RX | I2C_SWR_REF) ^ (I2C_BPF & I2C_RX & I2C_SWR_REF))  /* xor */



#define ledPin      13     // define LED pin number


uint8_t RX_Relays=0, RX_Relays_old=0xff;
uint8_t BPF_Relays=0, BPF_Relays_old=0xff;
uint8_t I2C_Data;
uint8_t rec=0, I2C_Address;
const uint8_t REL_BPF_val[REL_BPF_val_num] = {REL_LPF2_val, REL_BPF6_val, REL_BPF12_val, REL_BPF24_val, REL_BPF40_val};
const uint8_t REL_ATT_val[REL_ATT_val_num] = {REL_PRE_10_val, REL_ATT_30_val, REL_ATT_20_val, REL_ATT_10_val, REL_ATT_00_val};



unsigned long nextMillis;
#define LOOP_PERIOD_MILLIS  500


/*****************************************************************************************/
void setup() 
{
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(115200);  //choose the right clock for Arduino Pro Mini at Tools Processor
  for(int i=0; i<50; i++)    // wait some time for Serial to open
  {
  digitalWrite(ledPin, 1);  //toggle led
  delay(50);                       // wait
  digitalWrite(ledPin, 0);  //toggle led
  delay(50);                       // wait
  if(Serial)  //serial opened
    break;
  }  // If the serial does not open, the print commands will have no effect
  Serial.println("\nArduino I2C Slave Multi Address");
  //Serial.print("FREQ CPU: ");
  //Serial.println(F_CPU);   //prints the clock frequency, chose the right clock for Arduino Pro Mini at Tools Processor

  
  pinMode(REL_LPF2_pin, OUTPUT);
  pinMode(REL_BPF6_pin, OUTPUT);
  pinMode(REL_BPF12_pin, OUTPUT);
  pinMode(REL_BPF24_pin, OUTPUT);
  pinMode(REL_BPF40_pin, OUTPUT);
  pinMode(REL_ATT_20_pin, OUTPUT);
  pinMode(REL_ATT_10_pin, OUTPUT); 
  pinMode(REL_PRE_10_pin, OUTPUT);

  digitalWrite(REL_LPF2_pin, 1);
  digitalWrite(REL_BPF6_pin, 0);        
  digitalWrite(REL_BPF12_pin, 0);
  digitalWrite(REL_BPF24_pin, 0);
  digitalWrite(REL_BPF40_pin, 0);
  digitalWrite(REL_ATT_20_pin, 0);
  digitalWrite(REL_ATT_10_pin, 0);        
  digitalWrite(REL_PRE_10_pin, 0);
    

  pinMode(vForwardPin, INPUT);
  pinMode(vReflectedPin, INPUT);

  Wire.begin(I2C_ADDR, I2C_MASK);       // base address for all slaves running here (valid only for ATmega328P Arduinos)
  Wire.onRequest(requestEvent);  // register callback function for I2C = master read
  Wire.onReceive(receiveEvent);  // register callback function for I2C = master write



  nextMillis = millis() + LOOP_PERIOD_MILLIS;

}


/*****************************************************************************************/
void requestEvent (){    // master read = request data from slave
  switch (Wire.getLastAddress()) {   // address from last byte on the bus
    case (I2C_SWR):
      Wire.write(swr_8bits);   // send back 8bits value
      //Wire.write((byte *)&swr, 2);   //send 2 bytes
      break;

    case (I2C_SWR_FOR):
      Wire.write(swr_for);   // send back 8bits value
      break;

    case (I2C_SWR_REF):
      Wire.write(swr_ref);   // send back 8bits value
      break;

    default:
      break;
  }
}


/*****************************************************************************************/
void receiveEvent(int howManyBytesReceived) {   // master write = send data to slave
  switch (Wire.getLastAddress()) {   // address from last byte on the bus
    case (I2C_BPF):
      BPF_Relays = Wire.read();   // receive byte
      break;

    case (I2C_RX):
      RX_Relays = Wire.read();   // receive byte
      break;

    default:
      break;
  }

}



/*****************************************************************************************/
void Set_BPF_Relays() {

  Serial.print("Set BPF Relays ");  

  if(BPF_Relays == REL_LPF2_val)    
  {
    digitalWrite(REL_LPF2_pin, 1);
    digitalWrite(REL_BPF6_pin, 0);        
    digitalWrite(REL_BPF12_pin, 0);
    digitalWrite(REL_BPF24_pin, 0);
    digitalWrite(REL_BPF40_pin, 0);
    
    Serial.print(BPF_Relays);  
    Serial.println("  REL_LPF2_val   band=0");  
  }  
  else if(BPF_Relays == REL_BPF6_val) 
  {
    digitalWrite(REL_LPF2_pin, 0);
    digitalWrite(REL_BPF6_pin, 1);        
    digitalWrite(REL_BPF12_pin, 0);
    digitalWrite(REL_BPF24_pin, 0);
    digitalWrite(REL_BPF40_pin, 0);
    
    Serial.print(BPF_Relays);  
    Serial.println("  REL_BPF6_val   band=1");  
  }  
  else if(BPF_Relays == REL_BPF12_val) 
  {
    digitalWrite(REL_LPF2_pin, 0);
    digitalWrite(REL_BPF6_pin, 0);        
    digitalWrite(REL_BPF12_pin, 1);
    digitalWrite(REL_BPF24_pin, 0);
    digitalWrite(REL_BPF40_pin, 0);
    
    Serial.print(BPF_Relays);  
    Serial.println("  REL_BPF12_val   band=3");  
  }  
  else if(BPF_Relays == REL_BPF24_val) 
  {
    digitalWrite(REL_LPF2_pin, 0);
    digitalWrite(REL_BPF6_pin, 0);        
    digitalWrite(REL_BPF12_pin, 0);
    digitalWrite(REL_BPF24_pin, 1);
    digitalWrite(REL_BPF40_pin, 0);
    
    Serial.print(BPF_Relays);  
    Serial.println("  REL_BPF24_val   band=4");  
  }  
  else //BPF40   at least 1 filter connected
  {
    digitalWrite(REL_LPF2_pin, 0);
    digitalWrite(REL_BPF6_pin, 0);        
    digitalWrite(REL_BPF12_pin, 0);
    digitalWrite(REL_BPF24_pin, 0);
    digitalWrite(REL_BPF40_pin, 1);
    
    Serial.print(BPF_Relays);  
    Serial.println("  REL_BPF40_val   band=2");  
  }  
  
//  digitalWrite(ledPin, !digitalRead(ledPin));    //toggle led
}



/*****************************************************************************************/
void Set_RX_Relays() {

  Serial.print("Set RX Relays ");  
  Serial.print(RX_Relays);    

  if((RX_Relays & REL_ATT_20_val) == REL_ATT_20_val) 
  {
    digitalWrite(REL_ATT_20_pin, 1);
    Serial.print("   REL_ATT_20_val");      
  }
  else
  {
    digitalWrite(REL_ATT_20_pin, 0);        
  }  

  if((RX_Relays & REL_ATT_10_val) == REL_ATT_10_val) 
  {
    digitalWrite(REL_ATT_10_pin, 1);        
    Serial.print("   REL_ATT_10_val");      
  }  
  else
  {
    digitalWrite(REL_ATT_10_pin, 0);        
  }   

  if((RX_Relays & REL_PRE_10_val) == REL_PRE_10_val) 
  {
    digitalWrite(REL_PRE_10_pin, 1);
    Serial.print("   REL_PRE_10_pin");      
  }       
  else
  {
    digitalWrite(REL_PRE_10_pin, 0);
  } 
  
  Serial.println(" ");      
//  digitalWrite(ledPin, !digitalRead(ledPin));    //toggle led
}




/*****************************************************************************************/
void SWR_read() 
{
  uint16_t vForward, vForward0;
  static uint16_t  vForward1;  //last AD reading
  uint16_t vReflected, vReflected0;
  static uint16_t vReflected1;  //last AD reading
  uint16_t swr, swr_unid, swr_dec;
  
  /* read the AD for SWR */
  vForward0 = analogRead(vForwardPin);      //actual value
  vForward = (vForward0 + vForward1) >> 1;   //average with last value
  vForward1 = vForward0;                //save last value
    
  vReflected0 = analogRead(vReflectedPin);  //actual value
  vReflected = (vReflected0 + vReflected1) >> 1;   //average with last value
  vReflected1 = vReflected0;                //save last value

  if(vForward < VMIN)
  {
    swr = SWR_MIN;
  }    
  else if(vForward <= vReflected)
  {
    swr = SWR_MAX;
  }    
  else
  {    
    swr = (SWR_BASE10 * (vForward + vReflected)) / (vForward - vReflected);
    if(swr > SWR_MAX)      
    {
      swr = SWR_MAX;
    }      
  }
  /* swr value will be from 1.0 to 15.9 */
  /* 1 byte -> bit7-bit4 = integer    bit3-bit0 = decimal base 10 */

  swr_unid = swr/SWR_BASE10;  //integer part  max = 15
  if(swr_unid > 15) swr_unid = 15;
  swr_dec = swr%SWR_BASE10;   //decimal part
  if(swr_dec > 9) swr_dec = 9;  //double check, just in case SWR_BASE10 changed
  swr_8bits = (swr_unid << 4) + swr_dec;
  swr_for = vForward >> 2;  //10 bits AD to 8 bits
  swr_ref = vReflected >> 2;

  if(vForward > 0)
    {
    Serial.print("SWR  For= ");  
    Serial.print(vForward);
    Serial.print("   Ref= ");  
    Serial.print(vReflected);
    Serial.print("   swr= ");  
    Serial.print(swr_8bits>>4);  //swr_unid
    Serial.print('.');
    Serial.println(swr_8bits&0x0f);   //swr_dec
    }
}




uint8_t InitCheck(void)
{
  static uint16_t cont_BPF_relay = 0;
  static uint16_t cont_ATT_relay = 0;
  static uint16_t cont_test = 0;
  uint8_t retValue;

  if(cont_test < 10) 
  {
    /*******************************************************************************************************/  
    /* initial test switching relays after reset  (just switch through all relays) */
    /*******************************************************************************************************/  
    if(cont_test < 5)  
    {    
      BPF_Relays = REL_BPF_val[cont_BPF_relay];
      //Serial.println("Set BPF Relays " + String(cont_BPF_relay) + "  " + String(BPF_Relays));
      Serial.print(cont_BPF_relay);   Serial.print("  ");
      cont_BPF_relay++;
      if(cont_BPF_relay >= REL_BPF_val_num)
        cont_BPF_relay = 0;
      Set_BPF_Relays();
    }
    else
    {    
      RX_Relays = REL_ATT_val[cont_ATT_relay];
      //Serial.println("Set RX Relays " + String(cont_ATT_relay) + "  " + String(RX_Relays));
      Serial.print(cont_ATT_relay);   Serial.print("  ");
      cont_ATT_relay++;
      if(cont_ATT_relay >= REL_ATT_val_num)
        cont_ATT_relay = 0;
      Set_RX_Relays();
    }
    cont_test++;

    delay(250);  //ms

    retValue = false;
  }
  else  
  {
    retValue = true;
  }


return retValue;

}







/*****************************************************************************************/
void loop() {
 


  if(InitCheck() == true)  /* it runs a test sequence after reset, true means ready for normal processing */
  {  
    /**************************/  
    /* loop normal processing */
    /**************************/  
    digitalWrite(ledPin, 1);


    SWR_read();  /* read the ADC for SWR calculation */


    digitalWrite(ledPin, 0);

    /* wait loop period to end - pocesses loop at specific period */
    while(millis() < nextMillis)
    {

      /* check if received new value to set the BPF relays */
      if(BPF_Relays_old != BPF_Relays)
      {
        Set_BPF_Relays();
        BPF_Relays_old = BPF_Relays;
      }

      /* check if received new value to set the RX relays */
      if(RX_Relays_old != RX_Relays)
      {
        Set_RX_Relays();
        RX_Relays_old = RX_Relays;
      }  

    }
    /* time for next loop period */  
    nextMillis += LOOP_PERIOD_MILLIS; 

  }

}





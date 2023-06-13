/*


Receiving commands as more than one I2C slave (multiple addresses)
   to replace the PCF8574 (can replace two of them, and adding read swr option) in uSDX Pico FFT project

Reference:
https://stackoverflow.com/questions/34691478/arduino-as-slave-with-multiple-i2c-addresses
https://github.com/alexisgaziello/TwoWireSimulator

I2C Wire library with multi address:
https://github.com/arduino/ArduinoCore-avr/pull/90/files#diff-e4603cea13a2a6370bdf819d929e8fb9b272c812bc1df9a9190b365875c47db3



Arduino Pro Mini 3V3 8Mhz
 
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
#define I2C_SWR 		I2C_BPF	//0x20 read (use the same address to make easy to build the multi I2C mask)
#define swrPin      A0      // select the input pin for the swr analog reading
uint16_t swr, swr0, swr1;
uint8_t swr8bits;

#define I2C_ADDR (I2C_BPF | I2C_RX)
#define I2C_MASK ((I2C_BPF | I2C_RX) ^ (I2C_BPF & I2C_RX))



#define ledPin      13     // define LED pin number


uint8_t RX_Relays=0, RX_Relays_old=0;
uint8_t BPF_Relays=0, BPF_Relays_old=0;
uint8_t I2C_Data;
uint8_t rec=0, I2C_Address;
const uint8_t REL_BPF_val[REL_BPF_val_num] = {REL_LPF2_val, REL_BPF6_val, REL_BPF12_val, REL_BPF24_val, REL_BPF40_val};
const uint8_t REL_ATT_val[REL_ATT_val_num] = {REL_PRE_10_val, REL_ATT_30_val, REL_ATT_20_val, REL_ATT_10_val, REL_ATT_00_val};


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
  }  // If the serial is not open, the print commands will have no effect
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
    

  pinMode(swrPin, INPUT);

  Wire.begin(I2C_ADDR, I2C_MASK);       // base address for all slaves running here
  Wire.onRequest(requestEvent);  // register callback function for I2C = master read
  Wire.onReceive(receiveEvent);  // register callback function for I2C = master write
}


/*****************************************************************************************/
void requestEvent (){    // master read = request data from slave
  switch (Wire.getLastAddress()) {   // address from last byte on the bus
    case (I2C_SWR):
      Wire.write(swr8bits);   // send back 8bits value
      //Wire.write((byte *)&swr, 2);   //send 2 bytes
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
  if(BPF_Relays == REL_LPF2_val)    
  {
    digitalWrite(REL_LPF2_pin, 1);
    digitalWrite(REL_BPF6_pin, 0);        
    digitalWrite(REL_BPF12_pin, 0);
    digitalWrite(REL_BPF24_pin, 0);
    digitalWrite(REL_BPF40_pin, 0);
  }  
  else if(BPF_Relays == REL_BPF6_val) 
  {
    digitalWrite(REL_LPF2_pin, 0);
    digitalWrite(REL_BPF6_pin, 1);        
    digitalWrite(REL_BPF12_pin, 0);
    digitalWrite(REL_BPF24_pin, 0);
    digitalWrite(REL_BPF40_pin, 0);
  }  
  else if(BPF_Relays == REL_BPF12_val) 
  {
    digitalWrite(REL_LPF2_pin, 0);
    digitalWrite(REL_BPF6_pin, 0);        
    digitalWrite(REL_BPF12_pin, 1);
    digitalWrite(REL_BPF24_pin, 0);
    digitalWrite(REL_BPF40_pin, 0);
  }  
  else if(BPF_Relays == REL_BPF24_val) 
  {
    digitalWrite(REL_LPF2_pin, 0);
    digitalWrite(REL_BPF6_pin, 0);        
    digitalWrite(REL_BPF12_pin, 0);
    digitalWrite(REL_BPF24_pin, 1);
    digitalWrite(REL_BPF40_pin, 0);
  }  
  else //BPF40   at least 1 filter connected
  {
    digitalWrite(REL_LPF2_pin, 0);
    digitalWrite(REL_BPF6_pin, 0);        
    digitalWrite(REL_BPF12_pin, 0);
    digitalWrite(REL_BPF24_pin, 0);
    digitalWrite(REL_BPF40_pin, 1);
  }  
  
  digitalWrite(ledPin, !digitalRead(ledPin));    //toggle led
  Serial.print("Set BPF Relays ");  
  Serial.println(BPF_Relays);  
}



/*****************************************************************************************/
void Set_RX_Relays() {
  if((RX_Relays & REL_ATT_20_val) == REL_ATT_20_val) 
  {
    digitalWrite(REL_ATT_20_pin, 1);
  }
  else
  {
    digitalWrite(REL_ATT_20_pin, 0);        
  }  

  if((RX_Relays & REL_ATT_10_val) == REL_ATT_10_val) 
  {
    digitalWrite(REL_ATT_10_pin, 1);        
  }  
  else
  {
    digitalWrite(REL_ATT_10_pin, 0);        
  }   

  if((RX_Relays & REL_PRE_10_val) == REL_PRE_10_val) 
  {
    digitalWrite(REL_PRE_10_pin, 1);
  }       
  else
  {
    digitalWrite(REL_PRE_10_pin, 0);
  } 
  
  digitalWrite(ledPin, !digitalRead(ledPin));    //toggle led
  Serial.print("Set RX Relays ");  
  Serial.println(RX_Relays);    
}





uint16_t cont_BPF_relay = 0;
uint16_t cont_ATT_relay = 0;
uint16_t cont_test = 0;

/*****************************************************************************************/
void loop() {
 

  if(cont_test < 10) 
  {
    /*******************************************************************************************************/  
    /* special case doing initial test switching relays just after reset  (just switch through all relays) */
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

    delay(500);  //ms
  }
  else  
  {  
    /**************************/  
    /* loop normal processing */
    /**************************/  

    /* check if received new value to set the relays */
    if(BPF_Relays_old != BPF_Relays)
    {
      Set_BPF_Relays();
      BPF_Relays_old = BPF_Relays;
    }
    

    /* check if received new value to set the relays */
    if(RX_Relays_old != RX_Relays)
    {
      Set_RX_Relays();
      RX_Relays_old = RX_Relays;
    }  
      


    /* read the AD for SWR */
    swr0 = analogRead(swrPin);  //actual value
    swr = (swr0 + swr1) >> 1;   //average with last value
    swr8bits = swr >> 2;            //8 bits through I2C
    swr1 = swr0;                //save last value
 


    /* loop processes at each 10ms */  
    delay(10);  //ms
  }


 
/* future tasks */
/* send  tx power, swr, Vbat ? */
/* read the direct tx power */
/* read the reflected power */
/* calculate the swr */

}





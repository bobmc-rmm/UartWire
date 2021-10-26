// -*- C++ -*-
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
/**@file  UartWire2.ino 
 * @brief 1wire bus via Uart
 * 2021-oct-21
 * 
 */
#include <Arduino.h>
#include "UartWire.h"

/// required by Arduino to initialize
void setup() {
   pinMode(ePIN_LED, OUTPUT);
   pinMode(ePIN_SCOPE, OUTPUT);
   dso_strobe(HIGH);
   Serial.begin(115200);
   while(!Serial) delay(1);
   blink_pulse(LOW);
   Serial.println("\nUartWire.ino..");
   uw_init(); // prepare the UartWire
}

/// required by Arduino for run-time task
void loop() {
   int choice = 3;
   if(choice==1) uw_test_single(); // when only 1 DS18B20 is used
   if(choice==2) uw_list_rom();  // list the next ROM in the array
   if(choice==3) uw_next_sample(); // default after setup

   do{
      // DS18B20 needs 750ms to update temperature. Meanwhile,
      // something (not 1-wire) can be done here for a total time of 800ms.
      delay(800); //
      // Serial.println("-----------");
   }while(0);
   
}

/// used as trigger for optional bitscope
void dso_strobe(int state){
   digitalWrite(ePIN_SCOPE, state);
}

void blink_pulse(int state){
   digitalWrite(ePIN_LED, state & 1);
}
// UartWire.ino


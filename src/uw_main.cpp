// -*- C++ -*-
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
/**@file  uw_main.cpp
 * @brief Application of 1wire bus via UART
 * 2021-nov-01
 * 
 */
#include <Arduino.h>
#include "uw_main.h"
#include "UartWire.h"

/// required by Arduino to initialize
void setup() {
   blink_pulse( eInit, HIGH);
   dso_strobe(HIGH);
   Serial.begin(115200);   while(!Serial) delay(1);
   Serial.println("\nUartWire.ino..");
   uw_init(); // prepare the UartWire
}

/// required by Arduino for run-time task
void loop() {
   static unsigned times;
   int choice = 3;
   if(choice==1) uw_test_single(); // when only 1 DS18B20 is used
   if(choice==2) uw_list_rom();  // list the next ROM in the array
   if(choice==3) uw_next_sample(); // default after setup

   do{
      // DS18B20 needs 750ms to update temperature. Meanwhile,
      // something (not 1-wire) can be done here for a total time of 800ms.
      delay(800); //
      if( USE_MCU != 3 ){
	 // normally not used
	 //Serial.println("times: " + String(times));  times++;
      }
   }while(0);
   
}

/// used as trigger for optional bitscope
void dso_strobe(int state){
   digitalWrite(ePIN_SCOPE, state);
}

//------------------------------------------------------------------
/// note: NodeMcu uses GPIO2 for UART1 TX
/// \brief set the MCU LED, if random change is allowed
int  blink_pulse(CHOICE_T choice, int state){
   int rv = -1;
   if( USE_MCU == 3 ){ return rv; }

   if( choice == eInit ){
      pinMode(ePIN_LED, OUTPUT);
      digitalWrite(ePIN_LED, state & 1);
      return 0;
   }
   if( choice == eSet){
      digitalWrite(ePIN_LED, state & 1);
      return 0;
   }
   // if( choice == eGet ){ rv = digitalRead(ePIN_LED);  }
   return rv;
}
// UartWire.ino


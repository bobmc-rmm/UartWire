// -*- C++ -*-
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
/**@file  UartWire.cpp
 * @brief UART as 1wire driver and master
 *
 * 2021-oct-13
 * based on Maxim note 126 and others about 1-wire
 
 */ 
#include <Arduino.h>
#include "UartWire.h"

static float uw_degrees( u8t lsb, u8t msb);
static u8t * uw_drop_addrs(void);
static void  uw_rx_flush(void);
static void  uw_set_rate(u32t rate);
static int   uw_status(int linenum, int status);

static int   uw_array( u8t *pa, int choice );
static u8t   uw_crc(u8t *pBuf, int len);
static int   uw_print_rom(void);
static int   uw_reset(void);
static int   uw_rx_bit(void);
static int   uw_rx_byte(void);
static int   uw_rx_data(u8t *pb, int numBytes);
static int   uw_tx_bit(u8t flags);
static int   uw_tx_byte(u8t dataByte);
static int   uw_tx_data(u8t *pb, int numBytes);
static int   uw_tx_echo(u8t ub);
static int   uw_select(u8t *pb);

#define UART  Serial1
static u32t baudrate;
static int W1_ident = 0;
static int W1_node = 0;
static int W1_mode = W1_MULTI_DROP;
static u8t ubuff[ARRAY_PAD_SZ+2]; // temporary copy of ROM or scratchpad

//------------------------------------------------------------------
/// init the aux serial port for 1-wire service
void uw_init( void ){
   uw_set_rate(0);
   uw_set_rate(9600);

#if USE_ESP32
   UART.begin(9600, SERIAL_8N1,ePIN_RX1,ePIN_TX1);  // esp32
#endif
   
#if USE_MEGA2560
   UART.begin(9600, SERIAL_8N1);
#endif
   
   W1_mode = W1_MULTI_DROP;
}

//------------------------------------------------------------------
/// \brief reset 1-wire bus, detect sensor presence
/// \returns 1 if a device asserted a presence pulse, 0 otherwise.
static int uw_reset(void){
   int status;
   blink_pulse(1);
   uw_set_rate(9600);
   status = uw_tx_echo(0xF0);
   blink_pulse(0);
   uw_status(1,status);
   if (status < 1) return status;
   uw_set_rate((u32t)115200);
   u8t bs = status & 0xFF;
   if( bs == 0xF0 ) return -1; // no slave devices
   return ( bs == 0xE0 );
}

//------------------------------------------------------------------
/// \brief list ROMs in circular order 1,2,3,1,2....
int  uw_list_rom(void){
   u8t *pDrop = 0;
   W1_mode = W1_MULTI_DROP;
   pDrop = uw_drop_addrs();  // next ROM
   if( pDrop ){
      uw_array( pDrop, ARRAY_ROM );
   }
   return 0;
}

//------------------------------------------------------------------
/// \brief take the next temperature reading from next node
int uw_next_sample(void){
   u8t *pDrop = 0;
   int status = -1;
   W1_mode = W1_MULTI_DROP;
   if(status = uw_reset(), status < 0 ) return status;
   pDrop = uw_drop_addrs();
   uw_select(pDrop);
   uw_tx_byte(READ_PAD);
   uw_rx_data(&ubuff[0],ARRAY_PAD_SZ);
   uw_array( &ubuff[0], ARRAY_PAD );
   //
   uw_reset();
   uw_select(pDrop);
   uw_tx_byte(CONVERT_T);
   return status;
}

//------------------------------------------------------------------
/// flush the UART rx input
static void uw_rx_flush(void){
   int status;
   do{ delay(1); status = UART.read(); }while(status >= 0);
}

//------------------------------------------------------------------
/// Since TX->RX loopback, every byte sent must be consumed. Only
/// some of them are interesting because the 1wire response modifies
/// the sent byte.
/// \brief return the echo from tx-rx loopback
static int uw_tx_echo(u8t ub){
   UART.write(ub);
   delay(2);
   int status = UART.read();
   return status;
}

//------------------------------------------------------------------
/// \brief change the baud rate if the setting is new 
static void uw_set_rate(u32t rate){
   if( baudrate != rate ){
      baudrate = rate;
#if USE_ESP32
      UART.updateBaudRate( rate );  // esp32
#endif

#if USE_MEGA2560
       UART.begin(rate, SERIAL_8N1);  // meg2560
#endif
   }
   uw_rx_flush();  // not sure if this needed
}

//------------------------------------------------------------------
/// write 1 .. uses UART 0xFF start bit only (1x8.68)
/// write 0 .. uses (7x8.68)=60.76us low pulse (0xC0)
/// \brief Write a 1-wire pulse (8.68 or (8.68x7)) for 115200 baud
/// \return response byte; else -1 = error
static int uw_tx_bit(u8t flags){
   int status;
   u8t mask = (flags !=0 ) ? 0xFF : 0xC0;
   status = uw_tx_echo(mask);
   uw_status(2,status);
   return status;
}

//------------------------------------------------------------------
/// Starting with bit 0
/// \brief issue one data byte, bit-by-bit, on the 1-wire
static int uw_tx_byte(u8t dataByte){
   int status=0;

   for(int i=0; i<8; i++)   {
      u8t ub = dataByte & 1;
      status = uw_tx_bit(ub);
      if( status < 0 ) break;
      dataByte >>= 1;
   }
   return status;
}

//------------------------------------------------------------------
/// \brief issue a set of data bytes on the 1-wire
static int uw_tx_data(u8t *pb, int numBytes){
   int status=-1;
   if( numBytes < 1 ) return status;
   for(int i=0; i < numBytes; i++)   {
      status = uw_tx_byte(pb[i]);
      if( status < 0 ) break;
   }
   return status;
}

//------------------------------------------------------------------
/// \brief request a bit from the 1wire bus
static int uw_rx_bit(void){
   int status;
   status = uw_tx_echo(0xFF);
   if(status < 0) return status;
   if(0){ // can print (ff) or (fc) for bits 1 and 0
      Serial.print("(" + String(status,HEX) + ")" );
   }
   return status;
}

//------------------------------------------------------------------
/// \brief request a byte from the 1wire bus
static int uw_rx_byte(void) {
   u8t mask=1, ub=0;
   int status=0;
   for( int i=0; i < 8; i++ ){
      status = uw_rx_bit();
      if( status < 0 ) return status;
      if( (status & 1) ){
	 ub |= mask;
      }
      mask <<= 1;
   }
   return ub & 0xff;
}

//------------------------------------------------------------------
/// \brief read a set of data bytes from the 1-wire scratchpad or ROM
static int uw_rx_data(u8t *pb, int numBytes){
   u16t status=0;
   for(int i=0; i < numBytes; i++)   {
      status = uw_rx_byte();
      if( status < 0 ) break;
      pb[i] = status;
   }
   return status;
}

//------------------------------------------------------------------
/// cyclic redundancy check
static u8t uw_crc(u8t *pBuf, int len){
   u8t shiftedBit, crc = 0x00;
   for(int i=0; i<len; i++) {
      crc = (crc ^ pBuf[i]);
      for(int j=8; j>0; j--) {
	 shiftedBit= (crc & 0x01);
	 crc >>= 1;
	 if(shiftedBit){
	    crc = (crc ^ 0x8c);
	 }
      }
   }
   return crc;
}

//------------------------------------------------------------------
/// \brief select a Node/Drop by ROM addrs[8] (64-bit)
/// \param pDrop ..ROM addrs of the candidate mode or drop
static	int uw_select(u8t *pDrop){
   int status = -1;
   W1_ident = pDrop[0];
   if(W1_mode == W1_MULTI_DROP){
      uw_tx_byte(MATCH_ROM);
      status = uw_tx_data(pDrop,ARRAY_ROM_SZ);
      return status;
   }
   // if W1_SINGLE_DROP, no match is needed
   status = uw_tx_byte(SKIP_ROM); // there can be only 1 on bus
   return status;
}

//------------------------------------------------------------------
/// \brief show recent function result
static int uw_status(int linenum, int status){
   if(0){ // prints a line flag and the status from that line
      String sb = String(linenum,DEC) + " ;" + String(status,HEX);
      Serial.println(sb);
   }
   return status;
}

//------------------------------------------------------------------
/// See Maxim table1 which has 8 counts per 0.5 degrees
/// \brief convert DS18B20 scratchpad bytes to C-degrees
static float uw_degrees( u8t lsb, u8t msb){
   float fv = 0;
   if( W1_ident == W1_DS18B20 ){
      i16t iv = (msb<<8 | lsb);
      fv = iv/8.0 * 0.5;
   }
   return fv;
}

//------------------------------------------------------------------
/// shows 8-byte addrs of a single-drop bus
static int uw_print_rom(void){
   int status = -1;
   if( W1_mode != W1_SINGLE_DROP ) return status;
   if( status = uw_reset(), status < 0 ) return status;
   if( status = uw_tx_byte(READ_ROM), status < 0) return status;
   uw_status(4,status);
   status = uw_rx_data(&ubuff[0],ARRAY_ROM_SZ);
   if( status < 0 ) return status;
   status = uw_array( &ubuff[0], ARRAY_ROM );
   W1_ident = ubuff[0];
   return status;
}

//------------------------------------------------------------------
/// First run ref uw_test(W1_SINGLE_DROP,0) which prints a node
/// address. Then replace the nodes in the drop lines below. These
/// will be used only for W1_MULTI_DROP.
/// 
/// The 1-wire protocol has search features that require complicated
/// coding.  These are not implemented here. Instead there is a manual
/// procedure for mapping the bus where each sensor is identified and
/// assigned to a specific node or drop from the wire.
///
/// \brief get next MATCH_ROM addrs of 1wire node/drop
/// @return addrs of next ROM
static u8t * uw_drop_addrs(void){
   static int idx; // index of next 8 byte array
   const int NumDrops = 3; // update to match addrs lines
   static u8t data[NumDrops][8] = {
      {0x28,0x64,0x26,0x62,0x0D,0x00,0x00,0xE2}, // sensor at drop1
      {0x28,0x72,0x32,0xE9,0x0D,0x00,0x00,0x25}, // sensor at drop2
      {0x28,0x2A,0x5D,0x62,0x0D,0x00,0x00,0x99}	 // sensor at drop3
   };
   idx++;
   if( idx < 0 || idx >= NumDrops ) idx = 0;
   W1_node = idx+1;  // node/drop number 1...
   return &data[idx][0];
}

//------------------------------------------------------------------
/// Run \ref uw_test_single() when only 1 node is connected.  Reading
/// the Pad is done before the conversion is started.  The very first
/// reading is invalid. After that, the latest temperature is
/// available without waiting between the read and the
/// conversion. This differs from the Maxim DS18B20 flowchart which is
/// informative but idles the system during temperature conversion.
///
/// \brief prints ROM addrs (times % 5), then single DS18B20 temperatures
int uw_test_single(void ){
   static unsigned steps;
   int status = -1;
   W1_mode = W1_SINGLE_DROP;
   if( (++steps % 5) == 0  ){
      // list ROM 8-byte addrs for setting in uw_drop_addrs()
      status = uw_print_rom(); // ROM at single drop
   }
   if(status = uw_reset(), status < 0 ) return status;
   uw_tx_byte(SKIP_ROM); // there can be only 1 on bus
   uw_tx_byte(READ_PAD);
   uw_rx_data(&ubuff[0],ARRAY_PAD_SZ);
   uw_array( &ubuff[0], ARRAY_PAD );
   // start another conversion, then return to the application loop
   // where other work can be done for the 800 ms
   uw_reset();
   status = uw_tx_byte(SKIP_ROM); // there can be only 1 on bus
   uw_tx_byte(CONVERT_T);
   return status;
}

//------------------------------------------------------------------
// print rom/scratchpad, CRC, C-degrees
static int uw_array( u8t *pa, int choice ){
   //char cbuff[64];
   //char tmp[16];
   u8t ub, H, L;
   String sb = "[";
   u8t len = (choice==ARRAY_PAD) ? ARRAY_PAD_SZ : ARRAY_ROM_SZ;
   
   for( int i=0; i<len; i++ ){
      H = pa[i]>>4 & 0xf, L = pa[i] & 0xf;
      sb += String(H,HEX) + String(L,HEX) +":";
   }
   sb += "] ";
   ub = uw_crc(pa, len-1);

   H = ub>>4 & 0xf, L = ub & 0xf;
   sb += String(H,HEX) + String(L,HEX) +" ";
   if( pa[len-1] != ub ){
      sb += " CRC error ";
   }
   if(choice==ARRAY_PAD){
      float fv = uw_degrees( pa[0], pa[1] );
      sb += " " + String(fv,2) + " ";
   }
   sb += "N=" + String(W1_node,DEC);
   Serial.println(sb);
   return 0;
}

// end of file UartWire.cpp

// -*- C++ -*-
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
/**@file  UartWire.h 
 * @brief UartWire symbols
 * 
 */ 
#if !defined UartWire_h
#    define UartWire_h


// list of 1wire devices types tested
enum {
   W1_SINGLE_DROP = 1,		// only a single unit on bus
   W1_MULTI_DROP  = 2,
   W1_DS18B20     = 0x28,	// seen in first byte of ROM
   W1_BUTTON      = 'x' // NA
};

enum {
   ALARM_SEARCH = 0xEC,
   CONVERT_T    = 0x44,
   COPY_PAD     = 0x48,
   MATCH_ROM    = 0x55,		// select a 1wire node
   READ_PAD     = 0xBE,
   READ_POWER   = 0xB4,
   READ_ROM     = 0x33,
   RECALL       = 0xB8,
   SEARCH_ROM   = 0xF0,
   SKIP_ROM     = 0xCC,
   WRITE_PAD    = 0x4E
};

enum  {
   ARRAY_LSB = 0,
   ARRAY_MSB = 1,
   ARRAY_TH  = 2,		// alarm
   ARRAY_TL  = 3,
   ARRAY_CFG = 4,		// configuration
   ARRAY_xFF = 5,		// reserve
   ARRAY_RSV = 6,		// reserve
   ARRAY_x10 = 7,		// reserve
   ARRAY_CRC = 8,		// cyclic redundancy check
   ARRAY_PAD_SZ  = 9,		// scratchpad bytes
   ARRAY_ROM_SZ = 8,		// ROM bytes
   ARRAY_ROM = 1,		// print choice
   ARRAY_PAD = 2 		// print choice
};

void  dso_strobe(int state);
int   blink_pulse(CHOICE_T choice, int state);

void  uw_init( void );
// int   uw_test( u8t t_mode, int testnum);
int   uw_test_single(void );
int   uw_next_sample(void);
int   uw_list_rom(void);

#endif // UartWire_h

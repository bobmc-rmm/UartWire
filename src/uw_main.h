// -*- C++ -*-
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
/**@file  uw_main.h 
 * @brief UartWire application header
 * 
 */ 
#if !defined uw_main_h
#    define uw_main_h

typedef uint8_t    u8t;
typedef int16_t    i16t;
typedef uint16_t   u16t;
typedef uint32_t   u32t;
typedef int32_t    i32t;

typedef enum{ eInit, eSet, eGet }CHOICE_T;

#if USE_MCU==1   // GPIO feather32
enum{
   ePIN_LED       = 13,
   ePIN_SCOPE     = 21,		// NA
   ePIN_RX1       = 16,
   ePIN_TX1       = 17
};
#endif

#if USE_MCU==2   // GPIO mega2560
enum{
   ePIN_LED       = 13,
   ePIN_SCOPE     = 21,		// NA
   ePIN_RX1       = 16,		// NA
   ePIN_TX1       = 17		// NA
};
#endif

#if USE_MCU==3   // GPIO NodeMcu 8266 12-E
enum{
   ePIN_LED       = 2,
   ePIN_SCOPE     = 21,		// NA
   ePIN_RX1       = 16,		// NA
   ePIN_TX1       = 17		// NA
};
#endif

#endif //  uw_main_h

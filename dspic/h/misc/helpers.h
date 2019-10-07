

#include "stdio.h"
#include "device/dev_lcd.h"
#include "device/dev_uart1.h"

#define TEMPSTR_LCD_SIZE        40
#define TEMPSTR_SERIAL_SIZE    140

//void PrintLcd(uint8_t linenumber, ...);

//void PrintSerialInit();

//void PrintSerial( char *format, ...);

//#define __print_lcd_size    32
// PrintLcd helps to make printing on the Lcd easier
#define PrintLcd(LINE, ...)       do{char __print_utils_string[TEMPSTR_LCD_SIZE]; sprintf(__print_utils_string, __VA_ARGS__); Dev_Lcd_WriteStringXY(0, LINE, __print_utils_string); } while(0)

#define PrintSerialInit()  do{  UartSendText("\n\r"); } while(0)
#define PrintSerial(...)   do{char __print_utils_string[TEMPSTR_SERIAL_SIZE]; sprintf(__print_utils_string, __VA_ARGS__); Dev_UART1_WriteStringBlocking(__print_utils_string); } while(0)


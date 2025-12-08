#include <Arduino.h>
// UWB config 
#define TAG 
//#define ANCHOR 
#define UWB_INDEX 6

// #define FREQ_850K
#define FREQ_6800K
#define UWB_TAG_COUNT 2
#define ANT_Delay 16385

#define RESET 16
#define IO_RXD2 18
#define IO_TXD2 17
#define I2C_SDA 39
#define I2C_SCL 38

#define SERIAL_LOG Serial
#define SERIAL_AT mySerial2

//======================== Wi-Fi and Server Config ============================
const char* ssid = "Galaxy S23 Ultra AD07";
const char* password = "mdup0749";

const char* HOST = "10.11.69.192";
const uint16_t PORT = 3334;
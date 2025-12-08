#include "config.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>       // <--- เพิ่มส่วนนี้ (สำหรับรับ OSC)
// #include <OSCMessage.h>    // <--- เพิ่มส่วนนี้
#include <OSCBundle.h>     // <--- เพิ่มส่วนนี้
#include <FastLED.h>       // <--- เพิ่มส่วนนี้ (สำหรับคุมไฟ)

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ====== ตั้งค่า LED STRIP ตรงนี้ ======
#define LED_PIN     5      // <--- แก้เป็นขาที่ต่อกับ LED Strip
#define NUM_LEDS    53     // <--- แก้เป็นจำนวนเม็ดไฟที่มี
#define LED_TYPE    WS2813 // <--- แก้รุ่นไฟ (WS2812, WS2813, WS2811)
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
// ===================================
// ตัวแปรเก็บสีปัจจุบัน
int valR = 0;
int valG = 0;
int valB = 0;
// ====== ตั้งค่า OSC ตรงนี้ ======
WiFiUDP Udp;
const unsigned int localPort = 9000; // Port ที่จะรับจาก TouchDesigner
// ===================================

HardwareSerial SERIAL_AT(2);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
WiFiClient client;

String wifiStatus = "WiFi: connecting...";
String ipStr = "-";
String response = "";

// ================= Wi-Fi =================
bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  wifiStatus = "WiFi: connecting...";

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(150);
  }  
  if (WiFi.status() == WL_CONNECTED) {
    wifiStatus = "WiFi: connected";
    ipStr = WiFi.localIP().toString();
    
    // เริ่มฟัง UDP หลังจากต่อ WiFi ติด
    Udp.begin(localPort); // <--- เพิ่มส่วนนี้
    SERIAL_LOG.printf("UDP Listening on port: %d\n", localPort);

    return true;
  } else {
    wifiStatus = "WiFi: FAILED";
    ipStr = "-";
    return false;
  }
}

bool connectServer() {
  if (client.connected()) return true;
  SERIAL_LOG.printf("Connecting to server %s:%u...\n", HOST, PORT);
  bool ok = client.connect(HOST, PORT);
  if (ok) {
    client.setNoDelay(true); // ลด Nagle latency
    SERIAL_LOG.println("[TCP] Connected");
  }
  else{
    SERIAL_LOG.println("[TCP] Connected Failed");
  }
  
  return ok;
}

void updateColor() {
  fill_solid(leds, NUM_LEDS, CRGB(valR, valG, valB));
  FastLED.show();
  
  // (Optional) แสดงค่าสีบนจอ OLED เพื่อ Debug
  /*
  display.fillRect(0, 30, 128, 10, SSD1306_BLACK);
  display.setCursor(0, 30);
  display.printf("R%d G%d B%d", valR, valG, valB);
  display.display();
  */
}

// ฟังก์ชันรับค่าแต่ละสี
void funcR(OSCMessage &msg) {
  if (msg.isFloat(0)) valR = (int)msg.getFloat(0);
  else if (msg.isInt(0)) valR = msg.getInt(0);
  updateColor();
}

void funcG(OSCMessage &msg) {
  if (msg.isFloat(0)) valG = (int)msg.getFloat(0);
  else if (msg.isInt(0)) valG = msg.getInt(0);
  updateColor();
}

void funcB(OSCMessage &msg) {
  if (msg.isFloat(0)) valB = (int)msg.getFloat(0);
  else if (msg.isInt(0)) valB = msg.getInt(0);
  updateColor();
}

// ====== ส่งระยะเป็น NDJSON: {"tag_id":..., "ranges_cm":[r0,r1,r2,r3]} \n ======
void sendRanges(int tag_id, float r0_cm, float r1_cm, float r2_cm, float r3_cm) {
  if (!client.connected()) return;
  char line[112];
  int n = snprintf(
      line, sizeof(line),
      "{\"tag_id\":%d,\"ranges_cm\":[%.2f,%.2f,%.2f,%.2f]}\n",
      tag_id, r0_cm, r1_cm, r2_cm, r3_cm);
  client.write((const uint8_t*)line, n);
}

void setup() {
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);

  SERIAL_LOG.begin(115200);

  // Setup LED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(100);
  fill_solid(leds, NUM_LEDS, CRGB::Red); // ติดสีแดงก่อนเพื่อเช็คว่าไฟดี
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB::Black); // ดับไฟ
  FastLED.show();

  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    connectServer();
  }
  
  xTaskCreatePinnedToCore(Task_UWB,
                          "Task_UWB",
                          1024 * 20,
                          NULL,
                          tskIDLE_PRIORITY,
                          NULL,
                          0);
}

void loop() {
  // 1. จัดการ Reconnect WiFi/TCP (ของเดิม)
  static uint32_t lastRetry = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastRetry > 3000) {
      lastRetry = millis();
      connectWiFi();
    }
  } else if (!client.connected()) {
    if (millis() - lastRetry > 3000) {
      lastRetry = millis();
      connectServer();
    }
  }

  // 2. รับค่า OSC (เพิ่มส่วนนี้)
  OSCBundle bundle;
  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      bundle.fill(Udp.read());
    }
    
    if (!bundle.hasError()) {
      // แกะซองแล้วแยกจดหมาย
      bundle.dispatch("/r", funcR); 
      bundle.dispatch("/g", funcG); 
      bundle.dispatch("/b", funcB); 
    } 
  }
  delay(2);
}

void drawScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("UWB TAG -> TCP"));
  display.setCursor(0, 10);

  String freqText = "Freq: ";
#ifdef FREQ_850K
  freqText += "850k";
#else
  freqText += "6.8M";
#endif
  display.println(freqText);

  display.setTextSize(1);
  display.setCursor(0, 22);
  display.println("Ranges (cm):");
  display.display();
  delay(500);
}

String sendData(String command, const int timeout, boolean debug) {
  String response = "";

  SERIAL_LOG.println(command);
  SERIAL_AT.println(command);

  long int time = millis();
  while ((time + timeout) > millis()) {
    while (SERIAL_AT.available()) {
      char c = SERIAL_AT.read();
      response += c;
    }
  }
  return response;
}

// ====== ตั้งค่าให้เป็น TAG เสมอ (role = 0) ======
String config_cmd() {
  String temp = "AT+SETCFG=";
  temp += UWB_INDEX;   // device id
  temp += ",0";        // role: 0 = TAG
#ifdef FREQ_850K
  temp += ",0";        // 850k
#endif
#ifdef FREQ_6800K
  temp += ",1";        // 6.8M
#endif
  temp += ",1";        // range filter ON
  return temp;
}

String cap_cmd() {
  String temp = "AT+SETCAP=";
  temp += UWB_TAG_COUNT;
#ifdef FREQ_850K
  temp += ",15";
#endif
#ifdef FREQ_6800K
  temp += ",10";
#endif
  return temp;
}

void Task_UWB(void *pvParameters) {
  SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);

  SERIAL_AT.println("AT");
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(1000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    SERIAL_LOG.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  drawScreen();

  sendData("AT?", 600, 1);
  // sendData("AT+RESTORE", 5000, 1);

  sendData(config_cmd(), 600 , 1);
  sendData(cap_cmd(), 600 , 1);

  sendData("AT+SETRPT=1", 600, 1);
  //sendData(Pan_cmd(), 2000, 1);

  sendData(String("AT+SETANT=") + ANT_Delay, 600, 1);  // <<== ใส่ตรงนี้

  sendData("AT+SAVE", 600 , 1);
  sendData("AT+RESTART", 600 , 1);  

  while (1) {
    tag_action();   // ทำงานแบบ TAG เท่านั้น
  }
}

// ====== อ่านบรรทัดจากโมดูล UWB แล้วส่ง ranges_cm ไป PC ======
void tag_action() {
  while (SERIAL_LOG.available() > 0) {
    SERIAL_AT.write(SERIAL_LOG.read());
    yield();
  }
  while (SERIAL_AT.available() > 0) {
    char c = SERIAL_AT.read();

    if (c == '\r') continue;

    if (c == '\n') {
      SERIAL_LOG.println(response);

      float rangeValues_cm[4];
      if (parseData(response, rangeValues_cm)) {
        uint32_t t_us = micros();

        // ส่งไปทาง TCP → Python (หน่วย cm)
        sendRanges(UWB_INDEX, rangeValues_cm[0], rangeValues_cm[1], rangeValues_cm[2], rangeValues_cm[3]);

        // อัปเดตหน้าจอ
        char buf[64];
        snprintf(buf, sizeof(buf), "A0=%.2f  A1=%.2f  A2=%.2f  A3=%.2f", rangeValues_cm[0], rangeValues_cm[1], rangeValues_cm[2], rangeValues_cm[3]);
        display.fillRect(0, 32, 128, 32, SSD1306_BLACK);
        display.setCursor(0, 32);
        display.print(buf);
        display.display();
      }
      response = "";
    } else {
      response += c;
    }
  }
}

// ====== parse ค่า range:() เป็น float (หน่วย cm) ======
bool parseData(String dataString, float rangeValues_cm[4]) {
  int rangeStart = dataString.indexOf("range:(");
  if (rangeStart == -1) return false;

  int rangeEnd = dataString.indexOf(")", rangeStart);
  if (rangeEnd == -1) return false;

  String rangePart = dataString.substring(rangeStart + 7, rangeEnd);
  rangePart.trim();

  char charArray[64];
  rangePart.toCharArray(charArray, sizeof(charArray));

  char *saveptr = nullptr;
  char *token = strtok_r(charArray, ",", &saveptr);
  int valueIndex = 0;
  while (token != NULL && valueIndex < 4) {
    rangeValues_cm[valueIndex] = atof(token); // หน่วย cm
    valueIndex++;
    token = strtok_r(NULL, ",", &saveptr);
  }
  return (valueIndex == 4);
}
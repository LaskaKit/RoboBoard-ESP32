#include <Adafruit_NeoPixel.h>
#include <ESP32AnalogRead.h> 
#include "ADS1X15.h"  //Rob Tillaart ADS1X15 Library
#include "PCF8575.h"  //Rob Tillaart PCF8575 Library
#include <Wire.h>
#include "WiFi.h"
#include <esp_now.h>
#include <esp_task_wdt.h>

// #define CERVENE
#define MODRE

#define WDT_TIMEOUT 5

ADS1115 ADS(0x49);
PCF8575 PCF(0x20);

#define BRIGHTNESS 10
#define LED_PIN 4
#define NUM_LED 3

ESP32AnalogRead adc;
#define ADCpin 0
#define DeviderRatio 1.7693877551

#define OFF_PIN 5

#ifdef CERVENE
  uint8_t slaveAddress[] = {0x08, 0xD1, 0xF9, 0xC4, 0x3B, 0xA4};  //cervene auto 08:D1:F9:C4:3B:A4
  const char * ssid = "rcar";
#else
  uint8_t slaveAddress[] = {0xA8, 0x42, 0xE3, 0x80, 0xE2, 0xC4};  //modre auto  A8:42:E3:80:E2:C5 A8:42:E3:80:E2:C4
  const char * ssid = "bcar";
#endif
const char * wifipw = "er4xyfsrfAAA5111";  //zxc123sen85

typedef struct struct_message {
  long batt;
  long lx;
  long ly;
  long rx;
  long ry;
  long b;
  bool off;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

uint16_t buttons = 0;
int i = 0, ii = 0;
bool ss = false, poweroff = false;
int last = millis();
float batt_min = 3.0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  if (status == ESP_NOW_SEND_SUCCESS){
    ss = true;
  }else{
    ss = false;
  }
}

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LED, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pinMode(OFF_PIN, INPUT);

  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  adc.attach(ADCpin);
  Wire.begin(8, 10);
  ADS.begin();
  PCF.begin();

  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // R, G, B
  pixels.show();

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
         
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  return;


  int ii = 0;
  WiFi.begin(ssid, wifipw);
  Serial.println("Connecting Wifi");
  while ((WiFi.status() != WL_CONNECTED) && (ii < 50)) {
    Serial.print(".");
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // R, G, B
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // R, G, B
    pixels.show();
    delay(100);
    ii++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED){
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // R, G, B
    pixels.show();
  }else{
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // R, G, B
    pixels.show();
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
         
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
}

// the loop function runs over and over again forever
void loop() {
  // esp_task_wdt_reset();

  if (!digitalRead(OFF_PIN)){
    ii++;
  }else{
    ii = 0;
  }

  if (ii > 20){
    poweroff = true;
    ii = 0;
  }else{
    poweroff = false;
  }

  Serial.print(!digitalRead(OFF_PIN));
  Serial.print("  ");
  Serial.print(ii);
  Serial.print("  ");
  Serial.println(poweroff);
  
  ADS.setGain(0);

  int16_t val_0 = 0, val_1 = 0, val_2 = 0, val_3 = 0;

  //myMap(myData.ly, 17490, 7, 0, 20000);
  
  val_0 = ADS.readADC(0);
  val_1 = ADS.readADC(1);
  val_2 = ADS.readADC(2);
  val_3 = ADS.readADC(3);

  Serial.print("LY: "); Serial.print(val_0);
  Serial.print(" LX: "); Serial.print(val_1);
  Serial.print(" RY: "); Serial.print(val_2);
  Serial.print(" RX: "); Serial.println(val_3);

  buttons = ~(PCF.read16());

  Serial.print("READ: ");
  Serial.print(buttons, BIN);
  Serial.println();

  float batt_voltage = adc.readVoltage() * DeviderRatio;
  Serial.print("Battery Voltage = " );
  Serial.print(batt_voltage);
  Serial.println("V");

  if (batt_voltage <= batt_min){
    poweroff = true;
  }

  myData.batt = batt_voltage;
  myData.ry = val_2;
  myData.rx = val_3;
  myData.ly = val_0;
  myData.lx = val_1;
  myData.b = buttons;
  myData.off = poweroff;

  esp_err_t result = esp_now_send(slaveAddress, (uint8_t *) &myData, sizeof(myData));
  return;
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }else{
    Serial.println("Error sending the data");
  }

  if (WiFi.status() == WL_CONNECTED){
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // R, G, B
    pixels.show();
  }else{
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // R, G, B
    pixels.show();
  }

  if (ss){
    pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // R, G, B
    pixels.show();
  }else{
    pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // R, G, B
    pixels.show();
  }

  if ((batt_voltage <= batt_min) || poweroff){
    pinMode(OFF_PIN, OUTPUT);
    digitalWrite(OFF_PIN, LOW);
  }

  vTaskDelay(1);
  delay(10);
}

long myMap(long x, long in_min, long in_max, long out_min, long out_max)
{
  long in_size = in_max - in_min;
  long out_size = out_max - out_min;
  if( abs(in_size) > abs(out_size) )
  {
    if( in_size < 0 ) in_size--; else in_size++;
    if( out_size < 0 ) out_size--; else out_size++;
  }
  return (x - in_min) * (out_size) / (in_size) + out_min;
}

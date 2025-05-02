#include <Adafruit_NeoPixel.h>
#include <ESP32AnalogRead.h> 
#include "ADS1X15.h"  //Rob Tillaart ADS1X15 Library
#include "PCF8575.h"  //Rob Tillaart PCF8575 Library
#include <Wire.h>
#include "WiFi.h"
#include <esp_now.h>

#include "laskagamepad.hpp"

// #define CERVENE
#define MODRE

#define WDT_TIMEOUT 5


// adc sticks, L2, R2
ADS1115 ADS(0x49);

// buttons, io expander
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

LaskaGamepad laska_gamepad;

void setup() {
  pinMode(OFF_PIN, INPUT);

  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  adc.attach(ADCpin);
  Wire.begin(8, 10);
  ADS.begin();
  // voltage on gamepad stick is 0-3V3
  // so the raw value is between 0 and 26400 theoretically
  // there are some deadzones
  ADS.setGain(1);  // +- 4.096 V
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
}

int16_t handle_deadzones(int16_t value) {
  // deadzone ends 100
  // deadzone center 750
  if (abs(value) < 750) {
    return 0;
  }
  if (value > (1 << 15) - 1 - 100) {
    return (1 << 15) - 1;
  }
  if (value < -(1 << 15) + 100) {
    return -(1 << 15);
  }
  return value;
}

int16_t convert_to_16_bit(int32_t raw_value) {
  return raw_value * 2048 / 825 - (1 << 16) / 2;
}

int16_t read_gamepad_stick_adc(uint8_t n) {
  return handle_deadzones(convert_to_16_bit(ADS.readADC(n)));
}

// the loop function runs over and over again forever
void loop() {
  // if (!digitalRead(OFF_PIN)){
  //   ii++;
  // }else{
  //   ii = 0;
  // }

  // if (ii > 20){
  //   poweroff = true;
  //   ii = 0;
  // }else{
  //   poweroff = false;
  // }

  // Serial.print(!digitalRead(OFF_PIN));
  // Serial.print("  ");
  // Serial.print(ii);
  // Serial.print("  ");
  // Serial.println(poweroff);
  
  // ADS.setGain(0);

  int16_t val_0 = 0, val_1 = 0, val_2 = 0, val_3 = 0;

  //myMap(myData.ly, 17490, 7, 0, 20000);

  // read sticks
  laska_gamepad.left_stick_y = read_gamepad_stick_adc(0);
  laska_gamepad.left_stick_x = read_gamepad_stick_adc(1);
  laska_gamepad.right_stick_y = read_gamepad_stick_adc(2);
  laska_gamepad.right_stick_x = read_gamepad_stick_adc(3);

  // Serial.print("LY: "); Serial.print(laska_gamepad.left_stick_y);
  // Serial.print(" LX: "); Serial.print(laska_gamepad.left_stick_x);
  // Serial.print(" RY: "); Serial.print(laska_gamepad.right_stick_y);
  // Serial.print(" RX: "); Serial.println(laska_gamepad.right_stick_x);

  // read buttons
  laska_gamepad.buttons = ~(PCF.read16());

  // read battery
  float batt_voltage = adc.readVoltage() * DeviderRatio;
  Serial.print("Battery Voltage = " );
  Serial.print(batt_voltage);
  Serial.println("V");

  // if (batt_voltage <= batt_min){
  //   poweroff = true;
  // }


  // esp_err_t result = esp_now_send(slaveAddress, (uint8_t *) &myData, sizeof(myData));
  return;
   
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }else{
  //   Serial.println("Error sending the data");
  // }

  // if (WiFi.status() == WL_CONNECTED){
  //   pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // R, G, B
  //   pixels.show();
  // }else{
  //   pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // R, G, B
  //   pixels.show();
  // }

  // if (ss){
  //   pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // R, G, B
  //   pixels.show();
  // }else{
  //   pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // R, G, B
  //   pixels.show();
  // }

  // if ((batt_voltage <= batt_min) || poweroff){
  //   pinMode(OFF_PIN, OUTPUT);
  //   digitalWrite(OFF_PIN, LOW);
  // }
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

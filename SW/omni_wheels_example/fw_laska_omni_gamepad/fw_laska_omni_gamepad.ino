#include <Adafruit_NeoPixel.h>
#include <ESP32AnalogRead.h> 
#include "ADS1X15.h"  //Rob Tillaart ADS1X15 Library
#include "PCF8575.h"  //Rob Tillaart PCF8575 Library
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

#include "laskagamepad.hpp"


// adc sticks, L2, R2
ADS1115 ADS(0x49);

// buttons, io expander
PCF8575 PCF(0x20);

// neopixel led
#define BRIGHTNESS 10
#define LED_PIN 4
#define NUM_LED 3
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LED, LED_PIN, NEO_GRB + NEO_KHZ800);


// battery
ESP32AnalogRead adc;
#define ADCpin 0
#define DividerRatio 1.7693877551


// hw button?
#define OFF_PIN 5

uint8_t cars[3][6] = {
  // 08:D1:F9:C4:3B:A4
  {0xA8, 0x42, 0xE3, 0x80, 0xE2, 0xC4},  // blue car
  {0x08, 0xD1, 0xF9, 0xC4, 0x3B, 0xA4},  // red car
  {0x08, 0xD1, 0xF9, 0xC4, 0x60, 0xE4}   // my esp for testing
  // {0xA8, 0x42, 0xE3, 0x80, 0xE2, 0xC4}  // blue car A8:42:E3:80:E2:C4
};

uint8_t active_car = 0;

// napad: prepinat slave adresy `select` tlacitkem
// uint8_t slaveAddress[] = {0x08, 0xD1, 0xF9, 0xC4, 0x3B, 0xA4};  //cervene auto 08:D1:F9:C4:3B:A4
// uint8_t slaveAddress[] = {0xA8, 0x42, 0xE3, 0x80, 0xE2, 0xC4};  //modre auto  A8:42:E3:80:E2:C5 A8:42:E3:80:E2:C4
// uint8_t slaveAddress[] = {0x08, 0xD1, 0xF9, 0xC4, 0x60, 0xE4};  // backup auto 

esp_now_peer_info_t peerInfo;

struct Message {
  LaskaGamepad gamepad_inputs;
  uint8_t battery_v;  // voltage * 10
  uint8_t off;
};

Message message;


// power off
int i = 0, ii = 0;
bool poweroff = false;
int last = millis();
float batt_min = 3.0;
float batt_warn = 3.2;


LaskaGamepad laska_gamepad;

void print_inputs(LaskaGamepad* lg) {
  int16_t* lgg = (int16_t*)lg;
  for (int j = 0; j < 5; j++) {
    Serial.print(j); Serial.print(" ");
    for (int i = 0; i < 16; i++) {
      Serial.print((*lgg & (1 << i)) >> i);
    }
    lgg++;
    Serial.println();
  }
}

void print_message(Message* m) {
  print_inputs(&m->gamepad_inputs);
}

void setup() {
  pinMode(OFF_PIN, INPUT);

  Serial.begin(115200);
  
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);

  adc.attach(ADCpin);  // battery

  // i2c
  Wire.begin(8, 10);
  ADS.begin();
  // voltage on gamepad stick is 0-3V3 (3.31) safe margin
  // so the raw value is between 0 and 26400 (26480) in theory
  // there are some deadzones
  ADS.setGain(1);  // +- 4.096 V
  PCF.begin();

  pixels.setPixelColor(active_car, pixels.Color(0, 255, 0)); // R, G, B
  pixels.show();

  WiFi.mode(WIFI_STA);
  esp_now_init();
  // esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, cars[active_car], 6);
  // memcpy(peerInfo.peer_addr, cars[0], 6);
  
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
         
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

int16_t handle_deadzones(int16_t value) {
  // deadzone ends 250
  // deadzone center 800
  if (abs(value) < 800) {
    return 0;
  }
  if (value > (1 << 15) - 1 - 250) {
    return (1 << 15) - 1;
  }
  if (value < -(1 << 15) + 250) {
    return -(1 << 15);
  }
  return value;
}

uint16_t convert_to_16_bit(int32_t raw_value) {
  // 65535 / 26490
  return raw_value * 4369 / 1766 - (1 << 15);
}

int16_t read_gamepad_stick_adc(uint8_t n) {
  return handle_deadzones(convert_to_16_bit(ADS.readADC(n)));
}

void read_gamepad() {
  // read sticks
  laska_gamepad.left_stick_y = read_gamepad_stick_adc(0);
  laska_gamepad.left_stick_x = read_gamepad_stick_adc(1);
  laska_gamepad.right_stick_y = read_gamepad_stick_adc(2);
  laska_gamepad.right_stick_x = read_gamepad_stick_adc(3);

  // read buttons
  laska_gamepad.buttons = ~(PCF.read16());
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


  read_gamepad();

  // check for peer select
  if (laska_gamepad.get_button(LaskaGamepadButton::LEFT)) {
    esp_now_del_peer(cars[active_car]);
    active_car = 0;
  } else if (laska_gamepad.get_button(LaskaGamepadButton::UP)) {
    esp_now_del_peer(cars[active_car]);
    active_car = 1;
  } else if (laska_gamepad.get_button(LaskaGamepadButton::RIGHT)) {
    esp_now_del_peer(cars[active_car]);
    active_car = 2;
  }

  if (laska_gamepad.buttons &
      (LaskaGamepadButton::LEFT |
       LaskaGamepadButton::UP |
       LaskaGamepadButton::RIGHT)) {
    pixels.clear();
    pixels.setPixelColor(active_car, pixels.Color(0, 255, 0));
    memcpy(peerInfo.peer_addr, cars[active_car], 6);
    esp_now_add_peer(&peerInfo);
  }

  pixels.show();

  message.gamepad_inputs = laska_gamepad;

  // read battery
  float batt_voltage = adc.readVoltage() * DividerRatio * 10;
  uint8_t bv = batt_voltage;
  // Serial.print(bv); Serial.print(" ");
  // Serial.println(batt_voltage);
  message.battery_v = bv;
  // Serial.print("Battery Voltage = " );
  // Serial.print(batt_voltage);
  // Serial.println("V");
  // Serial.println(sizeof(m));

  // if (batt_voltage <= batt_min){
  //   poweroff = true;
  // }

  

  // print_message(&message);
  // Serial.println();
  esp_err_t result = esp_now_send(cars[active_car], (uint8_t *) &message, sizeof(message));
  // Serial.print("ESP status: ");
  // Serial.println(result);
  delay(10);
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }else{
    Serial.println("Error sending the data");
  }


  // send success
  // if (ss){
  //   pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // R, G, B
  //   pixels.show();
  // }else{
  //   pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // R, G, B
  //   pixels.show();
  // }


  // // turn off if the battery voltage is low
  // if (batt_voltage <= batt_min){
  //   pinMode(OFF_PIN, OUTPUT);
  //   digitalWrite(OFF_PIN, LOW);
  // } else if (batt_voltage <= batt_warn) {
  //   pixels.setPixelColor(2, pixels.Color(255, 0, 0));
  //   pixels.show();
  // }
}

/* 
  Example code for LaskaKit RoboBoard ESP32. This example contains
  the FW for the 'car' with omni wheels that is controlled from gamepad
  using esp-now protocol.
  
  
  Firmware for the gamepad can be found here:
  TODO: gamepad fw link

  Board:  LaskaKit RoboBoard ESP32      https://www.laskakit.cz/laskakit-roboboard-esp32/?variantId=13424
  Motors: TT motor - plastic gears      https://www.laskakit.cz/tt-motor-s-prevodovkou-plastove-prevody/
  Wheels: Mecanum Omni Wheels           https://www.laskakit.cz/sada-4ks-mecanum-omni-kol-60mm/

  Email: podpora@laskakit.cz
  Web: laskakit.cz
*/

#include <Adafruit_MotorShield.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ESP32AnalogRead.h>
#include <esp_now.h>
#include <WiFi.h>

#include "laskacar.hpp"

#define CHANNEL 1
#define DIVIDER_RATIO 2.599   //new

#define ADCpin 34
ESP32AnalogRead adc;

#define PIN_BUZZ 27
#define low_batt 6.7
#define AUTO_OFF 23

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x40);
Adafruit_DCMotor *BackLeftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *BackRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *FrontRightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *FrontLeftMotor = AFMS.getMotor(3);
LaskaOmniCar laska_omni_car(BackLeftMotor, BackRightMotor, FrontLeftMotor, FrontRightMotor);

typedef struct gamepad_message {
  long batt;
  long lx;
  long ly;
  long rx;
  long ry;
  long b;
  int b2;
  bool off;
} GAMEPAD_MESSAGE;

GAMEPAD_MESSAGE myData;

int motorSpeed = 255;
int lyval = 127;
int lxval = 127;
int ryval = 127;
int rxval = 127;
int triangle;
int cross;
int up;
int dwn;
int left;
int right;
int beac_run;
int spd = 100;

float reading;
unsigned long previousMillis = 0;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void setup() {
  pinMode(AUTO_OFF, OUTPUT);
  digitalWrite(AUTO_OFF, LOW);
  pinMode(16, INPUT_PULLUP);
  Wire.begin(21, 22);
  Serial.begin(115200);
  delay(1000);
  adc.attach(ADCpin);

  tone(PIN_BUZZ,1000,200);

  // WiFi.mode(WIFI_MODE_STA);
  // Serial.println(WiFi.macAddress());
  // return;

  WiFi.mode(WIFI_STA);
  // WiFi.mode(WIFI_AP);
  // configure device AP mode
  // configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  // Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  // InitESPNow();
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp now failed");
  } else {
    Serial.println("esp now OK");
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  // start adafruit motor shield
  AFMS.begin();
  reading = adc.readVoltage();
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  memcpy(&myData, data, sizeof(myData));
  Serial.print("Recv from: "); Serial.println(macStr);
  Serial.print("BATT: "); Serial.print(myData.batt);
  Serial.print("\tLX: "); Serial.print(myData.lx);
  Serial.print("\tLY: "); Serial.print(myData.ly);
  Serial.print("\tRX: "); Serial.print(myData.rx);
  Serial.print("\tRY: "); Serial.print(myData.ry);
  Serial.print("\tB: "); Serial.print(myData.b, BIN);
  Serial.print("\tB2: "); Serial.print(myData.b2, BIN);
  Serial.println("");
  // tone(PIN_BUZZ,500,500);

  lyval = myMap(myData.ly, 17000, 0, 0, 255);
  lxval = myMap(myData.lx, 17000, 0, 0, 255);
  rxval = myMap(myData.rx, 17000, 0, 0, 255);
  ryval = myMap(myData.ry, 17000, 0, 0, 255);

  // if (myData.off) {
  //   Serial.println("mydataofff"); 
  //   poweroff();
  // }
  laska_omni_car.move(ryval, rxval, lxval);
}

void loop() {}

void poweroff(void){
  tone(PIN_BUZZ,500,500);
  //pinMode(AUTO_OFF, OUTPUT);
  digitalWrite(AUTO_OFF, HIGH);
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

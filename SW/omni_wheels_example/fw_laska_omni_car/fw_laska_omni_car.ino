/* 
  Example code for LaskaKit RoboBoard ESP32. This example contains
  the FW for the 'car' with omni wheels that is controlled from gamepad
  using esp-now protocol.
  
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

#include "laskagamepad.hpp"
#include "laskacar.hpp"
#include "laskacar_beacon.hpp"

#define CHANNEL 1
#define DIVIDER_RATIO 2.599   //new
// 8.4 100
// 6.0   0


#define ADCpin 34
ESP32AnalogRead adc;

#define PIN_BUZZ 27
#define low_batt 6.7
#define AUTO_OFF 23

// disable Adafruit splash logo
#define SH110X_NO_SPLASH

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x40);
Adafruit_DCMotor *BackLeftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *BackRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *FrontRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *FrontLeftMotor = AFMS.getMotor(2);
LaskaOmniCar laska_omni_car(BackLeftMotor, BackRightMotor, FrontLeftMotor, FrontRightMotor);

struct Message {
  LaskaGamepad gamepad_inputs;
  uint8_t battery_v;  // voltage * 10
  uint8_t off;
};
Message m;

float battery_car_v;
unsigned long previousMillis = 0;

Beacon beacon(16);
LeftRightBeaconPattern lrbp(&beacon);

Adafruit_SH1106G display(128, 64, &Wire);


int display_refresh = 0;
unsigned long gamepad_watchdog_timer = 0;


uint8_t gamepads[2][6] = {
  {0x80, 0x65, 0x99, 0x96, 0x6B, 0x3C},  // black
  {0x24, 0x58, 0x7C, 0x01, 0xF9, 0xA8} // pink
};

esp_now_peer_info_t peerInfo;


bool mac_compare(const uint8_t mac1[6], const uint8_t mac2[6]) {
  for (int i = 0; i < 1; i++) {
    if (mac1[i] != mac2[i]) {
      return false;
    }
  }
  return true;
}


// esp-now callback
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  memcpy(&m, data, sizeof(m));

  if (data_len != 12) {
    return;
  }

  laska_omni_car.move(m.gamepad_inputs.right_stick_y >> 7,
                      m.gamepad_inputs.right_stick_x >> 7,
                      m.gamepad_inputs.left_stick_x >> 7);
  gamepad_watchdog_timer = millis();
}


uint8_t get_battery_status() {
  float empty = 6.0, full = 8.4;
  battery_car_v = adc.readVoltage() * DIVIDER_RATIO;
  uint8_t battery_car_percent = 100 * (battery_car_v - empty) / (full - empty);
  return battery_car_percent;
}

void display_battery_status() {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.print(get_battery_status());
  display.print("%");
}


void poweroff(void){
  tone(PIN_BUZZ,500,500);
  digitalWrite(AUTO_OFF, HIGH);
}


void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AUTO_OFF, OUTPUT);
  digitalWrite(AUTO_OFF, LOW);

  // display
  display.begin(0x3C, true); // Address 0x3C default
  display.setTextColor(SH110X_WHITE);
  // display.setTextSize(1);
  // display.setCursor(0, 0);
  // display.print("Battery voltage:");
  display.setTextSize(5);

  // battery voltage measurement
  adc.attach(ADCpin);
  // battery_car_v = adc.readVoltage() * DIVIDER_RATIO;
  // display.clearDisplay();
  // display.setCursor(0, 20);
  // display.print(battery_car_v);
  // Serial.println(battery_car_v);

  display_battery_status();

  WiFi.mode(WIFI_STA);

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


  // init the beacon pattern
  // lrbp.init();

  // signal that setup is complete
  tone(PIN_BUZZ,1000,200);

  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, gamepads[0], 6);
  esp_now_add_peer(&peerInfo);
  // memcpy(peerInfo.peer_addr, gamepads[1], 6);
  // esp_now_add_peer(&peerInfo);
}


void loop() {
  // gamepad connection watchdog
  unsigned long current = millis();
  if (current - gamepad_watchdog_timer >= 200) {
    laska_omni_car.move(0, 0, 0);  // stop the car
    m.gamepad_inputs.buttons = 0;  // unpress all buttons
  }


  display_refresh++;
  // refresh once every 10s
  if (display_refresh > 20 * 10) {
    display_battery_status();
    display.display();
    display_refresh = 0;
  }

  // make noise when cross is pressed
  if (m.gamepad_inputs.get_button(LaskaGamepadButton::CROSS)) {
    tone(PIN_BUZZ, 500, 55);
  }

  // start beacon when triangle is pressed
  if (m.gamepad_inputs.get_button(LaskaGamepadButton::TRIANGLE)) {
    lrbp.init();
  }

  // stop beacon when square is pressed
  if (m.gamepad_inputs.get_button(LaskaGamepadButton::SQUARE)) {
    beacon.leds_state = 0;
  }

  // power off the car when select is pressed or battery is <= 30%
  if (m.gamepad_inputs.get_button(LaskaGamepadButton::START) { // ||  get_battery_status() <= 30) {
    poweroff();
  }

  // handle the beacon lighting
  lrbp.step();
  beacon.write();

  // full light front when l1 is pressed
  // must be handled after beacon lighting to override it
  if (m.gamepad_inputs.get_button(LaskaGamepadButton::L1)) {
    beacon.leds.fill(beacon.leds.Color(255, 255, 255), 8, 8);
  }

  beacon.leds.show();
  delay(50);
}



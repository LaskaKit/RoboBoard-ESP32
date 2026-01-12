/* LED test for LaskaKit RoboBoard ESP32 V2
 * ESP32-S3-DEVKit just changing LED color 
 *
 * Board:   LaskaKit RoboBoard ESP32 V2   https://www.laskakit.cz/laskakit-roboboard-esp32/
 * 
 * Library: https://github.com/adafruit/Adafruit_NeoPixel
 *
 * Email:podpora@laskakit.cz
 * Web:laskakit.cz
 */

#include <Adafruit_NeoPixel.h>

#define BRIGHTNESS 10
#define LED_PIN 4
Adafruit_NeoPixel statusled = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  statusled.begin();
  statusled.show();
  statusled.setBrightness(BRIGHTNESS);

  statusled.setPixelColor(0, statusled.Color(255, 0, 0)); // R, G, B
  statusled.show();
  delay(2000);
  statusled.setPixelColor(0, statusled.Color(0, 255, 0)); // R, G, B
  statusled.show();
  delay(2000);
  statusled.setPixelColor(0, statusled.Color(0, 0, 255)); // R, G, B
  statusled.show();
  delay(2000);

}

void loop() {
  rainbow(10);
}

void rainbow(int wait) {
  for(long pixelHue = 0; pixelHue < 5*65536; pixelHue += 256) {
    statusled.setPixelColor(0, statusled.gamma32(statusled.ColorHSV(pixelHue)));
    statusled.show();
    delay(wait);
  }
}

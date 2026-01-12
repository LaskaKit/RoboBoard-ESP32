/* Buzzer test for LaskaKit RoboBoard ESP32 V2
 * ESP32-S3-DEVKit just changing LED color 
 *
 * Board:   LaskaKit RoboBoard ESP32 V2   https://www.laskakit.cz/laskakit-roboboard-esp32/
 * 
 *
 * Email:podpora@laskakit.cz
 * Web:laskakit.cz
 */

#define PIN_BUZZ 27

void setup() {
  tone(PIN_BUZZ, 1000, 200);
}

void loop() {

}
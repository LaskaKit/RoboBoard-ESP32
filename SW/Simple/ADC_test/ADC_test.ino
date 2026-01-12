/* ADC test for LaskaKit RoboBoard ESP32 V2
 *
 * Board:   LaskaKit RoboBoard ESP32 V2   https://www.laskakit.cz/laskakit-roboboard-esp32/
 * 
 * Email:podpora@laskakit.cz
 * Web:laskakit.cz
 */

#define DIVIDER_RATIO 3.5677966

#define ADCpin 34

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(ADCpin, ADC_11db);
}

void loop() {
  float battery_volt = (analogReadMilliVolts(ADCpin) * 0.001f) * DIVIDER_RATIO;
  Serial.printf("Battery voltage: %.3f\r\n", battery_volt);
  delay(2000);
}

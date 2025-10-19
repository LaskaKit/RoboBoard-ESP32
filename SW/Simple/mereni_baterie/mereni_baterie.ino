// mereni napeti baterie
// arduino-esp32 verze 3.x.x - (3.3.2)

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

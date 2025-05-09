#include <WiFi.h>
 
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  Serial.println();
  delay(1000);
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}
 
void loop() {
}

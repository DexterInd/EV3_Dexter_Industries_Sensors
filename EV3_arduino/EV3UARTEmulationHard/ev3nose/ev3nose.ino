#include <EV3UARTEmulationHard.h>
#include <Serial.h>

EV3UARTEmulation sensor(&Serial1, 99, 38400);

void setup() {
  Serial.begin(9600);
  while(!Serial1);
  sensor.create_mode("TEST", true, DATA16, 1, 3, 0);
  sensor.reset();
}

unsigned long last_reading = 0;

void loop() {
  sensor.heart_beat();
  if (millis() - last_reading > 100) {
    int r = analogRead(0);
    sensor.send_data16(r);
    Serial.print("Reading: ");
    Serial.println(r);
    last_reading = millis();
  }
}

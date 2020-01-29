#include <SoftwareSerial.h>
#include <EV3UARTEmulation.h>
#include <Serial.h>

EV3UARTEmulation sensor(10, 11, 99, 38400);

void setup() {
  Serial.begin(115200);
  sensor.create_mode("TEST", true, DATA8, 1, 2, 0);
  sensor.reset();
}

unsigned long last_reading = 0;
byte data = 0;

void loop() {
  sensor.heart_beat();
  if (millis() - last_reading > 100) {
    sensor.send_data8(data++);
    if (data > 99) data = 0;
	last_reading = millis();
  }
}

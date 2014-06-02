#include <EV3UARTSensor.h>
#include <Serial.h>
#include <SoftwareSerial.h>

EV3UARTSensor sensor(10,11);

void setup() {
  Serial.begin(115200);
  sensor.begin();
}

unsigned long lastMessage = 0;

void loop() {
  sensor.check_for_data();

  if (sensor.get_status() == DATA_MODE && (millis() - lastMessage) > 1000 ) {
    for(int i=0;i<sensor.get_number_of_modes();i++) {
      EV3UARTMode* mode = sensor.get_mode(i);
      Serial.print(i);
      Serial.print(" ");
      Serial.print(mode->name); 
      Serial.print("\t");
      Serial.print(mode->get_data_type_string());
      Serial.print("\t");
      Serial.print(mode->symbol);
      Serial.print("\t");
      Serial.print(mode->sets);
      Serial.print("\t");
      Serial.print(mode->raw_low);
      Serial.print(" - ");
      Serial.println(mode->raw_high);
    }
    if (Serial.available()) {
      int cmd = Serial.read();
      if (cmd >= '0' && cmd <= '9') {
         int mode = cmd - '0';
         if (mode >=0 && mode < sensor.get_number_of_modes()) {
           Serial.print("Setting mode to ");
           Serial.println(mode);
           sensor.set_mode(mode);
         }
      } else if (cmd == 'r') sensor.reset();
    }
    Serial.print("Current mode is ");
    Serial.println(sensor.get_current_mode());
    Serial.print("Sample size is ");
    Serial.println(sensor.sample_size());
    Serial.print("Sensor type is ");
    Serial.println(sensor.get_type());
 
    float sample[sensor.sample_size()];
    sensor.fetch_sample(sample, 0);
    Serial.print("Sample is ");
	for(int i=0;i<sensor.sample_size();i++) {
	  Serial.print(sample[i]);
	  Serial.print(" ");
	}
	Serial.println();
    lastMessage = millis();
  }
}
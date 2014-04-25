#include <Wire.h>

#define SLAVE_ADDRESS 0x04
void setup() 
{
    Serial.begin(9600);         // start serial for output
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
    Serial.println("Ready!");
}
int val,flag=0,index=0;
uint8_t c[]="DEXTERIN";
char buf[8];
void loop()
{
}

void receiveData(int byteCount)
{
    while(Wire.available()>0) 
    {
      val=Wire.read();
      Serial.print((char)val);
      flag=1;
    }
}

// callback for sending data
void sendData()
{
  Wire.write(c,8);
}


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
int pin,st,flag=0;
void loop()
{
  if(flag==1)
  {
    flag=0;
    pinMode(pin,OUTPUT);
    digitalWrite(pin,st);
  }
}

void receiveData(int byteCount)
{
    while(Wire.available()>0) 
    {
      pin=Wire.read();
      st=Wire.read();
      flag=1;
    }
}

// callback for sending data
void sendData()
{
  Wire.write("~");
}


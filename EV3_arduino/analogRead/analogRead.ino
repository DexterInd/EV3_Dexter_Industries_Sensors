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
int pin,st,val=0,flag=0,index=0;
char buf[8];
byte b[2];
void loop()
{
  if(flag==1)
  {
    flag=0;
    Serial.println(pin);
    val=analogRead(pin);
    b[0]=val/256;
    b[1]=val%256;
  }
}

void receiveData(int byteCount)
{
    while(Wire.available()>0) 
    {
      pin=Wire.read();
      flag=1;
    }
}

// callback for sending data
void sendData()
{
  Wire.write(b,2);
}


#include "EV3UARTEmulation.h"
#include <Serial.h>

/**
 * Create a mode object
**/
EV3UARTMode::EV3UARTMode() {
}

/**
 * Create the sensor emulation with the specified RX and TX pins
**/
EV3UARTEmulation::EV3UARTEmulation(byte rx_pin, byte tx_pin, byte type, unsigned long speed) {
  uart = new SoftwareSerial(rx_pin, tx_pin);
  this->type = type;
  this->speed = speed;
  status = 0;
  modes = 0;
  views = 0;
  current_mode = 0;
}

void EV3UARTEmulation::create_mode(String name, boolean view, 
		                           byte data_type, byte sample_size, 
						           byte figures, byte decimals) {
  EV3UARTMode* mode = new EV3UARTMode();
  mode->name = name;
  mode->view = view;
  mode->data_type = data_type;
  mode->sample_size = sample_size;
  mode->figures = figures;
  mode->decimals = decimals;
  mode_array[modes] = mode;
  modes++;
  if (view) views++;
}	

byte EV3UARTEmulation::get_status() {
  return status;
}

void EV3UARTEmulation::reset () {
  for(;;) {
#ifdef DEBUG
      Serial.println("Reset");
#endif
	  uart->begin(2400);
	  byte b[4];
	  b[0] = type;
	  send_cmd(CMD_TYPE, b, 1);
	  b[0] = modes - 1;
	  b[1] = views - 1;
	  send_cmd(CMD_MODES, b, 2);
	  get_long(speed, b);
	  send_cmd(CMD_SPEED, b, 4);
	  for(int i=modes-1;i>=0;i--) {
		EV3UARTMode* mode = get_mode(i);
		byte l = mode->name.length();
		byte ll = next_power2(l);
		byte bb[ll+2];
		bb[0] = 0;  // name
		mode->name.getBytes(&bb[1],ll+1); // Leave room for null terminator
		byte lll = log2(ll);
		// Send name 
		send_cmd(CMD_INFO | (lll << CMD_LLL_SHIFT) | i, bb, ll+1);
		byte bbb[5];
		bbb[0] = 0x80;
		bbb[1] = mode->sample_size;
		bbb[2] = mode->data_type;
		bbb[3] = mode->figures;
		bbb[4] = mode->decimals;
		send_cmd(CMD_INFO | (2 << CMD_LLL_SHIFT) | i, bbb, 5);
	  }
#ifdef DEBUG
      Serial.println("Sending ACK");
#endif
	  send_byte(BYTE_ACK);
	  unsigned long m = millis();
	  while(!uart->available() && millis() - m < ACK_TIMEOUT);
	  if (uart->available()) {
		byte b = uart->read();
		if (b == BYTE_ACK) {
		  uart->end();
		  uart->begin(speed);
		  delay(80);
		  last_nack = millis();
		  break;
		}
	  }
	}
}

void EV3UARTEmulation::heart_beat() {
  if (millis() - last_nack > HEARTBEAT_PERIOD) reset();
  if (uart->available()) {
      byte b = uart->read();
      if (b == BYTE_NACK) {
#ifdef DEBUG
        Serial.println("Received NACK");
#endif	  
	    last_nack = millis();
	  } else {
#ifdef DEBUG
        Serial.print("Received: ");
		Serial.println(b, HEX);
#endif	
         if (b == CMD_SELECT) {
		   byte checksum = 0xff ^ b;
		   byte mode = read_byte();
		   checksum ^= mode;
		   if (checksum == read_byte()) {
		     if (mode < modes) current_mode = mode;
		   }
		 }
      } 
  }
}

EV3UARTMode* EV3UARTEmulation::get_mode(byte mode) {
  return mode_array[mode];
}	

void EV3UARTEmulation::send_data8(byte mode, byte b) {
  byte bb[1];
  bb[0] = b;
  
  send_cmd(CMD_DATA | mode, bb, 1);
}

void EV3UARTEmulation::send_data16(byte mode, short s) {
  byte bb[1];
  bb[0] = s & 0xff;
  bb[1] = s >> 8;
  send_cmd(CMD_DATA | (1 << CMD_LLL_SHIFT) | mode, bb, 2);
}	

void EV3UARTEmulation::send_data16(byte mode, short* s, int len) {
  byte bb[len*2];
  for(int i=0;i<len;i++) {
    bb[2*i] = s[i] & 0x255;
    bb[2*i+1] = s[i] >> 8;   
  }
  send_cmd(CMD_DATA | (log2(len*2) << CMD_LLL_SHIFT) | mode, bb, len*2);
}

void EV3UARTEmulation::send_data32(byte mode, long l) {
  byte bb[4];
  for(int i=0;i<4;i++) {
    bb[i] = (l >> (i * 8)) && 0xff;
  }
  send_cmd(CMD_DATA | (2 << CMD_LLL_SHIFT) | mode, bb, 4);
}

void EV3UARTEmulation::send_dataf(byte mode, float f) {
  union Data {
    unsigned long l;
    float f;
  } data;
  data.f = f;
  send_data32(mode, data.l);
}			

void EV3UARTEmulation::send_cmd(byte cmd, byte* data, byte len) {
  byte checksum = 0xff ^ cmd;
#ifdef DEBUG
      Serial.print("Cmd: ");
	  Serial.print(cmd, HEX);
	  Serial.print(" ");
#endif
  uart->write(cmd);
  for(int i=0;i<len;i++) {
    checksum ^= data[i];
#ifdef DEBUG
      Serial.print(data[i],HEX);
	  Serial.print(" ");
#endif
    uart->write(data[i]);
  }
  uart->write(checksum);
#ifdef DEBUG
      Serial.println(checksum, HEX);
#endif
}

void EV3UARTEmulation::send_byte(byte b) {
  uart->write(b);
}

/**
 * Utility method to read a long from a byte array
**/
void EV3UARTEmulation::get_long(unsigned long l, byte* bb) {
  for(int i=0;i<4;i++) {
    bb[i] = (l >> (i * 8)) & 0xff;
  }
}

/**
 * Utility method to return a small power of 2
**/
int EV3UARTEmulation::log2(int val) {
  switch(val) {
    case 1: return 0;
    case 2: return 1;
    case 4: return 2;
    case 8: return 3;
    case 16: return 4;
    case 32: return 5;
    default: return 0;
  }
}

int EV3UARTEmulation::next_power2(int val) {
  if (val == 1 || val == 2) return val;
  else if (val <= 4) return 4;
  else if (val <= 8) return 8;
  else if (val <= 16) return 16;
  else if (val <= 32) return 32;
  else return 0;
}

/**
 * Utility method to read a byte synchronously
**/
byte EV3UARTEmulation::read_byte() {
  while(!uart->available());
  return uart->read();
}

byte EV3UARTEmulation::get_current_mode() {
  return current_mode;
}
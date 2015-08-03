// EV3UARTSensor.h
//
// Interface to LEGO Mindstorms EV3 UART Sensors
// 
// Author: Lawrie Griffiths (lawrie.griffiths@ntlworld.com)
// Copyright (C) 2014 Lawrie Griffiths

#include <Arduino.h>
#include <Serial.h>

// Message values
#define   BYTE_ACK                      0x04
#define   BYTE_NACK                     0x02
#define   CMD_SELECT                    0x43
#define   CMD_TYPE                      0x40
#define   CMD_MODES                     0x49
#define   CMD_SPEED                     0x52
#define   CMD_MASK                      0xC0
#define   CMD_INFO                      0x80
#define   CMD_LLL_MASK                  0x38
#define   CMD_LLL_SHIFT                 3
#define   CMD_MMM_MASK                  0x07
#define   CMD_DATA                      0xc0
#define   CMD_WRITE                     0x44

#define   TYPE_COLOR                    29

// Values for status
#define RESET 0
#define STARTED 1
#define DATA_MODE 2

// Maximum number of modes supported
#define MAX_MODES 10

// The maximum number of data items in a sample
#define MAX_DATA_ITEMS 10

// The time between heartbeats in milliseconds
#define HEART_BEAT 100

// Set to get message debbugging
//#define DEBUG

/**
* Represent a specific sensor mode
**/
class EV3UARTMode {
	public:
		EV3UARTMode();
		String name;                      // The mode name
		String symbol;                    // The unit symbol
		byte sets;                        // The number of samples
		byte data_type;                   // The data type 0= 8bits, 1=16 bits, 2=32 bits, 3=float
		byte figures;                     // Number of significant digits
		byte decimals;                    // Number of decimal places
		float raw_low, raw_high;          // Low and high values for raw data
		float si_low, si_high;            // Low and high values for SI data
		float pct_low, pct_high;	      // Low and high values for Percentage values
		String get_data_type_string();    // Get the data type as a string
};

/**
* Represent a generic EV3 UART Sensor
**/
class EV3UARTSensor {
	public:
		EV3UARTSensor(HardwareSerial* serial);         // Create the sensor and specify the pins for SoftwareSerial
		void begin();                                  // Start communicating with the sensor
		void end();                                    // End communication with      
		void check_for_data();                         // Called from Arduino loop to process all data from the sensor
		int get_number_of_modes();                     // Number of modes supported
		void set_mode(int mode);                       // Set the sensor to the specific mode
		int get_current_mode();                        // The current sensor mode
		int sample_size();                             // The number of items in a sample for the current mode
		void fetch_sample(float* sample, int offset);  // Fetch a sample in the current mode
		int get_status();                              // Get the status of the connection
		EV3UARTMode* get_mode(int mode);               // Get the EV3UARTMode object for a specific mode
		void reset();                                  // Make the sensor reset
		void send_write(byte* bb, int len);            // Send a WRITE command to the sensor
		int get_type();                                // Get the LEGO type code for the sensor
	private:
	    byte read_byte();                              // Read a byte from the sensor (synchronous)
		unsigned long get_long(byte* bb, int offset);  // Helper method to get a long value
		String get_string(byte* bb, int len);          // Helper method to get a String value
		float get_float(byte* bb, int len);            // Helper method to get a float value
		int exp2(int val);                             // Helper method for powers of 2
		String get_info_type(int val);                 // Helper method to get type of INFO message
		String get_data_type(int val);                 // Helper method to get the data type as a string
		void send_select(byte mode);                   // Send a CMD_SELECT command t change modes
		int get_int(byte* bb, int offset);             // Helper method to get an int
		unsigned long speed;                           // The required bit rate of the sensor
		byte mode;                                     // The current sensor mode
		byte status;                                   // The current status of the connection
		byte modes;                                    // The number of modes supported
		byte views;                                    // The number of views supported
		unsigned long last_nack;                       // The time of the last heartbeat NACK
		byte type;                                     // The internal type encoding of the sensor
		HardwareSerial *ss;                            // Reference to the SoftwareSerial object
		int data_errors;                               // Total number of data errors
		float value[MAX_DATA_ITEMS];                   // The current value
		int num_samples;                               // The current number of samples
		EV3UARTMode* mode_array[MAX_MODES] ;           // An array of EV3UARTMode objects
		byte consecutive_errors;                       // Number of sonsective errors
		int recent_messages;                           // Number of recent messages
};

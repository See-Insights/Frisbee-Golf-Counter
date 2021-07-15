/*	Accelerometer-Interrupt-Test.ino
	Chip McClelland (chip@seeinsights.com)
	July 15, 2021
	https://github.com/sparkfun/SparkFun_MMA8452Q_Particle_Library

	In this skech we use the interrupt capabilities of the MMA8452Q

	Development environment specifics:
	Particle Build environment (https://www.particle.io/build)
	Particle Boron
	Distributed as-is; no warranty is given. 
*/
// Include the library:
#include "ModMMA8452Q.h"

// Prototypes and system calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.

// For monitoring / debugging, you can uncomment the next line
SerialLogHandler logHandler(LOG_LEVEL_ALL);

// Create an MMA8452Q object, used throughout the rest of the sketch.
MMA8452Q accel; // Default constructor, SA0 pin is HIGH

// The above works if the MMA8452Q's address select pin (SA0) is high.
// If SA0 is low (if the jumper on the back of the SparkFun MMA8452Q breakout
// board is closed), initialize it like this:
// MMA8452Q accel(MMA8452Q_ADD_SA0_);

// Pin constants
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor

// Pin Constants - Sensor
const int intPin =        D2;                      // Accelerometer Interrupt Pin - I2

// program variables
unsigned long timeSinceILastCheckedForATap;         // Temp for polling
char pinMessage[64] = "NA";

// Sensor Variables
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered
byte sensitivity = 1;                               // This is the sensitivity of the accelerometer - 0 is lowest and 10 is the highest



void setup()
{
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output

  // Pressure / PIR Module Pin Setup
  pinMode(intPin,INPUT);                            // sensor interrupt (push/ pull)
  digitalWrite(blueLED,HIGH);                       // Turn on the led so we can see how long the Setup() takes

  delay(1000);                                                        // Time to start monitoring

  // Initialize the accelerometer with begin():
	// begin can take two parameters: full-scale range, and output data rate (ODR).
	// Full-scale range can be: SCALE_2G, SCALE_4G, or SCALE_8G (2, 4, or 8g)
	// ODR can be: ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_12, ODR_6 or ODR_1
  accel.begin(SCALE_2G, ODR_100); // Set up accel with +/-2g range, and 100Hz ODR

  accel.setupTapInts(sensitivity);                                    // Set up taps on x,y and z defaults otherwise

  Log.info(pinMessage);                                               // Report on interrupt initial state

  accel.clearTapInts();

  attachInterrupt(intPin, sensorISR, RISING);                          // Accelerometer interrupt from low to high

  digitalWrite(blueLED,LOW);                                           // Signal the end of startup

}

void loop()
{
  if (sensorDetect == true) {                                         // Interrupt flag raised - need to report a tap
      detachInterrupt(intPin);
      Log.info("Accelerometer Interrupt Triggered");                  // Report entering tap handling
      sensorDetect = false;                                           // Reset the flag
      if (sensitivity < 10) sensitivity++;
      else sensitivity = 0;
      snprintf(pinMessage, sizeof(pinMessage), "Sensitivity set to %i",sensitivity);
      Log.info(pinMessage);
      accel.setupTapInts(sensitivity);
      delay(2000);                                                    // debounce
      accel.clearTapInts();                                           // Reads the interrupt register clearing the latched interrupt
      attachInterrupt(intPin, sensorISR, RISING);                          // Accelerometer interrupt from low to high
  }
}

void sensorISR() {
  sensorDetect = true;                                              // sets the sensor flag for the main loop
}

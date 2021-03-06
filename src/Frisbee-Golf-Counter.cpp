/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/Frisbee-Golf-Counter/src/Frisbee-Golf-Counter.ino"
/*
* Project Frisbee Golf counter for Morrisville City
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland & Sanjna Jotwani
* Date: June 14, 2021
*/

/*  This is a refinement on the Boron Connected Counter firmware and incorporates new watchdog and RTC
*   capabilities as laid out in AN0023 - https://github.com/particle-iot/app-notes/tree/master/AN023-Watchdog-Timers
*   This software will work with both pressure and PIR sensor counters
*/

//v1    - Adapted from the Visitation Counters Code at release v11.06
//v1.01 - Working on interrupt driven alerts
//v1.02 - Added polling back in as well
//v1.03 - Multiple fixes for connectivity
//v1.05 - Working on getting the hardware interrupt working
//v1.06 - Hardware interrupts
//v1.07 - Fixed issue with sensitivity and debounce variable readings
//v2.00 - Issues with multiple hourly reporting and sleep / debounce interference, need to detach interrupt with sleep
//v2.01 - Added a check before napping that the intPin is not high
//v3.00 - Update to reduce change of napping with the interrupt flag set
//v3.01 - Still more safeguards needed
//v4.00 - One more check and will put into use
//v5.00 - Changed record count to include a debounce on the interrupt for stability
//v6.00 - Trying to make it less likely that the devices get hung up
//v7.00 - We will need to periodically wake and make sure the device has cleared the interrupt: https://community.particle.io/t/how-is-this-possible-frustrations-with-i2c-interrupts-and-sleep/60875/11
//v8.00 - Still leaving in the periodic wakes but moving to a pulse interrupt not latching
//v9.00 - Too many resets - reactivating the watchdog

// Particle Product definitions
void setup();
void loop();
void recordCount();
void sendEvent();
void UbidotsHandler(const char *event, const char *data);
void takeMeasurements();
bool isItSafeToCharge();
void getSignalStrength();
int getTemperature();
void outOfMemoryHandler(system_event_t event, int param);
void sensorISR();
void countSignalTimerISR();
int setPowerConfig();
void loadSystemDefaults();
void checkSystemValues();
void makeUpParkHourStrings();
bool disconnectFromParticle();
int resetCounts(String command);
int hardResetNow(String command);
int sendNow(String command);
void resetEverything();
int setSolarMode(String command);
int setVerboseMode(String command);
String batteryContextMessage();
int setOpenTime(String command);
int setCloseTime(String command);
int setDailyCount(String command);
int setSensitivity(String command);
int setDebounceSec(String command);
int setLowPowerMode(String command);
void publishStateTransition(void);
void fullModemReset();
void dailyCleanup();
#line 32 "/Users/chipmc/Documents/Maker/Particle/Projects/Frisbee-Golf-Counter/src/Frisbee-Golf-Counter.ino"
PRODUCT_ID(PLATFORM_ID);                            // No longer need to specify - but device needs to be added to product ahead of time.
PRODUCT_VERSION(9);
#define DSTRULES isDSTusa
char currentPointRelease[6] ="9.00";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 3;                    // Increment this number each time the memory map is changed


struct currentCounts_structure {                    // currently 10 bytes long
  int hourlyCount;                                  // In period hourly count
  int hourlyCountInFlight;                          // In flight and waiting for Ubidots to confirm
  int dailyCount;                                   // In period daily count
  unsigned long lastCountTime;                      // When did we record our last count
  int temperature;                                  // Current Temperature
  int alertCount;                                   // What is the current alert count
  int maxMinValue;                                  // Highest count in one minute in the current period
  uint16_t maxConnectTime = 0;                      // Longest connect time for the day
  int minBatteryLevel = 100;                        // Lowest Battery level for the day
} current;

// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                  // Pinout Documentation File
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "PublishQueueAsyncRK.h"                    // Async Particle Publish
#include "ModMMA8452Q.h"                            // Modified SparkFun library

// Libraries with helper functionsB40TAB9228FTJVL LXUML5Y3EE3YW4X
#include "time_zone_fn.h"
#include "sys_status.h"

struct systemStatus_structure sysStatus;

// This is the maximum amount of time to allow for connecting to cloud. If this time is
// exceeded, do a deep power down. This should not be less than 10 minutes. 11 minutes
// is a reasonable value to use.
unsigned long connectMaxTimeSec = 11 * 60;   // Timeout for trying to connect to Particle cloud in seconds

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
FuelGauge fuel;                                     // Enable the fuel gauge API     
MMA8452Q accel;                                     // Default constructor, SA0 pin is HIGH

// For monitoring / debugging, you can uncomment the next line
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE};
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Battery Conect variables
// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};

// Pin Constants - Boron Carrier Board v1.2a
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int intPin =        D2;                      // Accelerometer Interrupt Pin - I2

// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 30000;            // How long will we wait for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
char currentOffsetStr[10];                          // What is our offset from UTC
unsigned long lastReportedTime = 0;                 // Need to keep this separate from time so we know when to report
unsigned long connectionStartTime;
unsigned long timeSinceILastCheckedForATap;         // Temp for polling 


// Program Variables
volatile bool watchdogFlag;                         // Flag to let us know we need to pet the dog
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char lowPowerModeStr[16];                           // In low power mode?
char openTimeStr[8]="NA";                           // Park Open Time
char closeTimeStr[8]="NA";                          // Park close Time
char debounceStr[8]="NA";
char sensitivityStr[8]="NA";
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write
bool currentCountsWriteNeeded = false;
bool particleConnectionNeeded = false;              // Need to connect to Particle

// These variables are associated with the watchdog timer and will need to be better integrated
int outOfMemory = -1;
time_t RTCTime;

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Pressure Sensor Variables
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered

Timer countSignalTimer(1000, countSignalTimerISR, true);  // This is how we will ensure the BlueLED stays on long enough for folks to see it.

void setup()                                        // Note: Disconnected Setup()
{
  delay(2000);
  Log.info("Starting Setup");
  /* Setup is run for three reasons once we deploy a sensor:
       1) When you deploy the sensor
       2) Each hour while the device is sleeping
       3) After a reset event
    All three of these have some common code - this will go first then we will set a conditional
    to determine which of the three we are in and finish the code
  */
  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  
  // Pressure / PIR Module Pin Setup
  pinMode(intPin,INPUT);                            // sensor interrupt
  
  digitalWrite(blueLED,HIGH);                       // Turn on the led so we can see how long the Setup() takes

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", current.hourlyCount);                // Define my Particle variables
  Particle.variable("DailyCount", current.dailyCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature",current.temperature);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("OpenTime", openTimeStr);
  Particle.variable("CloseTime",closeTimeStr);
  Particle.variable("Alerts",current.alertCount);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextMessage);
  Particle.variable("Sensitivity", sensitivityStr);
  Particle.variable("Debounce", debounceStr);

  Particle.function("setDailyCount", setDailyCount);                          // These are the functions exposed to the mobile app and console
  Particle.function("resetCounts",resetCounts);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);
  Particle.function("Set-Sensitivity", setSensitivity);
  Particle.function("Set-Debounce", setDebounceSec);
  
  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));  // Don't disconnect abruptly

  // Load FRAM and reset variables to their correct values
  fram.begin();                                                       // Initialize the FRAM module

  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                             // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                     // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                   // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                         // See if this worked
    if (tempVersion != FRAMversionNumber) {
      state = ERROR_STATE;                                            // Device will not work without FRAM
      Log.info("FRAM Test Error");
    }
    else loadSystemDefaults();                                        // Out of the box, we need the device to be awake and connected
  }
  else {
    Log.info("loading FRAM values");
    fram.get(FRAM::systemStatusAddr,sysStatus);                       // Loads the System Status array from FRAM
    fram.get(FRAM::currentCountsAddr,current);                        // Loead the current values array from FRAM
  }

  checkSystemValues();                                                // Make sure System values are all in valid range

  makeUpParkHourStrings();                                            // Create the strings for the console

  // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.
  System.on(out_of_memory, outOfMemoryHandler);

  // Here is where we setup the Watchdog timer and Real Time Clock
  ab1805.withFOUT(D8).setup();                                        // The carrier board has D8 connected to FOUT for wake interrupts
  sysStatus.clockSet = ab1805.isRTCSet();                             // Note whether the RTC is set 
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);                        // Enable watchdog

  // Next we set the timezone and check is we are in daylight savings time
  Time.setDSTOffset(sysStatus.dstOffset);                             // Set the value from FRAM if in limits
  DSTRULES() ? Time.beginDST() : Time.endDST();                       // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                      // Set the Time Zone for our device
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string

  // Initialize the accelerometer with begin():
	// begin can take two parameters: full-scale range, and output data rate (ODR).
	// Full-scale range can be: SCALE_2G, SCALE_4G, or SCALE_8G (2, 4, or 8g)
	// ODR can be: ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_12, ODR_6 or ODR_1
  accel.begin(SCALE_2G, ODR_100); // Set up accel with +/-2g range, and 100Hz ODR

  accel.setupTapIntsPulse(sysStatus.sensitivity);                          // Initialize the accelerometer

  countSignalTimer.changePeriod(sysStatus.debounceSec*1000);           // This keeps the device awake during debounce

  (sysStatus.lowPowerMode) ? strncpy(lowPowerModeStr,"Low Power",sizeof(lowPowerModeStr)) : strncpy(lowPowerModeStr,"Not Low Power",sizeof(lowPowerModeStr));

  if (System.resetReason() == RESET_REASON_PIN_RESET) {
    Log.info("Restarted due to a pin reset");
    sysStatus.resetCount++;
    systemStatusWriteNeeded = true;                                    // If so, store incremented number - watchdog must have done This    
  }
  else if (System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    Log.info("Restarted due to a user reset");
    sysStatus.resetCount++;
    systemStatusWriteNeeded = true;                                    // If so, store incremented number - watchdog must have done This
  }

  // Done with the System Stuff - now we will focus on the current counts values
  if (current.hourlyCount) lastReportedTime = current.lastCountTime;
  else lastReportedTime = Time.now();                                  // Initialize it to now so that reporting can begin as soon as the hour changes

  setPowerConfig();                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  if (!digitalRead(userSwitch)) loadSystemDefaults();                  // Make sure the device wakes up and connects

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over  
  if (Time.day() != Time.day(current.lastCountTime)) {                 // Check to see if the device was last on in a different day
    resetEverything();                                                 // Zero the counts for the new day
  }

  if ((Time.hour() >= sysStatus.openTime) && (Time.hour() < sysStatus.closeTime)) { // Park is open let's get ready for the day                                                            
    attachInterrupt(intPin, sensorISR, RISING);                       // Pressure Sensor interrupt from low to high
    if (sysStatus.connectedStatus && !Particle.connected()) {         // If the system thinks we are connected, let's make sure that we are
      particleConnectionNeeded = true;                                // This may happen if there was an unexpected reset during park open hours
      sysStatus.connectedStatus = false;                              // We will fix this.
    }
    takeMeasurements();                                               // Populates values so you can read them before the hour
    stayAwake = stayAwakeLong;                                        // Keeps Boron awake after reboot - helps with recovery
  }

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;              // IDLE unless otherwise from above code

  Log.info("Startup Complete");
  digitalWrite(blueLED,LOW);                                          // Signal the end of startup
}


void loop()
{
  switch(state) {
  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (state != oldState) publishStateTransition();
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;         // When in low power mode, we can nap between taps
    if (Time.hour() != Time.hour(lastReportedTime)) state = REPORTING_STATE;                                  // We want to report on the hour but not after bedtime
    if ((Time.hour() >= sysStatus.closeTime) || (Time.hour() < sysStatus.openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    if (particleConnectionNeeded) state = CONNECTING_STATE;                                                   // Someone raised the connection neeeded flag - will return to IDLE once attempt is completed

    break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    if (state != oldState) publishStateTransition();
    detachInterrupt(intPin);                                          // Done sensing for the day
    if (current.hourlyCount) {                                        // If this number is not zero then we need to send this last count
      state = REPORTING_STATE;
      break;
    }
    if (sysStatus.connectedStatus) disconnectFromParticle();          // Disconnect cleanly from Particle
    ab1805.stopWDT();                                                 // No watchdogs interrupting our slumber
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .duration(wakeInSeconds * 1000);
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device reboots from here   
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    if (result.wakeupReason() == SystemSleepWakeupReason::BY_GPIO) {   // Awoken by GPIO pin
      if (result.wakeupPin() == intPin) {                              // Executions starts here after sleep - time or sensor interrupt?
        stayAwakeTimeStamp = millis();
      }
      else if (result.wakeupPin() == userSwitch) {
        setLowPowerMode("0");
        sysStatus.openTime = 0;
        sysStatus.closeTime = 24;
      }
    }

    if (Time.hour() < sysStatus.closeTime && Time.hour() >= sysStatus.openTime) { // We might wake up and find it is opening time.  Park is open let's get ready for the day
      attachInterrupt(intPin, sensorISR, RISING);                      // Pressure Sensor interrupt from low to high
      stayAwake = stayAwakeLong;                                       // Keeps Boron awake after deep sleep - may not be needed
    }
    state = IDLE_STATE;                                                // Head back to the idle state to see what to do next
    } break;

  case NAPPING_STATE: {  // This state puts the device in low power mode quickly
    if (state != oldState) publishStateTransition();
    if (sensorDetect || countSignalTimer.isActive()) break;           // Don't nap until we are done with event
    if (sysStatus.connectedStatus) disconnectFromParticle();          // If we are in connected mode we need to Disconnect from Particle
    stayAwake = 1000;                                                 // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    state = IDLE_STATE;                                               // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .gpio(intPin,RISING)
      .duration(60 * 1000);                                            // Only nap for one minute so we can check for a stuck interrupt
    ab1805.stopWDT();
    if (pinReadFast(intPin)) recordCount();
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep
    ab1805.resumeWDT();
    if (result.wakeupReason() == SystemSleepWakeupReason::BY_GPIO) {   // Awoken by GPIO pin
      if (result.wakeupPin() == intPin) {                              // Executions starts here after sleep - time or sensor interrupt?
        stayAwakeTimeStamp = millis();
      }
      else if (result.wakeupPin() == userSwitch) setLowPowerMode("0");
    }
    } break;

  case CONNECTING_STATE:{
    static unsigned long connectionStartTime;
    char connectionStr[32];
    static bool returnToReporting;

    if (state != oldState) {
      if (oldState == REPORTING_STATE) returnToReporting = true;
      else returnToReporting = false;                                 // Need to set value each time - just to be clear
      publishStateTransition();
      connectionStartTime = Time.now();                 // Start the clock first time we enter the state
      Cellular.on();                                                  // Needed until they fix this: https://github.com/particle-iot/device-os/issues/1631
      Particle.connect();                                             // Told the Particle to connect, now we need to wait
    }

    if (Particle.connected()) {
      particleConnectionNeeded = false;                               // Connected so we don't need this flag
      sysStatus.connectedStatus = true;
      sysStatus.lastConnection = Time.now();                          // This is the last time we attempted to connect
    }
    else if ((Time.now() - connectionStartTime) > connectMaxTimeSec) {
      particleConnectionNeeded = false;                               // Timed out so we will give up until the next hour
      if ((Time.now() - sysStatus.lastConnection) > 7200) {             // Only sends to ERROR_STATE if it has been over 2 hours
        state = ERROR_STATE;     
        resetTimeStamp = millis();
        break;
      }
      sysStatus.connectedStatus = false;
      Log.info("cloud connection unsuccessful");
    } 

    if (!particleConnectionNeeded) {                                  // Whether the connection was successful or not, we will collect and publish metrics
      sysStatus.lastConnectionDuration = Time.now() - connectionStartTime;
      if (sysStatus.lastConnectionDuration > connectMaxTimeSec) sysStatus.lastConnectionDuration = connectMaxTimeSec;           // This is clearly an erroneous result
      if (sysStatus.lastConnectionDuration > current.maxConnectTime) current.maxConnectTime = sysStatus.lastConnectionDuration; // Keep track of longest each day
      snprintf(connectionStr, sizeof(connectionStr),"Connected in %i secs",sysStatus.lastConnectionDuration);                   // Make up connection string and publish
      Log.info(connectionStr);
      if (sysStatus.verboseMode) publishQueue.publish("Cellular",connectionStr,PRIVATE);
      systemStatusWriteNeeded = true;
      currentCountsWriteNeeded = true;
      if (sysStatus.connectedStatus && returnToReporting) state = REPORTING_STATE;    // If we came here from reporting, this will send us back
      else state = IDLE_STATE;                                             // We are connected so, we can go to the IDLE state
    }
  } break;

  case REPORTING_STATE:
    if (state != oldState) publishStateTransition();

    lastReportedTime = Time.now();                                    // We are only going to try once

    if (!sysStatus.connectedStatus) {                                 // Asking us to report but not connected
      particleConnectionNeeded = true;                                // Set the flag to connect us to Particle
      state = CONNECTING_STATE;                                       // Will send us to connecting state - and it will send us back here                                             
      break;
    }

    if (!sysStatus.lowPowerMode) takeMeasurements();                  // Do this here as device not in lowPowerMode will not meausre coming out of sleep / napping

    if (sysStatus.connectedStatus) {
      if (Time.hour() == sysStatus.openTime) dailyCleanup();          // Once a day, clean house and publish to Google Sheets
      else sendEvent();                                               // Send data to Ubidots but not at opening time as there is nothing to publish
      if (Time.hour() == sysStatus.openTime && sysStatus.openTime==0) sendEvent();    // Need this so we can get 24 hour reporting for non-sleeping devices
      webhookTimeStamp = millis();                                    // This is for a webHook response timeout
      state = RESP_WAIT_STATE;                                        // Wait for Response
    }
    else {                                                            // In this case, the connection failed.  We will therefore skip this reporting period and go back to IDLE - try again next hour
      stayAwake = stayAwakeLong;                                      // Keeps device awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
      state = IDLE_STATE;
    }
    break;

  case RESP_WAIT_STATE:
    if (state != oldState) publishStateTransition();
    if (!dataInFlight)  {                                             // Response received --> back to IDLE state
      stayAwake = stayAwakeLong;                                      // Keeps device awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
      state = IDLE_STATE;

      if (current.hourlyCountInFlight) {                                // Cleared here as there could be counts coming in while "in Flight"
        current.hourlyCount -= current.hourlyCountInFlight;             // Confirmed that count was recevied - clearing
        current.hourlyCountInFlight = current.maxMinValue = current.alertCount = 0; // Zero out the counts until next reporting period
        currentCountsWriteNeeded=true;
      }
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      state = ERROR_STATE;                                            // Response timed out
    }
    break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait) {

      // The first two conditions imply that there is a connectivity issue - reset the modem
      if ((Time.now() - sysStatus.lastConnection) > 7200L) {           // It is been over two hours since we last connected to the cloud - time for a reset
        sysStatus.lastConnection = Time.now() - 3600;                 // Wait an hour before we come back to this condition
        fram.put(FRAM::systemStatusAddr,sysStatus);
        Log.error("failed to connect to cloud, doing deep reset");
        delay(100);
        fullModemReset();                                             // Full Modem reset and reboot
      }
      else if (Time.now() - sysStatus.lastHookResponse > 7200L) {     //It has been more than two hours since a sucessful hook response
        if (sysStatus.connectedStatus) publishQueue.publish("State","Error State - Full Modem Reset", PRIVATE, WITH_ACK);  // Broadcast Reset Action
        delay(2000);                                                  // Time to publish
        sysStatus.resetCount = 0;                                     // Zero the ResetCount
        sysStatus.lastHookResponse = Time.now() - 3600;               // Give it an hour before we act on this condition again
        systemStatusWriteNeeded=true;
        fullModemReset();                                             // Full Modem reset and reboot
      }
      // The next two are more general so a simple reset is all you need
      else if (sysStatus.resetCount <= 3) {                                // First try simple reset
        if (sysStatus.connectedStatus) publishQueue.publish("State","Error State - System Reset", PRIVATE, WITH_ACK);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (sysStatus.connectedStatus) publishQueue.publish("State","Error State - Full Modem Reset", PRIVATE, WITH_ACK);            // Brodcase Reset Action
        delay(2000);
        sysStatus.resetCount = 0;                                     // Zero the ResetCount
        fram.put(FRAM::systemStatusAddr,sysStatus);                   // Won't get back to the main loop
        ab1805.deepPowerDown();                                       // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
      }
    }
    break;      
  }
  // Take care of housekeeping items here

  if (sensorDetect) {
    recordCount();
  }

  ab1805.loop();                                                      // Keeps the RTC synchronized with the Boron's clock

  if (systemStatusWriteNeeded) {
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentCountsWriteNeeded) {
    fram.put(FRAM::currentCountsAddr,current);
    currentCountsWriteNeeded = false;
  }

  if (outOfMemory >= 0) {                                               // In this function we are going to reset the system if there is an out of memory error
    char message[64];
    snprintf(message, sizeof(message), "Out of memory occurred size=%d",outOfMemory);
    Log.info(message);
    delay(100);
    publishQueue.publish("Memory",message,PRIVATE);                     // Publish to the console - this is important so we will not filter on verboseMod
    delay(2000);
    System.reset();                                                     // An out of memory condition occurred - reset device.
  }
}


void recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the device
{
  static byte currentMinutePeriod;                                      // Current minute
  static unsigned long lastTapTime;                                     // When did we last record a count?
  char data[64];                                                        // Store the date in this character array - not global

  if (sensorDetect) {
    detachInterrupt(intPin);
    Log.info("Cleared Interrupt");
    sensorDetect = false;                                               // Reset the flag
    delay(1000);                                                        // Reset as there can be "ringing"
    attachInterrupt(intPin, sensorISR, RISING);                         // Sensor interrupt from low to high
  }

  if (Time.now() - lastTapTime > sysStatus.debounceSec) {
    lastTapTime = Time.now();

    countSignalTimer.reset();                                           // Keep the LED on for a set time so we can see it.

    if (currentMinutePeriod != Time.minute()) {                         // Done counting for the last minute
      currentMinutePeriod = Time.minute();                              // Reset period
      current.maxMinValue = 1;                                          // Reset for the new minute
    }
    current.maxMinValue++;

    current.lastCountTime = Time.now();
    current.hourlyCount++;                                              // Increment the PersonCount
    current.dailyCount++;                                               // Increment the PersonCount
    snprintf(data, sizeof(data), "Count, hourly: %i, daily: %i",current.hourlyCount,current.dailyCount);
    
    if (sysStatus.verboseMode && sysStatus.connectedStatus) publishQueue.publish("Count",data, PRIVATE);                      // Helpful for monitoring and calibration
    Log.info(data);
    currentCountsWriteNeeded = true;                                    // Write updated values to FRAM
  }
  else if (!countSignalTimer.isActive()) pinResetFast(blueLED);
}


void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  unsigned long timeStampValue;                                       // Going to start sending timestamps - and will modify for midnight to fix reporting issue
  if (current.hourlyCount) {
    timeStampValue = current.lastCountTime;                           // If there was an event in the past hour, send the most recent event's timestamp
  }
  else {                                                              // If there were no events in the past hour/recording period, send the time when the last report was sent
    timeStampValue = lastReportedTime;                                // This should be the beginning of the previous hour
  }
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i,\"key1\":\"%s\",\"temp\":%i, \"resets\":%i, \"alerts\":%i,\"maxmin\":%i,\"connecttime\":%i,\"timestamp\":%lu000}",current.hourlyCount, current.dailyCount, sysStatus.stateOfCharge, batteryContext[sysStatus.batteryState], current.temperature, sysStatus.resetCount, current.alertCount, current.maxMinValue, sysStatus.lastConnectionDuration, timeStampValue);
  publishQueue.publish("Ubidots-Counter-Hook-v1", data, PRIVATE);
  dataInFlight = true;                                                // set the data inflight flag
  current.hourlyCountInFlight = current.hourlyCount;                  // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
}


void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
    sysStatus.lastHookResponse = Time.now();                          // Record the last successful Webhook Response
    systemStatusWriteNeeded = true;
    dataInFlight = false;                                             // Data has been received
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  if (sysStatus.verboseMode) publishQueue.publish("Ubidots Hook", responseString, PRIVATE, WITH_ACK);
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready

  getTemperature();                                                   // Get Temperature at startup as well
  
  // Battery Releated actions
  sysStatus.batteryState = System.batteryState();                     // Call before isItSafeToCharge() as it may overwrite the context
  if (!isItSafeToCharge()) current.alertCount++;                      // Increment the alert count
  sysStatus.stateOfCharge = int(System.batteryCharge());              // Percentage of full charge
  if (sysStatus.stateOfCharge < 65 && sysStatus.batteryState == 1) {
    System.setPowerConfiguration(SystemPowerConfiguration());         // Reset the PMIC
  }
  if (sysStatus.stateOfCharge < current.minBatteryLevel) current.minBatteryLevel = sysStatus.stateOfCharge; // Keep track of lowest value for the day
  if (sysStatus.stateOfCharge < 30) sysStatus.lowBatteryMode = true;  // Check to see if we are in low battery territory
  else sysStatus.lowBatteryMode = false;                              // We have sufficient to continue operations

  systemStatusWriteNeeded = true;
  currentCountsWriteNeeded = true;
}

bool isItSafeToCharge()                                               // Returns a true or false if the battery is in a safe charging range.  
{         
  PMIC pmic(true);                                 
  if (current.temperature < 36 || current.temperature > 100 )  {      // Reference: https://batteryuniversity.com/learn/article/charging_at_high_and_low_temperatures (32 to 113 but with safety)
    pmic.disableCharging();                                           // It is too cold or too hot to safely charge the battery
    sysStatus.batteryState = 1;                                       // Overwrites the values from the batteryState API to reflect that we are "Not Charging"
    return false;
  }
  else {
    pmic.enableCharging();                                            // It is safe to charge the battery
    return true;
  }
}

void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature() {                                                // Get temperature and make sure we are not getting a spurrious value

  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  if (reading < 400) {                                                // This ocrresponds to 0 degrees - less than this and we should take another reading to be sure
    delay(50);
    reading = analogRead(tmp36Pin);
  }
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  currentCountsWriteNeeded=true;
  return current.temperature;
}


// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}


void sensorISR()
{
  sensorDetect = true;                                              // sets the sensor flag for the main loop
  pinSetFast(blueLED);                                                // Turn on the blue LED
}

void countSignalTimerISR() {
  digitalWrite(blueLED,LOW);
}


// Power Management function
int setPowerConfig() {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());  // To restore the default configuration
  if (sysStatus.solarPowerMode) {
    conf.powerSourceMaxCurrent(900) // Set maximum current the power source can provide (applies only when powered through VIN)
        .powerSourceMinVoltage(5080) // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(1024) // Set battery charge current
        .batteryChargeVoltage(4208) // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST); // For the cases where the device is powered through VIN
                                                                     // but the USB cable is connected to a USB host, this feature flag
                                                                     // enforces the voltage/current limits specified in the configuration
                                                                     // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
  else  {
    conf.powerSourceMaxCurrent(900)                                   // default is 900mA 
        .powerSourceMinVoltage(4208)                                  // This is the default value for the Boron
        .batteryChargeCurrent(900)                                    // higher charge current from DC-IN when not solar powered
        .batteryChargeVoltage(4112)                                   // default is 4.112V termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
    int res = System.setPowerConfiguration(conf); // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
}


void loadSystemDefaults() {                                         // Default settings for the device - connected, not-low power and always on
  particleConnectionNeeded = true;                                  // Get connected to Particle - sets sysStatus.connectedStatus to true
  if (sysStatus.connectedStatus) publishQueue.publish("Mode","Loading System Defaults", PRIVATE, WITH_ACK);
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = false;
  sysStatus.clockSet = false;
  sysStatus.lowBatteryMode = false;
  setLowPowerMode("0");
  sysStatus.timezone = -5;                                          // Default is East Coast Time
  sysStatus.dstOffset = 1;
  sysStatus.openTime = 6;
  sysStatus.closeTime = 21;
  sysStatus.solarPowerMode = true;  
  sysStatus.lastConnectionDuration = 0;                             // New measure
  fram.put(FRAM::systemStatusAddr,sysStatus);                       // Write it now since this is a big deal and I don't want values over written
}

 /**
  * @brief This function checks to make sure all values that we pull from FRAM are in bounds
  * 
  * @details As new devices are comissioned or the sysStatus structure is changed, we need to make sure that values are 
  * in bounds so they do not cause unpredectable execution.
  * 
  */
void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range 
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.openTime < 0 || sysStatus.openTime > 12) {
    Log.info("openTime value of %i resetting to default",sysStatus.openTime);
    sysStatus.openTime = 0; 
  } 
  if (sysStatus.closeTime < 12 || sysStatus.closeTime > 24) sysStatus.closeTime = 24;
  if (sysStatus.lastConnectionDuration < 0 || sysStatus.lastConnectionDuration > connectMaxTimeSec) sysStatus.lastConnectionDuration = 0;
  sysStatus.solarPowerMode = true;                                  // Need to reset this value across the fleet

  if (current.maxConnectTime > connectMaxTimeSec) {
    current.maxConnectTime = 0;
    currentCountsWriteNeeded = true;
  }
  // None for lastHookResponse
  systemStatusWriteNeeded = true;
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

 /**
  * @brief Simple Function to construct a string for the Open and Close Time
  * 
  * @details Looks at the open and close time and makes them into time strings.  Also looks at the special case of open 24 hours
  * and puts in an "NA" for both strings when this is the case.
  * 
  */
void makeUpParkHourStrings() {
  if (sysStatus.openTime == 0 && sysStatus.closeTime == 24) {
    snprintf(openTimeStr, sizeof(openTimeStr), "NA");
    snprintf(closeTimeStr, sizeof(closeTimeStr), "NA");
    return;
  }
  snprintf(sensitivityStr, sizeof(sensitivityStr), "%i", sysStatus.sensitivity);
  snprintf(debounceStr, sizeof(debounceStr), "%i sec", sysStatus.debounceSec);
    
  snprintf(openTimeStr, sizeof(openTimeStr), "%i:00", sysStatus.openTime);
  snprintf(closeTimeStr, sizeof(closeTimeStr), "%i:00", sysStatus.closeTime);
  return;
}

bool disconnectFromParticle()                                     // Ensures we disconnect cleanly from Particle
                                                                  // Updated based onthis thread: https://community.particle.io/t/waitfor-particle-connected-timeout-does-not-time-out/59181
{
  Particle.disconnect();
  waitForNot(Particle.connected, 15000);                          // make sure before turning off the cellular modem
  Cellular.disconnect();                                          // Disconnect from the cellular network
  Cellular.off();                                                 // Turn off the cellular modem
  waitFor(Cellular.isOff, 30000);                                 // As per TAN004: https://support.particle.io/hc/en-us/articles/1260802113569-TAN004-Power-off-Recommendations-for-SARA-R410M-Equipped-Devices
  sysStatus.connectedStatus = false;
  systemStatusWriteNeeded = true;
  return true;
}

int resetCounts(String command)                                   // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    current.dailyCount = 0;                                           // Reset Daily Count in memory
    current.hourlyCount = 0;                                          // Reset Hourly Count in memory
    sysStatus.resetCount = 0;                                            // If so, store incremented number - watchdog must have done This
    current.alertCount = 0;                                           // Reset count variables
    current.hourlyCountInFlight = 0;                                  // In the off-chance there is data in flight
    dataInFlight = false;
    currentCountsWriteNeeded = true;                                  // Make sure we write to FRAM back in the main loop
    systemStatusWriteNeeded = true;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    publishQueue.publish("Reset","Hard Reset in 2 seconds",PRIVATE);
    ab1805.deepPowerDown(10);
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

/**
 * @brief Resets all counts to start a new day.
 * 
 * @details Once run, it will reset all daily-specific counts and trigger an update in FRAM.
 */
void resetEverything() {                                              // The device is waking up in a new day or is a new install
  current.dailyCount = 0;                                             // Reset the counts in FRAM as well
  current.hourlyCount = 0;
  current.hourlyCountInFlight = 0;
  current.lastCountTime = Time.now();                                 // Set the time context to the new day
  sysStatus.resetCount = current.alertCount = 0;                      // Reset everything for the day
  current.maxConnectTime = 0;                                         // Reset values for this time period
  current.minBatteryLevel = 100;
  currentCountsWriteNeeded = true;

  currentCountsWriteNeeded=true;                                      // Make sure that the values are updated in FRAM
  systemStatusWriteNeeded=true;
  //lastReportedTime = Time.now();
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    setPowerConfig();                                               // Change the power management Settings
    systemStatusWriteNeeded=true;
    if (sysStatus.connectedStatus) publishQueue.publish("Mode","Set Solar Powered Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    systemStatusWriteNeeded=true;
    setPowerConfig();                                                // Change the power management settings
    if (sysStatus.connectedStatus) publishQueue.publish("Mode","Cleared Solar Powered Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else return 0;
}


/**
 * @brief Turns on/off verbose mode.
 * 
 * @details Extracts the integer command. Turns on verbose mode if the command is "1" and turns
 * off verbose mode if the command is "0".
 *
 * @param command A string with the integer command indicating to turn on or off verbose mode.
 * Only values of "0" or "1" are accepted. Values outside this range will cause the function
 * to return 0 to indicate an invalid entry.
 * 
 * @return 1 if successful, 0 if invalid command
 */
int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    systemStatusWriteNeeded = true;
    if (sysStatus.connectedStatus) publishQueue.publish("Mode","Set Verbose Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    systemStatusWriteNeeded = true;
    if (sysStatus.connectedStatus) publishQueue.publish("Mode","Cleared Verbose Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else return 0;
}

/**
 * @brief Returns a string describing the battery state.
 * 
 * @return String describing battery state.
 */
String batteryContextMessage() {
  return batteryContext[sysStatus.batteryState];
}

/**
 * @brief Sets the closing time of the facility.
 * 
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                                    // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                            // Make sure it falls in a valid range or send a "fail" result
  sysStatus.openTime = tempTime;
  makeUpParkHourStrings();                                                    // Create the strings for the console
  systemStatusWriteNeeded = true;                                            // Need to store to FRAM back in the main loop
  if (sysStatus.connectedStatus) {
    snprintf(data, sizeof(data), "Open time set to %i",sysStatus.openTime);
    publishQueue.publish("Time",data, PRIVATE, WITH_ACK);
  }
  return 1;
}

/**
 * @brief Sets the closing time of the facility.
 * 
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.closeTime = tempTime;
  makeUpParkHourStrings();                                                    // Create the strings for the console
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",sysStatus.closeTime);
  if (sysStatus.connectedStatus) publishQueue.publish("Time",data, PRIVATE, WITH_ACK);
  return 1;
}

/**
 * @brief Sets the daily count for the park - useful when you are replacing sensors.
 * 
 * @details Since the hourly counts are not retained after posting to Ubidots, seeding a value for
 * the daily counts will enable us to follow this process to replace an old counter: 1) Execute the "send now"
 * command on the old sensor.  Note the daily count.  2) Install the new sensor and perform tests to ensure
 * it is counting correclty.  3) Use this function to set the daily count to the right value and put the 
 * new device into operation.
 *
 * @param command A string for the new daily count.  
 * Inputs outside of "0" - "1000" will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setDailyCount(String command)
{
  char * pEND;
  char data[256];
  int tempCount = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempCount < 0) || (tempCount > 1000)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  current.dailyCount = tempCount;
  current.lastCountTime = Time.now();
  currentCountsWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Daily count set to %i",current.dailyCount);
  if (sysStatus.connectedStatus) publishQueue.publish("Daily",data, PRIVATE, WITH_ACK);
  return 1;
}

/**
 * @brief Sets the sensitivity of the detector.
 * 
 * @details Extracts the integer from the string passed in and re-initializes the accelerometer with the sensitivity value passed
 *
 * @param Looking for a sensitivity level from 0 - not sensitive to 10 - very sensitive
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setSensitivity(String command)
{
  char * pEND;
  char data[256];
  int tempValue = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempValue < 0) || (tempValue > 10)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.sensitivity = tempValue;
    accel.setupTapIntsPulse(sysStatus.sensitivity);                           // Initialize the accelerometer
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(sensitivityStr, sizeof(sensitivityStr), "%i", sysStatus.sensitivity);
  snprintf(data, sizeof(data), "Sensitivity set to %i",sysStatus.sensitivity);
  if (sysStatus.connectedStatus) publishQueue.publish("Time",data, PRIVATE, WITH_ACK);
  return 1;
}

/**
 * @brief Sets the debounce delay between "Taps".
 * 
 * @details Extracts the integer from the string passed in and updates the debounce value
 *
 * @param Looking for a value from 0 to 60 second.  Sets the system value and changes the period of the stay awake timer
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setDebounceSec(String command)
{
  char * pEND;
  char data[256];
  int tempValue = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempValue < 0) || (tempValue > 60)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.debounceSec = tempValue;
  countSignalTimer.changePeriod(sysStatus.debounceSec*1000);           // This keeps the device awake during debounce
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(debounceStr, sizeof(debounceStr), "%i sec", sysStatus.debounceSec);
  snprintf(data, sizeof(data), "Debounce set to %i seconds",sysStatus.debounceSec);
  if (sysStatus.connectedStatus) publishQueue.publish("Time",data, PRIVATE, WITH_ACK);
  return 1;
}

/**
 * @brief Toggles the device into low power mode based on the input command.
 * 
 * @details If the command is "1", sets the device into low power mode. If the command is "0",
 * sets the device into normal mode. Fails if neither of these are the inputs.
 *
 * @param command A string indicating whether to set the device into low power mode or into normal mode.
 * A "1" indicates low power mode, a "0" indicates normal mode. Inputs that are neither of these commands
 * will cause the function to return 0 to indicate an invalid entry.
 * 
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (sysStatus.connectedStatus) {
      publishQueue.publish("Mode","Low Power Mode", PRIVATE, WITH_ACK);
    }
    sysStatus.lowPowerMode = true;
    strncpy(lowPowerModeStr,"Low Power", sizeof(lowPowerModeStr));
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (!sysStatus.connectedStatus) {                                      // In case we are not connected, we will do so now.
      particleConnectionNeeded = true;
    }
    publishQueue.publish("Mode","Normal Operations", PRIVATE, WITH_ACK);
    delay(1000);                                                      // Need to make sure the message gets out.
    sysStatus.lowPowerMode = false;                                   // update the variable used for console status
    strncpy(lowPowerModeStr,"Not Low Power", sizeof(lowPowerModeStr));        // Use capitalization so we know that we set this.
  }
  systemStatusWriteNeeded = true;
  return 1;
}

/**
 * @brief Publishes a state transition to the Log Handler and to the Particle monitoring system.
 * 
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if (sysStatus.verboseMode && sysStatus.connectedStatus) publishQueue.publish("State Transition",stateTransitionString, PRIVATE, WITH_ACK);
  Log.info(stateTransitionString);
}

/**
 * @brief Fully resets modem.
 * 
 * @details Disconnects from the cloud, resets modem and SIM, and deep sleeps for 10 seconds.
 * Adapted form Rikkas7's https://github.com/rickkas7/electronsample.
 */
/**
 * @brief Fully resets modem.
 * 
 * @details Disconnects from the cloud, resets modem and SIM, and deep sleeps for 10 seconds.
 * Adapted form Rikkas7's https://github.com/rickkas7/electronsample.
 */
void fullModemReset() {  // 
	Particle.disconnect(); 	                                          // Disconnect from the cloud    
	waitFor(Particle.connected, 15000);                               // Wait up to 15 seconds to disconnect
	// Reset the modem and SIM card
  Cellular.off();                                                   // Turn off the Cellular modem
  waitFor(Cellular.isOff, 30000);                                   // New feature with deviceOS@2.1.0

  ab1805.stopWDT();                                                 // No watchdogs interrupting our slumber
                                             
  config.mode(SystemSleepMode::ULTRA_LOW_POWER)
    .gpio(userSwitch,CHANGE)
    .duration(10 * 1000);


  System.sleep(config);                                             // Put the device to sleep device reboots from here   
  ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
}


/**
 * @brief Cleanup function that is run at the beginning of the day.
 * 
 * @details Syncs time with remote service and sets low power mode. Called from Reporting State ONLY.
 * Clean house at the end of the day
 */
void dailyCleanup() {
  publishQueue.publish("Daily Cleanup","Running", PRIVATE, WITH_ACK);            // Make sure this is being run
  sysStatus.verboseMode = false;
  Particle.syncTime();                                                 // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                                // Wait for up to 30 seconds for the SyncTime to complete
  if (sysStatus.solarPowerMode || sysStatus.stateOfCharge <= 70) {     // If Solar or if the battery is being discharged
    setLowPowerMode("1");
  }

  resetEverything();                                               // If so, we need to Zero the counts for the new day

  systemStatusWriteNeeded = true;
}
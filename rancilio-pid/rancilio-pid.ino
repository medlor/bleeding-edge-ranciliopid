/********************************************************
 * BLEEDING EDGE FORK OF RANCILIO-PID.
 *
 * This enhancement implementation is initially based on 
 * rancilio-pid (http://rancilio-pid.de/) but was greatly
 * enhanced. 
 * In case of questions just contact, Tobias <medlor@web.de>
 *****************************************************/

#include <Arduino.h>

//Libraries for OTA
#include <ArduinoOTA.h>
#ifdef ESP32
#include <WiFi.h>
#include <Preferences.h>
Preferences preferences;
#else
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#endif

#include <WiFiUdp.h>
#include <math.h>
#include <float.h>

#include "rancilio-pid.h"

#include "display.h"
#include "MQTT.h"


RemoteDebug Debug;

const char* sysVersion PROGMEM  = "2.7.0 beta 3";

/********************************************************
  definitions below must be changed in the userConfig.h file
******************************************************/
const int OnlyPID = ONLYPID;
const int TempSensor = TEMPSENSOR;
const int TempSensorRecovery = TEMPSENSORRECOVERY;
const int brewDetection = BREWDETECTION;
const int triggerType = TRIGGERTYPE;
const bool ota = OTA;
const int grafana=GRAFANA;

// Wifi
const char* hostname = HOSTNAME;
const char* ssid = D_SSID;
const char* pass = PASS;

unsigned long lastWifiConnectionAttempt = millis();
const unsigned long wifiReconnectInterval = 60000; // try to reconnect every 60 seconds
unsigned long wifiConnectWaitTime = 6000; //ms to wait for the connection to succeed
unsigned int wifiReconnects = 0; //number of reconnects

// OTA
const char* OTApass = OTAPASS;

//Blynk
const char* blynkaddress = BLYNKADDRESS;
const int blynkport = BLYNKPORT;
const char* blynkauth = BLYNKAUTH;
unsigned long blynk_lastReconnectAttemptTime = 0;
unsigned int blynk_reconnectAttempts = 0;
unsigned long blynk_reconnect_incremental_backoff = 180000 ; //Failsafe: add 180sec to reconnect time after each connect-failure.
unsigned int blynk_max_incremental_backoff = 5 ; // At most backoff <mqtt_max_incremenatl_backoff>+1 * (<mqtt_reconnect_incremental_backoff>ms)

WiFiClient espClient;

// MQTT
#if (MQTT_ENABLE==1)
#include "src/PubSubClient/PubSubClient.h"
//#include <PubSubClient.h>  // uncomment this line AND delete src/PubSubClient/ folder, if you want to use system lib
PubSubClient mqtt_client(espClient);
#elif (MQTT_ENABLE==2)
#include "uMQTTBroker.h"
#endif
const int MQTT_MAX_PUBLISH_SIZE = 120; //see https://github.com/knolleary/pubsubclient/blob/master/src/PubSubClient.cpp
const char* mqtt_server_ip = MQTT_SERVER_IP;
const int mqtt_server_port = MQTT_SERVER_PORT;
const char* mqtt_username = MQTT_USERNAME;
const char* mqtt_password = MQTT_PASSWORD;
const char* mqtt_topic_prefix = MQTT_TOPIC_PREFIX;
char topic_will[256];
char topic_set[256];
char topic_actions[256];
unsigned long lastMQTTStatusReportTime = 0;
unsigned long lastMQTTStatusReportInterval = 5000; //mqtt send status-report every 5 second
const bool mqtt_flag_retained = true;
unsigned long mqtt_dontPublishUntilTime = 0;
unsigned long mqtt_dontPublishBackoffTime = 60000; // Failsafe: dont publish if there are errors for 10 seconds
unsigned long mqtt_lastReconnectAttemptTime = 0;
unsigned int mqtt_reconnectAttempts = 0;
unsigned long mqtt_reconnect_incremental_backoff = 210000 ; //Failsafe: add 210sec to reconnect time after each connect-failure.
unsigned int mqtt_max_incremental_backoff = 5 ; // At most backoff <mqtt_max_incremenatl_backoff>+1 * (<mqtt_reconnect_incremental_backoff>ms)
bool mqtt_disabled_temporary = false;
unsigned long mqtt_connectTime = 0;  // time of last successfull mqtt connection

/********************************************************
   Vorab-Konfig
******************************************************/
int pidON = 1 ;             // 1 = control loop in closed loop
int relayON, relayOFF;      // used for relay trigger type. Do not change!
int activeState = 3;        // (0:= undefined / EMERGENCY_TEMP reached)
                            // 1:= Coldstart required (machine is cold)
                            // 2:= Stabilize temperature after coldstart
                            // 3:= (default) Inner Zone detected (temperature near setPoint)
                            // 4:= Brew detected
                            // 5:= Outer Zone detected (temperature outside of "inner zone")
                            // 6:= steam mode activated
                            // (7:= steam ready, TODO?)
bool emergencyStop = false; // Notstop bei zu hoher Temperatur

/********************************************************
   history of temperatures
*****************************************************/
const int numReadings = 75;             // number of values per Array
double readingstemp[numReadings];       // the readings from Temp
float readingstime[numReadings];        // the readings from time
int readIndex = 0;                      // the index of the current reading
int totaltime = 0 ;                     // the running time
unsigned long  lastBrewTime = 0 ;
int timerBrewDetection = 0 ;
int i = 0;

/********************************************************
   PID Variables
*****************************************************/
const unsigned int windowSizeSeconds = 5;            // How often should PID.compute() run? must be >= 1sec
unsigned int windowSize = windowSizeSeconds * 1000;  // 1000=100% heater power => resolution used in TSR() and PID.compute().
volatile unsigned int isrCounter = 0;                // counter for heater ISR
#ifdef ESP32
hw_timer_t * timer = NULL;
#endif
const float heater_overextending_factor = 1.2;
unsigned int heater_overextending_isrCounter = windowSize * heater_overextending_factor;
unsigned long pidComputeLastRunTime = 0;
double Input = 0, Output = 0;
double previousInput = 0;
double previousOutput = 0;
int pidMode = 1;                   //1 = Automatic, 0 = Manual

double setPoint = SETPOINT;
double setPointSteam = SETPOINT_STEAM;
double* activeSetPoint = &setPoint;
double starttemp = STARTTEMP;
double steamReadyTemp = STEAM_READY_TEMP;

// State 1: Coldstart PID values
const int coldStartStep1ActivationOffset = 5;
// ... none ...

// State 2: Coldstart stabilization PID values
// ... none ...

// State 3: Inner Zone PID values
double aggKp = AGGKP;
double aggTn = AGGTN;
double aggTv = AGGTV;
#if (aggTn == 0)
double aggKi = 0;
#else
double aggKi = aggKp / aggTn;
#endif
double aggKd = aggTv * aggKp;

// State 4: Brew PID values
// ... none ...
double brewDetectionPower = BREWDETECTION_POWER;

// State 5: Outer Zone Pid values
double aggoKp = AGGOKP;
double aggoTn = AGGOTN;
double aggoTv = AGGOTV;
#if (aggoTn == 0)
double aggoKi = 0;
#else
double aggoKi = aggoKp / aggoTn;
#endif
double aggoKd = aggoTv * aggoKp ;
const double outerZoneTemperatureDifference = 1;
//const double steamZoneTemperatureDifference = 3;

/********************************************************
   PID with Bias (steadyPower) Temperature Controller
*****************************************************/
#include "PIDBias.h"
double steadyPower = STEADYPOWER; // in percent
double steadyPowerSaved = steadyPower;
double steadyPowerSavedInBlynk = 0;
double steadyPowerMQTTDisableUpdateUntilProcessed = 0;  //used as semaphore
unsigned long steadyPowerMQTTDisableUpdateUntilProcessedTime = 0;
int burstShot      = 0;   // this is 1, when the user wants to immediatly set the heater power to the value specified in burstPower
double burstPower  = 20;  // in percent

const int lastBrewTimeOffset = 4 * 1000;  //compensate for lag in software brew-detection

// If the espresso hardware itself is cold, we need additional power for steadyPower to hold the water temperature
double steadyPowerOffset     = STEADYPOWER_OFFSET;  // heater power (in percent) which should be added to steadyPower during steadyPowerOffsetTime
double steadyPowerOffsetModified = steadyPowerOffset;
int steadyPowerOffsetTime   = STEADYPOWER_OFFSET_TIME;  // timeframe (in ms) for which steadyPowerOffset_Activated should be active
unsigned long steadyPowerOffset_Activated = 0;
unsigned long steadyPowerOffsetDecreaseTimer = 0;
unsigned long lastUpdateSteadyPowerOffset = 0;  //last time steadyPowerOffset was updated
bool MachineColdOnStart = true;
double starttempOffset = 0;  //Increasing this lead to too high temp and emergency measures taking place. For my rancilio it is best to leave this at 0.

PIDBias bPID(&Input, &Output, &steadyPower, &steadyPowerOffsetModified, &steadyPowerOffset_Activated, &steadyPowerOffsetTime, &activeSetPoint, aggKp, aggKi, aggKd);

/********************************************************
   BREWING / PREINFUSSION
******************************************************/
double brewtime          = BREWTIME;
double preinfusion       = PREINFUSION;
double preinfusionpause  = PREINFUSION_PAUSE;
int brewing              = 0;  //Attention: "brewing" must only be changed in brew() (ONLYPID=0) or brewingAction() (ONLYPID=1)!
bool waitingForBrewSwitchOff = false;
int brewState            = 0;
unsigned long totalbrewtime = 0;
unsigned long bezugsZeit = 0;
unsigned long startZeit  = 0;
unsigned long previousBrewCheck = 0;
unsigned long lastBrewMessage   = 0;

/********************************************************
   STEAMING
******************************************************/
int steaming             = 0;
unsigned long lastSteamMessage = 0;

/********************************************************
   Sensor check
******************************************************/
bool sensorError    = false;
int error           = 0;
int maxErrorCounter = 10 ;  //define maximum number of consecutive polls (of intervaltempmes* duration) to have errors

/********************************************************
 * Rest
 *****************************************************/
unsigned long userActivity = 0;
unsigned long previousMillistemp;       // initialisation at the end of init()
unsigned long previousMillis_ota_handle = 0;
unsigned long previousMillis_mqtt_handle = 0;
unsigned long previousMillis_blynk_handle = 0;
unsigned long previousMillis_debug_handle = 0;
unsigned long previousMillis_pidCheck = 0;
const long refreshTempInterval = 1000;  //How often to read the temperature sensor
unsigned long best_time_to_call_refreshTemp = refreshTempInterval;
unsigned int estimated_cycle_refreshTemp = 25;  // for my TSIC the hardware refresh happens every 76ms
int tsic_validate_count = 0;
int tsic_stable_count = 0;
unsigned int estimated_cycle_refreshTemp_stable_next_save = 1;
#ifdef EMERGENCY_TEMP
const unsigned int emergency_temperature = EMERGENCY_TEMP;  // temperature at which the emergency shutdown should take place. DONT SET IT ABOVE 120 DEGREE!!
#else
const unsigned int emergency_temperature = 120;             // fallback
#endif
double brewDetectionSensitivity = BREWDETECTION_SENSITIVITY ; // if temperature decreased within the last 6 seconds by this amount, then we detect a brew.
#ifdef BREW_READY_DETECTION
const int enabledHardwareLed = ENABLE_HARDWARE_LED;
const int enabledHardwareLedNumber = ENABLE_HARDWARE_LED_NUMBER;
float marginOfFluctuation = float(BREW_READY_DETECTION);
#if (ENABLE_HARDWARE_LED == 2)  // WS2812b based LEDs
  #define FASTLED_ESP8266_RAW_PIN_ORDER
  #include <FastLED.h>
  CRGB leds[enabledHardwareLedNumber];
#endif
#else
const int enabledHardwareLed = 0;       // 0 = disable functionality
float marginOfFluctuation = 0;          // 0 = disable functionality
#endif
char* blynkReadyLedColor = "#000000";
unsigned long lastCheckBrewReady = 0;
unsigned long lastBrewReady = 0;
unsigned long lastBrewEnd = 0;    //used to determime the time it takes to reach brewReady==true
bool brewReady = false;
const int expected_eeprom_version = 6;        // EEPROM values are saved according to this versions layout. Increase if a new layout is implemented.
unsigned long eeprom_save_interval = 28*60*1000UL;  //save every 28min
unsigned long last_eeprom_save = 0;
char debugline[200];
unsigned long output_timestamp = 0;
unsigned long all_services_lastReconnectAttemptTime = 0;
unsigned long all_services_min_reconnect_interval = 160000; // 160sec minimum wait-time between service reconnections
bool force_offline = FORCE_OFFLINE;
unsigned long force_eeprom_sync = 0 ;
const int force_eeprom_sync_waitTime = 3000;  // after updating a setting wait this number of milliseconds before writing to eeprom

unsigned long loops = 0;
unsigned long max_micros = 0;
unsigned long last_report_micros = 0;
static unsigned long cur_micros;
unsigned long cur_micros_previous_loop = 0;
const unsigned long loop_report_count = 100;

/********************************************************
   DALLAS TEMP
******************************************************/
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS pinTemperature  // Data wire is plugged into port 2 on the Arduino
OneWire oneWire(ONE_WIRE_BUS);       // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
DeviceAddress sensorDeviceAddress;   // arrays to hold device address

/********************************************************
   TSIC 30x TEMP
******************************************************/
#define USE_ZACWIRE_TSIC
#ifdef USE_ZACWIRE_TSIC
#include "src/ZACwire-Library/ZACwire.h"
#ifdef ESP32
ZACwire<pinTemperature> TSIC(306,130,10,0);
#else
ZACwire<pinTemperature> TSIC;
#endif
#endif

uint16_t temperature = 0;
float Temperatur_C = 0;
volatile uint16_t temp_value[2] = {0};
volatile byte tsicDataAvailable = 0;
unsigned int isrCounterStripped = 0;
const int isrCounterFrame = 1000;


/********************************************************
   CONTROLS
******************************************************/
#include "controls.h"
controlMap* controlsConfig = NULL;
unsigned long lastCheckGpio = 0;

/********************************************************
   BLYNK
******************************************************/
#define BLYNK_PRINT Serial
#ifdef ESP32
#include <BlynkSimpleEsp32.h>
#else
#include <BlynkSimpleEsp8266.h>
#endif

#define BLYNK_GREEN     "#23C48E"
#define BLYNK_YELLOW    "#ED9D00"
#define BLYNK_RED       "#D3435C"
unsigned long previousMillisBlynk = 0;
const long intervalBlynk = 1000;      // Update Intervall zur App
int blynksendcounter = 1;
bool blynk_sync_run_once = false;
String PreviousError = "";
String PreviousOutputString = "";
String PreviousPastTemperatureChange = "";
String PreviousInputString = "";
bool blynk_disabled_temporary = false;

/******************************************************
 * Receive following BLYNK PIN values from app/server
 ******************************************************/
BLYNK_CONNECTED() {
  if (!blynk_sync_run_once) {
    blynk_sync_run_once = true;
    Blynk.syncAll();  //get all values from server/app when connected
  }
}
// This is called when Smartphone App is opened
BLYNK_APP_CONNECTED() {
  DEBUG_print("Blynk Client Connected.\n");
  print_settings();
  printControlsConfig(controlsConfig);
}
// This is called when Smartphone App is closed
BLYNK_APP_DISCONNECTED() {
  DEBUG_print("Blynk Client Disconnected.\n");
}
BLYNK_WRITE(V4) {
  aggKp = param.asDouble();
}
BLYNK_WRITE(V5) {
  aggTn = param.asDouble();
}
BLYNK_WRITE(V6) {
  aggTv = param.asDouble();
}
BLYNK_WRITE(V7) {
  setPoint = param.asDouble();
}
BLYNK_WRITE(V8) {
  brewtime = param.asDouble();
}
BLYNK_WRITE(V9) {
  preinfusion = param.asDouble();
}
BLYNK_WRITE(V10) {
  preinfusionpause = param.asDouble();
}
BLYNK_WRITE(V12) {
  starttemp = param.asDouble();
}
BLYNK_WRITE(V13) {
  pidON = param.asInt();
}
BLYNK_WRITE(V30) {
  aggoKp = param.asDouble();
}
BLYNK_WRITE(V31) {
  aggoTn = param.asDouble();
}
BLYNK_WRITE(V32) {
  aggoTv = param.asDouble();
}
BLYNK_WRITE(V34) {
  brewDetectionSensitivity = param.asDouble();
}
BLYNK_WRITE(V36) {
  brewDetectionPower = param.asDouble();
}
BLYNK_WRITE(V40) {
  burstShot = param.asInt();
}
BLYNK_WRITE(V41) {
  steadyPower = param.asDouble();
  // TODO fix this bPID.SetSteadyPowerDefault(steadyPower); //TOBIAS: working?
}
BLYNK_WRITE(V42) {
  steadyPowerOffset = param.asDouble();
}
BLYNK_WRITE(V43) {
  steadyPowerOffsetTime = param.asInt();
}
BLYNK_WRITE(V44) {
  burstPower = param.asDouble();
}
BLYNK_WRITE(V50) {
  setPointSteam = param.asDouble();
}


/******************************************************
 * Type Definition of "sending" BLYNK PIN values from
 * hardware to app/server (only defined if required)
 ******************************************************/
WidgetLED brewReadyLed(V14);

/******************************************************
 * HELPER
 ******************************************************/
bool wifi_working() {
  #ifdef ESP32
  //DEBUG_print("status=%d ip=%s\n", WiFi.status() == WL_CONNECTED, WiFi.localIP().toString());
  return ((!force_offline) && (WiFi.status() == WL_CONNECTED));  //TODO 2.7.x correct to remove IPAddress(0) check?
  #else
  return ((!force_offline) && (WiFi.status() == WL_CONNECTED) && (WiFi.localIP() != IPAddress(0U)));
  #endif
}

bool blynk_working() {
  return ((BLYNK_ENABLE == 1) && (wifi_working()) && (Blynk.connected()));
}

bool in_sensitive_phase() {
  return (Input >=(setPointSteam -5) || brewing || activeState==4 || isrCounter > 1000);
}

/********************************************************
  Emergency Stop when temp too high
*****************************************************/
void testEmergencyStop(){
  if (getCurrentTemperature() >= emergency_temperature) {
    if (emergencyStop != true) {
      snprintf(debugline, sizeof(debugline), "EmergencyStop because temperature>%u (temperature=%0.2f)", emergency_temperature, getCurrentTemperature());
      ERROR_println(debugline);
      mqtt_publish("events", debugline);
      emergencyStop = true;
    }
  } else if (emergencyStop == true && getCurrentTemperature() < emergency_temperature) {
    snprintf(debugline, sizeof(debugline), "EmergencyStop ended because temperature<%0.2f (temperature=%0.2f)", emergency_temperature, getCurrentTemperature());
    ERROR_println(debugline);
    mqtt_publish("events", debugline);
    emergencyStop = false;
  }
}

/********************************************************
  history temperature data
*****************************************************/
void updateTemperatureHistory(double myInput) {
  if (readIndex >= numReadings -1) {
    readIndex = 0;
  } else {
    readIndex++;
  }
  readingstime[readIndex] = millis();
  readingstemp[readIndex] = myInput;
}

//calculate the temperature difference between NOW and a datapoint in history
double pastTemperatureChange(int lookback) {
  if (lookback >= numReadings) lookback=numReadings -1;
  int offset = lookback % numReadings;
  int historicIndex = (readIndex - offset);
  if ( historicIndex < 0 ) {
    historicIndex += numReadings;
  }
  //ignore not yet initialized values
  if (readingstime[readIndex] == 0 || readingstime[historicIndex] == 0) return 0;
  return readingstemp[readIndex] - readingstemp[historicIndex];
}

//calculate the average temperature over the last (lookback) temperatures samples
double getAverageTemperature(int lookback) {
  double averageInput = 0;
  int count = 0;
  if (lookback >= numReadings) lookback=numReadings -1;
  for (int offset = 0; offset < lookback; offset++) {
    int thisReading = readIndex - offset;
    if (thisReading < 0) thisReading = numReadings + thisReading;
    if (readingstime[thisReading] == 0) break;
    averageInput += readingstemp[thisReading];
    count += 1;
  }
  if (count > 0) {
    return averageInput / count;
  } else {
    DEBUG_print("getAverageTemperature() returned 0\n");
    return 0;
  }
}

double getCurrentTemperature() {
  return readingstemp[readIndex];
}

double getTemperature(int lookback) {
  if (lookback >= numReadings) lookback=numReadings -1;
  int offset = lookback % numReadings;
  int historicIndex = (readIndex - offset);
  if ( historicIndex < 0 ) {
    historicIndex += numReadings;
  }
  //ignore not yet initialized values
  if (readingstime[historicIndex] == 0) return 0;
  return readingstemp[historicIndex];
}

//returns heater utilization in percent
double convertOutputToUtilisation(double Output) {
  return (100 * Output) / windowSize;
}

//returns heater utilization in Output
double convertUtilisationToOutput(double utilization) {
  return (utilization / 100 ) * windowSize;
}

bool checkBrewReady(double setPoint, float marginOfFluctuation, int lookback) {
  if (almostEqual(marginOfFluctuation, 0)) return false;
  if (lookback >= numReadings) lookback=numReadings -1;
  for (int offset = 0; offset < lookback; offset++) {
    int thisReading = readIndex - offset;
    if (thisReading < 0) thisReading = numReadings + thisReading;
    if (readingstime[thisReading] == 0) return false;
    if (fabs(setPoint - readingstemp[thisReading]) > (marginOfFluctuation + FLT_EPSILON)) return false;
  }
  return true;
}

void setHardwareLed(bool mode) {
  #if (ENABLE_HARDWARE_LED == 0)
  return;
  #elif (ENABLE_HARDWARE_LED == 1)
  static bool previousMode = false;
  if (enabledHardwareLed == 1 && mode != previousMode) {
    digitalWrite(pinLed, mode);
    previousMode = mode;
  }
  #elif (ENABLE_HARDWARE_LED == 2)
  static bool previousMode = false;
  if(enabledHardwareLed == 2 && mode != previousMode){
    previousMode = mode;
    if (mode){
      fill_solid(leds, enabledHardwareLedNumber, CRGB::ENABLE_HARDWARE_LED_RGB_ON);
    }
    else{
      fill_solid(leds, enabledHardwareLedNumber, CRGB::ENABLE_HARDWARE_LED_RGB_OFF);
    }
    FastLED.show();
  }
  #endif
}

/*****************************************************
 * fast temperature reading with TSIC 306
 * Code by Adrian with minor adaptions
******************************************************/
void ICACHE_RAM_ATTR readTSIC() { //executed every ~100ms by interrupt
  isrCounterStripped = isrCounter % isrCounterFrame;
  if (isrCounterStripped >= (isrCounterFrame - 20 - 100) && isrCounterStripped < (isrCounterFrame - 20)) {
    byte strobelength = 6;
    byte timeout = 0;
    for (byte ByteNr=0; ByteNr<2; ++ByteNr) {
      if (ByteNr) {                                    //measure strobetime between bytes
        for (timeout = 0; digitalRead(pinTemperature); ++timeout){
          delayMicroseconds(10);
          if (timeout > 20) return;
        }
        strobelength = 0;
        for (timeout = 0; !digitalRead(pinTemperature); ++timeout) {    // wait for rising edge
          ++strobelength;
          delayMicroseconds(10);
          if (timeout > 20) return;
        }
      }
      for (byte i=0; i<9; ++i) {
        for (timeout = 0; digitalRead(pinTemperature); ++timeout) {    // wait for falling edge
          delayMicroseconds(10);
          if (timeout > 20) return;
        }
        if (!i) temp_value[ByteNr] = 0;            //reset byte before start measuring
        delayMicroseconds(10 * strobelength);
        temp_value[ByteNr] <<= 1;
        if (digitalRead(pinTemperature)) temp_value[ByteNr] |= 1;        // Read bit
        for (timeout = 0; !digitalRead(pinTemperature); ++timeout) {     // wait for rising edge
          delayMicroseconds(10);
          if (timeout > 20) return;
        }
      }
    }
    tsicDataAvailable++;
  }
}

double temperature_simulate_steam() {
    unsigned long now = millis();
    //if ( now <= 20000 ) return 102;
    //if ( now <= 26000 ) return 99;
    //if ( now <= 33000 ) return 96;
    //if (now <= 45000) return setPoint;  //TOBIAS remove
    if ( now <= 20000 ) return 114;
    if ( now <= 26000 ) return 117;
    if ( now <= 29000 ) return 120;
    if (now <= 32000) return 116;
    if (now <= 35000) return 113;
    if (now <= 37000) return 109;
    if (now <= 39000) return 105;
    if (now <= 40000) return 101;
    if (now <= 43000) return 97;
    return setPoint;
}

double temperature_simulate_normal() {
    unsigned long now = millis();
    if ( now <= 12000 ) return 82;
    if ( now <= 15000 ) return 85;
    if ( now <= 19000 ) return 88;
    if ( now <= 25000 ) return 91;
    if (now <= 28000) return 92;
    return setPoint;
}

double getTSICvalue() {
    byte parity1 = 0;
    byte parity2 = 0;
    noInterrupts();                               //no ISRs because temp_value might change during reading
    uint16_t temperature1 = temp_value[0];        //get high significant bits from ISR
    uint16_t temperature2 = temp_value[1];        //get low significant bits from ISR
    interrupts();
    for (uint8_t i = 0; i < 9; ++i) {
      if (temperature1 & (1 << i)) ++parity1;
      if (temperature2 & (1 << i)) ++parity2;
    }
    if (!(parity1 % 2) && !(parity2 % 2)) {       // check parities
      temperature1 >>= 1;                         // delete parity bits
      temperature2 >>= 1;
      temperature = (temperature1 << 8) + temperature2; //joints high and low significant figures
      // TSIC 20x,30x
      return (float((temperature * 250L) >> 8) - 500) / 10;
      // TSIC 50x
      // return (float((temperature * 175L) >> 9) - 100) / 10;
    }
    else return -50;    //set to -50 if reading failed
}


/********************************************************
  check sensor value. If < 0 or difference between old and new >10, then increase error.
  If error is equal to maxErrorCounter, then set sensorError
*****************************************************/
bool checkSensor(float tempInput, float temppreviousInput) {
  bool sensorOK = false;
  if (!sensorError) {
    if ( ( tempInput < 0 || tempInput > 150 || fabs(tempInput - temppreviousInput) > 5)) {
      error++;
      DEBUG_print("temperature sensor reading: consec_errors=%d, temp_current=%0.2f, temp_prev=%0.2f\n", error, tempInput, temppreviousInput);
    } else {
      error = 0;
      sensorOK = true;
    }
    if (error >= maxErrorCounter) {
      sensorError = true;
      snprintf(debugline, sizeof(debugline), "temperature sensor malfunction: temp_current=%0.2f, temp_prev=%0.2f", tempInput, previousInput);
      ERROR_println(debugline);
      mqtt_publish("events", debugline);
    }
  } else if (TempSensorRecovery == 1 &&
             (!(tempInput < 0 || tempInput > 150))) {
      sensorError = false;
      error = 0;
      sensorOK = true;
  }
  return sensorOK;
}


/********************************************************
  Refresh temperature.
  Each time checkSensor() is called to verify the value.
  If the value is not valid, new data is not stored.
*****************************************************/
void refreshTemp() {
  unsigned long currentMillistemp = millis();
  if (TempSensor == 1) {
    long millis_elapsed = currentMillistemp - previousMillistemp ;
    if ( floor(millis_elapsed / refreshTempInterval) >= 2) {
        snprintf(debugline, sizeof(debugline), "Main loop() hang: refreshTemp() missed=%g, millis_elapsed=%lu, isrCounter=%u", floor(millis_elapsed / refreshTempInterval) -1, millis_elapsed, isrCounter);
        ERROR_println(debugline);
        mqtt_publish("events", debugline);
    }
    if (currentMillistemp >= previousMillistemp + refreshTempInterval)
    {
      previousInput = getCurrentTemperature();
      previousMillistemp = currentMillistemp;
      sensors.requestTemperatures();
      if (!checkSensor(sensors.getTempCByIndex(0), previousInput)) return;  //if sensor data is not valid, abort function
      updateTemperatureHistory(sensors.getTempCByIndex(0));
      Input = getAverageTemperature(5);
    }
  } else if (TempSensor == 2) {
      #ifdef USE_ZACWIRE_TSIC
      if (currentMillistemp >= previousMillistemp + refreshTempInterval)
      {
        previousInput = getCurrentTemperature();
        previousMillistemp = currentMillistemp;
        //Temperatur_C = temperature_simulate_steam();
        //Temperatur_C = temperature_simulate_normal();
        Temperatur_C = TSIC.getTemp();
        if (checkSensor(Temperatur_C, previousInput)) {
            updateTemperatureHistory(Temperatur_C);
            Input = getAverageTemperature(5);
        }
      }
      #else
      if (tsicDataAvailable >0) { // TODO Failsafe? || currentMillistemp >= previousMillistemp + 1.2* refreshTempInterval ) {
        previousInput = getCurrentTemperature();
        Temperatur_C = getTSICvalue();
        tsicDataAvailable = 0;
        if (checkSensor(Temperatur_C, previousInput)) {
            updateTemperatureHistory(Temperatur_C);
            Input = getAverageTemperature(5);
        }
      }
      #endif
  }
}


/********************************************************
    PreInfusion, Brew , if not Only PID
******************************************************/
void brew() {
  if (OnlyPID) {
    return;
  }
  unsigned long aktuelleZeit = millis();
  if (simulatedBrewSwitch && (brewing == 1 || waitingForBrewSwitchOff == false) ) {
    totalbrewtime = (preinfusion + preinfusionpause + brewtime) * 1000;

    if (brewing == 0) {
      brewing = 1;  // Attention: For OnlyPID==0 brewing must only be changed in this function! Not externally.
      startZeit = aktuelleZeit;
      waitingForBrewSwitchOff = true;

    }
    bezugsZeit = aktuelleZeit - startZeit;

    if (aktuelleZeit >= lastBrewMessage + 500) {
      lastBrewMessage = aktuelleZeit;
      DEBUG_print("brew(): bezugsZeit=%lu totalbrewtime=%lu\n", bezugsZeit/1000, totalbrewtime/1000);
    }
    if (bezugsZeit <= totalbrewtime) {
      if (preinfusion > 0 && bezugsZeit <= preinfusion*1000) {
        if (brewState != 1) {
          brewState = 1;
          DEBUG_println("preinfusion");
          digitalWrite(pinRelayVentil, relayON);
          digitalWrite(pinRelayPumpe, relayON);
        }
      } else if (preinfusion > 0 && bezugsZeit > preinfusion*1000 && bezugsZeit <= (preinfusion + preinfusionpause)*1000) {
        if (brewState != 2) {
          brewState = 2;
          DEBUG_println("Pause");
          digitalWrite(pinRelayVentil, relayON);
          digitalWrite(pinRelayPumpe, relayOFF);
        }
      } else if (preinfusion == 0 || bezugsZeit > (preinfusion + preinfusionpause)*1000) {
        if (brewState != 3) {
          brewState = 3;
          DEBUG_println("Brew");
          digitalWrite(pinRelayVentil, relayON);
          digitalWrite(pinRelayPumpe, relayON);
        }
      }
    } else {
      brewState = 0;
      DEBUG_print("End brew()\n");
      brewing = 0;
      digitalWrite(pinRelayVentil, relayOFF);
      digitalWrite(pinRelayPumpe, relayOFF);
    }
  } else if (simulatedBrewSwitch && !brewing) {  //corner-case: switch=On but brewing==0
    waitingForBrewSwitchOff = true;  //just to be sure
    //digitalWrite(pinRelayVentil, relayOFF);  //already handled by brewing var
    //digitalWrite(pinRelayPumpe, relayOFF);
    bezugsZeit = 0;
    brewState = 0;
  } else if (!simulatedBrewSwitch) {
    if (waitingForBrewSwitchOff) {
      DEBUG_print("simulatedBrewSwitch=off\n");
    }
    waitingForBrewSwitchOff = false;
    if (brewing == 1) {
      digitalWrite(pinRelayVentil, relayOFF);
      digitalWrite(pinRelayPumpe, relayOFF);
      brewing = 0;
    }
    bezugsZeit = 0;
    brewState = 0;
  }
}

 /********************************************************
   Check if Wifi is connected, if not reconnect
 *****************************************************/
 void checkWifi() {checkWifi(false, wifiConnectWaitTime); }
 void checkWifi(bool force_connect, unsigned long wifiConnectWaitTime_tmp) {
  if (force_offline) return;  //remove this to allow wifi reconnects even when DISABLE_SERVICES_ON_STARTUP_ERRORS=1
  if ((!force_connect) && (wifi_working() || in_sensitive_phase())) return;
  if (force_connect || (millis() > lastWifiConnectionAttempt + 5000 + (wifiReconnectInterval * wifiReconnects))) {
    lastWifiConnectionAttempt = millis();
    //noInterrupts();
    DEBUG_print("Connecting to WIFI with SID %s ...\n", ssid);
    WiFi.persistent(false);   // Don't save WiFi configuration in flash
    WiFi.disconnect(true);    // Delete SDK WiFi config
    #ifndef ESP32
    WiFi.setSleepMode(WIFI_NONE_SLEEP);  // needed for some disconnection bugs?
    #endif
    //WiFi.setSleep(false);              // needed?
    //displaymessage(0, "Connecting Wifi", "");
    #ifdef STATIC_IP
    IPAddress STATIC_IP;
    IPAddress STATIC_GATEWAY;
    IPAddress STATIC_SUBNET;
    WiFi.config(ip, gateway, subnet);
    #endif
    /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
      would try to act as both a client and an access-point and could cause
      network-issues with your other WiFi-devices on your WiFi-network. */
    WiFi.mode(WIFI_STA);
    //WiFi.enableSTA(true);
    delay(100);  //required for esp32?
    WiFi.setAutoConnect(false);    //disable auto-connect
    WiFi.setAutoReconnect(false);  //disable auto-reconnect
    #ifdef ESP32
    WiFi.begin(ssid, pass);
    WiFi.setHostname(hostname);  //XXX1 Reihenfolge wirklich richtig?
    #else
    WiFi.hostname(hostname);
    WiFi.begin(ssid, pass);
    #endif
    
    while (!wifi_working() && (millis() < lastWifiConnectionAttempt + wifiConnectWaitTime_tmp)) {
        yield(); //Prevent Watchdog trigger
    }
    if (wifi_working()) {
      DEBUG_print("Wifi connection attempt (#%u) successfull (%lu secs)\n", wifiReconnects, (millis() - lastWifiConnectionAttempt) /1000);
      wifiReconnects = 0;
    } else {
      ERROR_print("Wifi connection attempt (#%u) not successfull (%lu secs)\n", wifiReconnects, (millis() - lastWifiConnectionAttempt) /1000);
      wifiReconnects++;
    }
  }
}

/********************************************************
  send data to Blynk server
*****************************************************/
void sendToBlynk() {
  if (force_offline || !blynk_working() || blynk_disabled_temporary) return;
  unsigned long currentMillisBlynk = millis();
  if (currentMillisBlynk >= previousMillisBlynk + intervalBlynk) {
    previousMillisBlynk = currentMillisBlynk;
    if (brewReady) {
      if (blynkReadyLedColor != BLYNK_GREEN) {
        blynkReadyLedColor = BLYNK_GREEN;
        brewReadyLed.setColor(blynkReadyLedColor);
      }
    } else if (marginOfFluctuation != 0 && checkBrewReady(setPoint, marginOfFluctuation * 2, 40)) {
      if (blynkReadyLedColor != BLYNK_YELLOW) {
        blynkReadyLedColor = BLYNK_YELLOW;
        brewReadyLed.setColor(blynkReadyLedColor);
      }
    } else {
      if (blynkReadyLedColor != BLYNK_RED) {
        brewReadyLed.on();
        blynkReadyLedColor = BLYNK_RED;
        brewReadyLed.setColor(blynkReadyLedColor);
      }
    }
    if (grafana == 1 && blynksendcounter == 1) {
      Blynk.virtualWrite(V60, Input, Output, bPID.GetKp(), bPID.GetKi(), bPID.GetKd(), *activeSetPoint );
    }
    //performance tests has shown to only send one api-call per sendToBlynk()
    if (blynksendcounter == 1) {
      if (steadyPower != steadyPowerSavedInBlynk) {
        Blynk.virtualWrite(V41, steadyPower);  //auto-tuning params should be saved by Blynk.virtualWrite()
        steadyPowerSavedInBlynk = steadyPower;
      } else {
        blynksendcounter++;
      }
    }
    if (blynksendcounter == 2) {
      if (String(pastTemperatureChange(10)/2, 2) != PreviousPastTemperatureChange) {
        Blynk.virtualWrite(V35, String(pastTemperatureChange(10)/2, 2));
        PreviousPastTemperatureChange = String(pastTemperatureChange(10)/2, 2);
      } else {
        blynksendcounter++;
      }
    }
    if (blynksendcounter == 3) {
      if (String(Input - setPoint, 2) != PreviousError) {
        Blynk.virtualWrite(V11, String(Input - *activeSetPoint, 2));
        PreviousError = String(Input - *activeSetPoint, 2);
      } else {
        blynksendcounter++;
      }
    }
    if (blynksendcounter == 4) {
      if (String(convertOutputToUtilisation(Output), 2) != PreviousOutputString) {
        Blynk.virtualWrite(V23, String(convertOutputToUtilisation(Output), 2));
        PreviousOutputString = String(convertOutputToUtilisation(Output), 2);
      } else {
        blynksendcounter++;
      }
    }
    if (blynksendcounter >= 5) {
      if (String(Input, 2) != PreviousInputString) {
        Blynk.virtualWrite(V2, String(Input, 2)); //send value to server
        PreviousInputString = String(Input, 2);
      }
      //Blynk.syncVirtual(V2);  //get value from server
      blynksendcounter = 0;
    }
    blynksendcounter++;
  }
}

/********************************************************
    state Detection
******************************************************/
void updateState() {
  static bool runOnce = false;
  switch (activeState) {
    case 1: // state 1 running, that means full heater power. Check if target temp is reached
    {
      if (!runOnce) {
        runOnce = true;
        const int machineColdStartLimit = 45;
        if (Input <= starttemp && Input >= machineColdStartLimit) {  //special auto-tuning settings when maschine is already warm
          MachineColdOnStart = false;
          steadyPowerOffsetDecreaseTimer = millis();
          steadyPowerOffsetModified /= 2;   //OK
          snprintf(debugline, sizeof(debugline), "steadyPowerOffset halved because maschine is already warm");
        }
      }
      bPID.SetFilterSumOutputI(100);
      if (Input >= starttemp + starttempOffset  || !pidMode ) {  //80.5 if 44C. | 79,7 if 30C |
        snprintf(debugline, sizeof(debugline), "** End of Coldstart. Transition to step 2 (constant steadyPower)");
        DEBUG_println(debugline);
        mqtt_publish("events", debugline);
        bPID.SetSumOutputI(0);
        activeState = 2;
      }
      break;
    }
    case 2: // state 2 running, that means heater is on steadyState and we are waiting to temperature to stabilize
    {
      bPID.SetFilterSumOutputI(30);

      if ( (Input - setPoint >= 0) || (Input - setPoint <= -20) ||
           (Input - setPoint <= 0  && pastTemperatureChange(20) <= 0.3) ||
           (Input - setPoint >= -1.0  && pastTemperatureChange(10) > 0.2) ||
           (Input - setPoint >= -1.5  && pastTemperatureChange(10) >= 0.45) ||
           !pidMode ) {
          //auto-tune starttemp
          if (millis() < 400000 && steadyPowerOffset_Activated > 0 && pidMode && MachineColdOnStart) {  //ugly hack to only adapt setPoint after power-on
          double tempChange = pastTemperatureChange(10);
          if (Input - setPoint >= 0) {
            if (tempChange > 0.05 && tempChange <= 0.15) {
              DEBUG_print("Auto-Tune starttemp(%0.2f -= %0.2f) | steadyPowerOffset=%0.2f | steadyPowerOffsetTime=%d\n", starttemp, 0.5, steadyPowerOffset, steadyPowerOffsetTime);
              starttemp -= 0.5;
            } else if (tempChange > 0.15) {
              DEBUG_print("Auto-Tune starttemp(%0.2f -= %0.2f) | steadyPowerOffset=%0.2f | steadyPowerOffsetTime=%d\n", starttemp, 1.0, steadyPowerOffset, steadyPowerOffsetTime);
              starttemp -= 1;
            }
          } else if (Input - setPoint >= -1.5 && tempChange >= 0.8) {  //
            DEBUG_print("Auto-Tune starttemp(%0.2f -= %0.2f, too fast) | steadyPowerOffset=%0.2f | steadyPowerOffsetTime=%d\n", starttemp, 0.4, steadyPowerOffset, steadyPowerOffsetTime);
            starttemp -= 0.4;
          } else if (Input - setPoint >= -1.5 && tempChange >= 0.45) {  // OK (-0.10)!
            DEBUG_print("Auto-Tune starttemp(%0.2f -= %0.2f, too fast) | steadyPowerOffset=%0.2f | steadyPowerOffsetTime=%d\n", starttemp, 0.2, steadyPowerOffset, steadyPowerOffsetTime);
            starttemp -= 0.2;
          } else if (Input - setPoint >= -1.0 && tempChange > 0.2) {  // OK (+0.10)!
            DEBUG_print("Auto-Tune starttemp(%0.2f -= %0.2f, too fast) | steadyPowerOffset=%0.2f | steadyPowerOffsetTime=%d\n", starttemp, 0.1, steadyPowerOffset, steadyPowerOffsetTime);
            starttemp -= 0.1;
          } else if (Input - setPoint <= -1.2) {
            DEBUG_print("Auto-Tune starttemp(%0.2f += %0.2f) | steadyPowerOffset=%0.2f | steadyPowerOffsetTime=%d\n", starttemp, 0.3, steadyPowerOffset, steadyPowerOffsetTime);
            starttemp += 0.3;
          } else if (Input - setPoint <= -0.6) {
            DEBUG_print("Auto-Tune starttemp(%0.2f += %0.2f) | steadyPowerOffset=%0.2f | steadyPowerOffsetTime=%d\n", starttemp, 0.2, steadyPowerOffset, steadyPowerOffsetTime);
            starttemp += 0.2;
           } else if (Input - setPoint >= -0.4) {
            DEBUG_print("Auto-Tune starttemp(%0.2f -= %0.2f) | steadyPowerOffset=%0.2f | steadyPowerOffsetTime=%d\n", starttemp, 0.1, steadyPowerOffset, steadyPowerOffsetTime);
            starttemp -= 0.1;
          }
          //persist starttemp auto-tuning setting
          mqtt_publish("starttemp/set", number2string(starttemp));
          mqtt_publish("starttemp", number2string(starttemp));
          Blynk.virtualWrite(V12, String(starttemp, 1));
          force_eeprom_sync = millis();
        } else {
          DEBUG_print("Auto-Tune starttemp disabled\n");
        }

        snprintf(debugline, sizeof(debugline), "** End of stabilizing. Transition to step 3 (normal mode)");
        DEBUG_println(debugline);
        mqtt_publish("events", debugline);
        bPID.SetSumOutputI(0);
        activeState = 3;
        bPID.SetAutoTune(true);
      }
      break;
    }
    case 4: // state 4 running = Brew running
    {
      bPID.SetFilterSumOutputI(100);
      bPID.SetAutoTune(false);
      if ( !brewing ||
           (OnlyPID && brewDetection == 2 && bezugsZeit >= lastBrewTimeOffset + 3 &&
            (bezugsZeit >= brewtime*1000 ||
              setPoint - Input < 0
            )
           )
        ) {
        if (OnlyPID && brewDetection == 2) brewing = 0;
        //DEBUG_print("Out Zone Detection: past(2)=%0.2f, past(3)=%0.2f | past(5)=%0.2f | past(10)=%0.2f | bezugsZeit=%lu\n", pastTemperatureChange(2), pastTemperatureChange(3), pastTemperatureChange(5), pastTemperatureChange(10), bezugsZeit / 1000);
        //DEBUG_print("t(0)=%0.2f | t(1)=%0.2f | t(2)=%0.2f | t(3)=%0.2f | t(5)=%0.2f | t(10)=%0.2f | t(13)=%0.2f\n", getTemperature(0), getTemperature(1), getTemperature(2), getTemperature(3), getTemperature(5), getTemperature(7), getTemperature(10), getTemperature(13));
        snprintf(debugline, sizeof(debugline), "** End of Brew. Transition to step 2 (constant steadyPower)");
        DEBUG_println(debugline);
        mqtt_publish("events", debugline);
        bPID.SetAutoTune(true);
        bPID.SetSumOutputI(0);
        timerBrewDetection = 0;
        mqtt_publish("brewDetected", "0");
        activeState = 2;
        lastBrewEnd = millis();
      }
      break;
    }
    case 5: // state 5 in outerZone
    {
      if (Input >= setPoint - outerZoneTemperatureDifference - 1.5) {
        bPID.SetFilterSumOutputI(4.5);
      } else {
        bPID.SetFilterSumOutputI(9);
      }

      if ( fabs(Input - *activeSetPoint) < outerZoneTemperatureDifference || steaming == 1 || (OnlyPID && brewDetection == 1 && simulatedBrewSwitch) ) {
        snprintf(debugline, sizeof(debugline), "** End of outerZone. Transition to step 3 (normal mode)");
        DEBUG_println(debugline);
        mqtt_publish("events", debugline);
        if (pidMode == 1) bPID.SetMode(AUTOMATIC);
        bPID.SetSumOutputI(0);
        bPID.SetAutoTune(true);
        timerBrewDetection = 0 ;
        activeState = 3;
      }
      break;
    }
    case 6: //state 6 heat up because we want to steam
    {
      bPID.SetAutoTune(false);  //do not tune during steam phase
      bPID.SetSumOutputI(100);

      if (!steaming) {  ///YYY1
        snprintf(debugline, sizeof(debugline), "** End of Steaming phase. Now cooling down. Transition to step 3 (normal mode)");
        DEBUG_println(debugline);
        mqtt_publish("events", debugline);
        if (*activeSetPoint != setPoint) {
          activeSetPoint = &setPoint;  //TOBIAS rename setPoint -> brewSetPoint
          DEBUG_print("set activeSetPoint: %0.2f\n", *activeSetPoint);
        }
        if (pidMode == 1) bPID.SetMode(AUTOMATIC);
        bPID.SetSumOutputI(0);
        bPID.SetAutoTune(false);
        Output = 0;
        timerBrewDetection = 0;
        activeState = 3;
      }
      break;
    }

    case 3: // normal PID mode
    default:
    {
      if (!pidMode) break;

      //set maximum allowed filterSumOutputI based on error/marginOfFluctuation
      if (Input >= *activeSetPoint - marginOfFluctuation) {
        if (bPID.GetFilterSumOutputI() != 1.0) {
          bPID.SetFilterSumOutputI(0);
        }
        bPID.SetFilterSumOutputI(1.0);
      } else if ( Input >= *activeSetPoint - 0.5) {
        bPID.SetFilterSumOutputI(2.0);
      } else {
        bPID.SetFilterSumOutputI(4.5);
      }

      /* STATE 1 (COLDSTART) DETECTION */
      if (Input <= starttemp - coldStartStep1ActivationOffset) {
        snprintf(debugline, sizeof(debugline), "** End of normal mode. Transition to step 1 (coldstart)");
        DEBUG_println(debugline);
        mqtt_publish("events", debugline);
        steadyPowerOffset_Activated = millis();
        DEBUG_print("Enable steadyPowerOffset (%0.2f)\n", steadyPowerOffset);
        bPID.SetAutoTune(false);  //do not tune during coldstart + phase2
        bPID.SetSumOutputI(0);
        activeState = 1;
        break;
      }

      /* STATE 4 (BREW) DETECTION */
      if (brewDetection == 1 || (brewDetectionSensitivity != 0 && brewDetection == 2) ) {
        //enable brew-detection if not already running and diff temp is > brewDetectionSensitivity
        if ( brewing ||
             (OnlyPID && brewDetection == 2 && (pastTemperatureChange(3) <= -brewDetectionSensitivity) &&
               fabs(getTemperature(5) - setPoint) <= outerZoneTemperatureDifference &&
               millis() - lastBrewTime >= BREWDETECTION_WAIT * 1000)
           ) {
          //DEBUG_print("Brew Detect: prev(5)=%0.2f past(3)=%0.2f past(5)=%0.2f | Avg(3)=%0.2f | Avg(10)=%0.2f Avg(2)=%0.2f\n", getTemperature(5), pastTemperatureChange(3), pastTemperatureChange(5), getAverageTemperature(3), getAverageTemperature(10), getAverageTemperature(2));
          //Sample: Brew Detection: past(3)=-1.70 past(5)=-2.10 | Avg(3)=91.50 | Avg(10)=92.52 Avg(20)=92.81
          if (OnlyPID) {
            bezugsZeit = 0;
            if (brewDetection == 2) {
              brewing = 1;
              lastBrewTime = millis() - lastBrewTimeOffset;
            } else {
              lastBrewTime = millis() - 200;
            }
          }
          userActivity = millis();
          timerBrewDetection = 1 ;
          mqtt_publish("brewDetected", "1");
          snprintf(debugline, sizeof(debugline), "** End of normal mode. Transition to step 4 (brew)");
          DEBUG_println(debugline);
          mqtt_publish("events", debugline);
          bPID.SetSumOutputI(0);
          activeState = 4;
          break;
        }
      }

      /* STATE 6 (Steam) DETECTION */
      if (steaming) {
        snprintf(debugline, sizeof(debugline), "Steaming Detected. Transition to state 6 (Steam)");
        //digitalWrite(pinRelayVentil, relayOFF);
        DEBUG_println(debugline);
        mqtt_publish("events", debugline);
        if (*activeSetPoint != setPointSteam) {
          activeSetPoint = &setPointSteam;
          DEBUG_print("set activeSetPoint: %0.2f\n", *activeSetPoint);
        }
        activeState = 6;
        break;
      }

      /* STATE 5 (OUTER ZONE) DETECTION */
      if ( Input > starttemp - coldStartStep1ActivationOffset &&
           (fabs(Input - *activeSetPoint) > outerZoneTemperatureDifference)
         ) {
        //DEBUG_print("Out Zone Detection: Avg(3)=%0.2f | Avg(5)=%0.2f Avg(20)=%0.2f Avg(2)=%0.2f\n", getAverageTemperature(3), getAverageTemperature(5), getAverageTemperature(20), getAverageTemperature(2));
        snprintf(debugline, sizeof(debugline), "** End of normal mode. Transition to step 5 (outerZone)");
        DEBUG_println(debugline);
        mqtt_publish("events", debugline);
        bPID.SetSumOutputI(0);
        activeState = 5;
        if (Input > setPoint) {  // if we are above setPoint always disable heating (primary useful after steaming)  YYY1
          bPID.SetAutoTune(false);
          bPID.SetMode(MANUAL);
        }
        break;
      }

      break;
    }
  }

  // steadyPowerOffset_Activated handling
  if ( steadyPowerOffset_Activated > 0) {
    if (Input - setPoint >= 1) {
      steadyPowerOffset_Activated = 0;
      snprintf(debugline, sizeof(debugline), "ATTENTION: Disabled steadyPowerOffset because its too large or starttemp too high");
      ERROR_println(debugline);
      mqtt_publish("events", debugline);
      bPID.SetAutoTune(true);
    } else if (Input - setPoint >= 0.4  && millis() >= steadyPowerOffsetDecreaseTimer + 90000) {
      steadyPowerOffsetDecreaseTimer = millis();
      steadyPowerOffsetModified /= 2;
      snprintf(debugline, sizeof(debugline), "ATTENTION: steadyPowerOffset halved because its too large or starttemp too high");
      ERROR_println(debugline);
      mqtt_publish("events", debugline);
    } else if (millis() >= steadyPowerOffset_Activated + steadyPowerOffsetTime*1000) {
      steadyPowerOffset_Activated = 0;
      DEBUG_print("Disable steadyPowerOffset\n");
      bPID.SetAutoTune(true);
    }
  }
}

/***********************************
 * PID & HEATER ISR
 ***********************************/
unsigned long pidComputeDelay = 0;
void pidCompute() {
  float Output_save;
  //certain activeState set Output to fixed values
  if (activeState == 1 || activeState == 2 || activeState == 4) {
    Output_save = Output;
  }
  int ret = bPID.Compute();
  if ( ret == 1) {  // compute() did run successfully
    if (isrCounter>(windowSize+100)) {
      ERROR_print("pidCompute() delay: isrCounter=%d, heater_overextending_isrCounter=%d, heater=%d\n", isrCounter, heater_overextending_isrCounter, digitalRead(pinRelayHeater));
    }
    isrCounter = 0; // Attention: heater might not shutdown if bPid.SetSampleTime(), windowSize, timer1_write() and are not set correctly!
    pidComputeDelay = millis()+5 - pidComputeLastRunTime - windowSize;
    if (pidComputeDelay > 50 && pidComputeDelay < 100000000) {
      DEBUG_print("pidCompute() delay of %lu ms (loop() hang?)\n", pidComputeDelay);
    }
    pidComputeLastRunTime = millis();
    if (activeState == 1 || activeState == 2 || activeState == 4) {
      Output = Output_save;
    }
    DEBUG_print("Input=%6.2f | error=%5.2f delta=%5.2f | Output=%6.2f = b:%5.2f + p:%5.2f + i:%5.2f(%5.2f) + d:%5.2f\n",
      Input,
      (*activeSetPoint - Input),
      pastTemperatureChange(10)/2,
      convertOutputToUtilisation(Output),
      steadyPower + bPID.GetSteadyPowerOffsetCalculated(),
      convertOutputToUtilisation(bPID.GetOutputP()),
      convertOutputToUtilisation(bPID.GetSumOutputI()),
      convertOutputToUtilisation(bPID.GetOutputI()),
      convertOutputToUtilisation(bPID.GetOutputD())
    );
  } else if (ret == 2) { // PID is disabled but compute() should have run
    isrCounter = 0;
    pidComputeLastRunTime = millis();
    DEBUG_print("Input=%6.2f | error=%5.2f delta=%5.2f | Output=%6.2f (PID disabled)\n",
      Input,
      (*activeSetPoint - Input),
      pastTemperatureChange(10)/2,
      convertOutputToUtilisation(Output));
  }
}

#ifdef ESP32
void IRAM_ATTR onTimer1ISR() {;
  timerAlarmWrite(timer, 10000, true);  //10ms
  if (isrCounter >= heater_overextending_isrCounter) {
    //turn off when when compute() is not run in time (safetly measure)
    digitalWrite(pinRelayHeater, LOW);
    //ERROR_print("onTimer1ISR has stopped heater because pid.Compute() did not run\n");
    //TODO: add more emergency handling?
  } else if (isrCounter > windowSize) {
    //dont change output when overextending withing overextending_factor threshold
    //DEBUG_print("onTimer1ISR over extending due to processing delays: isrCounter=%u\n", isrCounter);
  } else if (isrCounter >= Output) {  // max(Output) = windowSize
    digitalWrite(pinRelayHeater, LOW);
  } else {
    digitalWrite(pinRelayHeater, HIGH);
  }
  if (isrCounter <= (heater_overextending_isrCounter + 100)) {
    isrCounter += 10; // += 10 because one tick = 10ms
  }
}
#else
void ICACHE_RAM_ATTR onTimer1ISR() {
  timer1_write(50000); // set interrupt time to 10ms
  if (isrCounter >= heater_overextending_isrCounter) {
    //turn off when when compute() is not run in time (safetly measure)
    digitalWrite(pinRelayHeater, LOW);
    //ERROR_print("onTimer1ISR has stopped heater because pid.Compute() did not run\n");
    //TODO: add more emergency handling?
  } else if (isrCounter > windowSize) {
    //dont change output when overextending withing overextending_factor threshold
    //DEBUG_print("onTimer1ISR over extending due to processing delays: isrCounter=%u\n", isrCounter);
  } else if (isrCounter >= Output) {  // max(Output) = windowSize
    digitalWrite(pinRelayHeater, LOW);
  } else {
    digitalWrite(pinRelayHeater, HIGH);
  }
  if (isrCounter <= (heater_overextending_isrCounter + 100)) {
    isrCounter += 10; // += 10 because one tick = 10ms
  }
}
#endif

/***********************************
 * LOOP()
 ***********************************/
void loop() {
  refreshTemp();        // save new temperature values
  testEmergencyStop();  // test if Temp is to high

  //brewReady
  if (millis() > lastCheckBrewReady + refreshTempInterval) {
    lastCheckBrewReady = millis();
    bool brewReadyCurrent = checkBrewReady(setPoint, marginOfFluctuation, 60);
    if (!brewReady && brewReadyCurrent) {
      snprintf(debugline, sizeof(debugline), "brewReady (Tuning took %lu secs)", ((lastCheckBrewReady - lastBrewEnd) / 1000) - 60);
      DEBUG_println(debugline);
      mqtt_publish("events", debugline);
      lastBrewReady = millis() - 60000;
    }
    brewReady = brewReadyCurrent;
  }
  setHardwareLed(brewReady || Input >=setPointSteam || Input >= steamReadyTemp);

  //network related stuff
  if (!force_offline) {
    if (!wifi_working()) {
      #if (MQTT_ENABLE == 2)
      MQTT_server_cleanupClientCons();
      #endif
      checkWifi();
    } else {

      static bool runOnceOTASetup = true;
      if (runOnceOTASetup) {
        runOnceOTASetup = false;
        // Disable interrupt when OTA starts, otherwise it will not work
        ArduinoOTA.onStart([](){
          DEBUG_print("OTA update initiated\n");
          Output = 0;
          #ifdef ESP32
          timerAlarmDisable(timer);
          #else
          timer1_disable();
          #endif
          digitalWrite(pinRelayHeater, LOW); //Stop heating
        });
        ArduinoOTA.onError([](ota_error_t error) {
          ERROR_print("OTA update error\n");
          #ifdef ESP32
          timerAlarmEnable(timer);
          #else
          timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
          #endif
        });
        // Enable interrupts if OTA is finished
        ArduinoOTA.onEnd([](){
          #ifdef ESP32
          timerAlarmEnable(timer);
          #else
          timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
          #endif
        });
      }
      if ( millis() >= previousMillis_ota_handle + 500 ) {
        previousMillis_ota_handle = millis();
        ArduinoOTA.handle();
      }

      if (BLYNK_ENABLE && !blynk_disabled_temporary) {
        if (blynk_working()) {
          if ( millis() >= previousMillis_blynk_handle + 500 ) {
            previousMillis_blynk_handle = millis();
            Blynk.run(); //Do Blynk household stuff. (On reconnect after disconnect, timeout seems to be 5 seconds)
          }
        } else {
          unsigned long now = millis();
          if ((now > blynk_lastReconnectAttemptTime + (blynk_reconnect_incremental_backoff * (blynk_reconnectAttempts)))
               && now > all_services_lastReconnectAttemptTime + all_services_min_reconnect_interval
               && !in_sensitive_phase() ) {
              blynk_lastReconnectAttemptTime = now;
              all_services_lastReconnectAttemptTime = now;
              ERROR_print("Blynk disconnected. Reconnecting...\n");
              if ( Blynk.connect(2000) ) { // Attempt to reconnect
                blynk_lastReconnectAttemptTime = 0;
                blynk_reconnectAttempts = 0;
                DEBUG_print("Blynk reconnected in %lu seconds\n", (millis() - now)/1000);
              } else if (blynk_reconnectAttempts < blynk_max_incremental_backoff) {
                blynk_reconnectAttempts++;
              }
          }
        }
      }

      //Check mqtt connection
      if (MQTT_ENABLE && !mqtt_disabled_temporary) {
        if (!mqtt_working()) {

          mqtt_reconnect(false);
        } else {
          #if (MQTT_ENABLE == 1)
          if ( millis() >= previousMillis_mqtt_handle + 200 ) {
            previousMillis_mqtt_handle = millis();
            mqtt_client.loop(); // mqtt client connected, do mqtt housekeeping
          }
          #endif
          unsigned long now = millis();
          if (now >= lastMQTTStatusReportTime + lastMQTTStatusReportInterval) {
            lastMQTTStatusReportTime = now;
            mqtt_publish("temperature", number2string(Input));
            mqtt_publish("temperatureAboveTarget", number2string((Input - *activeSetPoint)));
            mqtt_publish("heaterUtilization", number2string(convertOutputToUtilisation(Output)));
            mqtt_publish("pastTemperatureChange", number2string(pastTemperatureChange(10)));
            mqtt_publish("brewReady", bool2string(brewReady));
            //mqtt_publish_settings();  //not needed because we update live on occurence
           }
        }
      }
    }
  }


  if ( millis() >= previousMillis_debug_handle + 200 ) {
    previousMillis_debug_handle = millis();
    Debug.handle();
  }
  
  #if (1==0)
  performance_check();
  return;
  #endif

  #if (DEBUG_FORCE_GPIO_CHECK == 1)
  if (millis() > lastCheckGpio + 2000) {
    lastCheckGpio = millis();
    debugControlHardware(controlsConfig);
  }
  return;
  #endif

  pidCompute();         // call PID for Output calculation

  checkControls(controlsConfig);  //transform controls to actions

  //handle brewing if button is pressed (ONLYPID=0 for now, because ONLYPID=1_with_BREWDETECTION=1 is handled in actionControl)
  //ideally brew() should be controlled in our state-maschine (TODO)
  brew();
  
  //check if PID should run or not. If not, set to manuel and force output to zero
  if (millis() > previousMillis_pidCheck + 300) {
    previousMillis_pidCheck = millis();
    if (pidON == 0 && pidMode == 1) {
      DEBUG_println("ins1");
      pidMode = 0;
      bPID.SetMode(pidMode);
      Output = 0 ;
      DEBUG_print("Current config has disabled PID\n");
    } else if (pidON == 1 && pidMode == 0 && !emergencyStop) {
      DEBUG_println("ins2");
      Output = 0; // safety: be 100% sure that PID.compute() starts fresh.
      pidMode = 1;
      bPID.SetMode(pidMode);
      if ( millis() - output_timestamp > 21000) {
        DEBUG_print("Current config has enabled PID\n");
        output_timestamp = millis();
      }
    }
  }

  //Sicherheitsabfrage
  if (!sensorError && !emergencyStop && Input > 0) {
    updateState();

    /* state 1: Water is very cold, set heater to full power */
    if (activeState == 1) {
      Output = windowSize;

    /* state 2: ColdstartTemp reached. Now stabilizing temperature after coldstart */
    } else if (activeState == 2) {
      //Output = convertUtilisationToOutput(steadyPower + bPID.GetSteadyPowerOffsetCalculated());
      Output = convertUtilisationToOutput(steadyPower);

    /* state 4: Brew detected. Increase heater power */
    } else if (activeState == 4) {
      if (Input > setPoint + outerZoneTemperatureDifference) {
        Output = convertUtilisationToOutput(steadyPower + bPID.GetSteadyPowerOffsetCalculated());
      } else {
        Output = convertUtilisationToOutput(brewDetectionPower);
      }
      if (OnlyPID == 1) {
        if (timerBrewDetection == 1) {
          bezugsZeit = millis() - lastBrewTime;
        }
      }
      /*
      if (OnlyPID == 0) {  //TODO TOBIAS
         Output = convertUtilisationToOutput(brewDetectionPower);
      } else if (OnlyPID == 1) {
        if (setPoint - Input <= (outerZoneTemperatureDifference + 0.5)
         ) {
          //DEBUG_print("BREWDETECTION_POWER(%0.2f) might be too high\n", brewDetectionPower);
          Output = convertUtilisationToOutput(steadyPower + bPID.GetSteadyPowerOffsetCalculated());
        } else {
          Output = convertUtilisationToOutput(brewDetectionPower);
        }
        if (timerBrewDetection == 1) {
          bezugsZeit = millis() - lastBrewTime;
        }
      }
      */

    /* state 5: Outer Zone reached. More power than in inner zone */
    } else if (activeState == 5) {
      if (Input > setPoint) {
        Output = 0;
      } else {
        if (aggoTn != 0) {
          aggoKi = aggoKp / aggoTn ;
        } else {
          aggoKi = 0;
        }
        aggoKd = aggoTv * aggoKp ;
        bPID.SetTunings(aggoKp, aggoKi, aggoKd);
        if (pidMode == 1) bPID.SetMode(AUTOMATIC);
      }

    /* state 6: Steaming state active*/
    } else if (activeState == 6) {
      bPID.SetMode(MANUAL);
      if (!pidMode) {
        Output = 0;
      } else {
        if (Input <= setPointSteam) {
          //full heat when temp below steam-temp
          Output = windowSize;
        } else if (Input > setPointSteam && (pastTemperatureChange(2) < 0) ) {
          //full heat when >setPointSteam BUT temp goes down!
          Output = windowSize;
        } else {
          Output = 0;
        }
        /*
        if (millis() >= lastSteamMessage + 1000) {
          lastSteamMessage = millis();
          DEBUG_print("*nput=%6.2f | error=%5.2f delta=%5.2f | Output=%6.2f\n",
            Input,
            (*activeSetPoint - Input),
            pastTemperatureChange(2),
            convertOutputToUtilisation(Output)
          );
        }
        */
      }

    /* state 3: Inner zone reached = "normal" low power mode */
    } else {
      if (!pidMode) {
        Output = 0;
      } else {
        bPID.SetMode(AUTOMATIC);
        if (aggTn != 0) {
          aggKi = aggKp / aggTn;
        } else {
          aggKi = 0 ;
        }
        aggKd = aggTv * aggKp ;
        bPID.SetTunings(aggKp, aggKi, aggKd);
      }
    }

    if (burstShot == 1 && pidMode == 1) {
      burstShot = 0;
      bPID.SetBurst(burstPower);
      snprintf(debugline, sizeof(debugline), "BURST Output=%0.2f", convertOutputToUtilisation(Output));
      DEBUG_println(debugline);
      mqtt_publish("events", debugline);
    }

    displaymessage(activeState, "", "");
    sendToBlynk();
    #if (1==0)
    performance_check();
    return;
    #endif
  
  } else if (sensorError) {
    //Deactivate PID
    if (pidMode == 1) {
      pidMode = 0;
      bPID.SetMode(pidMode);
      Output = 0 ;
      if ( millis() - output_timestamp > 15000) {
        ERROR_print("sensorError detected. Shutdown PID and heater\n");
        output_timestamp = millis();
      }
    }
    digitalWrite(pinRelayHeater, LOW); //Stop heating
    char line2[17];
    snprintf(line2, sizeof(line2), "Temp. %0.2f", getCurrentTemperature());
    displaymessage(0, "Check Temp. Sensor!", line2);

  } else if (emergencyStop) {
    //Deactivate PID
    if (pidMode == 1) {
      pidMode = 0;
      bPID.SetMode(pidMode);
      Output = 0 ;
      if ( millis() - output_timestamp > 10000) {
         ERROR_print("emergencyStop detected. Shutdown PID and heater (temp=%0.2f)\n", getCurrentTemperature());
         output_timestamp = millis();
      }
    }
    digitalWrite(pinRelayHeater, LOW); //Stop heating
    char line2[17];
    snprintf(line2, sizeof(line2), "%0.0f\xB0""C", getCurrentTemperature());
    displaymessage(0, "Emergency Stop!", line2);

  } else {
    if ( millis() - output_timestamp > 15000) {
       ERROR_print("unknown error\n");
       output_timestamp = millis();
    }
  }

  //persist steadyPower auto-tuning setting
  if (!almostEqual(steadyPower,steadyPowerSaved) && steadyPowerMQTTDisableUpdateUntilProcessed == 0) { //prevent race conditions by semaphore
    steadyPowerSaved = steadyPower;
    steadyPowerMQTTDisableUpdateUntilProcessed = steadyPower;
    steadyPowerMQTTDisableUpdateUntilProcessedTime = millis();
    mqtt_publish("steadyPower/set", number2string(steadyPower)); //persist value over shutdown
    mqtt_publish("steadyPower", number2string(steadyPower));
    if (force_eeprom_sync == 0) {
      force_eeprom_sync = millis() + 600000; // reduce writes on eeprom
    }
  }
  if ((steadyPowerMQTTDisableUpdateUntilProcessedTime >0) && (millis() >= steadyPowerMQTTDisableUpdateUntilProcessedTime + 20000)) {
    ERROR_print("steadyPower setting not saved for over 20sec (steadyPowerMQTTDisableUpdateUntilProcessed=%0.2f)\n", steadyPowerMQTTDisableUpdateUntilProcessed);
    steadyPowerMQTTDisableUpdateUntilProcessedTime = 0;
    steadyPowerMQTTDisableUpdateUntilProcessed = 0;
  }

  //persist settings to eeprom on interval or when required
  if (!in_sensitive_phase() &&
      (millis() >= last_eeprom_save + eeprom_save_interval ||
       (force_eeprom_sync > 0 && (millis() >= force_eeprom_sync + force_eeprom_sync_waitTime))
      )
     ) {
    last_eeprom_save = millis();
    force_eeprom_sync = 0;
    noInterrupts();
    sync_eeprom();
    interrupts();
  }
}


/***********************************
 * EEPROM
 ***********************************/
void sync_eeprom() { sync_eeprom(false, false); }

#ifdef ESP32
void sync_eeprom(bool startup_read, bool force_read) {
  DEBUG_print("EEPROM: sync_eeprom(startup_read=%d, force_read=%d) called\n", startup_read, force_read);
  preferences.begin("config");
  int current_version = preferences.getInt("current_version", 0);
  DEBUG_print("EEPROM: Detected Version=%d Expected Version=%d\n", current_version, expected_eeprom_version);
  if (current_version != expected_eeprom_version) {
    ERROR_print("EEPROM: Version has changed or settings are corrupt or not previously set. Ignoring..\n");
    //preferences.clear();
    preferences.putInt("current_version", expected_eeprom_version);
  }

  //if variables are not read from blynk previously, always get latest values from EEPROM
  if (force_read && (current_version == expected_eeprom_version)) {
    DEBUG_print("EEPROM: Blynk not active and not using external mqtt server. Reading settings from EEPROM\n");
    aggKp = preferences.getDouble("aggKp", 0.0);
    aggTn = preferences.getDouble("aggTn", 0.0);
    aggTv = preferences.getDouble("aggTv", 0.0);
    setPoint = preferences.getDouble("setPoint", 0.0);
    brewtime = preferences.getDouble("brewtime", 0.0);
    preinfusion = preferences.getDouble("preinf", 0.0);
    preinfusionpause = preferences.getDouble("preinfpau", 0.0);
    starttemp = preferences.getDouble("starttemp", 0.0);
    aggoKp = preferences.getDouble("aggoKp", 0.0);
    aggoTn = preferences.getDouble("aggoTn", 0.0);
    aggoTv = preferences.getDouble("aggoTv", 0.0);
    brewDetectionSensitivity = preferences.getDouble("bDetSen", 0.0);
    steadyPower = preferences.getDouble("stePow", 0.0);
    steadyPowerOffset = preferences.getDouble("stePowOff", 0.0);
    steadyPowerOffsetTime = preferences.getInt("stePowOT", 0);
    burstPower = preferences.getDouble("burstPower", 0.0);
    brewDetectionPower = preferences.getDouble("bDetPow", 0.0);
    pidON = preferences.getInt("pidON", 0);
    setPointSteam = preferences.getDouble("sPointSte", 0.0);
  }
  //always read the following values during setup() (which are not saved in blynk)
  if (startup_read && (current_version == expected_eeprom_version)) {
    estimated_cycle_refreshTemp = preferences.getUInt("estCycleRT");
  }

  //if blynk vars are not read previously, get latest values from EEPROM
  double aggKp_sav = 0;
  double aggTn_sav = 0;
  double aggTv_sav = 0;
  double aggoKp_sav = 0;
  double aggoTn_sav = 0;
  double aggoTv_sav = 0;
  double setPoint_sav = 0;
  double brewtime_sav = 0;
  double preinf_sav = 0;
  double preinfpau_sav = 0;
  double starttemp_sav = 0;
  double bDetSen_sav = 0;
  double stePow_sav = 0;
  double stePowOff_sav = 0;
  int stePowOT_sav = 0;
  double burstPower_sav = 0;
  unsigned int estCycleRT_sav = 0;
  double bDetPow_sav = 0;
  int pidON_sav = 0;
  double sPointSte_sav = 0;

  if (current_version == expected_eeprom_version) {
    aggKp_sav = preferences.getDouble("aggKp", 0.0);
    aggTn_sav = preferences.getDouble("aggTn", 0.0);
    aggTv_sav = preferences.getDouble("aggTv",0.0);
    setPoint_sav = preferences.getDouble("setPoint", 0.0);
    brewtime_sav = preferences.getDouble("brewtime", 0);
    preinf_sav = preferences.getDouble("preinf", 0.0);
    preinfpau_sav = preferences.getDouble("preinfpau", 0.0);
    starttemp_sav = preferences.getDouble("starttemp", 0.0);
    aggoKp_sav = preferences.getDouble("aggoKp", 0.0);
    aggoTn_sav = preferences.getDouble("aggoTn", 0.0);
    aggoTv_sav = preferences.getDouble("aggoTv", 0.0);
    bDetSen_sav = preferences.getDouble("bDetSen", 0.0);
    stePow_sav = preferences.getDouble("stePow", 0.0);
    stePowOff_sav = preferences.getDouble("stePowOff", 0.0);
    stePowOT_sav = preferences.getInt("stePowOT", 0);
    burstPower_sav = preferences.getDouble("burstPower", 0.0);
    estCycleRT_sav = preferences.getUInt("estCycleRT");
    bDetPow_sav = preferences.getDouble("bDetPow", 0.0);
    pidON_sav = preferences.getInt("pidON", 0);
    sPointSte_sav = preferences.getDouble("sPointSte", 0.0);
  }

  //get saved userConfig.h values
  double aggKp_cfg;
  double aggTn_cfg;
  double aggTv_cfg;
  double aggoKp_cfg;
  double aggoTn_cfg;
  double aggoTv_cfg;
  double setPoint_cfg;
  double brewtime_cfg;
  double preinf_cfg;
  double preinfpau_cfg;
  double starttemp_cfg;
  double bDetSen_cfg;
  double stePow_cfg;
  double stePowOff_cfg;
  int stePowOT_cfg;
  double burstPower_cfg;
  double bDetPow_cfg;
  double sPointSte_cfg;

  aggKp_cfg = preferences.getDouble("aggKp_cfg", 0.0);
  aggTn_cfg = preferences.getDouble("aggTn_cfg", 0.0);
  aggTv_cfg = preferences.getDouble("aggTv_cfg", 0.0);
  setPoint_cfg = preferences.getDouble("setPoint_cfg", 0.0);
  brewtime_cfg = preferences.getDouble("brewtime_cfg", 0.0);
  preinf_cfg = preferences.getDouble("preinf_cfg", 0.0);
  preinfpau_cfg = preferences.getDouble("preinfpau_cfg", 0.0);
  starttemp_cfg = preferences.getDouble("starttemp_cfg", 0.0);
  aggoKp_cfg = preferences.getDouble("aggoKp_cfg", 0.0);
  aggoTn_cfg = preferences.getDouble("aggoTn_cfg", 0.0);
  aggoTv_cfg = preferences.getDouble("aggoTv_cfg", 0.0);
  bDetSen_cfg = preferences.getDouble("bDetSen_cfg", 0.0);
  stePow_cfg = preferences.getDouble("stePow_cfg", 0.0);
  stePowOff_cfg = preferences.getDouble("stePowOff_cfg", 0.0);
  stePowOT_cfg = preferences.getInt("stePowOT_cfg");
  //burstPower_cfg = preferences.getDouble("burstPower_cfg", 0);
  bDetPow_cfg = preferences.getDouble("bDetPow_cfg", 0.0);
  sPointSte_cfg = preferences.getDouble("sPointSte_cfg", 0.0); 

  //use userConfig.h value if if differs from *_cfg
  if (AGGKP != aggKp_cfg) { aggKp = AGGKP; preferences.putDouble("aggKp_cfg", aggKp); }
  if (AGGTN != aggTn_cfg) { aggTn = AGGTN; preferences.putDouble("aggTn_cfg", aggTn); }
  if (AGGTV != aggTv_cfg) { aggTv = AGGTV; preferences.putDouble("aggTv_cfg", aggTv); }
  if (AGGOKP != aggoKp_cfg) { aggoKp = AGGOKP; preferences.putDouble("aggoKp_cfg", aggoKp); }
  if (AGGOTN != aggoTn_cfg) { aggoTn = AGGOTN; preferences.putDouble("aggoTn_cfg", aggoTn); }
  if (AGGOTV != aggoTv_cfg) { aggoTv = AGGOTV; preferences.putDouble("aggoTv_cfg", aggoTv); }
  if (SETPOINT != setPoint_cfg) { setPoint = SETPOINT; preferences.putDouble("setPoint_cfg", setPoint); DEBUG_print("EEPROM: setPoint (%0.2f) is read from userConfig.h\n", setPoint); }
  if (BREWTIME != brewtime_cfg) { brewtime = BREWTIME; preferences.putDouble("brewtime_cfg", brewtime); DEBUG_print("EEPROM: brewtime (%0.2f) is read from userConfig.h (prev:%0.2f)\n",brewtime, brewtime_cfg); }
  if (PREINFUSION != preinf_cfg) { preinfusion = PREINFUSION; preferences.putDouble("preinf_cfg", preinfusion); }
  if (PREINFUSION_PAUSE != preinfpau_cfg) { preinfusionpause = PREINFUSION_PAUSE; preferences.putDouble("preinfpau_cfg", preinfusionpause); }
  if (STARTTEMP != starttemp_cfg) { starttemp = STARTTEMP; preferences.putDouble("starttemp_cfg", starttemp); DEBUG_print("EEPROM: starttemp (%0.2f) is read from userConfig.h (prev:%0.2f)\n", starttemp, starttemp_cfg); }
  if (BREWDETECTION_SENSITIVITY != bDetSen_cfg) { brewDetectionSensitivity = BREWDETECTION_SENSITIVITY; preferences.putDouble("bDetSen_cfg", brewDetectionSensitivity); }
  if (STEADYPOWER != stePow_cfg) { steadyPower = STEADYPOWER; preferences.putDouble("stePow_cfg", steadyPower); }
  if (STEADYPOWER_OFFSET != stePowOff_cfg) { steadyPowerOffset = STEADYPOWER_OFFSET; preferences.putDouble("stePowOff_cfg", steadyPowerOffset); }
  if (STEADYPOWER_OFFSET_TIME != stePowOT_cfg) { steadyPowerOffsetTime = STEADYPOWER_OFFSET_TIME; preferences.putInt("stePowOT_cfg", steadyPowerOffsetTime); }
  //if (BURSTPOWER != burstPower_cfg) { burstPower = BURSTPOWER; preferences.putDouble(470, burstPower); }
  if (BREWDETECTION_POWER != bDetPow_cfg) { brewDetectionPower = BREWDETECTION_POWER; preferences.putDouble("bDetPow_cfg", brewDetectionPower); DEBUG_print("EEPROM: brewDetectionPower (%0.2f) is read from userConfig.h\n", brewDetectionPower); }
  if (SETPOINT_STEAM != sPointSte_cfg) { setPointSteam = SETPOINT_STEAM; preferences.putDouble("sPointSte_cfg", setPointSteam); DEBUG_print("EEPROM: setPointSteam (%0.2f) is read from userConfig.h\n", setPointSteam); }

  //save latest values to eeprom and sync back to blynk
  if ( aggKp != aggKp_sav) { preferences.putDouble("aggKp", aggKp); Blynk.virtualWrite(V4, aggKp); }
  if ( aggTn != aggTn_sav) { preferences.putDouble("aggTn", aggTn); Blynk.virtualWrite(V5, aggTn); }
  if ( aggTv != aggTv_sav) { preferences.putDouble("aggTv", aggTv); Blynk.virtualWrite(V6, aggTv); }
  if ( setPoint != setPoint_sav) { preferences.putDouble("setPoint", setPoint); Blynk.virtualWrite(V7, setPoint); DEBUG_print("EEPROM: setPoint (%0.2f) is saved\n", setPoint); }
  if ( brewtime != brewtime_sav) { preferences.putDouble("brewtime", brewtime); Blynk.virtualWrite(V8, brewtime); DEBUG_print("EEPROM: brewtime (%0.2f) is saved (previous:%0.2f)\n", brewtime, brewtime_sav); }
  if ( preinfusion != preinf_sav) { preferences.putDouble("preinf", preinfusion); Blynk.virtualWrite(V9, preinfusion); }
  if ( preinfusionpause != preinfpau_sav) { preferences.putDouble("preinfpau", preinfusionpause); Blynk.virtualWrite(V10, preinfusionpause); }
  if ( starttemp != starttemp_sav) { preferences.putDouble("starttemp", starttemp); Blynk.virtualWrite(V12, starttemp); DEBUG_print("EEPROM: starttemp (%0.2f) is saved\n", starttemp); }
  if ( aggoKp != aggoKp_sav) { preferences.putDouble("aggoKp", aggoKp); Blynk.virtualWrite(V30, aggoKp); }
  if ( aggoTn != aggoTn_sav) { preferences.putDouble("aggoTn", aggoTn); Blynk.virtualWrite(V31, aggoTn); }
  if ( aggoTv != aggoTv_sav) { preferences.putDouble("aggoTv", aggoTv); Blynk.virtualWrite(V32, aggoTv); }
  if ( brewDetectionSensitivity != bDetSen_sav) { preferences.putDouble("bDetSen", brewDetectionSensitivity); Blynk.virtualWrite(V34, brewDetectionSensitivity); }
  if ( steadyPower != stePow_sav) { preferences.putDouble("stePow", steadyPower); Blynk.virtualWrite(V41, steadyPower); DEBUG_print("EEPROM: steadyPower (%0.2f) is saved (previous:%0.2f)\n", steadyPower, stePow_sav); }
  if ( steadyPowerOffset != stePowOff_sav) { preferences.putDouble("stePowOff", steadyPowerOffset); Blynk.virtualWrite(V42, steadyPowerOffset); }
  if ( steadyPowerOffsetTime != stePowOT_sav) { preferences.putInt("stePowOT", steadyPowerOffsetTime); Blynk.virtualWrite(V43, steadyPowerOffsetTime); }
  if ( burstPower != burstPower_sav) { preferences.putDouble("burstPower", burstPower); Blynk.virtualWrite(V44, burstPower); }
  if ( estimated_cycle_refreshTemp != estCycleRT_sav) { preferences.putUInt("estCycleRT", estimated_cycle_refreshTemp); DEBUG_print("EEPROM: estimated_cycle_refreshTemp (%u) is saved (previous:%u)\n", estimated_cycle_refreshTemp, estCycleRT_sav); }
  if ( brewDetectionPower != bDetPow_sav) { preferences.putDouble("bDetPow", brewDetectionPower); Blynk.virtualWrite(V36, brewDetectionPower); DEBUG_print("EEPROM: brewDetectionPower (%0.2f) is saved (previous:%0.2f)\n", brewDetectionPower, bDetPow_sav); }
  if ( pidON != pidON_sav) { preferences.putInt("pidON", pidON); Blynk.virtualWrite(V13, pidON); DEBUG_print("EEPROM: pidON (%d) is saved (previous:%d)\n", pidON, pidON_sav); }
  if ( setPointSteam != sPointSte_sav) { preferences.putDouble("sPointSte", setPointSteam); Blynk.virtualWrite(V50, setPointSteam); DEBUG_print("EEPROM: setPointSteam (%0.2f) is saved\n", setPointSteam); }
  preferences.end();
  DEBUG_print("EEPROM: sync_eeprom() finished.\n");
}
#else
void sync_eeprom(bool startup_read, bool force_read) {
  int current_version;
  DEBUG_print("EEPROM: sync_eeprom(startup_read=%d, force_read=%d) called\n", startup_read, force_read);
  EEPROM.begin(512);
  EEPROM.get(290, current_version);
  DEBUG_print("EEPROM: Detected Version=%d Expected Version=%d\n", current_version, expected_eeprom_version);
  if (current_version != expected_eeprom_version) {
    ERROR_print("EEPROM: Version has changed or settings are corrupt or not previously set. Ignoring..\n");
    EEPROM.put(290, expected_eeprom_version);
  }

  //if variables are not read from blynk previously, always get latest values from EEPROM
  if (force_read && (current_version == expected_eeprom_version)) {
    DEBUG_print("EEPROM: Blynk not active and not using external mqtt server. Reading settings from EEPROM\n");
    EEPROM.get(0, aggKp);
    EEPROM.get(10, aggTn);
    EEPROM.get(20, aggTv);
    EEPROM.get(30, setPoint);
    EEPROM.get(40, brewtime);
    EEPROM.get(50, preinfusion);
    EEPROM.get(60, preinfusionpause);
    EEPROM.get(80, starttemp);
    EEPROM.get(90, aggoKp);
    EEPROM.get(100, aggoTn);
    EEPROM.get(110, aggoTv);
    EEPROM.get(130, brewDetectionSensitivity);
    EEPROM.get(140, steadyPower);
    EEPROM.get(150, steadyPowerOffset);
    EEPROM.get(160, steadyPowerOffsetTime);
    EEPROM.get(170, burstPower);
    //180 is used
    EEPROM.get(190, brewDetectionPower);
    EEPROM.get(200, pidON);
    EEPROM.get(210, setPointSteam);
    //Reminder: 290 is reserved for "version"
  }
  //always read the following values during setup() (which are not saved in blynk)
  if (startup_read && (current_version == expected_eeprom_version)) {
    EEPROM.get(180, estimated_cycle_refreshTemp);
  }

  //if blynk vars are not read previously, get latest values from EEPROM
  double aggKp_sav = 0;
  double aggTn_sav = 0;
  double aggTv_sav = 0;
  double aggoKp_sav = 0;
  double aggoTn_sav = 0;
  double aggoTv_sav = 0;
  double setPoint_sav = 0;
  double brewtime_sav = 0;
  double preinf_sav = 0;
  double preinfpau_sav = 0;
  double starttemp_sav = 0;
  double bDetSen_sav = 0;
  double stePow_sav = 0;
  double stePowOff_sav = 0;
  int stePowOT_sav = 0;
  double burstPower_sav = 0;
  unsigned int estCycleRT_sav = 0;
  double bDetPow_sav = 0;
  int pidON_sav = 0;
  double sPointSte_sav = 0;

  if (current_version == expected_eeprom_version) {
    EEPROM.get(0, aggKp_sav);
    EEPROM.get(10, aggTn_sav);
    EEPROM.get(20, aggTv_sav);
    EEPROM.get(30, setPoint_sav);
    EEPROM.get(40, brewtime_sav);
    EEPROM.get(50, preinf_sav);
    EEPROM.get(60, preinfpau_sav);
    EEPROM.get(80, starttemp_sav);
    EEPROM.get(90, aggoKp_sav);
    EEPROM.get(100, aggoTn_sav);
    EEPROM.get(110, aggoTv_sav);
    EEPROM.get(130, bDetSen_sav);
    EEPROM.get(140, stePow_sav);
    EEPROM.get(150, stePowOff_sav);
    EEPROM.get(160, stePowOT_sav);
    EEPROM.get(170, burstPower_sav);
    EEPROM.get(180, estCycleRT_sav);
    EEPROM.get(190, bDetPow_sav);
    EEPROM.get(200, pidON_sav);
    EEPROM.get(210, sPointSte_sav);
  }

  //get saved userConfig.h values
  double aggKp_cfg;
  double aggTn_cfg;
  double aggTv_cfg;
  double aggoKp_cfg;
  double aggoTn_cfg;
  double aggoTv_cfg;
  double setPoint_cfg;
  double brewtime_cfg;
  double preinf_cfg;
  double preinfpau_cfg;
  double starttemp_cfg;
  double bDetSen_cfg;
  double stePow_cfg;
  double stePowOff_cfg;
  int stePowOT_cfg;
  double burstPower_cfg;
  double bDetPow_cfg;
  double sPointSte_cfg;

  EEPROM.get(300, aggKp_cfg);
  EEPROM.get(310, aggTn_cfg);
  EEPROM.get(320, aggTv_cfg);
  EEPROM.get(330, setPoint_cfg);
  EEPROM.get(340, brewtime_cfg);
  EEPROM.get(350, preinf_cfg);
  EEPROM.get(360, preinfpau_cfg);
  EEPROM.get(380, starttemp_cfg);
  EEPROM.get(390, aggoKp_cfg);
  EEPROM.get(400, aggoTn_cfg);
  EEPROM.get(410, aggoTv_cfg);
  EEPROM.get(430, bDetSen_cfg);
  EEPROM.get(440, stePow_cfg);
  EEPROM.get(450, stePowOff_cfg);
  EEPROM.get(460, stePowOT_cfg);
  EEPROM.get(470, burstPower_cfg);
  EEPROM.get(490, bDetPow_cfg);
  EEPROM.get(334, sPointSte_cfg); 

  //use userConfig.h value if if differs from *_cfg
  if (AGGKP != aggKp_cfg) { aggKp = AGGKP; EEPROM.put(300, aggKp); }
  if (AGGTN != aggTn_cfg) { aggTn = AGGTN; EEPROM.put(310, aggTn); }
  if (AGGTV != aggTv_cfg) { aggTv = AGGTV; EEPROM.put(320, aggTv); }
  if (AGGOKP != aggoKp_cfg) { aggoKp = AGGOKP; EEPROM.put(390, aggoKp); }
  if (AGGOTN != aggoTn_cfg) { aggoTn = AGGOTN; EEPROM.put(400, aggoTn); }
  if (AGGOTV != aggoTv_cfg) { aggoTv = AGGOTV; EEPROM.put(410, aggoTv); }
  if (SETPOINT != setPoint_cfg) { setPoint = SETPOINT; EEPROM.put(330, setPoint); DEBUG_print("EEPROM: setPoint (%0.2f) is read from userConfig.h\n", setPoint); }
  if (BREWTIME != brewtime_cfg) { brewtime = BREWTIME; EEPROM.put(340, brewtime); DEBUG_print("EEPROM: brewtime (%0.2f) is read from userConfig.h\n", brewtime); }
  if (PREINFUSION != preinf_cfg) { preinfusion = PREINFUSION; EEPROM.put(350, preinfusion); }
  if (PREINFUSION_PAUSE != preinfpau_cfg) { preinfusionpause = PREINFUSION_PAUSE; EEPROM.put(360, preinfusionpause); }
  if (STARTTEMP != starttemp_cfg) { starttemp = STARTTEMP; EEPROM.put(380, starttemp); DEBUG_print("EEPROM: starttemp (%0.2f) is read from userConfig.h\n", starttemp); }
  if (BREWDETECTION_SENSITIVITY != bDetSen_cfg) { brewDetectionSensitivity = BREWDETECTION_SENSITIVITY; EEPROM.put(430, brewDetectionSensitivity); }
  if (STEADYPOWER != stePow_cfg) { steadyPower = STEADYPOWER; EEPROM.put(440, steadyPower); }
  if (STEADYPOWER_OFFSET != stePowOff_cfg) { steadyPowerOffset = STEADYPOWER_OFFSET; EEPROM.put(450, steadyPowerOffset); }
  if (STEADYPOWER_OFFSET_TIME != stePowOT_cfg) { steadyPowerOffsetTime = STEADYPOWER_OFFSET_TIME; EEPROM.put(460, steadyPowerOffsetTime); }
  //if (BURSTPOWER != burstPower_cfg) { burstPower = BURSTPOWER; EEPROM.put(470, burstPower); }
  if (BREWDETECTION_POWER != bDetPow_cfg) { brewDetectionPower = BREWDETECTION_POWER; EEPROM.put(490, brewDetectionPower); DEBUG_print("EEPROM: brewDetectionPower (%0.2f) is read from userConfig.h\n", brewDetectionPower); }
  if (SETPOINT_STEAM != sPointSte_cfg) { setPointSteam = SETPOINT_STEAM; EEPROM.put(334, setPointSteam); DEBUG_print("EEPROM: setPointSteam (%0.2f) is read from userConfig.h\n", setPointSteam); }

  //save latest values to eeprom and sync back to blynk
  if ( aggKp != aggKp_sav) { EEPROM.put(0, aggKp); Blynk.virtualWrite(V4, aggKp); }
  if ( aggTn != aggTn_sav) { EEPROM.put(10, aggTn); Blynk.virtualWrite(V5, aggTn); }
  if ( aggTv != aggTv_sav) { EEPROM.put(20, aggTv); Blynk.virtualWrite(V6, aggTv); }
  if ( setPoint != setPoint_sav) { EEPROM.put(30, setPoint); Blynk.virtualWrite(V7, setPoint); DEBUG_print("EEPROM: setPoint (%0.2f) is saved\n", setPoint); }
  if ( brewtime != brewtime_sav) { EEPROM.put(40, brewtime); Blynk.virtualWrite(V8, brewtime); DEBUG_print("EEPROM: brewtime (%0.2f) is saved (previous:%0.2f)\n", brewtime, brewtime_sav); }
  if ( preinfusion != preinf_sav) { EEPROM.put(50, preinfusion); Blynk.virtualWrite(V9, preinfusion); }
  if ( preinfusionpause != preinfpau_sav) { EEPROM.put(60, preinfusionpause); Blynk.virtualWrite(V10, preinfusionpause); }
  if ( starttemp != starttemp_sav) { EEPROM.put(80, starttemp); Blynk.virtualWrite(V12, starttemp); DEBUG_print("EEPROM: starttemp (%0.2f) is saved\n", starttemp); }
  if ( aggoKp != aggoKp_sav) { EEPROM.put(90, aggoKp); Blynk.virtualWrite(V30, aggoKp); }
  if ( aggoTn != aggoTn_sav) { EEPROM.put(100, aggoTn); Blynk.virtualWrite(V31, aggoTn); }
  if ( aggoTv != aggoTv_sav) { EEPROM.put(110, aggoTv); Blynk.virtualWrite(V32, aggoTv); }
  if ( brewDetectionSensitivity != bDetSen_sav) { EEPROM.put(130, brewDetectionSensitivity); Blynk.virtualWrite(V34, brewDetectionSensitivity); }
  if ( steadyPower != stePow_sav) { EEPROM.put(140, steadyPower); Blynk.virtualWrite(V41, steadyPower); DEBUG_print("EEPROM: steadyPower (%0.2f) is saved (previous:%0.2f)\n", steadyPower, stePow_sav); }
  if ( steadyPowerOffset != stePowOff_sav) { EEPROM.put(150, steadyPowerOffset); Blynk.virtualWrite(V42, steadyPowerOffset); }
  if ( steadyPowerOffsetTime != stePowOT_sav) { EEPROM.put(160, steadyPowerOffsetTime); Blynk.virtualWrite(V43, steadyPowerOffsetTime); }
  if ( burstPower != burstPower_sav) { EEPROM.put(170, burstPower); Blynk.virtualWrite(V44, burstPower); }
  if ( estimated_cycle_refreshTemp != estCycleRT_sav) { EEPROM.put(180, estimated_cycle_refreshTemp); DEBUG_print("EEPROM: estimated_cycle_refreshTemp (%u) is saved (previous:%u)\n", estimated_cycle_refreshTemp, estCycleRT_sav); }
  if ( brewDetectionPower != bDetPow_sav) { EEPROM.put(190, brewDetectionPower); Blynk.virtualWrite(V36, brewDetectionPower); DEBUG_print("EEPROM: brewDetectionPower (%0.2f) is saved (previous:%0.2f)\n", brewDetectionPower, bDetPow_sav); }
  if ( pidON != pidON_sav) { EEPROM.put(200, pidON); Blynk.virtualWrite(V13, pidON); DEBUG_print("EEPROM: pidON (%d) is saved (previous:%d)\n", pidON, pidON_sav); }
  if ( setPointSteam != sPointSte_sav) { EEPROM.put(210, setPointSteam); Blynk.virtualWrite(V50, setPointSteam); DEBUG_print("EEPROM: setPointSteam (%0.2f) is saved\n", setPointSteam); }
  EEPROM.commit();
  DEBUG_print("EEPROM: sync_eeprom() finished.\n");
}
#endif

void performance_check() {
  loops += 1;
  cur_micros = micros();
  if (max_micros < cur_micros-cur_micros_previous_loop) {
      max_micros = cur_micros-cur_micros_previous_loop;
  }
  if ( cur_micros >= last_report_micros + 100000 ) { //100ms
    snprintf(debugline, sizeof(debugline), "%lu loop() | loops/ms=%lu | spend_micros_last_loop=%lu | max_micros_since_last_report=%lu | avg_micros/loop=%lu", 
        cur_micros/1000, loops/100, (cur_micros-cur_micros_previous_loop), max_micros, (cur_micros - last_report_micros)/loops);
    DEBUG_println(debugline);
    last_report_micros = cur_micros;
    max_micros = 0;
    loops=0;
  }
  cur_micros_previous_loop = cur_micros;
}
  
void print_settings() {
  DEBUG_print("========================\n");
  DEBUG_print("Machine: %s | Version: %s\n", MACHINE_TYPE, sysVersion);
  DEBUG_print("aggKp: %0.2f | aggTn: %0.2f | aggTv: %0.2f\n", aggKp, aggTn, aggTv);
  DEBUG_print("aggoKp: %0.2f | aggoTn: %0.2f | aggoTv: %0.2f\n", aggoKp, aggoTn, aggoTv);
  DEBUG_print("starttemp: %0.2f | burstPower: %0.2f\n", starttemp, burstPower);
  DEBUG_print("setPoint: %0.2f | setPointSteam: %0.2f | activeSetPoint: %0.2f | \n", setPoint, setPointSteam, *activeSetPoint);
  DEBUG_print("brewDetection: %d | brewDetectionSensitivity: %0.2f | brewDetectionPower: %0.2f\n", brewDetection, brewDetectionSensitivity, brewDetectionPower);
  DEBUG_print("brewtime: %0.2f | preinfusion: %0.2f | preinfusionpause: %0.2f\n", brewtime, preinfusion, preinfusionpause);
  DEBUG_print("steadyPower: %0.2f | steadyPowerOffset: %0.2f | steadyPowerOffsetTime: %d\n", steadyPower, steadyPowerOffset, steadyPowerOffsetTime);
  DEBUG_print("pidON: %d\n", pidON);
  printControlsConfig(controlsConfig);
  DEBUG_print("========================\n");
}


/***********************************
 * SETUP()
 ***********************************/
void setup() {
  bool eeprom_force_read = true;
  DEBUGSTART(115200);

  //required for remoteDebug to work
  #ifdef ESP32
  WiFi.mode(WIFI_STA);
  #endif
  Debug.begin(hostname, Debug.DEBUG);
  Debug.setResetCmdEnabled(true); // Enable the reset command
  Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
  Debug.showColors(true); // Colors
  Debug.setSerialEnabled(true); // log to Serial also
  //Serial.setDebugOutput(true); // enable diagnostic output of WiFi libraries
  Debug.setCallBackNewClient(&print_settings);

  /********************************************************
    Define trigger type
  ******************************************************/
  if (triggerType)
  {
    relayON = HIGH;
    relayOFF = LOW;
  } else {
    relayON = LOW;
    relayOFF = HIGH;
  }

  /********************************************************
    Ini Pins
  ******************************************************/
  pinMode(pinRelayVentil, OUTPUT);
  digitalWrite(pinRelayVentil, relayOFF);
  pinMode(pinRelayPumpe, OUTPUT);
  digitalWrite(pinRelayPumpe, relayOFF);
  pinMode(pinRelayHeater, OUTPUT);
  digitalWrite(pinRelayHeater, LOW);
  #if (ENABLE_HARDWARE_LED == 1)
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);
  setHardwareLed(1);
  #elif (ENABLE_HARDWARE_LED == 2)
  pinMode(pinLed, OUTPUT);
  FastLED.addLeds<WS2812B, pinLed, GRB>(leds, enabledHardwareLedNumber);
  setHardwareLed(1);
  #endif

  DEBUG_print("\nMachine: %s\nVersion: %s\n", MACHINE_TYPE, sysVersion);

  #if defined(OVERWRITE_VERSION_DISPLAY_TEXT)
  displaymessage(0, (char*) DISPLAY_TEXT, (char*) OVERWRITE_VERSION_DISPLAY_TEXT);
  #else
  displaymessage(0, (char*) DISPLAY_TEXT, (char*) sysVersion);
  #endif

  delay(1000);

  controlsConfig = parseControlsConfig();
  configureControlsHardware(controlsConfig);

  //if simulatedBrewSwitch is already "on" on startup, then we brew should not start automatically
  if (simulatedBrewSwitch) {
    DEBUG_print("brewsitch is already on. Dont brew until it is turned off.\n");
    waitingForBrewSwitchOff=true;
  }

  /********************************************************
   Ini PID
  ******************************************************/
  bPID.SetSampleTime(windowSize);
  bPID.SetOutputLimits(0, windowSize);
  bPID.SetMode(AUTOMATIC);
  
  /********************************************************
     BLYNK & Fallback offline
  ******************************************************/
  if (!force_offline) {
    checkWifi(true, 12000); // wait up to 12 seconds for connection
    if (!wifi_working()) {
      ERROR_print("Cannot connect to WIFI %s. Disabling WIFI\n", ssid);
      if (DISABLE_SERVICES_ON_STARTUP_ERRORS) {
        force_offline = true;
        mqtt_disabled_temporary = true;
        blynk_disabled_temporary = true;
        lastWifiConnectionAttempt = millis();
      }
      displaymessage(0, "Cant connect to Wifi", "");
      delay(1000);
    } else {
      DEBUG_print("IP address: %s\n", WiFi.localIP().toString().c_str());

      // MQTT
      #if (MQTT_ENABLE == 1)
        snprintf(topic_will, sizeof(topic_will), "%s%s/%s", mqtt_topic_prefix, hostname, "will");
        snprintf(topic_set, sizeof(topic_set), "%s%s/+/%s", mqtt_topic_prefix, hostname, "set");
        snprintf(topic_actions, sizeof(topic_actions), "%s%s/actions/+", mqtt_topic_prefix, hostname);
        mqtt_client.setServer(mqtt_server_ip, mqtt_server_port);
        mqtt_client.setCallback(mqtt_callback);
        if (!mqtt_reconnect(true)) {
          if (DISABLE_SERVICES_ON_STARTUP_ERRORS) mqtt_disabled_temporary = true;
          ERROR_print("Cannot connect to MQTT. Disabling...\n");
          //displaymessage(0, "Cannt connect to MQTT", "");
          //delay(1000);
        } else {
          const bool useRetainedSettingsFromMQTT = true;
          if (useRetainedSettingsFromMQTT) {
            //read and use settings retained in mqtt and therefore dont use eeprom values
            eeprom_force_read = false;
            unsigned long started = millis();
            while (mqtt_working() && (millis() < started + 2000))  //attention: delay might not be long enough over WAN
            {
              mqtt_client.loop();
            }
            force_eeprom_sync = 0;
          }
        }
      #elif (MQTT_ENABLE == 2)
        DEBUG_print("Starting MQTT service\n");
        const unsigned int max_subscriptions = 30;
        const unsigned int max_retained_topics = 30;
        const unsigned int mqtt_service_port = 1883;
        snprintf(topic_set, sizeof(topic_set), "%s%s/+/%s", mqtt_topic_prefix, hostname, "set");
        snprintf(topic_actions, sizeof(topic_actions), "%s%s/actions/+", mqtt_topic_prefix, hostname);
        MQTT_server_onData(mqtt_callback);
        if (MQTT_server_start(mqtt_service_port, max_subscriptions, max_retained_topics)) {
          if (!MQTT_local_subscribe((unsigned char *)topic_set, 0) || !MQTT_local_subscribe((unsigned char *)topic_actions, 0)) {
            ERROR_print("Cannot subscribe to local MQTT service\n");
          }
        } else {
          if (DISABLE_SERVICES_ON_STARTUP_ERRORS) mqtt_disabled_temporary = true;
          ERROR_print("Cannot create MQTT service. Disabling...\n");
          //displaymessage(0, "Cannt create MQTT service", "");
          //delay(1000);
        }
      #endif

      if (BLYNK_ENABLE) {
        DEBUG_print("Connecting to Blynk ...\n");
        Blynk.config(blynkauth, blynkaddress, blynkport) ;
        if (!Blynk.connect(5000)) {
          if (DISABLE_SERVICES_ON_STARTUP_ERRORS) blynk_disabled_temporary = true;
          ERROR_print("Cannot connect to Blynk. Disabling...\n");
          //displaymessage(0, "Cannt connect to Blynk", "");
          //delay(1000);
        } else {
          //displaymessage(0, "3: Blynk connected", "sync all variables...");
          DEBUG_print("Blynk is online, get latest values\n");
          unsigned long started = millis();
          while (blynk_working() && (millis() < started + 2000))
          {
            Blynk.run();
          }
          eeprom_force_read = false;
        }
      }
    }

  } else {
    DEBUG_print("Staying offline due to force_offline=1\n");
  }


  /********************************************************
   * READ/SAVE EEPROM
   *  get latest values from EEPROM if not already fetched from blynk or remote mqtt-server
   *  Additionally this function honors changed values in userConfig.h (changed userConfig.h values have priority)
  ******************************************************/
  sync_eeprom(true, eeprom_force_read);

  print_settings();

  /********************************************************
   * PUBLISH settings on MQTT (and wait for them to be processed!)
   * + SAVE settings on MQTT-server if MQTT_ENABLE==1
  ******************************************************/
  steadyPowerSaved = steadyPower;
  if (mqtt_working()) {
    steadyPowerMQTTDisableUpdateUntilProcessed = steadyPower;
    steadyPowerMQTTDisableUpdateUntilProcessedTime = millis();
    mqtt_publish_settings();
    #if (MQTT_ENABLE == 1)
    unsigned long started = millis();
    while ((millis() < started + 5000) && (steadyPowerMQTTDisableUpdateUntilProcessed != 0))
    {
      mqtt_client.loop();
    }
    #endif
  }

  /********************************************************
     OTA
  ******************************************************/
  if (ota && !force_offline) {
    //wifi connection is done during blynk connection
    ArduinoOTA.setHostname(hostname);  //  Device name for OTA
    ArduinoOTA.setPassword(OTApass);  //  Password for OTA
    ArduinoOTA.begin();
  }

  /********************************************************
    movingaverage ini array
  ******************************************************/
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingstemp[thisReading] = 0;
    readingstime[thisReading] = 0;
  }

  /********************************************************
     TEMP SENSOR
  ******************************************************/
  //displaymessage(0, "Init. vars", "");
  if (TempSensor == 1) {
    sensors.begin();
    sensors.getAddress(sensorDeviceAddress, 0);
    sensors.setResolution(sensorDeviceAddress, 10) ;
    while (true) {
      sensors.requestTemperatures();
      previousInput = sensors.getTempCByIndex(0);
      delay(400);
      sensors.requestTemperatures();
      Input = sensors.getTempCByIndex(0);
      if (checkSensor(Input, previousInput)) {
        updateTemperatureHistory(Input);
        break;
      }
      displaymessage(0, "Temp. sensor defect", "");
      ERROR_print("Temp. sensor defect. Cannot read consistant values. Retrying\n");
      delay(400);
    }
  } else if (TempSensor == 2) {
    isrCounter = 950;  //required
    #ifdef USE_ZACWIRE_TSIC
    if (TSIC.begin() != true) {
      ERROR_println("TSIC Tempsensor cannot be initialized");
    }
    delay(120);
    while (true) {
      //previousInput = temperature_simulate_steam();
      //previousInput = temperature_simulate_normal();
      previousInput = TSIC.getTemp();
      delay(200);
      //Input = temperature_simulate_steam();
      //Input = temperature_simulate_normal();
      Input = TSIC.getTemp();
      if (checkSensor(Input, previousInput)) {
        updateTemperatureHistory(Input);
        break;
      }
      displaymessage(0, "Temp. sensor defect", "");
      ERROR_print("Temp. sensor defect. Cannot read consistant values. Retrying\n");
      delay(1000);
    }
    #else
    attachInterrupt(digitalPinToInterrupt(pinTemperature), readTSIC, RISING); //activate TSIC reading
    delay(200);
    while (true) {
      previousInput = getTSICvalue();
      delay(200);
      Input = getTSICvalue();
      if (checkSensor(Input, previousInput)) {
        updateTemperatureHistory(Input);
        break;
      }
      displaymessage(0, "Temp. sensor defect", "");
      ERROR_print("Temp. sensor defect. Cannot read consistant values. Retrying\n");
      delay(1000);
    }
    #endif
  }

  /********************************************************
     REST INIT()
  ******************************************************/
  setHardwareLed(0);
  //Initialisation MUST be at the very end of the init(), otherwise the time comparison in loop() will have a big offset
  unsigned long currentTime = millis();
  previousMillistemp = currentTime;
  previousMillisBlynk = currentTime + 800;
  lastMQTTStatusReportTime = currentTime + 300;
  pidComputeLastRunTime = currentTime;

  /********************************************************
    Timer1 ISR - Initialisierung
    TIM_DIV1 = 0,   //80MHz (80 ticks/us - 104857.588 us max)
    TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
    TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
  ******************************************************/
  isrCounter = 0;
  #ifdef ESP32
  timer = timerBegin(0, 80, true);  // 1Mhz  
  timerAttachInterrupt(timer, &onTimer1ISR, true);
  timerAlarmWrite(timer, 10000, true);  //10ms
  timerAlarmEnable(timer);  
  #else
  timer1_isr_init();
  timer1_attachInterrupt(onTimer1ISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(50000); // set interrupt time to 10ms
  #endif
  DEBUG_print("End of setup()\n");
}

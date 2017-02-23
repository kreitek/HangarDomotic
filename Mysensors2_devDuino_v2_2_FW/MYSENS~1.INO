/*
This sketch is for a devDuino SN v2.2 with the support OTA 
http://www.seeedstudio.com/wiki/DevDuino_Sensor_Node_V2.2_(ATmega_328)
and MySensors 1.5

Press the button D4 when a voltage is applied.
If you want to download the new firmware over the air.

This sketch is a modification of code written
Version 1.3 - Thomas Bowman Mørch
for sensor Sensebender Micro
http://www.mysensors.org/hardware/micro

 modified
 18 December 2015
 by greengo
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Select pin 9 (devduino2.2 LED) for debug. 
// Select NC pin (p.e. 5) for avoid battery energy consumption!
#define MY_DEFAULT_TX_LED_PIN 5
#define MY_WITH_LEDS_BLINKING_INVERSE

// Define a static node address, remove if you want auto address assignment
#define MY_NODE_ID 23

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Set ISP devduino pins for NRF24
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 6

#define MY_OTA_FIRMWARE_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <sha204_library.h>

// Uncomment the line below, to transmit battery voltage as a normal sensor value
#define BATT_SENSOR    2

#define RELEASE "2.0.1"

// How many milli seconds between each measurement
#define MEASURE_INTERVAL 50000 // Normal 50s. for Debug 10 sec

// How many milli seconds should we wait for OTA?
#define OTA_WAIT_PERIOD 300

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 30  // Normal 30. 10 for Debug

//LED Blink wait for OTA? LED blinks during data transmission. Greater battery energy consumption!
//#define LED_BLINK_WAIT_TRANSMIT

#define TEMP_TRANSMIT_THRESHOLD 0.5

#define OTA_ENABLE        4 // Button D4
#define LED_PIN           9 // LED 
#define ATSHA204_PIN     A2 // ATSHA204A 
#define TEMP_PIN         A3 // Temp sensor MCP9700
#define BUTTON_PIN        3 // Arduino Digital I/O pin for button/reed switch

#define CHILD_TEMP_ID  0
#define CHILD_TRIP_ID  1
#define CHILD_BATT_ID  BATT_SENSOR

int TEMP_SENSE_PIN = TEMP_PIN;
float TEMP_SENSE_OFFSET = -0.01;

const int sha204Pin = ATSHA204_PIN;
atsha204Class sha204(sha204Pin);

float temp = 0;

MyMessage msgTemp(CHILD_TEMP_ID, V_TEMP);

#ifdef BATT_SENSOR
MyMessage msgBatt(CHILD_BATT_ID, V_VOLTAGE);
#endif

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage msg(CHILD_TRIP_ID, V_TRIPPED);

// Global settings
int measureCount = 0;
boolean ota_enabled = false; 
int sendBattery = 0;
boolean highfreq = true;
boolean transmission_occured = false;
Bounce debouncer = Bounce();
bool awake = false;

// Storage of old measurements
float lastTemperature = 0;
long lastBattery = 0;
float sendVCC;
int oldValue=-1;

// the setup function runs once when you press reset or power the board
void setup() {
 
  // initialize digital pin 9 as an output.
  pinMode(LED_PIN, OUTPUT);

  pinMode(OTA_ENABLE, INPUT);
  digitalWrite(OTA_ENABLE, HIGH);
  if (!digitalRead(OTA_ENABLE)) {
    ota_enabled = true;
  }

  // Make sure that ATSHA204 is not floating
  pinMode(ATSHA204_PIN, INPUT);
  digitalWrite(ATSHA204_PIN, HIGH);
 
  digitalWrite(OTA_ENABLE, LOW); // remove pullup, save some power. 
  
  digitalWrite(LED_PIN, HIGH);

#ifndef MY_NODE_ID
  getNodeId();
#endif 
  
  digitalWrite(LED_PIN, LOW);

  // Setup the button
  pinMode(BUTTON_PIN,INPUT);
  //// Activate internal pull-up
  // not necery with grove magnetic switch
  //digitalWrite(BUTTON_PIN,HIGH);

  // After setting up the button, setup debouncer
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5);
  
  Serial.flush();
  Serial.println(F(" - Online!")); 

sendTempMeasurements(false);
sendBattLevel(false);
sendTripStatus();
 
 if (ota_enabled) Serial.println("OTA FW update enabled");
  
}

void presentation() {
  // void sendSketchInfo(const char *name, const char *version, bool ack);
  sendSketchInfo("devDuino SNv2.2", RELEASE);
  
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  present(CHILD_TRIP_ID, S_DOOR, "Magnetic window/door sensor");

  present(CHILD_TEMP_ID, S_TEMP, "Devduino2.2 temperature sensor");
  
#ifdef BATT_SENSOR
  present(CHILD_BATT_ID, S_MULTIMETER, "Avr internal Vcc sensor");
#endif

}

// the loop function runs over and over again forever
void loop() {
  measureCount ++;
  sendBattery ++;
  bool forceTransmit = false;
  transmission_occured = false;
  if ((measureCount == 5) && highfreq) 
  {
    if (!ota_enabled) clock_prescale_set(clock_div_8); // Switch to 1Mhz for the reminder of the sketch, save power.
    highfreq = false;
  } 
  
  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }
    
  //process();

  // send msg if awake casue was input change else only if input change
  sendTripStatus();
  sendTempMeasurements(forceTransmit);
  if (sendBattery > 60)
  {
     sendBattLevel(forceTransmit); // Not needed to send battery info that often
     sendBattery = 0;
  }

  if (ota_enabled & transmission_occured) {
      wait(OTA_WAIT_PERIOD);
  }

  #define MY_SLEEP smartSleep
  awake = MY_SLEEP(digitalPinToInterrupt(BUTTON_PIN), CHANGE, MEASURE_INTERVAL);

  if (awake)
      sendTripStatus();

}

/********************************************
 *
 * Sends battery information (battery percentage)
 *
 * Parameters 
 * - force : Forces transmission of a value
 *
 *******************************************/
void sendBattLevel(bool force)
{
   if (force) lastBattery = -1;
  long vcc = readVcc();
   sendVCC = vcc/1000.0;

  if (vcc != lastBattery) {
    lastBattery = vcc;

#ifdef BATT_SENSOR
    send(msgBatt.set(sendVCC,2));
#endif

    // Calculate percentage

    vcc = vcc - 1900; // subtract 1.9V from vcc, as this is the lowest voltage we will operate at
    
    long percent = vcc / 14.0;
    if (percent > 100); percent = 100;
    sendBatteryLevel(percent);
    transmission_occured = true;
  }
}

/*********************************************
 *
 * Sends temperature and humidity from Si7021 sensor
 *
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *
 *********************************************/
void sendTempMeasurements(bool force)
{
  bool tx = force;

 float temp = readMCP9700(TEMP_SENSE_PIN,TEMP_SENSE_OFFSET); //temp pin and offset for calibration
   
  float diffTemp = abs(lastTemperature - temp);

  Serial.print(F("TempDiff :"));Serial.println(diffTemp);

  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;

  if (tx) {
    measureCount = 0;
     
    Serial.print("T: ");Serial.println(temp);
 // LED 
#ifdef LED_BLINK_WAIT_TRANSMIT
     digitalWrite(LED_PIN, HIGH);  
     send(msgTemp.set(temp,1));
     digitalWrite(LED_PIN, LOW);
#else
     send(msgTemp.set(temp,1));
#endif
    lastTemperature = temp;
    transmission_occured = true;
  }
}

/*******************************************
 *
 * Internal TEMP sensor 
 *
 *******************************************/
float readMCP9700(int pin,float offset)
{
  analogReference(INTERNAL);
  
  analogRead(A0); //perform a dummy read to clear the adc
  delay(20);
    
  for (int n=0;n<5;n++)
    analogRead(pin);
  
  int adc=analogRead(pin);
  float tSensor=((adc*(1.1/1024.0))-0.5+offset)*100;
  float error=244e-6*(125-tSensor)*(tSensor - -40.0) + 2E-12*(tSensor - -40.0)-2.0;
  float temp=tSensor-error;
 
  return temp;
}

/*******************************************
 *
 * Internal battery ADC measuring 
 *
 *******************************************/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADcdMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
 
}

/*********************************************
 *
 * Sends tripped status from D3 Reed switch sensor
 *
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *
 *********************************************/
void sendTripStatus() {
  debouncer.update();
  // Get the update value
  int value = debouncer.read();

  Serial.print("--->>");
  Serial.println(value);

  if (value != oldValue) {
     // Send in the new value
     send(msg.set(value==HIGH ? 1 : 0));
     oldValue = value;
     transmission_occured = true;
  }
}


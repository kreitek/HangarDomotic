/*
This sketch is for a DevDuino 4.0 
http://www.seeedstudio.com/depot/devDuino-Sensor-Node-V4-ATmega-328-Integrated-temperature-humidity-sensor-p-2279.html
and MySensors 2.x
 
 modified
 31 December 2015
 by greengo

 modified
 19 April 2017
 by enen
 
*/

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Select pin 9 (devduino4 LED) for debug.
#define MY_DEFAULT_TX_LED_PIN 9
#define MY_WITH_LEDS_BLINKING_INVERSE

// Define a static node address, remove if you want auto address assignment
#define MY_NODE_ID 26

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Set ISP devduino pins for NRF24
#define MY_RF24_CE_PIN 8
#define MY_RF24_CS_PIN 7

#define MY_OTA_FIRMWARE_FEATURE

#include <MySensors.h> // Library of Mysensors.org (v >2)
#include <SPI.h>
//#include "HTU21D.h"
#include <SparkFunHTU21D.h>
#include <Wire.h>
#include <RunningAverage.h>
 
// Uncomment the line below, to transmit battery voltage as a normal sensor value
#define BATT_SENSOR    3
 
#define RELEASE "1.1"
 
#define AVERAGES 2
 
// How many milli seconds between each measurement
#define MEASURE_INTERVAL 50000 //for Real Work 50 sec
//#define MEASURE_INTERVAL  10000 //for Debug 10 sec
 
// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 30 
//#define FORCE_TRANSMIT_INTERVAL 10 //for Debug 
 
// LED blinks during data transmission. Greater battery energy consumption!
#define LED_BLINK_WAIT_TRANSMIT  
 
#define TEMP_TRANSMIT_THRESHOLD 0.5
#define HUMI_TRANSMIT_THRESHOLD 0.5
 
// Pin definitions
#define LED_PIN           9 // LED 
 
// Child sensor ID's
#define CHILD_ID_TEMP  1
#define CHILD_ID_HUM   2
#define CHILD_BATT_ID  BATT_SENSOR
 
int oldBattPct = 0;
float temp = 0;
 
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
 
#ifdef BATT_SENSOR
MyMessage msgBatt(BATT_SENSOR, V_VOLTAGE);
#endif
 
// Global settings
int measureCount = 0;
int sendBattery = 0;
boolean highfreq = true;
boolean transmission_occured = false;
 
HTU21D myHumidity;
 
// Storage of old measurements
float lastTemperature = 0;
int lastHumidity = 0;
long lastBattery = 0;
 
RunningAverage raHum(AVERAGES);
 
/****************************************************
 * Setup code 
 ****************************************************/
void setup() {
 
  // initialize digital pin 9 as an output.
  pinMode(LED_PIN, OUTPUT); 
 
  Serial.begin(115200);
  Serial.print(F("devDuino SNv4"));
  Serial.println(RELEASE);
  Serial.flush(); 
 
  digitalWrite(LED_PIN, HIGH); 

#ifndef MY_NODE_ID
  getNodeId();
#endif  
 
myHumidity.begin(); 
 
  digitalWrite(LED_PIN, LOW);
 
  Serial.flush();
  Serial.println(F(" - Online!"));

raHum.clear();
 
sendTempHumidityMeasurements(false);
sendBattLevel(false);
 
}

void presentation() {
  // void sendSketchInfo(const char *name, const char *version, bool ack);
  sendSketchInfo("devDuino SNv4", RELEASE);
  
  // Register binary input sensor to gw (they will be created as child devices)
  // You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage. 
  // If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  present(CHILD_ID_TEMP, S_TEMP, "Devduino4 temperature sensor");

  present(CHILD_ID_HUM, S_HUM, "Devduino4 humidity sensor");
  
#ifdef BATT_SENSOR
  present(CHILD_BATT_ID, S_MULTIMETER, "Avr internal Vcc sensor");
#endif

}

/***********************************************
 *  Main loop function
 ***********************************************/
void loop() {
  measureCount ++;
  sendBattery ++;
  bool forceTransmit = false;
  transmission_occured = false;
  if ((measureCount == 5) && highfreq) 
 
  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }
 
  //process();
 
  sendTempHumidityMeasurements(forceTransmit);
  if (sendBattery > 60) 
  {
     sendBattLevel(forceTransmit); // Not needed to send battery info that often
     sendBattery = 0;
  }
 
  sleep(MEASURE_INTERVAL);  
}
/*********************************************
 * Sends temperature and humidity sensor
 *
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *********************************************/
void sendTempHumidityMeasurements(bool force)
{
  bool tx = force;
 
    //get the Temperature from the onboard sensor.
  float temp = myHumidity.readTemperature();
  int humidity = myHumidity.readHumidity();
 
  raHum.addValue(humidity);
 
  float diffTemp = abs(lastTemperature - temp);
  float diffHum = abs(lastHumidity - raHum.getAverage());
 
  Serial.print(F("TempDiff :"));Serial.println(diffTemp);
  Serial.print(F("HumDiff  :"));Serial.println(diffHum); 
 
  if (isnan(diffHum)) tx = true; 
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;
  if (diffHum > HUMI_TRANSMIT_THRESHOLD) tx = true;
 
  if (tx) {
    measureCount = 0;
 
    Serial.print("T: ");Serial.println(temp);
    Serial.print("H: ");Serial.println(humidity);
 
 // LED 
#ifdef LED_BLINK_WAIT_TRANSMIT
   digitalWrite(LED_PIN, HIGH);      
    send(msgTemp.set(temp,1));
    send(msgHum.set(humidity));
    digitalWrite(LED_PIN, LOW);
 #else
   send(msgTemp.set(temp,1));
   send(msgHum.set(humidity));
#endif 

    lastTemperature = temp;
    lastHumidity = humidity;
    transmission_occured = true;
  }
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
  if (vcc != lastBattery) {
    lastBattery = vcc;
 
#ifdef BATT_SENSOR
    send(msgBatt.set(vcc));
#endif
 
  // Calculate percentage
    vcc = vcc - 1900; // subtract 1.9V from vcc, as this is the lowest voltage we will operate at
 
    long percent = vcc / 14.0;
    sendBatteryLevel(percent);
    transmission_occured = true;
  }
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

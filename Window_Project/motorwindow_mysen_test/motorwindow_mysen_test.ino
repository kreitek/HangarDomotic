/*
This sketch is for a DevDuino 4.0 
http://www.seeedstudio.com/depot/devDuino-Sensor-Node-V4-ATmega-328-Integrated-temperature-humidity-sensor-p-2279.html
and MySensors 2.x
*/

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Set ISP devduino pins for NRF24
#define MY_RF24_CE_PIN 8
#define MY_RF24_CS_PIN 7

#include <MyConfig.h>
#include <MySensors.h>

#define CHILD_ID_MOTOR 1

#define RELEASE "1.0-1"

// Percentage that marks window open grade
uint8_t openpos = 0;

// From:
// http://www.schmalzhaus.com/EasyDriver/Examples/EasyDriverExamples.html
// This code does basically the same thing as Example 2, but using
// acceleration/deceleration via the AccelStepper library, 
// and running for twice as many steps. 
// (Thanks Mr. Duffy for pointing out this important fact!) 
// The reason it runs twice as many steps is because we do 
// "pos = -pos" to keep things short and simple. 
// This means that it will run from 0 to 3600, then from 
// 3600 to -3600 (which is 7200 steps).

#define STEPS_PER_MM 200
#define ENABLE_PIN A1
#define RETURN_PIN A0

#include <AccelStepper.h>
// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 5, 3);
//int pos = 50 * STEPS_PER_MM;

#define STOPPED 0
#define OPENING 1
#define CLOSING 2
unsigned int state;

#define LED_PIN 9
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)

void before()
{
  openpos = loadState(CHILD_ID_MOTOR);
}

void setup()
{ 
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(300);
  
  pinMode(LED_PIN, OUTPUT);  
  pinMode(ENABLE_PIN, OUTPUT);
  
  pinMode(RETURN_PIN, INPUT);
  digitalWrite(RETURN_PIN, INPUT_PULLUP);
}

void presentation()
{
  // void sendSketchInfo(const char *name, const char *version, bool ack);
  sendSketchInfo("devDuino SNv4 Window test", RELEASE);

  present(CHILD_ID_MOTOR, S_COVER, "Devduino4 window motor");
  
}

void loop()
{
  if (stepper.distanceToGo() == 0)
    state = STOPPED;
  stepper.run();
  
  bool carriage_ret = return_triggered();
  blink_led(state != STOPPED && !carriage_ret);
  
  if (carriage_ret && state == CLOSING)
    _stop();
}

void receive(const MyMessage &message)
{
  if (message.sensor == CHILD_ID_MOTOR)
  {
    switch (message.type)
    {
      case V_UP:
        openpos = 100 - openpos;
        _moveTo(openpos);
        break;
      case V_DOWN:
        openpos = 0 - openpos;
        _moveTo(openpos);
        break;
      case V_STOP:
        openpos = currentPos();
        _stop();
        break;
      case V_PERCENTAGE:
        openpos = message.getByte() - openpos;
        _moveTo(openpos);
        break;
      //default:
    }
    saveState(CHILD_ID_MOTOR, openpos);
  }
}

void _stop() {
  digitalWrite(ENABLE_PIN, HIGH);
  state = STOPPED;
}

void _start() {
  state = openpos < 0 ? CLOSING : (openpos > 0 ? OPENING : STOPPED);
  if (state != CLOSING && !return_triggered())
    digitalWrite(ENABLE_PIN, LOW);
}

bool return_triggered() {
  return !digitalRead(RETURN_PIN);
}

void _moveTo(uint8_t pos) {
  delay(500);
  stepper.moveTo(percentageToSteps(pos));
  _start();
}

int percentageToSteps(uint8_t percentage)
{
  return max(percentage, 16) * STEPS_PER_MM;
}

uint8_t currentPos()
{
  return (stepper.distanceToGo() / STEPS_PER_MM) - openpos;
}

void blink_led(bool enable) {
  if (enable) { 
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW)
        ledState = HIGH;
      else
        ledState = LOW;
      digitalWrite(LED_PIN, ledState);
    }
  } else
    digitalWrite(LED_PIN, LOW);
}


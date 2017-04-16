/*
This sketch is for a DevDuino 4.0 
http://www.seeedstudio.com/depot/devDuino-Sensor-Node-V4-ATmega-328-Integrated-temperature-humidity-sensor-p-2279.html
and MySensors 2.x
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

#include <MyConfig.h>
#include <MySensors.h>

#define CHILD_ID_MOTOR 0

#define RELEASE "1.0-2"

// Percentage that marks window open grade
uint8_t openpos;

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

#define UP      0
#define DOWN    1
#define STOPPED 2
#define OPENING 3
#define CLOSING 4
unsigned int state = STOPPED;
static bool initial_state_sent = false;

MyMessage upMessage(CHILD_ID_MOTOR, V_UP);
MyMessage downMessage(CHILD_ID_MOTOR, V_DOWN);
MyMessage stopMessage(CHILD_ID_MOTOR, V_STOP);
MyMessage statusMessage(CHILD_ID_MOTOR, V_STATUS);
MyMessage percentMessage(CHILD_ID_MOTOR, V_PERCENTAGE);


#define LED_PIN 9
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)


void sendState() {
  // Send current state and status to gateway.
  send(upMessage.set(state == UP));
  send(downMessage.set(state == DOWN));
  send(stopMessage.set(state == STOPPED));
  send(statusMessage.set(state != DOWN));
  send(percentMessage.set(openpos));
}

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
  sendSketchInfo("devDuino SNv4 WProtest", RELEASE);

  present(CHILD_ID_MOTOR, S_COVER);
  
}

void loop()
{
  if (!initial_state_sent) {
    updateFixedState();
    sendState();
    initial_state_sent = true;
  }
  
  stepper.run();
  if (stepper.distanceToGo() == 0 && state > STOPPED)
  {
    updateFixedState();
    sendState();
  }
  
  bool carriage_ret = return_triggered();
  blink_led(state != STOPPED && !carriage_ret);
  
  if (carriage_ret && state == CLOSING)
    _disable();
}

void receive(const MyMessage &message)
{
  if (message.sensor == CHILD_ID_MOTOR)
  {
    switch (message.type)
    {
      case V_UP:
        _moveTo(100);
        break;
      case V_DOWN:
        _moveTo(0);
        break;
      case V_STOP:
        _disable();
        openpos = currentPos();
        stepper.setCurrentPosition(percentageToSteps(openpos));
        break;
      case V_PERCENTAGE:
        _moveTo(message.getByte());
        break;
      //default:
    }
    saveState(CHILD_ID_MOTOR, openpos);
  }
}

void _disable() {
  digitalWrite(ENABLE_PIN, HIGH);
  stepper.stop();
}

void _enable() {
  if (state != CLOSING || !return_triggered())
    digitalWrite(ENABLE_PIN, LOW);
}

void _moveTo(uint8_t pos) {
  delay(500);
  int moving_to = pos - openpos;
  if (moving_to)
  {
    stepper.moveTo(percentageToSteps(moving_to));
    state = moving_to < 0 ? CLOSING : (moving_to > 0 ? OPENING : STOPPED);
    _enable();
    openpos = pos;
  }
}

void updateFixedState()
{
  switch (openpos)
  {
    case 0:   state = DOWN; break;
    case 100: state = UP; break;
    default:  state = STOPPED;
  }
}

bool return_triggered() {
  return !digitalRead(RETURN_PIN);
}

int percentageToSteps(uint8_t percentage)
{
  return max(percentage, 16) * STEPS_PER_MM;
}

uint8_t currentPos()
{
  return openpos - (stepper.distanceToGo() / STEPS_PER_MM);
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


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
int pos = 50 * STEPS_PER_MM;

#define STOPPED 0
#define OPENING 1
#define CLOSING 2
unsigned int state;

#define LED_PIN 9
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)

void setup()
{ 
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(300);
  
  pinMode(LED_PIN, OUTPUT);  
  pinMode(ENABLE_PIN, OUTPUT);
  
  pinMode(RETURN_PIN, INPUT);
  digitalWrite(RETURN_PIN, INPUT_PULLUP);
}

void loop()
{
  if (stepper.distanceToGo() == 0)
  {
    delay(500);
    pos = -pos; 
    stepper.moveTo(pos);
  }
  stepper.run();
  
  bool carriage_ret = !digitalRead(RETURN_PIN);
  blink_led(!carriage_ret);
  state = pos < 0 ? CLOSING : (pos > 0 ? OPENING : STOPPED);
  
  if (carriage_ret && state == CLOSING)
    _stop();
  if (state == OPENING)
    _start();
}

void _stop() {
  digitalWrite(ENABLE_PIN, HIGH);
}

void _start() {
  digitalWrite(ENABLE_PIN, LOW);
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


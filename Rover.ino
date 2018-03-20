/*
 * ------- ------- ------- ------- ------- ------- -------
 * 
 * Basic remote controlled tracked rover with forwards/reverse & turning-on-the-spot.
 * 
 * Based/lifted from;
 * http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
 * 
 * ------- ------- ------- ------- ------- ------- -------
 * 
 * Tracked rover chassis with motors
 * https://www.banggood.com/DIY-RC-Robot-Chassis-Tank-Car-Tracking-Obstacle-Avoidance-With-Crawler-Set-p-1254211.html
 * 
 * Arduino Uno Rev3
 * https://store.arduino.cc/usa/arduino-uno-rev3
 * 
 * Adafruit Motor Shield v2.3
 * https://www.adafruit.com/product/1438
 * 
 * FrSky X4R-SB receiver
 * https://www.frsky-rc.com/x4rsb/
 * 
 * ------- ------- ------- ------- ------- ------- ------- 
 * 
 * Taranis setup (same as a differential thrust plane)
 * https://www.youtube.com/watch?v=v6W6FpL9zFg
 * 
 * Video in action
 * https://www.youtube.com/watch?v=FEWU_2hBrXM
 * 
 */

// ------- ------- ------- ------- ------- ------- -------

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define LEFT_THROTTLE_SIGNAL_IN 0
#define LEFT_THROTTLE_SIGNAL_IN_PIN 2

#define RIGHT_THROTTLE_SIGNAL_IN 1
#define RIGHT_THROTTLE_SIGNAL_IN_PIN 3

volatile int nLeftThrottleIn, nRightThrottleIn;

volatile unsigned long ulStartPeriod = 0;

// these are set in the interrupt handlers then read & fed to the motor shield in loop()
volatile boolean bNewLeftThrottleSignal = false;
volatile boolean bNewRightThrottleSignal = false;

// ------- ------- ------- ------- ------- ------- -------

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);

// ------- ------- ------- ------- ------- ------- -------

void setup() {
  
  attachInterrupt(LEFT_THROTTLE_SIGNAL_IN, calcLeftInput, CHANGE);
  attachInterrupt(RIGHT_THROTTLE_SIGNAL_IN, calcRightInput, CHANGE);

  AFMS.begin();  // create with the default frequency 1.6KHz

}

// ------- ------- ------- ------- ------- ------- -------

void loop() {

  if(bNewLeftThrottleSignal) {
    bNewLeftThrottleSignal = false;

    // ignore obviously invalid PWM values 
    if (nLeftThrottleIn > 2018 || nLeftThrottleIn < 980) {
      return;
    }

    // check whether we want to go forward (>1500uS) or backward (<1500uS)
    // using 1520 & 1480 gives a nice deadband, you could tweak this or use a global variable for deadband amount
    if (nLeftThrottleIn >= 1520) {
      leftMotor->run(FORWARD);

      // map the forward PWM value (which will be between neutral & ~2000) to the range of the motor shield (which is 0 to 255)
      leftMotor->setSpeed((map(nLeftThrottleIn, 1520, 2018, 0, 255)));
    } else if (nLeftThrottleIn <= 1480){
      leftMotor->run(BACKWARD);

      // likewise map backward PWM values (which will be between neutral & ~1000) to the range of the motor shield, remembering that
      // for backwards we want smaller values to go faster (eg a PWM value of 1200 should move faster backward than a PWM value of 1300)
      leftMotor->setSpeed((map(nLeftThrottleIn, 980, 1480, 255, 0)));
    } else {
      leftMotor->run(RELEASE);
    }

  }

  if(bNewRightThrottleSignal) {
    bNewRightThrottleSignal = false;

    if (nRightThrottleIn > 2018 || nRightThrottleIn < 980) {
      return;
    }

    if (nRightThrottleIn >= 1520) {
      rightMotor->run(FORWARD);
      rightMotor->setSpeed((map(nRightThrottleIn, 1520, 2018, 0, 255)));
    } else if (nRightThrottleIn <= 1480) {
      Serial.print("R: ");
      Serial.println(nRightThrottleIn);
      rightMotor->run(BACKWARD);
      rightMotor->setSpeed((map(nRightThrottleIn, 988, 1480, 255, 0)));
    } else {
      rightMotor->run(RELEASE);
    }

  }
}

// ------- ------- ------- ------- ------- ------- -------

// Taken directly from http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html

void calcLeftInput() {
  // if the pin is high, its the start of an interrupt
  if(digitalRead(LEFT_THROTTLE_SIGNAL_IN_PIN) == HIGH) { 
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its 
    // easy to understand and works very well
    ulStartPeriod = micros();
  } else {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the 
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriod && (bNewLeftThrottleSignal == false)) {
      nLeftThrottleIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;

      // tell loop we have a new signal on the throttle channel
      // we will not update nThrottleIn until loop sets
      // bNewThrottleSignal back to false
      bNewLeftThrottleSignal = true;
    }
  }
}

// ------- ------- ------- ------- ------- ------- -------

void calcRightInput() {
  if(digitalRead(RIGHT_THROTTLE_SIGNAL_IN_PIN) == HIGH) { 
    ulStartPeriod = micros();
  } else {
    if(ulStartPeriod && (bNewRightThrottleSignal == false)) {
      nRightThrottleIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;
      bNewRightThrottleSignal = true;
    }
  }
}

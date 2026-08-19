/*********
  Split Flap Arduino Nano Slave Unit Firmware
  Target: Arduino Nano (ATmega328P)
*********/

#include <Arduino.h>
#include <Wire.h>
#include <Stepper.h>
#include <EEPROM.h>
#include <avr/sleep.h>

// Pins of I2C address DIP switch (Active LOW with INPUT_PULLUP)
#define ADRESSSW1 6
#define ADRESSSW2 5
#define ADRESSSW3 4
#define ADRESSSW4 3

// Stepper Motor (28BYJ-48 via ULN2003 Driver)
#define STEPPERPIN1 11
#define STEPPERPIN2 10
#define STEPPERPIN3 9
#define STEPPERPIN4 8
#define STEPS 2038         // 28BYJ-48 stepper steps per revolution (~2038)
#define HALLPIN 7          // KY-003 Hall sensor pin (Active LOW when magnet detected)
#define AMOUNTFLAPS 45     // Total flaps per drum

// Constants
#define BAUDRATE 115200
#define ROTATIONDIRECTION -1 // -1 for reverse direction
#define OVERHEATINGTIMEOUT 2 // Seconds idle timeout before disabling coils
unsigned long lastRotation = 0;

// Globals
int displayedLetter = 0;   // Currently shown flap index (0 to AMOUNTFLAPS - 1)
volatile int desiredLetter = 0;     // Flap index requested via I2C
Stepper stepper(STEPS, STEPPERPIN1, STEPPERPIN3, STEPPERPIN2, STEPPERPIN4);

bool lastInd1 = false;
bool lastInd2 = false;
bool lastInd3 = false;
bool lastInd4 = false;
float missedSteps = 0;     // Fractional step accumulator

volatile int currentlyrotating = 0; // 1 = rotating, 0 = stationary
volatile int stepperSpeed = 10;     // Default speed RPM
int eeAddress = 0;         // EEPROM start address for offset
int calOffset = 0;         // Calibration offset in steps
int i2cAddress = 1;

// Sleep globals
const unsigned long WAIT_TIME = 2000; // ms idle before sleep
unsigned long previousSleepMillis = 0;

// Forward declarations
void startMotor();
void stopMotor();
int calibrate(bool initialCalibration);
void rotateToLetter(int toLetter);

// Return I2C address (1 to 15) from 4-bit DIP switches
int getaddress() {
  int address = (!digitalRead(ADRESSSW4) * 1) + 
                (!digitalRead(ADRESSSW3) * 2) + 
                (!digitalRead(ADRESSSW2) * 4) + 
                (!digitalRead(ADRESSSW1) * 8);
  return (address > 0) ? address : 1; // Fallback to 1 if all switches are off
}

// Fetch magnet sensor offset from EEPROM
void getOffset() {
  EEPROM.get(eeAddress, calOffset);
  if (calOffset < -2000 || calOffset > 2000) {
    calOffset = 0;
  }
}

// Calibrate drum using Hall-effect sensor
int calibrate(bool initialCalibration) {
  currentlyrotating = 1;
  bool reachedMarker = false;
  stepper.setSpeed(stepperSpeed > 0 ? stepperSpeed : 10);
  int i = 0;

  while (!reachedMarker) {
    int currentHallValue = digitalRead(HALLPIN);
    
    // If magnet is already present on startup (LOW), advance 50 steps to clear it
    if (currentHallValue == LOW && i == 0) {
      i = 50;
      stepper.step(ROTATIONDIRECTION * 50);
    } else if (currentHallValue == HIGH) {
      // Magnet not yet reached
      stepper.step(ROTATIONDIRECTION * 1);
    } else {
      // Reached magnet marker! Advance by calibration offset to align flap 0
      reachedMarker = true;
      if (calOffset != 0) {
        stepper.step(ROTATIONDIRECTION * calOffset);
      }
      displayedLetter = 0;
      missedSteps = 0;
      if (initialCalibration) {
        stopMotor();
      }
      return i;
    }

    if (i > 3 * STEPS) {
      // Timeout safety: Hall sensor not detected
      displayedLetter = 0;
      desiredLetter = 0;
      reachedMarker = true;
      stopMotor();
      return -1;
    }
    i++;
  }
  return i;
}

// Move to specified flap index
void rotateToLetter(int toLetter) {
  if (lastRotation == 0 || (millis() - lastRotation > (unsigned long)(OVERHEATINGTIMEOUT * 1000))) {
    lastRotation = millis();
    int posLetter = toLetter;
    int posCurrentLetter = displayedLetter;

    if (posLetter >= 0 && posLetter < AMOUNTFLAPS) {
      if (posLetter >= posCurrentLetter) {
        int diffPosition = posLetter - posCurrentLetter;
        startMotor();
        stepper.setSpeed(stepperSpeed > 0 ? stepperSpeed : 10);
        for (int i = 0; i < diffPosition; i++) {
          float preciseStep = (float)STEPS / (float)AMOUNTFLAPS;
          int roundedStep = (int)preciseStep;
          missedSteps += (preciseStep - (float)roundedStep);
          if (missedSteps > 1.0f) {
            roundedStep += 1;
            missedSteps -= 1.0f;
          }
          stepper.step(ROTATIONDIRECTION * roundedStep);
        }
      } else {
        // Forward wrap-around: perform homing calibration to re-index at flap 0
        calibrate(false);
        startMotor();
        stepper.setSpeed(stepperSpeed > 0 ? stepperSpeed : 10);
        for (int i = 0; i < posLetter; i++) {
          float preciseStep = (float)STEPS / (float)AMOUNTFLAPS;
          int roundedStep = (int)preciseStep;
          missedSteps += (preciseStep - (float)roundedStep);
          if (missedSteps > 1.0f) {
            roundedStep += 1;
            missedSteps -= 1.0f;
          }
          stepper.step(ROTATIONDIRECTION * roundedStep);
        }
      }
      displayedLetter = toLetter;
      delay(50);
      stopMotor();
    }
  }
}

// Power off motor coils to prevent heating when stationary
void stopMotor() {
  digitalWrite(STEPPERPIN1, LOW);
  digitalWrite(STEPPERPIN2, LOW);
  digitalWrite(STEPPERPIN3, LOW);
  digitalWrite(STEPPERPIN4, LOW);
  currentlyrotating = 0;
}

void startMotor() {
  currentlyrotating = 1;
}

// I2C Receive Event Handler (ESP32 -> Slave)
void receiveLetter(int numBytes) {
  if (numBytes >= 1) {
    int target = Wire.read();
    if (target >= 0 && target < AMOUNTFLAPS) {
      desiredLetter = target;
    }
  }
  if (numBytes >= 2) {
    int spd = Wire.read();
    if (spd > 0) {
      stepperSpeed = spd;
    }
  }
  // Flush any extraneous bytes to prevent buffer overflow
  while (Wire.available()) {
    Wire.read();
  }
}

// I2C Status Request Event Handler (Slave -> ESP32)
void requestEvent() {
  Wire.write((uint8_t)currentlyrotating);
}

void setup() {
  // DIP address switches
  pinMode(ADRESSSW1, INPUT_PULLUP);
  pinMode(ADRESSSW2, INPUT_PULLUP);
  pinMode(ADRESSSW3, INPUT_PULLUP);
  pinMode(ADRESSSW4, INPUT_PULLUP);

  // KY-003 Hall sensor (ensure internal pullup in case board lacks resistor)
  pinMode(HALLPIN, INPUT_PULLUP);

  i2cAddress = getaddress();

  Wire.begin(i2cAddress);
  Wire.onReceive(receiveLetter);
  Wire.onRequest(requestEvent);

  getOffset();
  calibrate(true);
}

void loop() {
  // Check if a new letter index was received over I2C
  if (displayedLetter != desiredLetter) {
    rotateToLetter(desiredLetter);
  }
}

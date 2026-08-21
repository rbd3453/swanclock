/*********
  Split Flap Arduino Nano Slave Unit Firmware
  Target: Arduino Nano (ATmega328P)
  High-Torque Half-Step Drive via AccelStepper with I2C Calibration
*********/

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <EEPROM.h>

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

// Stepper Constants (Half-step 8-phase high-torque sequence)
const int STEPS_PER_REV = 4096;
const int AMOUNTFLAPS = 45;
const float STEPS_PER_FLAP = (float)STEPS_PER_REV / (float)AMOUNTFLAPS; // 91.02222

#define HALLPIN 7 // KY-003 Hall sensor pin (Active LOW when magnet detected)

// Initialize AccelStepper in 4-wire half-step mode: pin sequence 1, 3, 2, 4
AccelStepper stepper(AccelStepper::HALF4WIRE, STEPPERPIN1, STEPPERPIN3, STEPPERPIN2, STEPPERPIN4);

// Globals
int displayedLetter = 0;   // Currently shown flap index (0 to AMOUNTFLAPS - 1)
volatile int desiredLetter = 0;     // Flap index requested via I2C
volatile int currentlyrotating = 0; // 1 = rotating, 0 = stationary

int eeAddress = 0;         // EEPROM start address for offset
int calOffset = 0;         // Calibration offset in steps
int i2cAddress = 1;

// I2C Command Buffer
volatile uint8_t pendingCmd = 0;
volatile int pendingParam = 0;

// Forward declarations
void calibrate(bool initialCalibration);
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
  if (calOffset < -4096 || calOffset > 4096) {
    calOffset = 0;
  }
}

// Calibrate drum using Hall-effect sensor and stored EEPROM offset
void calibrate(bool initialCalibration) {
  currentlyrotating = 1;
  stepper.enableOutputs();
  stepper.setSpeed(-500);

  // If magnet is already present on startup (LOW), advance 150 steps to clear it
  if (digitalRead(HALLPIN) == LOW) {
    long clearTarget = stepper.currentPosition() - 150;
    stepper.moveTo(clearTarget);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
  }

  // Seek magnet in homing direction
  long maxSearch = stepper.currentPosition() - (long)(STEPS_PER_REV * 2);
  while (digitalRead(HALLPIN) == HIGH && stepper.currentPosition() > maxSearch) {
    stepper.runSpeed();
  }

  // If magnet found
  if (digitalRead(HALLPIN) == LOW) {
    stepper.setCurrentPosition(0);
    // Apply calibration offset stored in EEPROM to land on Flap 0
    if (calOffset != 0) {
      stepper.moveTo(-1 * calOffset);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      stepper.setCurrentPosition(0);
    }
    displayedLetter = 0;
    desiredLetter = 0;
  }

  currentlyrotating = 0;
  if (initialCalibration) {
    stepper.disableOutputs(); // Cut coil power to stay cool
  }
}

// Move to specified flap index with high torque & smooth acceleration
void rotateToLetter(int toLetter) {
  if (toLetter < 0 || toLetter >= AMOUNTFLAPS) return;
  if (toLetter == displayedLetter) return;

  currentlyrotating = 1;
  stepper.enableOutputs();

  int flapsToAdvance = toLetter - displayedLetter;
  if (flapsToAdvance < 0) {
    flapsToAdvance += AMOUNTFLAPS;
  }

  long targetSteps = stepper.currentPosition() - (long)(flapsToAdvance * STEPS_PER_FLAP);
  stepper.moveTo(targetSteps);

  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  displayedLetter = toLetter;
  currentlyrotating = 0;
  stepper.disableOutputs(); // Power off motor coils to prevent heating
}

// I2C Receive Event Handler (ESP32 -> Slave)
void receiveLetter(int numBytes) {
  if (numBytes >= 1) {
    int cmd = Wire.read();
    if (cmd >= 0 && cmd < AMOUNTFLAPS) {
      desiredLetter = cmd;
    } else {
      pendingCmd = (uint8_t)cmd;
    }
  }
  if (numBytes >= 2) {
    pendingParam = Wire.read();
  }
  // Flush any extraneous bytes
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

  // KY-003 Hall sensor
  pinMode(HALLPIN, INPUT_PULLUP);

  // High-Torque Stepper Configuration
  stepper.setMaxSpeed(700);     // Optimal torque speed for 28BYJ-48 (~10 RPM)
  stepper.setAcceleration(500); // Smooth torque ramp to push past flap resistance

  i2cAddress = getaddress();

  Wire.begin(i2cAddress);
  Wire.onReceive(receiveLetter);
  Wire.onRequest(requestEvent);

  getOffset();
  calibrate(true);
}

void loop() {
  // Process any pending I2C calibration / jog commands
  if (pendingCmd != 0) {
    uint8_t cmd = pendingCmd;
    int param = pendingParam;
    pendingCmd = 0;

    if (cmd == 250) {
      // 0xFA: Jog Flaps
      int flaps = param > 0 ? param : 1;
      currentlyrotating = 1;
      stepper.enableOutputs();
      long delta = -1 * (long)(flaps * STEPS_PER_FLAP);
      stepper.move(delta);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      currentlyrotating = 0;
      stepper.disableOutputs();
    } 
    else if (cmd == 251) {
      // 0xFB: Fine Step Jog
      int steps = param > 0 ? param : 10;
      currentlyrotating = 1;
      stepper.enableOutputs();
      stepper.move(-1 * steps);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      currentlyrotating = 0;
      stepper.disableOutputs();
    } 
    else if (cmd == 252) {
      // 0xFC: Save current position as Home (Flap 0) into EEPROM
      long deltaSteps = -1 * stepper.currentPosition();
      calOffset = (int)(calOffset + deltaSteps);
      while (calOffset < 0) calOffset += STEPS_PER_REV;
      calOffset = calOffset % STEPS_PER_REV;
      EEPROM.put(eeAddress, calOffset);
      stepper.setCurrentPosition(0);
      displayedLetter = 0;
      desiredLetter = 0;
      stepper.disableOutputs();
    } 
    else if (cmd == 253) {
      // 0xFD: Re-Home drum
      calibrate(true);
    }
  }

  // Check if a new target flap index was received over I2C
  if (displayedLetter != desiredLetter) {
    rotateToLetter(desiredLetter);
  }
}

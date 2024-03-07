#include "DropTarget.h"
#include "Arduino.h"
#include "Interfaces.h"
#include <Adafruit_PWMServoDriver.h>

DropTarget::DropTarget( 
    Adafruit_PWMServoDriver* _pwm, 
    uint8_t _servonum, 
    uint16_t _servomin, 
    uint16_t _servomax,
    int _inputPin,
    bool _inputRestValue = HIGH,
    unsigned long _debounceDelay = 10
  ) {
  pinIn = _inputPin;
  inputRestValue = _inputRestValue;
  debounceDelay = _debounceDelay;

  lastInputState = inputRestValue;
  currentInputState = inputRestValue;

  pwm = _pwm;
  servonum = _servonum;
  servomax = _servomax;
  servomin = _servomin;

  pinMode(pinIn, INPUT_PULLUP);
}

void DropTarget::setMessageQueue(MessageQueue* queue) {
  writeQueue = queue;
}

void DropTarget::setComponentID(uint8_t id) {
  component_id = id;
}

void DropTarget::handleMessage(uint8_t id, unsigned char content) {
  isResetting = true;
}

void DropTarget::update() {
  if(shouldTriggerComm()) {
    triggerComm();
  } else {
    untriggerComm();
  }

  if(isResetting) {
    handleReset();
  }
}

void DropTarget::handleReset() {
  if(resetTime == 0) {
    pwm->setPWM(servonum, 0, servomax);
    resetTime = millis();
  } else if (millis() >= resetTime + 300) {
    pwm->setPWM(servonum, 0, servomin);
    resetTime = 0;
    isResetting = false;
  }
}

void DropTarget::triggerComm() {
  if(!alreadyTriggered) {
    alreadyTriggered = true; 
    char message = char(1);
    writeQueue->queueOutgoingMessage(component_id, message);
  }
}

void DropTarget::untriggerComm() {
  alreadyTriggered = false;
}

bool DropTarget::shouldTriggerComm() {
  return inputActivated();
}

bool DropTarget::inputActivated() {
  bool pinVal = debouncedInputRead();
  return inputRestValue == LOW ? pinVal : !pinVal;
}

bool DropTarget::debouncedInputRead() {
  bool pinVal = digitalRead(pinIn);

  if (pinVal != lastInputState) {
    debounceTime = millis();
    lastInputState = pinVal;
  }

  if (currentInputState != pinVal && (millis() - debounceTime) >= debounceDelay) {
    currentInputState = pinVal;
  }

  return currentInputState;
}
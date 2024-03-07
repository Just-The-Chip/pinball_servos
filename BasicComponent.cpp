#include "Arduino.h"
#include "BasicComponent.h"

BasicComponent::BasicComponent(int _pinIn, int _pinOut, bool _inputRestValue, bool _outputRestValue, unsigned long _debounceDelay) {
  pinIn = _pinIn;
  pinOut = _pinOut;
  inputRestValue = _inputRestValue;
  outputRestValue = _outputRestValue;
  debounceDelay = _debounceDelay;

  lastInputState = inputRestValue;
  currentInputState = inputRestValue;

  pinMode(pinIn, INPUT_PULLUP);
  pinMode(pinOut, OUTPUT);
}

void BasicComponent::update() {
  if(shouldTriggerOutput()) {
    triggerOutput();
  } else {
    untriggerOutput();
  }
}

void BasicComponent::triggerOutput() {
  digitalWrite(pinOut, !outputRestValue);
}

void BasicComponent::untriggerOutput() {
  digitalWrite(pinOut, outputRestValue);
}

bool BasicComponent::shouldTriggerOutput() {
  return inputActivated();
}

bool BasicComponent::inputActivated() {
  bool pinVal = debouncedInputRead();
  return inputRestValue == LOW ? pinVal : !pinVal;
}

bool BasicComponent::debouncedInputRead() {
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
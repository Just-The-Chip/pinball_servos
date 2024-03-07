#pragma once
#include <stdint.h>
#include "Arduino.h"

class BasicComponent {
  public:
    BasicComponent(
      int _pinIn, 
      int _pinOut,
      bool _inputRestValue = HIGH,
      bool _outputRestValue = LOW,
      unsigned long _debounceDelay = 10
    );
    virtual void update();
    virtual bool shouldTriggerOutput();
    virtual bool inputActivated();
    virtual bool debouncedInputRead();
    virtual void triggerOutput();
    virtual void untriggerOutput();

  protected:
    int	pinIn;
    int	pinOut;
    bool inputRestValue;
    bool outputRestValue;

    unsigned int debounceDelay;
    bool lastInputState;
    bool currentInputState;
    unsigned long debounceTime;
  };
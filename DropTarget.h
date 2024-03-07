#pragma once
#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>
#include "Interfaces.h"

class DropTarget : public MessageHandler {
  public:
    DropTarget( 
      Adafruit_PWMServoDriver* _pwm, 
      uint8_t _servonum, 
      uint16_t _servomin, 
      uint16_t _servomax,
      int _inputPin,
      bool _inputRestValue = HIGH,
      unsigned long _debounceDelay = 10
    );
    virtual void update();
    virtual bool shouldTriggerComm();
    virtual bool inputActivated();
    virtual bool debouncedInputRead();
    virtual void triggerComm();
    virtual void untriggerComm();
    virtual void setComponentID(uint8_t id);
    virtual void setMessageQueue(MessageQueue* queue);
    virtual void handleMessage(uint8_t id, unsigned char content);


  protected:
    virtual void handleReset();

    Adafruit_PWMServoDriver* pwm;
    uint8_t servonum;
    uint16_t servomin;
    uint16_t servomax;

    bool isResetting;
    unsigned long resetTime;

    uint8_t component_id;
    bool alreadyTriggered;
    MessageQueue* writeQueue;

    int	pinIn;
    bool inputRestValue;

    unsigned int debounceDelay;
    bool lastInputState;
    bool currentInputState;
    unsigned long debounceTime;
};
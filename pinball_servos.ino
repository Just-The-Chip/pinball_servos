#include "PiComm.h"
#include "DropTarget.h"
#include "Interfaces.h"

#define DROP_PIN_1 8
#define DROP_PIN_2 9
#define DROP_PIN_3 10

#define SERVOMIN  388 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  515 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// a bunch of handler IDS are on the other board so even though
// there is only three components we have to have space for all IDs. 
MessageHandler* handlers[14];

DropTarget *dropTarget1;
DropTarget *dropTarget2;
DropTarget *dropTarget3;

PiComm *comm;
Adafruit_PWMServoDriver *pwm = new Adafruit_PWMServoDriver();

void setup() {
  delay(500);

  pwm->begin();
  Serial.begin(115200);

  pwm->setOscillatorFrequency(27000000);
  pwm->setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  comm = new PiComm();

  // top left drop targets
  dropTarget1 = new DropTarget(pwm, 0, SERVOMAX, SERVOMAX, DROP_PIN_1, HIGH);
  dropTarget2 = new DropTarget(pwm, 1, SERVOMAX, SERVOMAX, DROP_PIN_2, HIGH);
  dropTarget3 = new DropTarget(pwm, 2, SERVOMAX, SERVOMAX, DROP_PIN_3, HIGH);

  setupDropTarget(dropTarget1, 4);
  setupDropTarget(dropTarget2, 5);
  setupDropTarget(dropTarget3, 6);
}

void setupDropTarget(DropTarget *component, uint8_t id) {
  component->setComponentID(id);
  component->setMessageQueue(comm);

  handlers[id] = component;
}

void loop() {
  comm->handleIncomingMessages(handlers);

  // at some point we will keep track of game mode to decide which components to update
  // but for now we will just update everything
  updatePlayModeComponents();

  comm->writeQueuedMessages();
}

void updatePlayModeComponents() {
  dropTarget1->update();
  dropTarget2->update();
  dropTarget3->update();
}

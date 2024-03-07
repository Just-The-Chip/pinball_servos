#include "PiComm.h"
#include <ArduinoQueue.h>

PiComm::PiComm() {
  writeQueue = new ArduinoQueue<PiMessage>(25);
  Serial.begin(115200);
}

// expected message format: 
void PiComm::handleIncomingMessages(MessageHandler* handlers[]) {
  uint8_t max_id = sizeof(handlers) / sizeof(handlers[0]);
  while(available() > 0) {
    unsigned char message[MESSAGE_LENGTH];

    Serial.readBytes(message, MESSAGE_LENGTH);

    if(message[2] == MESSAGE_TERMINATOR) {
      uint8_t handler_id = uint8_t(message[0]);
      unsigned char message_content = message[1];

      // if handler_id is within bounds of the handlers array
      if(handler_id < max_id && handlers[handler_id] != NULL) {
        handlers[handler_id]->handleMessage(handler_id, message_content);
      }
    }
  }
}

// not sure what happens when queue gets to max size
void PiComm::queueOutgoingMessage(uint8_t component_id, unsigned char message_content) {
  // Serial.println("queue boo");
  struct PiMessage message;
  message.id = component_id;
  message.content = message_content;
  
  writeQueue->enqueue(message);
}

void PiComm::writeQueuedMessages() {
  while(!writeQueue->isEmpty()) {
    writeMessage(writeQueue->dequeue());
  }
}

int PiComm::available() {
  return Serial.available() / MESSAGE_LENGTH;
}

void PiComm::writeMessage(PiMessage message) {
  char outgoing_message[3] = {message.id, message.content, MESSAGE_TERMINATOR};
  Serial.write(outgoing_message, 3);
}
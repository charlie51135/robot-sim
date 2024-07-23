#ifndef DEAD_MAN_SWITCH_H
#define DEAD_MAN_SWITCH_H

#include <Arduino.h>

class DeadManSwitch {
  public:
    DeadManSwitch(unsigned long timeoutPeriod) {
      this->timeoutPeriod = timeoutPeriod;
      reset();
    }

    void update(bool *newTarget) {
      if (*newTarget) {   // Reset timer
        *newTarget = false;
        reset();
      } else {            // Update timer
        if (millis() - lastReceivedTime > timeoutPeriod) {
          active = false;
        }
      }

    bool isActive() {
      return active;
    }

  private:
    unsigned long lastReceivedTime;
    unsigned long timeoutPeriod;
    bool active;

    void reset() {
      lastReceivedTime = millis();
      active = true;
    }
};

#endif // DEAD_MAN_SWITCH_H

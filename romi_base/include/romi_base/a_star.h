#ifndef A_STAR_H
#define A_STAR_H

#include <wiringPiI2C.h>
#include <unistd.h>
#include <cstring>
#include <array>
#include <tuple>
#include <iostream>

#define DEBUG_MODE false

class AStar {
private:
    int fd;

public:
    AStar();
    ~AStar();

    void setRedLed(int red);
    void setGreenLed(int green);
    void setYellowLed(int yellow);

    void playNotes(const char* notes);
    void setMotors(int left_speed, int right_speed);
    void setNewTarget(bool new_target);
    
    int readBatteryMillivolts();
    std::array<int, 6> readAnalog();
    std::tuple<int, int> readEncoders();
    std::tuple<bool, bool, bool> readButtons();

    void testRead8();
    void testWrite8();
};

#endif // A_STAR_H
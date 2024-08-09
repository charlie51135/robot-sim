/*
Description: All functions to read and write to the shared I2C registers on the Romi board. 

Notes: Changes made to this file must also be made to the data structure in RomiRPiSlave.ino
*/

#include "romi_base/a_star.h"
#include <chrono>
#include <thread>

void delayMicroseconds(unsigned int microseconds) {
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

AStar::AStar() {
    fd = wiringPiI2CSetup(20);  // Romi I2C address
}

AStar::~AStar() {
    close(fd);
}

// Writes the yellow LED state (0-255) to register 0
void AStar::setYellowLed(int yellow) {
    wiringPiI2CWriteReg8(fd, 0, yellow);

    if (DEBUG_MODE) {
        std::cout << "Set yellow LED to: " << yellow << std::endl;
    }
}


// Writes the green LED state (0-255) to register 1
void AStar::setGreenLed(int green) {
    wiringPiI2CWriteReg8(fd, 1, green);

    if (DEBUG_MODE) {
        std::cout << "Set green LED to: " << green << std::endl;
    }
}


// Writes the red LED state (0-255) to register 2
void AStar::setRedLed(int red) {
    wiringPiI2CWriteReg8(fd, 2, red);

    if (DEBUG_MODE) {
        std::cout << "Set red LED to: " << red << std::endl;
    }
}


// Writes the notes to play on the buzzer to registers 25-38
void AStar::playNotes(const char* notes) {
    wiringPiI2CWriteReg8(fd, 24, 1);
    for (int i = 0; i < strlen(notes); ++i) {
        wiringPiI2CWriteReg8(fd, 25 + i, notes[i]);
    }

    if (DEBUG_MODE) {
        std::cout << "Playing notes: " << notes << std::endl;
    }
}


// Writes the motor speeds to registers 6-9
void AStar::setMotors(int left_speed, int right_speed) {
    wiringPiI2CWriteReg16(fd, 6, left_speed);
    wiringPiI2CWriteReg16(fd, 8, right_speed);

    if (DEBUG_MODE) {
        std::cout << "Setting motors - Left speed: " << left_speed << ", Right speed: " << right_speed << std::endl;
    }
}


// Writes a boolean flag to register 43 to indicate a new command has been sent
void setNewTarget(bool new_target) {
    wiringPiI2CWriteReg8(fd, 43, new_target)

    if (DEBUG_MODE) {
        std::cout << "Set new target" << std::endl;
    }
}


// Reads the three button states from registers 3-5
std::tuple<bool, bool, bool> AStar::readButtons() {

    // Write the address of the buttons and then read the next 3 bytes
    wiringPiI2CWrite(fd,3);
    delayMicroseconds(100);
    bool buttonA = wiringPiI2CRead(fd);
    bool buttonB = wiringPiI2CRead(fd);
    bool buttonC = wiringPiI2CRead(fd);

    if (DEBUG_MODE) {
        std::cout << "Read buttons - A: " << buttonA << ", B: " << buttonB << ", C: " << buttonC << std::endl;
    }

    return std::make_tuple(buttonA, buttonB, buttonC);
}


// Reads the battery status in millivolts from registers 10-11
int AStar::readBatteryMillivolts() {

    // Write the address of the battery voltage and then read the next 2 bytes
    wiringPiI2CWrite(fd,10);
    delayMicroseconds(100);
    int millivolts = wiringPiI2CRead(fd) + (wiringPiI2CRead(fd) << 8);
    
    if (DEBUG_MODE) {
        std::cout << "Read battery millivolts: " << millivolts << std::endl;
    }

    return millivolts;
}


// Reads the 6 ADC values from register 12-23
std::array<int, 6> AStar::readAnalog() {
    std::array<int, 6> analog;

    // Write the address of the first analog byte and then read the next 12 bytes
    wiringPiI2CWrite(fd,12);
    delayMicroseconds(100);
    for (int i = 0; i < 6; ++i) {
        analog[i] = wiringPiI2CRead(fd) + (wiringPiI2CRead(fd) << 8);
    }

    if (DEBUG_MODE) {
        std::cout << "Read analog values: ";
        for (int i = 0; i < 6; ++i) {
            std::cout << analog[i] << " ";
        }
        std::cout << std::endl;
    }

    return analog;
}


// Reads the encoder values from registers 39-42
std::tuple<int, int> AStar::readEncoders() {

    // Write the address of the first encoder and then read the next 4 bytes
    wiringPiI2CWrite(fd,39);
    delayMicroseconds(100);
    int left = wiringPiI2CRead(fd) + (wiringPiI2CRead(fd) << 8);
    int right = wiringPiI2CRead(fd) + (wiringPiI2CRead(fd) << 8);

    if (DEBUG_MODE) {
        std::cout << "Read encoders - Left: " << left << ", Right: " << right << std::endl;
    }

    return std::make_tuple(left, right);
}


// Writes a byte of zeros to register 0
void AStar::testRead8() {
    wiringPiI2CReadReg8(fd, 0);
}


// Reads a byte from register 0
void AStar::testWrite8() {
    wiringPiI2CWriteReg8(fd, 0, 0);
}

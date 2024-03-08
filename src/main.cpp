#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>
#include <array>
#include "entities.h"
// #include <math.h>

int main(void)
{
    ModbusClient *client = new ModbusClient("/dev/ttyUSB0");
    Stepper *stepper1 = new Stepper(0, client);
    Stepper *stepper2 = new Stepper(1, client);

    stepper1->setMaxSpeed(7.13);
    std::cout << stepper1->radiansToDegrees(7.571899212) << std::endl;
    // stepper2->setMaxSpeed(7);

    stepper1->setAcceleration(30 * PI);
    // stepper2->setAcceleration(30 * PI);

    stepper1->rotate(PI);
    // stepper2->rotate(2*PI);

    float current_pos = stepper1->getCurrentPosition();

    std::cout << "Current speed: " << stepper1->getCurrentSpeed() << std::endl;
    std::cout << "Current position: " << fmod(current_pos, 360.0) << std::endl;
}

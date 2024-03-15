#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>
#include <array>
#include "entities.h"
// #include <math.h>

int main(void)
{
    //TODO: delete print of raw modbus request

    ModbusClient *client = new ModbusClient("/dev/ttyUSB0");
    Stepper *stepper1 = new Stepper(0, client);
    Stepper *stepper2 = new Stepper(1, client);

    stepper1->setMaxSpeed(PI);
    stepper2->setMaxSpeed(PI);

    //TODO: check acceleration in toolkit
    stepper1->setAcceleration(0 * PI);
    stepper2->setAcceleration(30 * PI);

    stepper1->rotate(1 * PI);
    stepper2->rotate(2 * PI);

    // float current_pos = stepper1->getCurrentPosition();

    std::cout << "Current acceleration stepper1: " << stepper1->getCurrentAcceleration() << std::endl;
    std::cout << "Current acceleration stepper2: " << stepper2->getCurrentAcceleration() << std::endl;

    std::cout << "Current speed stepper1: " << stepper1->getCurrentSpeed() << std::endl;
    std::cout << "Current speed stepper2: " << stepper2->getCurrentSpeed() << std::endl;

    std::cout << "Current position stepper1: " << stepper1->getCurrentPosition() << std::endl;
    std::cout << "Current position stepper2: " << stepper2->getCurrentPosition() << std::endl;

    std::cout << "Current rotation degree stepper1: " << stepper1->getCurrentRotationDegree() << std::endl;
    std::cout << "Current rotation degree stepper2: " << stepper2->getCurrentRotationDegree() << std::endl;
}

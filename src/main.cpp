#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>
#include <array>
#include "entities.h"
#include <thread>
// #include <math.h>

int main(void)
{
    //TODO: delete print of raw modbus request

    ModbusClient *client = new ModbusClient("/dev/ttyUSB0");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    Stepper *stepper1 = new Stepper(0, client);
    Stepper *stepper2 = new Stepper(1, client);

    for (int i=0; i <= 100; i++) {
    
        stepper1->setMaxSpeed(7.72 * PI);
        stepper2->setMaxSpeed(7.72 * PI);

        std::cout << "Max speed stepper1: " << stepper1->getCurrentSpeed() << std::endl;

        stepper1->setAcceleration(0 * PI);
        stepper2->setAcceleration(0 * PI);

        if (i%2) {
            stepper1->rotate(-2 * PI);
            stepper2->rotate(-2 * PI);
        } else {
            stepper1->rotate(2 * PI);
            stepper2->rotate(2 * PI);
        }
        

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::cout << "Current acceleration stepper1: " << stepper1->getCurrentAcceleration() << std::endl;
        std::cout << "Current acceleration stepper2: " << stepper2->getCurrentAcceleration() << std::endl;

        std::cout << "Current speed stepper1: " << stepper1->getCurrentSpeed() << std::endl;
        std::cout << "Current speed stepper2: " << stepper2->getCurrentSpeed() << std::endl;

        std::cout << "Current position stepper1: " << stepper1->getCurrentPosition() << std::endl;
        std::cout << "Current position stepper2: " << stepper2->getCurrentPosition() << std::endl;

        std::cout << "Current rotation degree stepper1: " << stepper1->getCurrentRotationDegree() << std::endl;
        std::cout << "Current rotation degree stepper2: " << stepper2->getCurrentRotationDegree() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}

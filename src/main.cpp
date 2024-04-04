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
    // TODO: delete print of raw modbus request

    ModbusClient *client = new ModbusClient("/dev/ttyUSB0");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    Stepper *stepper1 = new Stepper(0, client);
    Stepper *stepper2 = new Stepper(1, client);
    Stepper *stepper3 = new Stepper(2, client);

    // stepper1->setMaxSpeed(2 * M_PI);
    stepper2->setMaxSpeed(8 * M_PI);

    stepper2->rotate(-100 * M_PI);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // stepper2->brake();

    // stepper1->reset();
    stepper2->reset();
    // stepper3->reset();

    // stepper2->setMaxSpeed(8 * M_PI);

    // stepper2->rotate(-8 * M_PI);

    // stepper3->setMaxSpeed(8 * M_PI);

    // stepper3->rotate(-8 * M_PI);
}

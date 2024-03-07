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

    stepper1->setMaxSpeed(1);
    stepper2->setMaxSpeed(2*PI);

    stepper1->rotate(PI);
    stepper2->rotate(2 * PI);

    float current_pos = stepper1->getCurrentPosition();
    std::cout << "Current position: " << fmod(current_pos, 360.0) << std::endl;
}

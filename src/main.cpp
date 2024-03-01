#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>
#include <array>
#include "entities.h"

int main(void)
{
    ModbusClient *client = new ModbusClient("/dev/ttyUSB0");
    Stepper *stepper1 = new Stepper(0, client);
    Stepper *stepper2 = new Stepper(1, client);

    stepper1->rotate(PI);
    stepper2->rotate(2 * PI);

    float current_pos = stepper1->getCurrentPosition();
    std::cout << "Current position: " << current_pos << std::endl;
}

#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>
#include <array>
#include "entities.h"

int main(void)
{
    ModbusClient *client = new ModbusClient("/dev/ttyUSB0");
    Stepper *stepper = new Stepper(client);

    stepper->rotate(2*PI);
}

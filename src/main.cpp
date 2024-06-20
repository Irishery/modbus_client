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

    Stepper *stepper1 = new Stepper(0, client);
    Stepper *stepper2 = new Stepper(1, client);
    Stepper *stepper3 = new Stepper(2, client);
    Stepper *stepper4 = new Stepper(3, client);
    Stepper *stepper5 = new Stepper(4, client);   
    Stepper *stepper6 = new Stepper(5, client);

    Stepper steppers[] = {*stepper1, *stepper2, *stepper3, *stepper4, *stepper5, *stepper6};

    SteppersGroup *group = new SteppersGroup(client, steppers);
    group->setMaxSpeedAll(M_PI);
    group->resetAll();
}

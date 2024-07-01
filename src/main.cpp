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
    // TODO: refactor getCurrentPosition funcs due to transfer value
    // TODO: delete print of raw modbus request

    ModbusClient *client = new ModbusClient("/dev/ttyUSB0");

    Stepper *stepper1 = new Stepper(0, client);
    Stepper *stepper2 = new Stepper(1, client);
    Stepper *stepper3 = new Stepper(2, client);
    Stepper *stepper4 = new Stepper(3, client);
    Stepper *stepper5 = new Stepper(4, client);   
    Stepper *stepper6 = new Stepper(5, client);
    Servo *servo = new Servo(client);

    Stepper steppers[] = {*stepper1, *stepper2, *stepper3, *stepper4, *stepper5, *stepper6};

    SteppersGroup *group = new SteppersGroup(client, steppers);
    group->setMaxSpeedAll(M_PI);
    // group->resetAll();
    // stepper2->rotate(3 * 5);
    servo->setMaxSpeed(2);
    // servo->setAcceleration(0);
    // 1.74533
    // stepper2->rotate(0);
    // std::cout << stepper5->getCurrentPosition() << std::endl;
    // servo->rotate(1.74533);
    // servo->setPosition(0);
    // sleep(2);

    // servo->brake();

    // float pos = servo->getCurrentPosition();
    // std::cout << servo->getStatus() << std::endl;
    std::cout << servo->getCurrentPosition() << std::endl;
    // servo->rotate(0);
}

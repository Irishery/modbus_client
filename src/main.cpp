#include <iostream>
#include "entities.h"
#include <math.h>

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
    group->setMaxSpeedAll(2 * M_PI);
    // stepper5->setMaxSpeed(M_PI / 2);
    // stepper6->setMaxSpeed(M_PI / 2);
    // group->resetAll();
    // stepper5->reset();
    // group->resetAll();
    // stepper2->rotate(3 * 5);
    // servo->setMaxSpeed(2);
    // * 4
    // * 2
    // stepper5->rotate(0);
    // stepper6->rotate(-10);

    // stepper5->rotate(-1);
    // stepper6->rotate(1);

    stepper5->rotateNew(-4, -4);
    

    // servo->setAcceleration(0);
    // 1.74533
    // stepper2->rotate(0);
    std::cout << stepper5->getCurrentPosition() << std::endl;
    std::cout << stepper6->getCurrentPosition() << std::endl;
    // servo->rotate(1.74533);
    // servo->setPosition(0);
    // sleep(2);

    // servo->brake();

    // float pos = servo->getCurrentPosition();
    // std::cout << servo->getStatus() << std::endl;
    // std::cout << servo->getCurrentPosition() << std::endl;
    // servo->rotate(0);
}

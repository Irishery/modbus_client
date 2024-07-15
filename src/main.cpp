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
    // stepper6->setMaxSpeed(M_PI / 5);
    // stepper1->rotate(2);
    // stepper2->rotate(2);
    // stepper3->rotate(1);
    // stepper4->rotate(0);
    // stepper5->rotate(2);
    // stepper5->rotateNew(0, 0);
    group->rotateAll(-2, -2, -1, 0, 2, 3);
    // group->getCurrentPositionAll();
    // std::cout << stepper1->getLastTargetDegree() << std::endl;
    // float *positions = group->getCurrentPositionAll();
    // std::cout << positions[0] << std::endl;
    float *speeds = group->getCurrentPositionAll();
    for (int i = 0; i < STEPPERS_COUNT; i++)
    {
        std::cout << speeds[i] << std::endl;
    }
 
}

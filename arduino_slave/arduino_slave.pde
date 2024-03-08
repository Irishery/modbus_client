#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include "GyverStepper.h"
#define steppers_count 2


GStepper<STEPPER2WIRE> stepper1(200, 2, 3);
GStepper<STEPPER2WIRE> stepper2(200, 4, 5);
//Setup the brewtrollers register bank
//All of the data accumulated will be stored here
modbusDevice regBank;
//Create the modbus slave protocol handler
modbusSlave slave;

void setup()
{   
  
//Assign the modbus device ID.  
  regBank.setId(1);
  Serial.begin(9600);

  stepper1.setRunMode(FOLLOW_POS);
  stepper1.autoPower(true);
  //stepper1.autoPower(true);


// rotation 1|0
for (int i = 1; i <= 20; i++) {
    regBank.add(i);
}

// current position
for (int i = 1; i <= 20; i++) {
    regBank.add(30000 + i);
}

// current speed
for (int i = 1; i <= 20; i++) {
    regBank.add(40000 + i);
}

// rotation degree
for (int i = 21; i <= 40; i++) {
    regBank.add(40000 + i);
}

// acceleration speed
for (int i = 41; i <= 60; i++) {
    regBank.add(40000 + i);
}

/*
Assign the modbus device object to the protocol handler
This is where the protocol handler will look to read and write
register data.  Currently, a modbus slave protocol handler may
only have one device assigned to it.
*/
  slave._device = &regBank;  

// Initialize the serial port for coms at 9600 baud  
  slave.setBaud(9600);   
}

float modbus_get_float(const uint16_t *src)
{
    float f;
    uint32_t i;

    i = (((uint32_t) src[1]) << 16) + src[0];
    memcpy(&f, &i, sizeof(float));

    return f;
}

GStepper<STEPPER2WIRE> steppers[] = {stepper1, stepper2};

void loop()
{
  for (int i = 0; i < steppers_count; i++) {
    int is_must_run = regBank.get(i + 1);

    union {
      float asFloat;
      int asInt[2];
    }
    speed;

    float acceleration = regBank.get(40040 + i + 1);
    
    uint16_t data[2];

    data[0] = regBank.get(40000);
    data[1] = regBank.get(40001);
    
    float fln = modbus_get_float(data);

    Serial.println(fln);

    int degree = regBank.get(40020 + i + 1);

    steppers[i].setAcceleration(acceleration);
  
    steppers[i].setMaxSpeedDeg(fln);
    

    if (!steppers[i].tick()) {
      if (is_must_run) {
        steppers[i].setTargetDeg(degree, RELATIVE);
      }
    } else {
      regBank.set(i + 1, 0);
    }

    union {
      float asFloat;
      int asInt[2];
    }
    flreg;


    flreg.asFloat = steppers[0].getCurrentDeg();
    regBank.set(30000 + 1, flreg.asInt[0]);
    regBank.set(30000 + 1 + 1, flreg.asInt[1]);

  }

  slave.run();
}

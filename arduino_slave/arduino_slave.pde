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
  Serial.begin(115200);

  stepper1.setRunMode(FOLLOW_POS);
  // stepper1.autoPower(true);
  // stepper1.setSpeed(260);
  // stepper1.setAcceleration(120);
  

  stepper2.setRunMode(FOLLOW_POS);
  // stepper2.setSpeed(260);
  // stepper2.setAcceleration(120);
  //stepper1.autoPower(true);


// rotation 1|0
for (int i = 1; i <= 20; i++) {
    regBank.add(i);
}

// current position (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
for (int i = 1; i <= 40; i++) {
    regBank.add(30000 + i);
}

// current speed (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
for (int i = 1; i <= 40; i++) {
    regBank.add(40000 + i);
}

// rotation degree (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
for (int j = 41; j <= 60; j++) {
    regBank.add(40000 + j);
}

// acceleration speed (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
// for (int i = 81; i <= 120; i++) {
//     regBank.add(40000 + i);
// }

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
  // if (!stepper2.tick()) {
  //   stepper2.setTargetDeg(360, RELATIVE);
  //   stepper2.tick();
  // }
  // stepper2.setAcceleration(120);
  // stepper2.setSpeed(360);
  // stepper2.setTargetDeg(180);
  // stepper1.tick();
  for (int i = 0; i < steppers_count; i++) {
    int float_id = i + i + 1;

    steppers[i].tick();

    int is_must_run = regBank.get(i + 1);

    // Serial.println("Tick1");
    // uint16_t acceleration[2];
    // acceleration[0] = regBank.get(40080 + (i * 2));
    // acceleration[1] = regBank.get(40080 + (i * 2) + 1);
    // float acceleration_num = modbus_get_float(acceleration);

    uint16_t speed[2];
    speed[0] = regBank.get(40000 + float_id);
    speed[1] = regBank.get(40000 + float_id + 1);
    // Serial.println("SPEED");
    // Serial.println(regBank.get(40000));
    // Serial.println(regBank.get(40001));
    // Serial.println(regBank.get(40002));
    // Serial.println(regBank.get(40003));
    // Serial.println(regBank.get(40004));
    // Serial.println(regBank.get(40005));

    float speed_num = modbus_get_float(speed);
    // Serial.println(speed_num);
    // Serial.println("---");

    // Serial.println("Tick2");

    uint16_t degree[2];
    degree[0] = regBank.get(40040 + float_id);
    degree[1] = regBank.get(40040 + float_id + 1);
    // Serial.println("DEGREE");
    // Serial.println(regBank.get(40040));
    // Serial.println(regBank.get(40041));
    // Serial.println(regBank.get(40042));
    // Serial.println(regBank.get(40043));
    // Serial.println(regBank.get(40044));
    // Serial.println(regBank.get(40045));
    
    float degree_num = modbus_get_float(degree);
    // Serial.println(degree_num);
    // Serial.println("---");

    steppers[i].tick();
    // Serial.println("Tick3");
    steppers[i].setAcceleration(120);
    steppers[i].setMaxSpeedDeg(speed_num);
    // Serial.println("Tick4");

    if (!steppers[i].tick()) {
      // Serial.println("ping1");
      if (is_must_run) {
        steppers[i].setTargetDeg(degree_num, RELATIVE);
      }
    } else {
      // Serial.println("ping2");
      regBank.set(i + 1, 0);
    }

    steppers[i].tick();
    // Serial.println("Tick5");

    // union {
    //   float asFloat;
    //   int asInt[2];
    // }
    // flreg;


    // flreg.asFloat = steppers[0].getCurrentDeg();
    // regBank.set(30000 + float_id, flreg.asInt[0]);
    // regBank.set((30000 + float_id + 1), flreg.asInt[1]);

  }

  slave.run();
}

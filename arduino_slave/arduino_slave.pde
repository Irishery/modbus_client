#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include "GyverStepper.h"
#define STEPPERS_COUNT 2


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
  stepper1.setSpeed(260);
  stepper1.setAcceleration(120);
  

  stepper2.setRunMode(FOLLOW_POS);
  stepper1.setTargetDeg(180, ABSOLUTE);
  stepper2.setTargetDeg(180, ABSOLUTE);
  stepper2.setSpeed(260);
  stepper2.setAcceleration(120);
  //stepper1.autoPower(true);


// rotation 1|0
// for (int i = 1; i <= 3; i++) {
//     regBank.add(i);
// }

// // current position (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
// for (int i = 1; i <= (3*2); i++) {
//     regBank.add(30000 + i);
// }

// // current speed (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
// for (int i = 1; i <= (3*2); i++) {
//     regBank.add(40000 + i);
// }

// // rotation degree (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
// for (int j = (3 * 2 + 1); j <= (3 * 4); j++) {
//     regBank.add(40000 + j);
// }

// // acceleration speed (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
// for (int i = (3 * 4 + 1); i <= (3 * 6); i++) {
//     regBank.add(40000 + i);
// }

// rotation 1|0
for (int i = 1; i <= STEPPERS_COUNT; i++) {
    regBank.add(i);
}

// current position (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
for (int i = 1; i <= (STEPPERS_COUNT*2); i++) {
    regBank.add(30000 + i);
}

// current speed (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
for (int i = 1; i <= (STEPPERS_COUNT*2); i++) {
    regBank.add(40000 + i);
}

// rotation degree (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
for (int i = (STEPPERS_COUNT*2 + 1); i <= (STEPPERS_COUNT*4); i++) {
    regBank.add(40000 + i);
}

// acceleration speed (each stepper takes a 2 registers (0, 1), (2, 3) etc.)
for (int i = (STEPPERS_COUNT*4 + 1); i <= (STEPPERS_COUNT*6); i++) {
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
  slave.setBaud(115200);   
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

  for (int i = 0; i < STEPPERS_COUNT; i++) {
    int is_must_run = regBank.get(i + 1);
    if(is_must_run) {
    int float_id = i + i + 1;

    steppers[i].setRunMode(FOLLOW_POS);

    uint16_t speed[2];
    speed[0] = regBank.get(40000 + float_id);
    speed[1] = regBank.get(40000 + float_id + 1);
    float speed_num = modbus_get_float(speed);
    
    // Serial.println("SPEED");
    // Serial.println(speed_num);
    // Serial.println("-----------");

    uint16_t degree[2];
    degree[0] = regBank.get(40000 + STEPPERS_COUNT*2 + float_id);
    degree[1] = regBank.get(40000 + STEPPERS_COUNT*2 + float_id + 1);
    float degree_num = modbus_get_float(degree);

    // Serial.println("DEGREE");
    // Serial.println(degree_num);
    // Serial.println("-----------");

    uint16_t acceleration[2];
    acceleration[0] = regBank.get(40000 + STEPPERS_COUNT*2 + float_id);
    acceleration[1] = regBank.get(40000 + STEPPERS_COUNT*2 + float_id + 1);
    float acceleration_num = modbus_get_float(acceleration);

    // Serial.println("DEGREE");
    // Serial.println(acceleration_num);
    // Serial.println("-----------");

    //steppers[i].tick();

    steppers[i].setAcceleration(acceleration_num);
    steppers[i].setMaxSpeedDeg(speed_num);

    int is_must_run = regBank.get(i + 1);
    // if (!steppers[i].tick()) {
      // if (is_must_run) {
    // steppers[i].tick();
    steppers[i].setTargetDeg(degree_num, ABSOLUTE);
    // steppers[i].tick();
    //   }
    // } else {
    regBank.set(i + 1, 0);
    // }
    }
    //steppers[i].tick();

  }

  for(int i=0;i<STEPPERS_COUNT;i++)
  {
      //int is_must_run = regBank.get(i + 1);
      //stepper1.setTargetDeg(180, ABSOLUTE);
      //if(is_must_run)
      steppers[i].tick();
  }

  /*for(int i=0;i<STEPPERS_COUNT;i++)
  {
      int float_id = i + i + 1;
      union {
        float asFloat;
        int asInt[2];
      }
      flreg;

      flreg.asFloat = steppers[i].getCurrentDeg();
      regBank.set(30000 + float_id, flreg.asInt[0]);
      regBank.set((30000 + float_id + 1), flreg.asInt[1]);
  }
*/
  slave.run();
}

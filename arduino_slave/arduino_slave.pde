#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include "GyverStepper.h"
#include <ModbusRTUSlave.h>

#define ENDSTOP_PIN1 10
#define ENDSTOP_PIN2 11
// #define ENDSTOP_PIN3 12

#define STEPPERS_COUNT 2
#define ENDSTOPS_COUNT 2



GStepper<STEPPER2WIRE> stepper1(200, 2, 3);
GStepper<STEPPER2WIRE> stepper2(200, 4, 5);
// GStepper<STEPPER2WIRE> stepper2(200, 7, 8);
//Setup the brewtrollers register bank
//All of the data accumulated will be stored here
// modbusDevice regBank;
// //Create the modbus slave protocol handler
// modbusSlave slave;

ModbusRTUSlave modbus(Serial);

bool coils[STEPPERS_COUNT*4];
uint16_t holdingRegisters[STEPPERS_COUNT*6];
uint16_t inputRegisters[STEPPERS_COUNT*2];

void setup()
{   
  // delay(5000);
//Assign the modbus device ID.  
  Serial.begin(9600);

  stepper1.setRunMode(FOLLOW_POS);
  stepper1.setTargetDeg(0);

  stepper2.setRunMode(FOLLOW_POS);
  stepper2.setTargetDeg(0);

  pinMode(ENDSTOP_PIN1, INPUT_PULLUP);
  pinMode(ENDSTOP_PIN2, INPUT_PULLUP);
  // pinMode(ENDSTOP_PIN3, INPUT_PULLUP);


modbus.configureCoils(coils, STEPPERS_COUNT*4);                       // bool array of coil values, number of coils
modbus.configureHoldingRegisters(holdingRegisters, STEPPERS_COUNT*6); // unsigned 16 bit integer array of holding register values, number of holding registers
modbus.configureInputRegisters(inputRegisters, STEPPERS_COUNT*2); 

/*
Assign the modbus device object to the protocol handler
This is where the protocol handler will look to read and write
register data.  Currently, a modbus slave protocol handler may
only have one device assigned to it.
*/
//   slave._device = &regBank;  

// // Initialize the serial port for coms at 9600 baud  
  modbus.begin(1, 115200);
  // modbus.begin(1, 9600);
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
int endstops[] = {ENDSTOP_PIN1, ENDSTOP_PIN2};

void loop()
{
  for (int i = 0; i < STEPPERS_COUNT; i++) {
    int is_must_run = coils[i];
    int is_must_brake = coils[STEPPERS_COUNT + i];
    int is_must_reset = coils[STEPPERS_COUNT*2 + i];

    if(is_must_brake)
    {
      steppers[i].brake();
      coils[STEPPERS_COUNT + i] = 0;
      continue;
    }

    if(is_must_reset)
    {
      if (!steppers[i].tick()) {
        steppers[i].setRunMode(KEEP_SPEED);
        steppers[i].setSpeedDeg(180);
        steppers[i].tick();
      }

      if (digitalRead(endstops[i])) {
        steppers[i].reset();
        steppers[i].setRunMode(FOLLOW_POS);
        steppers[i].setTargetDeg(0);
        coils[STEPPERS_COUNT*2 + i] = 0;
      }

      continue;
    }


    if(is_must_run) 
    {
      int float_id = i + i;

      // Serial.println("CURRENT TARGET");
      // Serial.println(steppers[i].getTargetDeg());
      // Serial.println(steppers[i].getTarget());
      // Serial.println("-----------");

      uint16_t speed[2];
      speed[0] = holdingRegisters[float_id];
      speed[1] = holdingRegisters[float_id + 1];
      float speed_num = modbus_get_float(speed);

      uint16_t degree[2];
      degree[0] = holdingRegisters[STEPPERS_COUNT*2 + float_id];
      degree[1] = holdingRegisters[STEPPERS_COUNT*2 + float_id + 1];
      float degree_num = modbus_get_float(degree);

      uint16_t acceleration[2];
      acceleration[0] = holdingRegisters[STEPPERS_COUNT*4 + float_id];
      acceleration[1] = holdingRegisters[STEPPERS_COUNT*4 + float_id + 1];
      float acceleration_num = modbus_get_float(acceleration);

      steppers[i].setAcceleration(acceleration_num);
      steppers[i].setMaxSpeedDeg(speed_num);

      int is_must_run = coils[i];

      steppers[i].setTargetDeg(degree_num, ABSOLUTE);

      coils[i] = 0;
    }
  }

  for(int i=0;i<STEPPERS_COUNT;i++)
  {
    steppers[i].tick();
    if (steppers[i].getState()) 
    {
      coils[STEPPERS_COUNT*3 + i] = 1;
    } 
    else {
      coils[STEPPERS_COUNT*3 + i] = 0;
    }
  }

  for(int i=0;i<STEPPERS_COUNT;i++)
  {
      int float_id = i + i;
      union {
        float asFloat;
        int asInt[2];
      }
      flreg;
      uint16_t position[2];

      flreg.asFloat = steppers[i].getCurrentDeg();
      inputRegisters[float_id] = flreg.asInt[0];
      inputRegisters[float_id + 1] = flreg.asInt[1];
  }

  modbus.poll();
}

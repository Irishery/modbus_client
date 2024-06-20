#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include "GyverStepper.h"
#include <ModbusRTUSlave.h>

const int ENDSTOP_PIN1 = A0;
const int ENDSTOP_PIN2 = A1;
const int ENDSTOP_PIN3 = A2;
const int ENDSTOP_PIN4 = A3;
const int ENDSTOP_PIN5 = A4;

#define STEPPERS_COUNT 6
#define ENDSTOPS_COUNT 5

GStepper<STEPPER2WIRE> stepper1(3200, 8, A6);
GStepper<STEPPER2WIRE> stepper2(3200, 10, 11);
GStepper<STEPPER2WIRE> stepper3(3200, 12, 13);
GStepper<STEPPER2WIRE> stepper4(3200, 2, 3);
GStepper<STEPPER2WIRE> stepper5(3200, 6, 7);
GStepper<STEPPER2WIRE> stepper6(3200, 4, 5);

ModbusRTUSlave modbus(Serial);

bool coils[STEPPERS_COUNT*4];
uint16_t holdingRegisters[STEPPERS_COUNT*6];
uint16_t inputRegisters[STEPPERS_COUNT*2];

void setup()
{
  Serial.begin(9600);

  pinMode(ENDSTOP_PIN1, INPUT);
  pinMode(ENDSTOP_PIN2, INPUT);
  pinMode(ENDSTOP_PIN3, INPUT);
  pinMode(ENDSTOP_PIN4, INPUT);
  pinMode(ENDSTOP_PIN5, INPUT);


  modbus.configureCoils(coils, STEPPERS_COUNT*4);                       // bool array of coil values, number of coils
  modbus.configureHoldingRegisters(holdingRegisters, STEPPERS_COUNT*6); // unsigned 16 bit integer array of holding register values, number of holding registers
  modbus.configureInputRegisters(inputRegisters, STEPPERS_COUNT*2); 
  
  modbus.begin(1, 115200);
}

float modbus_get_float(const uint16_t *src)
{
    float f;
    uint32_t i;

    i = (((uint32_t) src[1]) << 16) + src[0];
    memcpy(&f, &i, sizeof(float));

    return f;
}

GStepper<STEPPER2WIRE> steppers[] = {stepper1, stepper2, stepper3, stepper4, stepper5, stepper6};
int endstops[] = {ENDSTOP_PIN1, ENDSTOP_PIN2, ENDSTOP_PIN3, ENDSTOP_PIN4, ENDSTOP_PIN5};
int endstops_type[] = {1, 1, 0, 0, 0}; // we have didffenet endstops models, so there is a need to separate conditions of end

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

      if ((digitalRead(endstops[i]) && endstops_type[i]) || (!digitalRead(endstops[i]) && !endstops_type[i])) {
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

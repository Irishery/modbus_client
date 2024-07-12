#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include "GyverStepper.h"
#include <ModbusRTUSlave.h>
#include "ServoSmooth.h"  

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

ServoSmooth servo;

ModbusRTUSlave modbus(Serial);

bool coils[STEPPERS_COUNT*4 + 3];
uint16_t holdingRegisters[STEPPERS_COUNT*6 + 6];
uint16_t inputRegisters[STEPPERS_COUNT*2 + 2];

float last_s5_pos = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(ENDSTOP_PIN1, INPUT);
  pinMode(ENDSTOP_PIN2, INPUT);
  pinMode(ENDSTOP_PIN3, INPUT);
  pinMode(ENDSTOP_PIN4, INPUT);
  pinMode(ENDSTOP_PIN5, INPUT);

  // servo
  servo.attach(9);

  modbus.configureCoils(coils, STEPPERS_COUNT*4 + 3);                       // bool array of coil values, number of coils
  modbus.configureHoldingRegisters(holdingRegisters, STEPPERS_COUNT*6 + 6); // unsigned 16 bit integer array of holding register values, number of holding registers
  modbus.configureInputRegisters(inputRegisters, STEPPERS_COUNT*2 + 2); 
  
  // modbus.begin(1, 115200);
  modbus.begin(1, 9600);
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

// 2 - rotate
// 1 - to endstoup
// 0 - out of endstop
// -1 - error
int get_action() {
  float t5 = steppers[4].getTargetDeg();
  float t6 = steppers[5].getTargetDeg();
  if ((t5 == t6) && (t5 != 0)) {
    return 2; // rotate
  } else if (t5 == -t6) {
    return 1; // lean
  }
  return -1; // error
}
int endstops[] = {ENDSTOP_PIN1, ENDSTOP_PIN2, ENDSTOP_PIN3, ENDSTOP_PIN4, ENDSTOP_PIN5};
int endstops_type[] = {1, 1, 0, 0, 0}; // we have didffenet endstops models, so there is a need to separate conditions of end

float target4_prev = 0;
float cur4_prev = 0;
float current4 = 0;
float target5_prev = 0;
float cur5_prev = 0;
float current5 = 0;
float dtarget4 = 0;
float dtarget5 = 0;

float dtarget4_prev = 0;
float dtarget5_prev = 0;

void loop() 
{
  for (int i = 0; i < STEPPERS_COUNT; i++) {
    int is_must_run = coils[i];
    int is_must_brake = coils[STEPPERS_COUNT + i];
    int is_must_reset = coils[STEPPERS_COUNT*2 + i]; 

    // float s5t = stepper[4].getTargetDeg();
    // float s6t = stepper[5].getTargetDeg();

    // int act = get_action();
    // if (act != -1) {
    //   steppers[5].tick();
    // }

    if(is_must_brake)
    {
      steppers[i].brake();
      coils[STEPPERS_COUNT + i] = 0;
      continue;
    }

    if(is_must_reset)
    
    {
      // steppers[5].tick();
      // stepper6.tick();
      if (!steppers[i].tick()) {
        // steppers[5].tick();
        steppers[i].setRunMode(KEEP_SPEED);
        steppers[i].setSpeedDeg(180);
        steppers[i].tick();
      }

      if ((digitalRead(endstops[i]) && endstops_type[i]) || (!digitalRead(endstops[i]) && !endstops_type[i])) {
        // steppers[5].tick();
        // stepper6.tick();
        steppers[i].reset();
        
        steppers[i].setRunMode(FOLLOW_POS);
        steppers[i].setTargetDeg(0);
        coils[STEPPERS_COUNT*2 + i] = 0;
      }
      // stepper6.tick();

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

      if(i==4 || i==5)
      {

      }
      else
        steppers[i].setTargetDeg(degree_num, ABSOLUTE);

      coils[i] = 0;
    }
  }

  for(int i=0;i<STEPPERS_COUNT-2;i++)
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

  int action = get_action();

  uint16_t degree[2];
  degree[0] = holdingRegisters[STEPPERS_COUNT*2+8];
  degree[1] = holdingRegisters[STEPPERS_COUNT*2 + 8 + 1];
  float target4 = modbus_get_float(degree);
  degree[0] = holdingRegisters[STEPPERS_COUNT*2+10];
  degree[1] = holdingRegisters[STEPPERS_COUNT*2 + 10 + 1];
  float target5 = modbus_get_float(degree);

  float cur4 = steppers[4].getCurrentDeg();
  float dcur4 = cur4 - cur4_prev;

  float cur5 = steppers[5].getCurrentDeg();
  float dcur5 = cur5 - cur5_prev;

  //current4 + dcur;
  union {
    float asFloat;
    int asInt[2];
  } flreg1;
  flreg1.asFloat = current4 + dtarget4*dcur4/(dtarget4+dtarget5+0.0001);
  inputRegisters[STEPPERS_COUNT * 2 - 4] = flreg1.asInt[0];
  inputRegisters[STEPPERS_COUNT * 2 - 3] = flreg1.asInt[1];
  union {
    float asFloat;
    int asInt[2];
  } flreg2;
  flreg2.asFloat = current5 + dtarget5*dcur5/(-dtarget4+dtarget5+0.0001);
  inputRegisters[STEPPERS_COUNT * 2 - 2] = flreg2.asInt[0];
  inputRegisters[STEPPERS_COUNT * 2 - 1] = flreg2.asInt[1];

  /*if (target4 != target4_prev && target5 == target5_prev) 
  { 
    float dtarget = target4 - target4_prev;
    target4_prev = target4;
    current4 += dcur4;
    cur4_prev = cur4;

    steppers[4].setTargetDeg(dtarget, RELATIVE);
    steppers[5].setTargetDeg(-dtarget, RELATIVE);
  }

  if (target5 != target5_prev && target4 == target4_prev) 
  { 
    float dtarget = target5 - target5_prev;
    target5_prev = target5;
    current5 += dcur5;
    cur5_prev = cur5;

    steppers[4].setTargetDeg(dtarget, RELATIVE);
    steppers[5].setTargetDeg(dtarget, RELATIVE);
  }*/

  if(abs(target4 - target4_prev)>0.01 && abs(target5 - target5_prev)>0.01)
  {
    dtarget4 = target4 - target4_prev;
    target4_prev = target4;

    dtarget5 = target5 - target5_prev;
    target5_prev = target5;

    current4 += dcur4*dtarget4_prev/(dtarget4_prev+dtarget5_prev+0.001);
    cur4_prev = cur4;

    current5 += dcur5*dtarget5_prev/(-dtarget4_prev+dtarget5_prev+0.001);
    cur5_prev = cur5;

    dtarget4_prev = dtarget4;
    dtarget5_prev = dtarget5;

    //if(dtarget4>0 && dtarget5>0)
    //{
     steppers[4].setTargetDeg(dtarget4 + dtarget5, RELATIVE);
     steppers[5].setTargetDeg(-dtarget4 + dtarget5, RELATIVE);
    //}
    //else if(dtarget4>0 && dtarget5<0)
    //{
    //  steppers[4].setTargetDeg(0, RELATIVE);
    //  steppers[5].setTargetDeg(dtarget4, RELATIVE);
    //}
  }

  uint16_t input_reg5[2] = {inputRegisters[STEPPERS_COUNT * 2 - 4], inputRegisters[STEPPERS_COUNT * 2 - 3]};
  uint16_t input_reg6[2] = {inputRegisters[STEPPERS_COUNT * 2 - 2], inputRegisters[STEPPERS_COUNT * 2 - 1]};


  for(int i=0;i<STEPPERS_COUNT;i++)
  {
    // steppers[5].tick();
    // stepper6.tick();
    steppers[i].tick();
    if (steppers[i].getState()) 
    {
      coils[STEPPERS_COUNT*3 + i] = 1;
    } 
    else {
      coils[STEPPERS_COUNT*3 + i] = 0;
    }
  }

  // Servo
  int is_servo_must_run = coils[STEPPERS_COUNT*4];
  int is_servo_must_brake = coils[STEPPERS_COUNT*4 + 1];

  if (is_servo_must_brake) 
  {
    servo.stop();
    coils[STEPPERS_COUNT*4] = 0;
    coils[STEPPERS_COUNT*4 + 1] = 0;
  }

  if (is_servo_must_run) 
  {
      servo.start();

      uint16_t speed[2];
      speed[0] = holdingRegisters[STEPPERS_COUNT*6];
      speed[1] = holdingRegisters[STEPPERS_COUNT*6 + 1];
      float speed_num = modbus_get_float(speed);

      uint16_t degree[2];
      degree[0] = holdingRegisters[STEPPERS_COUNT*6 + 2];
      degree[1] = holdingRegisters[STEPPERS_COUNT*6 + 3];
      float degree_num = modbus_get_float(degree);

      uint16_t acceleration[2];
      acceleration[0] = holdingRegisters[STEPPERS_COUNT*6 + 4];
      acceleration[1] = holdingRegisters[STEPPERS_COUNT*6 + 5];
      float acceleration_num = modbus_get_float(acceleration);

      servo.setSpeed(speed_num);
      servo.setAccel(acceleration_num);
      servo.setTargetDeg(degree_num);

      coils[STEPPERS_COUNT*4] = 0;

  }

  union {
    float asFloat;
    int asInt[2];
  } flreg;
  uint16_t position[2];

  flreg.asFloat = servo.getCurrentDeg();
  inputRegisters[STEPPERS_COUNT*2] = flreg.asInt[0];
  inputRegisters[STEPPERS_COUNT*2 + 1] = flreg.asInt[1];

  if (!servo.tick())
  {
    coils[STEPPERS_COUNT*4 + 2] = 1;
  } else 
  {
    coils[STEPPERS_COUNT*4 + 2] = 0;
  }
  servo.tick();

  modbus.poll();
}

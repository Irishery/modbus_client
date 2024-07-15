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

GStepper<STEPPER2WIRE> stepper1(3200, 8, 6);
GStepper<STEPPER2WIRE> stepper2(3200, 10, 11);
GStepper<STEPPER2WIRE> stepper3(3200, 12, 13);
GStepper<STEPPER2WIRE> stepper4(3200, 2, 3);
GStepper<STEPPER2WIRE> stepper5(3200, A6, 7);
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

int endstops[] = {ENDSTOP_PIN1, ENDSTOP_PIN2, ENDSTOP_PIN3, ENDSTOP_PIN4, ENDSTOP_PIN5};
int endstops_type[] = {1, 1, 0, 0, 0}; // we have didffenet endstops models, so there is a need to separate conditions of end

float targetLean_prev = 0;
float cur4_prev = 0;
float currentLean = 0;
float targetRotate_prev = 0;
float cur5_prev = 0;
float currentRotate = 0;
float dtargetLean = 0;
float dtargetRotate = 0;
float dtargetLean_prev = 0;
float dtargetRotate_prev = 0;

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
        steppers[i].setTargetDeg(degree_num);

      coils[i] = 0;
    }
  }
  // Serial.println("-------------------------");
  // Serial.println(steppers[0].getCurrentDeg());
  // Serial.println(steppers[0].getTargetDeg());
  // Serial.println("-------------------------");

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

  ///////////////////////////////////////////
  // Секция работы с узлом поворота и наклона
  ///////////////////////////////////////////

  // Получение таргетов
  uint16_t degree[2];
  degree[0] = holdingRegisters[STEPPERS_COUNT*2+8];
  degree[1] = holdingRegisters[STEPPERS_COUNT*2 + 8 + 1];
  float targetLean = modbus_get_float(degree);
  degree[0] = holdingRegisters[STEPPERS_COUNT*2+10];
  degree[1] = holdingRegisters[STEPPERS_COUNT*2 + 10 + 1];
  float targetRotate = modbus_get_float(degree);

  // // Расчет изменения положения
  float cur4 = steppers[4].getCurrentDeg();
  float dcur4 = cur4 - cur4_prev;
  float cur5 = steppers[5].getCurrentDeg();
  float dcur5 = cur5 - cur5_prev;

  // // Передача изменненного положения
  union {
    float asFloat;
    int asInt[2];
  } floatIntUnion;
  floatIntUnion.asFloat = currentLean + dtargetLean*dcur4/(dtargetLean+dtargetRotate+0.0001);
  inputRegisters[STEPPERS_COUNT * 2 - 4] = floatIntUnion.asInt[0];
  inputRegisters[STEPPERS_COUNT * 2 - 3] = floatIntUnion.asInt[1];
  floatIntUnion.asFloat = currentRotate + dtargetRotate*dcur5/(-dtargetLean+dtargetRotate+0.0001);
  inputRegisters[STEPPERS_COUNT * 2 - 2] = floatIntUnion.asInt[0];
  inputRegisters[STEPPERS_COUNT * 2 - 1] = floatIntUnion.asInt[1];

  // // Расчет и устрановка таргетов на моторы
  if(abs(targetLean - targetLean_prev)>0.01 || abs(targetRotate - targetRotate_prev)>0.01)
  {
    dtargetLean = targetLean - targetLean_prev;
    targetLean_prev = targetLean;

    dtargetRotate = targetRotate - targetRotate_prev;
    targetRotate_prev = targetRotate;

    currentLean += dcur4*dtargetLean_prev/(dtargetLean_prev+dtargetRotate_prev+0.001);
    cur4_prev = cur4;

    currentRotate += dcur5*dtargetRotate_prev/(-dtargetLean_prev+dtargetRotate_prev+0.001);
    cur5_prev = cur5;

    dtargetLean_prev = dtargetLean;
    dtargetRotate_prev = dtargetRotate;

    steppers[4].setTargetDeg(dtargetLean + dtargetRotate, RELATIVE);
    steppers[5].setTargetDeg(-dtargetLean + dtargetRotate, RELATIVE);
  }

  /////////////////////////////////////////////////
  // Конец секции работы с узлом поворота и наклона
  /////////////////////////////////////////////////

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

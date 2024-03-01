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
  //Serial.begin(9600);

  stepper1.setRunMode(FOLLOW_POS);
  stepper1.setMaxSpeed(120);
  stepper1.setAcceleration(500);
  stepper1.autoPower(true);
  //stepper1.autoPower(true);


// rotation 1|0
  regBank.add(1);
  regBank.add(2);
  regBank.add(3);
  regBank.add(4);
  regBank.add(5);
  regBank.add(6);
  regBank.add(7);
  regBank.add(8);
  regBank.add(9);
  regBank.add(10);
  regBank.add(11);
  regBank.add(12);
  regBank.add(13);
  regBank.add(14);
  regBank.add(15);
  regBank.add(16);
  regBank.add(17);
  regBank.add(18);
  regBank.add(19);
  regBank.add(20);

  regBank.add(10001);
  regBank.add(10002);
  regBank.add(10003);
  regBank.add(10004);
  regBank.add(10005);
  regBank.add(10006);
  regBank.add(10007);
  regBank.add(10008);


// current position
  regBank.add(30001);
  regBank.add(30002);
  regBank.add(30003);
  regBank.add(30004);
  regBank.add(30005);
  regBank.add(30006);
  regBank.add(30007);
  regBank.add(30008);
  regBank.add(30009);
  regBank.add(30010);
  regBank.add(30011);
  regBank.add(30012);
  regBank.add(30013);
  regBank.add(30014);
  regBank.add(30015);
  regBank.add(30016);
  regBank.add(30017);
  regBank.add(30018);
  regBank.add(30019);
  regBank.add(30020);

// current speed
  regBank.add(40001);
  regBank.add(40002);
  regBank.add(40003);
  regBank.add(40004);
  regBank.add(40005);
  regBank.add(40006);
  regBank.add(40007);
  regBank.add(40008);
  regBank.add(40009);
  regBank.add(40010);
  regBank.add(40011);
  regBank.add(40012);
  regBank.add(40013);
  regBank.add(40014);
  regBank.add(40015);
  regBank.add(40016);
  regBank.add(40017);
  regBank.add(40018);
  regBank.add(40019);
  regBank.add(40020);

// current angle
  regBank.add(40021);
  regBank.add(40022);
  regBank.add(40023);
  regBank.add(40024);
  regBank.add(40025);
  regBank.add(40026);
  regBank.add(40027);
  regBank.add(40028);
  regBank.add(40029);
  regBank.add(40030);
  regBank.add(40031);
  regBank.add(40032);
  regBank.add(40033);
  regBank.add(40034);
  regBank.add(40035);
  regBank.add(40036);
  regBank.add(40037);
  regBank.add(40038);
  regBank.add(40039);
  regBank.add(40040);

  // request to write
  regBank.add(40041); // register id
  regBank.add(40042); // value
  regBank.add(40043); // flag is request new or not

  // request to run
  regBank.add(40047);
  regBank.add(40048);
  regBank.add(40049);
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

GStepper<STEPPER2WIRE> steppers[] = {stepper1, stepper2};


void loop()
{
  for (int i = 0; i < steppers_count; i++) {
    int is_must_run = regBank.get(i + 1);
    int degree = regBank.get(40000 + i + 1);

    if (!steppers[i].tick()) {
      if (is_must_run) {
        steppers[i].setTargetDeg(degree, RELATIVE);
      }
    } else {
      regBank.set(i + 1, 0);
    }
  }

  slave.run();
}

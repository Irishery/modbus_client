#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>
#include <array>
#include <math.h>

// #define PI 3.14159265358979323
#define PI 3.14159265359

// ------------ Registers scheme ------------

// - **Rotation 1|0 Registers (1-20)**:
// These registers related to the rotation control with values ranging from 1 to 20.

// - **Current Position Registers (30001-30020)**:
// These registers store the current positions of certain elements or components, with values ranging from 30001 to 30020.

// - **Current Speed Registers (40001-40020)**:
// These registers hold the current speed values of the system, with values ranging from 40001 to 40020.

// - **Rotation Degree Registers (40021-40040)**:
// These registers used to store the rotation degrees of specific components, with values ranging from 40021 to 40040.

// - **Acceleration Speed Registers (40041-40060)**:
// These registers dedicated to storing the acceleration speeds of the system, with values ranging from 40041 to 40060.

// ------------ Modbus Client ------------
class ModbusClient
{
private:
    modbus_t *ctx;

public:
    ModbusClient(const char *port);

    void setSlave(int slave);

    uint16_t *readRegisters(int address, int count);

    int writeRegister(int address, int value);
    int writeRegisters(int address, uint16_t *values, int count);

    uint16_t *readInputRegisters(int address, int count);
    uint8_t *readBits(int address);

    int writeBit(int address, int status);
};

ModbusClient::ModbusClient(const char *port)
{
    this->ctx = modbus_new_rtu(port, 9600, 'N', 8, 1);
    if (ctx == NULL)
    {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return;
    }

    if (modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return;
    }

    this->setSlave(1);
};

void ModbusClient::setSlave(int slave)
{
    modbus_set_slave(ctx, slave);
};

uint16_t *ModbusClient::readRegisters(int address, int count)
{
    uint16_t *tab_reg = (uint16_t *)malloc(sizeof(uint16_t) * count);
    modbus_read_registers(this->ctx, address, count, tab_reg);

    return tab_reg;
};

uint16_t *ModbusClient::readInputRegisters(int address, int count)
{
    uint16_t *tab_reg = (uint16_t *)malloc(sizeof(uint16_t) * count);
    modbus_read_input_registers(this->ctx, address, count, tab_reg);

    return tab_reg;
};

int ModbusClient::writeRegister(int address, int value)
{
    int success = modbus_write_register(this->ctx, address, value); // 1|-1
    return success;
};

int ModbusClient::writeRegisters(int address, uint16_t *values, int count)
{
    int success = modbus_write_registers(this->ctx, address, count, values); // 1|-1
    return success;
};

uint8_t *ModbusClient::readBits(int address)
{
    uint8_t *tab_bits = (uint8_t *)malloc(sizeof(uint16_t));
    modbus_read_bits(this->ctx, address, 1, tab_bits);
    return tab_bits;
};

int ModbusClient::writeBit(int address, int status)
{
    int success = modbus_write_bit(this->ctx, address, status); // 1|-1
    return success;
};

// ------------ Stepper ------------

class Stepper
{
private:
    

public:
    float radiansToDegrees(float radian);
    int stepper_id;
    // int speed_id;

    ModbusClient *client;
    Stepper(int id, ModbusClient *client);

    void setRotationDegree(float radian);
    void setMaxSpeed(float rad_per_sec);
    void setAcceleration(float rad_per_sec_sq);

    void rotate(float degree);

    float getCurrentPosition();
    float getCurrentSpeed();
    int getCurrentAcceleration();
};

Stepper::Stepper(int id, ModbusClient *client)
{
    this->stepper_id = id;
    this->client = client;

    // if (stepper_id != 0) {
    //     this->speed_id = stepper_id + 2
    // };
};

float Stepper::radiansToDegrees(float radian)
{
    return (radian * (180 / PI));
};

void Stepper::setRotationDegree(float radian)
{

    int succes = this->client->writeRegister(this->stepper_id + 20, this->radiansToDegrees(radian)); // add 20 because of registers scheme
    if (succes == -1)
    {
        std::cout << "Error writing register" << std::endl;
    }
};

void Stepper::setMaxSpeed(float rad_per_sec)
{
    uint16_t speed_in_uint_format[2];
    // int speed = this->radiansToDegrees(rad_per_sec);
    float speed = this->radiansToDegrees(rad_per_sec);

    modbus_set_float(speed, speed_in_uint_format);

    int succes = this->client->writeRegisters(this->stepper_id, speed_in_uint_format, 2);

    if (succes == -1)
    {
        std::cout << "Error writing register" << std::endl;
    }
};

void Stepper::setAcceleration(float rad_per_sec_sq)
{
    int succes = this->client->writeRegister(this->stepper_id + 40, this->radiansToDegrees(rad_per_sec_sq));
    if (succes == -1)
    {
        std::cout << "Error writing register" << std::endl;
    }
};

void Stepper::rotate(float radian)
{
    this->setRotationDegree(radian);
    int succes = this->client->writeBit(this->stepper_id, 1);
    if (succes == -1)
    {
        std::cout << "Error writing bit" << std::endl;
    }
};

// TODO: rethink concept of using modbus directly from Stepper object
float Stepper::getCurrentPosition()
{
    uint16_t *position = this->client->readInputRegisters(this->stepper_id, 2);
    return modbus_get_float(position);
};

float Stepper::getCurrentSpeed()
{
    // uint16_t *speed = this->client->readRegisters(this->stepper_id + 20, 4);
    uint16_t *speed = this->client->readRegisters(this->stepper_id, 2);

    return modbus_get_float(speed);
};

// // ------------ Stepper Group ------------
// class StepperGroup {
//     private:
//         std::array<Stepper*, 20> stepper_group;

//     public:
//         StepperGroup(ModbusClient *client);

//         int stepper_count;

//         std::array<int, 20> stepper_ids;

// };

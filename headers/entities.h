#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>
#include <array>
#include <math.h>

#define STEPPERS_COUNT 3

#define READY 0
#define ROTATION 1
#define CALIBRATION 2

// ------------ Registers scheme ------------
// 1. Rotation Status Registers: 1 to STEPPERS_COUNT*4
// 1.1 Starts rotation
// 1.2 Stores info about ratation
// 1.3 Initiates brake
// 1.4 Initiates calibration
// 2. Current Position Registers: 30001 to (30000 + STEPPERS_COUNT*2)
// 3. Current Speed Registers: 40001 to (40000 + STEPPERS_COUNT*2)
// 4. Rotation Degree Registers: (40001 + STEPPERS_COUNT*2) to (40000 + STEPPERS_COUNT*4)
// 5. Acceleration Speed Registers: (40001 + STEPPERS_COUNT*4) to (40000 + STEPPERS_COUNT*6)

float radiansToDegrees(float radian)
{
    return (radian * (180 / M_PI));
};

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
    int writeBits(int address, uint8_t *values, int count);
};

ModbusClient::ModbusClient(const char *port)
{
    this->ctx = modbus_new_rtu(port, 115200, 'N', 8, 1);
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
    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing register" << std::endl;
    }

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

int ModbusClient::writeBits(int address, uint8_t *values, int count)
{
    int success = modbus_write_bits(this->ctx, address, count, values); // 1|-1
    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
    }

    return success;
};

// ------------ Stepper ------------

class Stepper
{
public:
    int stepper_id;
    int float_id; // refers to group of registers that stores a float value

    ModbusClient *client;
    Stepper(int id, ModbusClient *client);

    void setRotationDegree(float radian);
    void setMaxSpeed(float rad_per_sec);
    void setAcceleration(float rad_per_sec_sq);

    void rotate(float degree);
    void brake();
    void reset();

    float getCurrentPosition();
    float getCurrentSpeed();
    float getCurrentAcceleration();
    float getCurrentRotationDegree();
    int getStatus();
};

Stepper::Stepper(int id, ModbusClient *client)
{
    this->stepper_id = id;
    this->client = client;
    this->float_id = id * 2;
};

void Stepper::setRotationDegree(float radian)
{
    uint16_t radian_in_uint_format[2];
    float degree = radiansToDegrees(radian);

    modbus_set_float(degree, radian_in_uint_format);

    int success = this->client->writeRegisters(this->stepper_id * 2 + STEPPERS_COUNT * 2, radian_in_uint_format, 2); // add 20 because of registers scheme
    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing register" << std::endl;
    }
};

// TODO: make a logger
void Stepper::setMaxSpeed(float rad_per_sec)
{
    uint16_t speed_in_uint_format[2];
    float speed = radiansToDegrees(rad_per_sec);

    modbus_set_float(speed, speed_in_uint_format);
    int success = this->client->writeRegisters(this->stepper_id * 2, speed_in_uint_format, 2);

    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing register" << std::endl;
    }
};

void Stepper::setAcceleration(float rad_per_sec)
{

    uint16_t acceleration_in_uint_format[2];
    float acceleration = radiansToDegrees(rad_per_sec);

    modbus_set_float(acceleration, acceleration_in_uint_format);

    int success = this->client->writeRegisters(this->stepper_id * 2 + STEPPERS_COUNT * 4, acceleration_in_uint_format, 2);
    if (success == -1)
    {
        std::cout << "Error writing register" << std::endl;
    }
};

void Stepper::rotate(float radian)
{
    this->setRotationDegree(radian);

    int success = this->client->writeBit(this->stepper_id, 1);
    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing bit" << std::endl;
    }
};

void Stepper::brake()
{
    int success = this->client->writeBit(this->stepper_id + STEPPERS_COUNT, 1);
    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing bit" << std::endl;
    }
}

void Stepper::reset()
{
    // TODO: это костыль, чтобы останавливать двигатель, если в момент вызова калибровки он уже крутился к цели
    //  Убрать костыль, если получилось мигрировать на GStepper2
    this->brake();

    int success = this->client->writeBit(this->stepper_id + STEPPERS_COUNT * 2, 1);
    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing bit" << std::endl;
    }
};

// TODO: think if its correctly to use modbus directly from Stepper class
// TODO: sometimes it returns 0 when it shouldnt - figure out the reason
float Stepper::getCurrentPosition()
{
    uint16_t *position = this->client->readInputRegisters(this->stepper_id * 2, 2);

    return modbus_get_float(position);
};

float Stepper::getCurrentSpeed()
{
    uint16_t *speed = this->client->readRegisters(this->stepper_id * 2, 2);

    return modbus_get_float(speed);
};

float Stepper::getCurrentAcceleration()
{
    uint16_t *acceleration = this->client->readRegisters(this->stepper_id * 2 + STEPPERS_COUNT * 4, 2);

    return modbus_get_float(acceleration);
};

float Stepper::getCurrentRotationDegree()
{
    uint16_t *rotation_degree = this->client->readRegisters(this->stepper_id * 2 + STEPPERS_COUNT * 2, 2);

    return modbus_get_float(rotation_degree);
};

int Stepper::getStatus()
{
    uint8_t *rotation_status = this->client->readBits(this->stepper_id + STEPPERS_COUNT * 3);
    uint8_t *calibration_status = this->client->readBits(this->stepper_id + STEPPERS_COUNT * 2);

    int status;

    if (*calibration_status)
    {
        return CALIBRATION;
    }

    if (*rotation_status)
    {
        return ROTATION;
    }

    return READY;
};

// ------------ Stepper Group ------------

class SteppersGroup
{
public:
    Stepper *steppers[STEPPERS_COUNT];
    ModbusClient *client;

    SteppersGroup(ModbusClient *client, Stepper steppers[STEPPERS_COUNT]);

    // TODO: think about troubleshooting when param sets to a group
    void setMaxSpeedAll(float rad_per_sec);
    void setAccelerationAll(float rad_per_sec);

    void breakAll();
    void resetAll();

    float* getCurrentPositionAll();
    float* getCurrentSpeedAll();
    float* getCurrentAccelerationAll();
    float* getCurrentRotationDegreeAll();
};

SteppersGroup::SteppersGroup(ModbusClient *client, Stepper steppers[STEPPERS_COUNT])
{
    this->client = client;

    for (int i = 0; i < STEPPERS_COUNT; i++)
    {
        this->steppers[i] = &steppers[i];
    }
};

void SteppersGroup::setMaxSpeedAll(float rad_per_sec)
{
    uint16_t speed_in_uint_format[STEPPERS_COUNT * 2];
    float speed = radiansToDegrees(rad_per_sec);

    modbus_set_float(speed, speed_in_uint_format);

    for (int i = 2; i < STEPPERS_COUNT * 2; i += 2)
    {
        speed_in_uint_format[i] = speed_in_uint_format[0];
        speed_in_uint_format[i + 1] = speed_in_uint_format[1];
    }

    int success = this->client->writeRegisters(0, speed_in_uint_format, 2 * STEPPERS_COUNT);
};

void SteppersGroup::setAccelerationAll(float rad_per_sec)
{
    uint16_t acceleration_in_uint_format[STEPPERS_COUNT * 2];
    float acceleration = radiansToDegrees(rad_per_sec);

    modbus_set_float(acceleration, acceleration_in_uint_format);

    for (int i = 2; i < STEPPERS_COUNT * 2; i += 2)
    {
        acceleration_in_uint_format[i] = acceleration_in_uint_format[0];
        acceleration_in_uint_format[i + 1] = acceleration_in_uint_format[1];
    }

    int success = this->client->writeRegisters(STEPPERS_COUNT * 4, acceleration_in_uint_format, 2 * STEPPERS_COUNT);

    if (success == -1)
    {
        std::cout << "Error writing register" << std::endl;
    }
};

// WIP
void SteppersGroup::breakAll()
{
    uint8_t bits[STEPPERS_COUNT];
    std::fill(bits, bits + STEPPERS_COUNT, 1);

    int success = this->client->writeBits(STEPPERS_COUNT, bits, STEPPERS_COUNT);

    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing bit" << std::endl;
    }
};

void SteppersGroup::resetAll()
{
    // TODO: это костыль, чтобы останавливать двигатель, если в момент вызова калибровки он уже крутился к цели
    //  Убрать костыль, если получилось мигрировать на GStepper2
    this->breakAll();

    uint8_t bits[STEPPERS_COUNT];
    std::fill(bits, bits + STEPPERS_COUNT, 1);

    int success = this->client->writeBits(STEPPERS_COUNT * 2, bits, STEPPERS_COUNT);

    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing bit" << std::endl;
    }
};

float* SteppersGroup::getCurrentPositionAll()
{
    float* current_positions = new float[STEPPERS_COUNT];

    for (int i = 0; i < STEPPERS_COUNT; i++)
    {
        current_positions[i] = this->steppers[i]->getCurrentPosition();
    }

    return current_positions;
};

float* SteppersGroup::getCurrentSpeedAll()
{
    float* current_speeds = new float[STEPPERS_COUNT];

    for (int i = 0; i < STEPPERS_COUNT; i++)
    {
        current_speeds[i] = this->steppers[i]->getCurrentSpeed();
    }

    return current_speeds;
};

float* SteppersGroup::getCurrentAccelerationAll()
{
    float* current_accelerations = new float[STEPPERS_COUNT];

    for (int i = 0; i < STEPPERS_COUNT; i++)
    {
        current_accelerations[i] = this->steppers[i]->getCurrentAcceleration();
    }

    return current_accelerations;
};

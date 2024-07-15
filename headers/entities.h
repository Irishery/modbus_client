#include <modbus.h>
#include <iostream>
#include <math.h>

#define STEPPERS_COUNT 6

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

// ------------- Servo -------------
class Servo
{
public:
    // int servo_id;
    // int float_id;

    ModbusClient *client;
    Servo(ModbusClient *client);

    void setPosition(float radian);
    void setMaxSpeed(float rad_per_sec);
    void setAcceleration(float rad_per_sec_sq);

    void rotate(float radians);
    void brake();

    float getCurrentPosition();
    float getCurrentSpeed();
    float getCurrentAcceleration();
    float getLastTargetDegree();

    int getStatus();
};

Servo::Servo(ModbusClient *client)
{
    this->client = client;
};

void Servo::setMaxSpeed(float rad_per_sec)
{
    uint16_t speed_in_uint_format[2];
    float speed = radiansToDegrees(rad_per_sec);

    modbus_set_float(speed, speed_in_uint_format);

    int success = this->client->writeRegisters(STEPPERS_COUNT * 6, speed_in_uint_format, 2);
    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
    }
};

void Servo::setPosition(float radian)
{
    uint16_t radian_in_uint_format[2];
    float degree = radiansToDegrees(radian);

    modbus_set_float(degree, radian_in_uint_format);

    int success = this->client->writeRegisters(STEPPERS_COUNT * 6 + 2, radian_in_uint_format, 2);
    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
    }
};

void Servo::setAcceleration(float rad_per_sec_sq)
{
    uint16_t acceleration_in_uint_format[2];
    float acceleration = radiansToDegrees(rad_per_sec_sq);

    modbus_set_float(acceleration, acceleration_in_uint_format);

    int success = this->client->writeRegisters(STEPPERS_COUNT * 6 + 4, acceleration_in_uint_format, 2);
    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
    }
};

float Servo::getCurrentPosition()
{
    uint16_t *position = this->client->readInputRegisters(STEPPERS_COUNT * 2, 2);

    return modbus_get_float(position);
};

float Servo::getCurrentSpeed()
{
    uint16_t *speed = this->client->readRegisters(STEPPERS_COUNT * 6, 2);

    return modbus_get_float(speed);
};

float Servo::getLastTargetDegree()
{
    uint16_t *degree = this->client->readRegisters(STEPPERS_COUNT * 6 + 2, 2);

    return modbus_get_float(degree);
};

float Servo::getCurrentAcceleration()
{
    uint16_t *acceleration = this->client->readRegisters(STEPPERS_COUNT * 6 + 4, 2);

    return modbus_get_float(acceleration);
};

void Servo::rotate(float radians)
{
    this->setPosition(radians);

    int success = this->client->writeBit(STEPPERS_COUNT * 4, 1);
    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
    }
};

void Servo::brake()
{
    int success = this->client->writeBit(STEPPERS_COUNT * 4 + 1, 1);
    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
    }
};

int Servo::getStatus()
{
    uint8_t *status = this->client->readBits(STEPPERS_COUNT * 4 + 2);
    if (*status) {
        return ROTATION;
    }

    return READY;
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
    void setRotationDegreeNew(float radian1, float radian2);
    void setMaxSpeed(float rad_per_sec);
    void setAcceleration(float rad_per_sec_sq);

    void rotate(float degree);
    void rotateNew(float degree1, float degree2);
    void brake();
    void reset();

    float getCurrentPosition();
    float getCurrentSpeed();
    float getCurrentAcceleration();
    float getLastTargetDegree();
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

    int success = this->client->writeRegisters(this->stepper_id * 2 + STEPPERS_COUNT * 2, radian_in_uint_format, 2);
    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing register" << std::endl;
    }
};

void Stepper::setRotationDegreeNew(float radian1, float radian2)
{
    uint16_t radian_in_uint_format1[2];
    uint16_t radian_in_uint_format2[2];
    float degree1 = radiansToDegrees(radian1);
    float degree2 = radiansToDegrees(radian2);

    modbus_set_float(degree1, radian_in_uint_format1);
    modbus_set_float(degree2, radian_in_uint_format2);

    uint16_t * result = new uint16_t[4];
    std::copy(radian_in_uint_format1, radian_in_uint_format1 + 2, result);
    std::copy(radian_in_uint_format2, radian_in_uint_format2 + 2, result + 2);

    int success = this->client->writeRegisters(this->stepper_id * 2 + STEPPERS_COUNT * 2, result, 4);
    if (success == -1)
    {
        fprintf(stderr, "Error writing register: %s\n", modbus_strerror(errno));
        // std::cout << "Error writing register" << std::endl;
    }
};

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

void Stepper::rotateNew(float degree1, float degree2)
{
    this->setRotationDegreeNew(degree1, degree2);

    // int success = this->client->writeBit(this->stepper_id, 1);
    uint8_t bits[2];
    // std::fill(bits, bits + 2, 1);
    bits[0] = 1;
    bits[1] = 1;

    int success = this->client->writeBits(STEPPERS_COUNT - 2, bits, 2);
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
    //  Убрать костыль, если получилось мигрировать на GStepper3
    this->brake();

    int success = this->client->writeBit(this->stepper_id + STEPPERS_COUNT * 2, 1);
    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
    }
};

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

float Stepper::getLastTargetDegree()
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

    void setMaxSpeedAll(float rad_per_sec);
    void setAccelerationAll(float rad_per_sec);
    void setRotationDegreeAll(float s1, float s2, float s3, float s4, float s5, float s6);

    void rotateAll(float s1, float s2, float s3, float s4, float s5, float s6);
    void breakAll();
    void resetAll();

    float *getCurrentPositionAll();
    float *getCurrentSpeedAll();
    float *getCurrentAccelerationAll();
    float *getLastTargetDegreeAll();
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
    //  Убрать костыль, если получилось мигрировать на GStepper3
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

float *SteppersGroup::getLastTargetDegreeAll()
{
    float *degrees = new float[STEPPERS_COUNT];
    uint16_t *degrees_regs = this->client->readRegisters(STEPPERS_COUNT*2, STEPPERS_COUNT*2);

    for (int i = 0; i < STEPPERS_COUNT*2; i+=2)
    {
        uint16_t degree[2];
        degree[0] = degrees_regs[i];
        degree[1] = degrees_regs[i+1];

        degrees[i/2] = modbus_get_float(degree);
    }

    return degrees;
};

float *SteppersGroup::getCurrentSpeedAll()
{
    float *speeds = new float[STEPPERS_COUNT];
    uint16_t *speeds_regs = this->client->readRegisters(0, STEPPERS_COUNT*2);

    for (int i = 0; i < STEPPERS_COUNT*2; i+=2)
    {
        uint16_t speed[2];
        speed[0] = speeds_regs[i];
        speed[1] = speeds_regs[i+1];

        speeds[i/2] = modbus_get_float(speed);
    }

    return speeds;
};

float *SteppersGroup::getCurrentAccelerationAll()
{
    float *accelerations = new float[STEPPERS_COUNT];
    uint16_t *accelerations_regs = this->client->readRegisters(STEPPERS_COUNT*4, STEPPERS_COUNT*2);

    for (int i = 0; i < STEPPERS_COUNT*2; i+=2)
    {
        uint16_t acceleration[2];
        acceleration[0] = accelerations_regs[i];
        acceleration[1] = accelerations_regs[i+1];

        accelerations[i/2] = modbus_get_float(acceleration);
    }

    return accelerations;
};

float *SteppersGroup::getCurrentPositionAll()
{
    float *positions = new float[STEPPERS_COUNT];
    uint16_t *positions_regs = this->client->readInputRegisters(0, STEPPERS_COUNT*2);
    for (int i = 0; i < STEPPERS_COUNT*2; i+=2)
    {
        uint16_t position[2];
        position[0] = positions_regs[i];
        position[1] = positions_regs[i+1];

        positions[i/2] = modbus_get_float(position);
    }

    return positions;
};

void SteppersGroup::setRotationDegreeAll(float s1, float s2, float s3, float s4, float s5, float s6)
{
    uint16_t degrees_in_uint_format[STEPPERS_COUNT * 2];

    modbus_set_float(s1, degrees_in_uint_format);
    modbus_set_float(s2, degrees_in_uint_format + 2);
    modbus_set_float(s3, degrees_in_uint_format + 4);
    modbus_set_float(s4, degrees_in_uint_format + 6);
    modbus_set_float(s5, degrees_in_uint_format + 8);
    modbus_set_float(s6, degrees_in_uint_format + 10);

    int success = this->client->writeRegisters(STEPPERS_COUNT * 2, degrees_in_uint_format, 2 * STEPPERS_COUNT);

    if (success == -1)
    {
        std::cout << "Error writing register" << std::endl;
    }
};


void SteppersGroup::rotateAll(float s1, float s2, float s3, float s4, float s5, float s6)
{
    float s1_rad = radiansToDegrees(s1);
    float s2_rad = radiansToDegrees(s2);
    float s3_rad = radiansToDegrees(s3);
    float s4_rad = radiansToDegrees(s4);
    float s5_rad = radiansToDegrees(s5);
    float s6_rad = radiansToDegrees(s6);


    this->setRotationDegreeAll(s1_rad, s2_rad, s3_rad, s4_rad, s5_rad, s6_rad);

    uint8_t bits[STEPPERS_COUNT];
    std::fill(bits, bits + STEPPERS_COUNT, 1);

    int success = this->client->writeBits(0, bits, STEPPERS_COUNT);

    if (success == -1)
    {
        fprintf(stderr, "Error writing bit: %s\n", modbus_strerror(errno));
    }
};

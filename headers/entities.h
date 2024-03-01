#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>
#include <array>

#define PI 3.14159265359

// ------------ Modbus Client ------------
class ModbusClient {
    private:
        modbus_t *ctx;

    public:
        ModbusClient(const char *port);

        void setSlave(int slave);

        uint16_t* readRegisters(int address, int count);

        int writeRegister(int address, int value);

        uint8_t* readBits(int address);

        int writeBit(int address, int status);
};

ModbusClient::ModbusClient(const char *port) {
    this -> ctx = modbus_new_rtu(port, 9600, 'N', 8, 1);
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

    this -> setSlave(1);
};

void ModbusClient::setSlave(int slave) {
    modbus_set_slave(ctx, slave);
};

uint16_t* ModbusClient::readRegisters(int address, int count) {
    uint16_t *tab_reg = (uint16_t *) malloc(sizeof(uint16_t) * count);
    modbus_read_registers(this -> ctx, address, count, tab_reg);

    return tab_reg;
};

int ModbusClient::writeRegister(int address, int value) {
    int success = modbus_write_register(this -> ctx, address, value); // 1|-1
    return success;
};

uint8_t* ModbusClient::readBits(int address) {
    uint8_t *tab_bits = (uint8_t *) malloc(sizeof(uint16_t));
    modbus_read_bits(this -> ctx, address, 1, tab_bits);
    return tab_bits;
};

int ModbusClient::writeBit(int address, int status) {
    int success = modbus_write_bit(this -> ctx, address, status); // 1|-1
    return success;
};

// ------------ Stepper ------------

class Stepper {
    private:
        void setRotationDegree(double radian);

        double radiansToDegrees(double radian);

    public:
        int stepper_id;

        ModbusClient *client;
        Stepper(int id, ModbusClient *client);

        void rotate(double degree);
};

Stepper::Stepper(int id, ModbusClient *client) {
    this -> stepper_id = id;
    this -> client = client;
};

double Stepper::radiansToDegrees(double radian) {
    return (radian * (180 / PI));
};


void Stepper::setRotationDegree(double radian) {

    int succes = this -> client -> writeRegister(this -> stepper_id, this -> radiansToDegrees(radian));
    if (succes == -1) {
        std::cout << "Error writing register" << std::endl;
    }
};

void Stepper::rotate(double radian) {
    this -> setRotationDegree(radian);
    int succes = this -> client -> writeBit(this -> stepper_id, 1);
    if (succes == -1) {
        std::cout << "Error writing bit" << std::endl;
    }
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

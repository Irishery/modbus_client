#include <cerrno>
#include <stdio.h>
#include <modbus.h>
#include <iostream>

int main(void)
{
    const int REMOTE_ID = 1;
    modbus_t *ctx;
    uint16_t tab_reg[64];

    ctx = modbus_new_rtu("/dev/ttyUSB0", 9800, 'N', 8, 1);
    if (ctx == NULL)
    {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return -1;
    }

    if (modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    modbus_set_slave(ctx, REMOTE_ID);
    int success;
    success = modbus_write_register(ctx, 10, 666);
    std::cout << "Write register 10: " << success << std::endl;

    modbus_read_registers(ctx, 0, 25, tab_reg);

    std::cout << "Read 2 registers from address 11" << std::endl;
    std::cout << "Register 0: " << tab_reg[0] << std::endl;
    std::cout << "Register 1: " << tab_reg[1] << std::endl;
    std::cout << "Register 2: " << tab_reg[2] << std::endl;
    std::cout << "Register 3: " << tab_reg[3] << std::endl;
    std::cout << "Register 9: " << tab_reg[11] << std::endl;
    std::cout << "Register 11: " << tab_reg[10] << std::endl;
    std::cout << "Register 10: " << tab_reg[9] << std::endl;

    // int cmnd = 0;
    // std::cin >> cmnd;
    // if (cmnd) {
    //     int a = modbus_write_register(ctx, 10, 666);
    //     std::cout << a << std::endl;
    // }
}

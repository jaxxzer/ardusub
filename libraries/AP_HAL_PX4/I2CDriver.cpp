#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "I2CDriver.h"
#include "I2CWrapper.h"

extern const AP_HAL::HAL& hal;

using namespace PX4;

// constructor for main i2c class
PX4I2CDriver::PX4I2CDriver(void)
{
    px4_i2c = new PX4_I2C(PX4_I2C_BUS_EXPANSION);
    if (px4_i2c == nullptr) {
        AP_HAL::panic("Unable to allocate PX4 I2C driver");
    }
}

/*
  expose transfer function
 */
bool PX4I2CDriver::do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
    return px4_i2c->do_transfer(address, send, send_len, recv, recv_len);
}

/* write: for i2c devices which do not obey register conventions */
uint8_t PX4I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
    return do_transfer(addr, data, len, nullptr, 0) ? 0:1;
}

/* writeRegister: write a single 8-bit value to a register */
uint8_t PX4I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t d[2] = { reg, val };
    return do_transfer(addr, d, sizeof(d), nullptr, 0) ? 0:1;
}

/* writeRegisters: write bytes to contiguous registers */
uint8_t PX4I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                                     uint8_t len, uint8_t* data)
{
    return write(addr, 1, &reg) == 0 && write(addr, len, data) == 0;
}

/* read: for i2c devices which do not obey register conventions */
uint8_t PX4I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    return do_transfer(addr, nullptr, 0, data, len) ? 0:1;
}
    
/* readRegister: read from a device register - writes the register,
 * then reads back an 8-bit value. */
uint8_t PX4I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    return do_transfer(addr, &reg, 1, data, 1) ? 0:1;
}

/* readRegister: read contiguous device registers - writes the first 
 * register, then reads back multiple bytes */
uint8_t PX4I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                    uint8_t len, uint8_t* data)
{
    return do_transfer(addr, &reg, 1, data, len) ? 0:1;
}

#endif // CONFIG_HAL_BOARD

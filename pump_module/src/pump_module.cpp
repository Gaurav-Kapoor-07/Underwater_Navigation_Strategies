#include <pump_module/pump_module.h>

namespace pump_module {

bool pump_module::openDev(char const* bus, u16 address)
{
    ec = i2c.open(bus, address);
    return !ec;
}

void pump_module::writeRegister(u8 reg, u8 value)
{
    i2c.writeRegister(reg, value);
}

} // namespace pump_module

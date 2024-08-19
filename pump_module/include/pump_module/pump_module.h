#pragma once

#include <pump_module/i2c_device.h>
#include <monsun_util/types.h>

namespace pump_module {

class pump_module {
public:
    bool openDev(char const* bus, u16 address);
    void writeRegister(u8 reg, u8 value);
    monsun::ErrorCode errorCode() const { return ec; }

private:
    i2c::I2cDevice i2c;
    monsun::ErrorCode ec = monsun::ErrorCode(0);
};

} // namespace pump_module

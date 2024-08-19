#pragma once

#include <linux/i2c.h> // struct i2c_msg
#include <linux/i2c-dev.h> // I2C_RDWR_IOCTL_MAX_MSGS

#include <monsun_util/error_code.h>
#include <monsun_util/types.h>

namespace i2c {

enum I2cDirection { I2cWrite, I2cRead };
enum I2cBlockSize { SMBusMax = 32, I2cMax = 8192 };

class I2cDevice {
public:
    ~I2cDevice();
    monsun::ErrorCode open(char const* dev_name, u16 address);
    void close();
    void setBlockSize(i32 block_size);
    void setBlockSize(I2cBlockSize b) { setBlockSize(static_cast<i32>(b)); }
    explicit operator bool() const;

    monsun::ErrorCode finish();
    operator monsun::ErrorCode();
    I2cDevice& xfer();

    I2cDevice& write(u8 const* buf, u32 nbuf);
    I2cDevice& write(u8* value) { return write(value, 1); }
    I2cDevice& write(i8* value) { return write(reinterpret_cast<u8*>(value), 1); }
    I2cDevice& read(u8* buf, u32 nbuf);
    I2cDevice& read(u8* value) { return read(value, 1); }
    I2cDevice& read(i8* value) { return read(reinterpret_cast<u8*>(value), 1); }

    I2cDevice& writeChar(u8 value);
    I2cDevice& writeRegister(u8 reg, u8 value);

private:
    u32 addBuffer(u8* buf, u32 nbuf, I2cDirection dir);
    u32 addI2cMsg(u8* buf, u32 nbuf, I2cDirection dir);
    void xferQueue();
    bool queueFull() const { return nmsgs == I2C_RDWR_IOCTL_MAX_MSGS; }
    bool queueEmpty() const { return nmsgs == 0; }

    i32 file = -1;
    u32 nmsgs = 0;
    monsun::ErrorCode ec = monsun::ErrorCode(0);
    struct i2c_msg msgs[I2C_RDWR_IOCTL_MAX_MSGS];
    u16 block_max = 8192; // set maximum as default block size limit to avoid message fragmentation
};

} //  namespace i2c

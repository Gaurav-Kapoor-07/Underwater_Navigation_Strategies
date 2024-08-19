#include <pump_module/i2c_device.h>

// i2c
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

// open
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// read, write, close
#include <unistd.h>

// errno
#include <cerrno>
#include <cstring>

namespace i2c {

/// Implicitly close the file handle if is is still open on destruction
I2cDevice::~I2cDevice()
{
    close();
}

/// Open an I2C device. This will fail if the bus cannot be accessed. It does not probe the device.
monsun::ErrorCode I2cDevice::open(char const* dev_name, u16 address)
{
    close(); // ensure we are in a sane state
    if ((address < 0x08) || (address > 0x77)) {
        return monsun::ErrorCode(EINVAL);
    }
    file = ::open(dev_name, O_RDWR);
    if (file < 0) {
        return monsun::ErrorCode(errno);
    }
    // make sure the driver supports plain I2C I/O
    unsigned long i2c_funcs = 0;
    if (ioctl(file, I2C_FUNCS, &i2c_funcs) < 0) {
        return monsun::ErrorCode(errno);
    }
    if (!(i2c_funcs & I2C_FUNC_I2C)) {
        // I2C_RDWR is not supported
        return monsun::ErrorCode(ENOTSUP);
    }
    // prepare send/recv buffer
    for (u32 i = 0; i != I2C_RDWR_IOCTL_MAX_MSGS; ++i) {
        // we never reset the array but only update data pointers and flags
        msgs[i].addr = address;
    }
    return monsun::ErrorCode(0);
}

/// Close the I2C device file handle and return the handle to the operating system
void I2cDevice::close()
{
    if (*this) {
        ::close(file);
        file = -1;
    }
}

/// Set a block size limit for every read/write operation. Valid values are in the range
/// of 1...8192. The linux kernel sets a message size limit of 8192 bytes (see i2c-dev.c). SMBus
/// uses a maximal size of 32 bytes. A negative value or zero has no effect. A too large value is
/// saturated.
void I2cDevice::setBlockSize(i32 block_size)
{
    if (block_size < 1) {
        // does not make sense to limit transfers to 0 byte or -N bytes
        return;
    }
    if (block_size > 8192) {
        // limited by the linux kernel
        block_size = 8192;
    }
    block_max = block_size;
}

/// Return true if a device is opened
I2cDevice::operator bool() const
{
    return file >= 0;
}

/// Finish any outstanding transfers and return the result of the chained operations. If any of the
/// operations failed, it will return an error. In case all chained operations were successfull,
/// success is returned. An internal error flag prevents further read/write operations after the
/// first error ocurred. This shall prevent that some bytes are read immediately after a register
/// could not be written. finish() resets this internal error flag.
monsun::ErrorCode I2cDevice::finish()
{
    xfer();
    monsun::ErrorCode result = ec;
    ec.clear();
    return result;
}

/// Implicit conversion to ErrorCode to avoid the need to end every operation with .finish()
I2cDevice::operator monsun::ErrorCode()
{
    return finish();
}

/// If no error ocurred and there is outstanding data in the queue waiting to be sent/read then
/// start the transfer
I2cDevice& I2cDevice::xfer()
{
    if (!ec && !queueEmpty()) {
        xferQueue(); // finish outstanding i2c transfers if there was no error until now
    }
    return *this;
}

/// Add data to be sent to the queue. A subsequent call of finish() will send the data and return an
/// ErrorCode indicating if the transfer was successful. Calling xfer() will trigger the immediate
/// transfer as well but will not return an ErrorCode to enable to chain further read/write calls.
/// In case an error ocurred, no further operations will be executed until finish is called.
I2cDevice& I2cDevice::write(u8 const* buf, u32 nbuf)
{
    // Use const_cast because the ioctl api is const incorrect: The buffer referenced by
    // u8 const* buf is only read from when data is written to the I2C device.
    u32 nbytes = 0;
    while (!ec) {
        nbytes += addBuffer(const_cast<u8*>(buf) + nbytes, nbuf - nbytes, I2cDirection::I2cWrite);
        if (nbytes == nbuf) {
            // all data added
            break;
        }
        // incomplete buffer added to queue, need to empty queue and continue
        xferQueue();
    }
    return *this;
}

/// Add data to be read to the queue. A subsequent call of finish() will read the data and return an
/// ErrorCode indicating if the transfer was successful. Calling xfer() will trigger the immediate
/// transfer as well but will not return an ErrorCode to enable to chain further read/write calls.
/// In case an error ocurred, no further operations will be executed until finish is called.
I2cDevice& I2cDevice::read(u8* buf, u32 nbuf)
{
    u32 nbytes = 0;
    while (!ec) {
        nbytes += addBuffer(const_cast<u8*>(buf) + nbytes, nbuf - nbytes, I2cDirection::I2cRead);
        if (nbytes == nbuf) {
            // all data added
            break;
        }
        // incomplete buffer added to queue, need to empty queue and continue
        xferQueue();
    }
    return *this;
}

/// writeChar(u8 value) is a convenience function to simplify sending a single char: there is no
/// need have the data on the stack and use a pointer. This is useful to send values that have no
/// address (e.g.: const enum value). As a side effect, the queue is always flushed when using this
/// function.
I2cDevice& I2cDevice::writeChar(u8 value)
{
    write(&value, 1); // add to queue
    return xfer(); // empty queue as value is a temporary
}

/// writeRegister is a convenience function to simplify one of the most often used operations
I2cDevice& I2cDevice::writeRegister(u8 reg, u8 value)
{
    u8 tmp[2] = {reg, value};
    write(tmp, 2); // add to queue
    return xfer(); // empty queue as the buffer is temporary
}

/// addBuffer creates the necessary number of I2C messages to transfer all of the buffer data. If
/// the buffer is too large for a single system call, it will be transferred in several
/// system calls. Each syscall may consist of up to 42 messages with a maximum of 8192 bytes each.
u32 I2cDevice::addBuffer(u8* buf, u32 nbuf, I2cDirection dir)
{
    u32 nbytes = 0;
    while (nbytes != nbuf) {
        u32 res = addI2cMsg(buf + nbytes, nbuf - nbytes, dir);
        // check if queue is full
        if (res == 0) {
            // we can't add any more data
            break;
        }
        nbytes += res;
    }
    return nbytes;
}

/// Add partial or complete buffer to the message queue. If the maximal queue length has been
/// reached and no data can be enqueued, return 0. Return he number of bytes added to the queue
/// otherwise.
u32 I2cDevice::addI2cMsg(u8* buf, u32 nbuf, I2cDirection dir)
{
    if (queueFull()) {
        return 0;
    }
    u16 flags = (dir == I2cWrite) ? 0 : I2C_M_RD;
    u16 len = (nbuf > block_max) ? block_max : static_cast<u16>(nbuf);
    // no need to write .addr, address is already set
    msgs[nmsgs].flags = flags;
    msgs[nmsgs].len = len;
    msgs[nmsgs].buf = buf;
    nmsgs += 1;
    return len;
}

void I2cDevice::xferQueue()
{
    struct i2c_rdwr_ioctl_data data;
    data.msgs = msgs;
    data.nmsgs = nmsgs;
    if (ioctl(file, I2C_RDWR, &data) < 0) {
        ec = monsun::ErrorCode(errno);
    }
    nmsgs = 0; // pipe is now empty again
}

} //  namespace i2c

#pragma once

#include <cstdint>

#include <monsun_util/serde_big_base.h>

#include <ros/serialization.h>

namespace monsun_comm {

class ReceiveBuffer {
public:
    ReceiveBuffer(void const* buffer, size_t length)
        : buf(static_cast<uint8_t const*>(buffer)), sz(length)
    {
    }

    /// read() member function is used to read big endian encoded data from a buffer. It can be
    /// extended for custom data types by overloading.
    template <typename T>
    bool read(T* value)
    {
        size_t nbytes = monsun::read_be(buf + idx, remaining(), value);
        if(nbytes) {
            idx += nbytes;
            return true;
        }
        return false;
    }

    /// read_ros() member function is used to read ros messages in the ros specific format from a
    /// buffer using the ros::serialization functions using the code automatically generated for
    /// every ros message.
    template <typename T>
    bool read_ros(T& value)
    {
        // as the ros library is not const-correct, i need to const_cast. there is no mutating
        // access to the underlying buffer, hence this will not lead to undefined behaviour.
        ros::serialization::IStream istr(const_cast<uint8_t*>(buf + idx), remaining());
        try {
            ros::serialization::deserialize(istr, value);
        }
        catch(ros::serialization::StreamOverrunException const&) {
            // "Buffer Overrun"
            return false;
        }
        idx += remaining() - istr.getLength();
        return true;
    }

    /// Reset buffer read pointer to start of buffer
    void reset() { idx = 0; }

    /// Pointer to data. Do not read more than index() bytes!
    void const* data() const { return buf; }

    /// Number of read bytes
    size_t index() const { return idx; }

    /// Number of unread bytes
    size_t remaining() const { return capacity() - index(); }

    /// Number of bytes that are written into the buffer
    size_t capacity() const { return sz; }

private:
    uint8_t const* buf;
    size_t const sz;
    size_t idx = 0;
};

} // namespace monsun_comm

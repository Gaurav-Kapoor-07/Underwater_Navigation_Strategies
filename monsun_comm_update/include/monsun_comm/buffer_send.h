#pragma once

#include <cstdint>
#include <vector>

#include <monsun_util/serde_big_base.h>

#include <ros/serialization.h>

namespace monsun_comm {

class SendBuffer {
public:
    /// write() member function is used to write big endian encoded data into a buffer. It can be
    /// extended for custom data types by overloading.
    template <typename T>
    void write(T const& value)
    {
        uint8_t tmpbuf[sizeof(value)];
        monsun::write_be(tmpbuf, sizeof(value), value);
        write_block(tmpbuf, sizeof(value));
    }

    /// write_ros() member function is used to write ros messages in the ros specific format into
    /// the buffer using the ros::serialization functions using the code automatically generated for
    /// every ros message.
    template <typename T>
    void write_ros(T const& value)
    {
        // get pointer to current end of data in the vector
        size_t idx = buf.size();
        // calculate size of additional space needed
        uint32_t buf_len = ros::serialization::serializationLength(value);
        // preallocate enough space at the end of the vector (append zeros)
        buf.resize(idx + buf_len);
        // write directly into the underlying buffer
        ros::serialization::OStream ostr(buf.data() + idx, buf_len);
        // The only possible error "Buffer Overrun" cannot happen as we preallocated enough space
        ros::serialization::serialize(ostr, value);
    }

    /// write_block() member function is used to write <length> bytes of
    /// subsequent data, e.g. a packed struct, into the buffer.
    void write_block(uint8_t const* data, size_t length)
    {
        buf.insert(buf.end(), data, data + length);
    }

    /// Reset buffer and throw away previously written data
    void clear() { buf.clear(); }

    /// Pointer to data. Do not read more than len() bytes!
    uint8_t const* data() const { return buf.data(); }

    /// Pointer to mutable data. Do not read more than len() bytes!
    uint8_t* data() { return buf.data(); }

    /// Number of written bytes
    size_t len() const { return buf.size(); }

private:
    std::vector<uint8_t> buf;
};

} // namespace monsun_comm

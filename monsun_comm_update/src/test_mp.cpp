#include <cstdio>
#include <cstdlib>

#include <zmq.hpp>

#include <monsun_comm/portdef.h>

using namespace monsun_comm;

static void print(void const* buffer, unsigned length)
{
    auto buf = reinterpret_cast<char const*>(buffer);
    for(unsigned i = 0; i != length; ++i) {
        if(buf[i] < 32)
            printf(".");
        else if(buf[i] > 126)
            printf("'");
        else
            printf("%c", buf[i]);
    }
    printf("\n");
}

int main(int argc, char* argv[])
{
    if(argc > 2) {
        printf("Usage: %s [ip[:port]]\n", argv[0]);
        std::exit(1);
    }

    std::string endpoint("tcp://");
    if(argc == 2) {
        std::string s(argv[1]);
        if(s.find(':') == s.npos)
            s.append(":").append(std::to_string(CommPort::PortMultiplexer));
        endpoint.append(s);
    }
    else {
        endpoint.append("127.0.0.1:").append(std::to_string(CommPort::PortMultiplexer));
    }

    zmq::context_t context(1);

    zmq::socket_t sub(context, ZMQ_SUB);
    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    try {
        sub.connect(endpoint.c_str());
    }
    catch(zmq::error_t& err) {
        printf("[ERROR] Could not connect to %s: %s\n", endpoint.c_str(), err.what());
        std::exit(1);
    }
    printf("[INFO] connect to %s\n", endpoint.c_str());

    for(;;) {
        zmq::message_t msg;
        if(sub.recv(&msg)) {
            printf("[INFO] received %3zu bytes: ", msg.size());
            print(msg.data(), msg.size());
        }
    }
    return 0;
}

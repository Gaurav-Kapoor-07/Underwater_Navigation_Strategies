#include <cstdio>
#include <cstdlib>

#include <zmq.hpp>

#include <monsun_comm/portdef.h>

using namespace std;
using namespace monsun_comm;

int const nips = 7;
char const* default_ips[nips] = {"",
                                 "127.0.0.1",
                                 "192.168.0.141",
                                 "192.168.0.142",
                                 "192.168.0.143",
                                 "192.168.0.144",
                                 "192.168.0.145"};

int main(int argc, char const* argv[])
{
    if(argc == 1) {
        argc = nips;
        argv = default_ips;
    }

    zmq::context_t context(1);
    zmq::socket_t pub(context, ZMQ_PUB);

    std::string bindaddr =
        std::string("tcp://0.0.0.0:").append(std::to_string(CommPort::PortMultiplexer));
    pub.bind(bindaddr);

    zmq::socket_t sub(context, ZMQ_SUB);
    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    int count = 0;
    for(int i = 1; i != argc; ++i) {
        // add default port if no port is given
        string address(argv[i]);
        if(address.find(':') == address.npos)
            address.append(":").append(std::to_string(CommPort::PortPublisher));
        string endpoint("tcp://");
        endpoint.append(address);
        try {
            sub.connect(endpoint);
            count += 1;
            printf("[INFO] %i: %s\n", count, endpoint.c_str());
        }
        catch(zmq::error_t& err) {
            printf("[WARN] Could not connect to %s: %s\n", endpoint.c_str(), err.what());
        }
        catch(exception& ex) {
            printf("[ERROR] Could not connect to %s: %s\n", endpoint.c_str(), ex.what());
            exit(1);
        }
    }

    if(count == 0) {
        printf("[ERROR] Could not connect to any end point\n");
        exit(1);
    }

    printf("[INFO] republishing on: %s\n", bindaddr.c_str());

    zmq::proxy(sub.operator void*(), pub.operator void*(), nullptr);
}

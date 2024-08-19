#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <vegetation_boundary_message/VegetationBoundary.h>
#include <cmath>

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <unistd.h>
#include <string> 

using namespace std;

class SocketServer 
{
    private:
        ros::NodeHandle n;

        int server_fd;
        int new_socket;
        struct sockaddr_in address;
        int addrlen = sizeof(address);
        int buffer_size = 1024;
        char buffer[1024] = {0};

        int port_number = 10000;
        int valread = 0;

        // double vb_msg_publish_freq{1.0};

        ros::Publisher vegetation_boundary_detection;
        
    public:
        SocketServer() 
        { 
            n.getParam("vegetation_boundary_detection/port_number", port_number);
            n.getParam("vegetation_boundary_detection/buffer_size", buffer_size);
            // n.getParam("vegetation_boundary_detection/vb_msg_publish_freq", vb_msg_publish_freq);

            vegetation_boundary_detection = n.advertise<vegetation_boundary_message::VegetationBoundary>("/vegetation_boundary", 1000);
            
            // Create a socket
            if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
                ROS_ERROR("vegetation_boundary_detection: Socket creation error");
                return;
            }

            bzero((char *) &address, addrlen);

            // Set up server address
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY; // Use any available IP address of the machine
            address.sin_port = htons(port_number); // Choose a port number

            // Bind the socket to the chosen port
            if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
                ROS_ERROR("vegetation_boundary_detection: Bind failed");
                return;
            }

            // Listen for incoming connections
            if (listen(server_fd, 5) < 0) {
                ROS_ERROR("vegetation_boundary_detection: Listen failed");
                return;
            }

            ROS_INFO("vegetation_boundary_detection: Server listening...");

            // Accept incoming connection
            if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
                ROS_ERROR("vegetation_boundary_detection: Accept failed");
                return;
            }

            receiveData();
        }

        void receiveData()
        {
            // Receive data from the client

            bzero(buffer, 1024);

            double time_now = 0.0;

            while (true)
            {
                valread = read(new_socket, buffer, buffer_size);

                if (valread < 0)
                {
                    ROS_ERROR("vegetation_boundary_detection: Accept failed");
                }

                else
                {
                    ROS_INFO("vegetation_boundary_detection: Received: %s", buffer);
                    // ROS_INFO("vegetation_boundary_detection: Received: %c", buffer[0]);
                    // ROS_INFO("vegetation_boundary_detection: Received: %c", buffer[1]);
                    // ROS_INFO("vegetation_boundary_detection: Received: %c", buffer[2]);
                    // ROS_INFO("vegetation_boundary_detection: Received: %c", buffer[3]);
                    // ROS_INFO("vegetation_boundary_detection: Received: %c", buffer[4]);
                    // ROS_INFO("vegetation_boundary_detection: Received: %c", buffer[5]);
                    // ROS_INFO("vegetation_boundary_detection: Received: %c", buffer[6]);
                    // std::cout << "Received: " << buffer << std::endl;

                    string buffer_str(buffer);

                    std::size_t found_opening = buffer_str.find("[");
                    std::size_t found_blank = buffer_str.find(" ");
                    std::size_t found_closing = buffer_str.find("]");

                    // ROS_INFO("vegetation_boundary_detection: Received: %i", found_closing);

                    vegetation_boundary_message::VegetationBoundary vb;
                    vb.header.stamp = ros::Time::now();
                    vb.header.frame_id = "vegetation_boundary";
                    vb.vegetation_boundary_detected = std::stoi(buffer_str.substr(found_opening + 1, 1));
                    vb.confidence_percent = std::stod(buffer_str.substr(found_blank + 1, found_closing - found_blank - 1));
                    vegetation_boundary_detection.publish(vb);

                    time_now = vb.header.stamp.sec + (vb.header.stamp.nsec * 1e-9);
                    // ROS_INFO("time now: %f", time_now);
                    string time_now_string = to_string(time_now);

                    send(new_socket, time_now_string.c_str(), time_now_string.length(), 0);
                }

                // ros::Duration(1.0 / vb_msg_publish_freq).sleep();
            }
        }

        ~SocketServer()
        {
            close(new_socket);
            close(server_fd);
        }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "vegetation_boundary_detection");
    SocketServer socketserver;
    ROS_INFO("vegetation_boundary_detection: started");
    ros::spin();
    return 0;
}
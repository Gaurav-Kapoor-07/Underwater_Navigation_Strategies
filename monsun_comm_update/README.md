### monsun_comm

This package aims to provide a complete solution to the inter-robot-communication of MONSUN AUVs 
that are surfaced and have access to the WLAN. It follows the publish-subscribe-pattern of ROS 
closely. monsun_comm reuses the automatically generated code of ROS messages, hence all data that 
is present in a ROS message (like e.g. time stamp in the header) is preserved when forwarded with 
monsun_comm. 

#### Dependencies

This package depends on the ZeroMQ messaging library in version 4. If using ubuntu, install ZMQ 
with ```apt install libzmq3-dev```. 

#### Usage

Run ```publisher``` to forward ROS mesages via ZMQ. You do not need to specify ROS 
parameters. 

For every remote AUV, you need to run an instance of ```subscriber```. Every instance 
needs to know the IP and the ID of the respective AUV. Republishing of remote ROS messages takes 
place in the ```/mnX```-namespace, with ```X``` being the given MONSUN ID of the endpoint. 

There is an additional helper tool to multiplex several monsun_comm messages into one single port.
Start multi_sub and 

#### Development

To include a new message in the forwarding loop, it has to be given a unique ID and 
a ROS subscriber needs to be added in publisher as well as a ROS publisher in subscriber.

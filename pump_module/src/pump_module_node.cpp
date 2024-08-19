#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/SetBool.h>
#include <pump_module/pump_module.h>

class PumpModuleNode
{
private:
    pump_module::pump_module pump_module;
    
    ros::NodeHandle n;

    ros::ServiceServer enable_pump_module;
    bool pump_module_enabled = false;

    u8 pump_module_i2c_register = 0x0;

public:
    PumpModuleNode() : n("~")
    {   
        std::string bus_name;
        if (!n.getParam("i2c_bus", bus_name)) {
            ROS_FATAL("pump_module: no bus name");
            n.shutdown();
            return;
        }

        i32 slave_address;
        if (!n.getParam("address", slave_address)) {
            ROS_FATAL("pump_module: no slave address");
            n.shutdown();
            return;
        }
    
        auto addr = static_cast<u16>(slave_address);
        
        if (!pump_module.openDev(bus_name.c_str(), addr)) {
        ROS_FATAL("pump_module: could not open slave %#04x on bus %s: %s", addr, bus_name.c_str(),
            pump_module.errorCode().message());
        n.shutdown();
        return;
        }
        
        enable_pump_module = n.advertiseService("pump_module/enable", &PumpModuleNode::enable_pump_module_callback, this);
    }
    
    bool enable_pump_module_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
        // pump_module enable
        if (req.data) 
        {
            if (pump_module_enabled) 
            {
                ROS_ERROR("pump_module: already on");
                res.success = false;
                return true;
            }

            pump_module.writeRegister(pump_module_i2c_register, 1);
            
            pump_module_enabled = true;
            ROS_INFO("pump_module: on");
            res.success = true;
            return true;
        }

        // pump_module disable
        if (!pump_module_enabled) 
        {
            ROS_WARN("pump_module: already off");
            res.success = false;
            return true;
        }

        pump_module.writeRegister(pump_module_i2c_register, 0);

        pump_module_enabled = false;
        ROS_INFO("pump_module: off");
        res.success = true;

        return true;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pump_module");
    PumpModuleNode PumpModuleNode;
    ROS_INFO("pump_module: started");
    ros::spin();
    return 0;
}
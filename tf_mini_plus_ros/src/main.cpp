#include <tf_mini_plus_ros/range_sensor.h>
#include <vector> 


int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_sensor_node");
    ros::Time::init();
    ros::NodeHandle n("~");
    ros::Rate r(30.0);
    
    //Read parameters
    int base_address = TFMP_DEFAULT_ADDRESS, bus = TFMP_DEFAULT_BUS, num_devices = 1;
    n.getParam("tfmp_base_address", base_address);
    n.getParam("tfmp_bus", bus);
    n.getParam("tfmp_num_devices", num_devices);

    RangeSensor *range_sensors[num_devices];
    for(uint8 i = 0; i < num_devices; ++i){
        std::string topicId = "sensor" + std::to_string(i+1);
        ROS_INFO("%s", topicId.c_str());
        range_sensors[i] = new RangeSensor(&n, bus, base_address + i, topicId);
        if(range_sensors[i]->isOpenned())
            ROS_INFO("Range sensor ready on i2c address: 0x%x and bus: %i", base_address + i, bus);
    }

    while (n.ok())
    {
        for(uint8 i = 0; i < num_devices; ++i){
            if(range_sensors[i]->isOpenned())
                range_sensors[i]->PubData();
        }
        ros::spinOnce();
        r.sleep();
    }
}
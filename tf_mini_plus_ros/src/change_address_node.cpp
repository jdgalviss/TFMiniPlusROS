#include <tf_mini_plus_ros/range_sensor.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "change_address_node");
    ros::Time::init();
    ros::NodeHandle n("~");
    int address1, bus, address2;
    n.getParam("address1", address1);
    n.getParam("address2", address2);
    n.getParam("bus", bus);

    RangeSensor *range_sensor = new RangeSensor(&n, (uint8)bus, (uint8)address1); 
    ROS_INFO("changing device from address: 0x%x to address: 0x%x in bus: %i", address1, address2, bus);
    range_sensor->ChangeAddress(address2);
}
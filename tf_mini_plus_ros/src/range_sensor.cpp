#include <tf_mini_plus_ros/range_sensor.h>

RangeSensor::RangeSensor(ros::NodeHandle *nh, uint8 bus, uint8 address, std::string topic_id)
{
    n_ = nh;
    std::string topic_name = "/range_sensor/" + topic_id;
    range_sensor_pub_ = nh->advertise<sensor_msgs::Range>(topic_name, 10);
    range_msg_.header.frame_id = "range_sensor";
    range_msg_.header.frame_id = "range_sensor";
    range_msg_.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg_.field_of_view = 0.0; //Need to double check this value
    range_msg_.min_range = 0.2;
    range_msg_.max_range = 1.5;
    try
    {
        tf_mini_plus_ = new TFMiniPro(address, bus);
        device_oppened_ = true;
    }
    catch (const std::system_error &error)
    {
        ROS_WARN("ERROR opening sensor on bus %i and address 0x%x ->%s", bus, address, error.what());
        device_oppened_ = false;
    }
}

void RangeSensor::PubData()
{
    // read_channel computes the raw value * 0.000125
    range_ = tf_mini_plus_->ReadData();

    // Distance calculation in cm
    range_msg_.range = range_;
    range_msg_.header.stamp = ros::Time::now();

    // Publishing the distance
    range_sensor_pub_.publish(range_msg_);
}

void RangeSensor::ChangeAddress(uint8 new_address)
{
    if(tf_mini_plus_->ChangeAddress(new_address)) 
        ROS_INFO("Address changed succesfuly to 0x%x", new_address);
    else
        ROS_WARN("Address could not be changed");
    
}

bool RangeSensor::isOpenned(){
    return device_oppened_;
}



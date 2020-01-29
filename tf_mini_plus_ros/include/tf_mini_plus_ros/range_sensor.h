#ifndef RANGE_SENSOR_H_INCLUDED
#define RANGE_SENSOR_H_INCLUDED

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <sensor_msgs/Range.h>
#include <tf_mini_plus_ros/drivers/tf_mini_plus.h>

class RangeSensor
{
    public:
        RangeSensor(ros::NodeHandle *nh, uint8 bus, uint8 address, std::string deviceId = "sensor0");
        void PubData();
        void ChangeAddress(uint8 new_address);
        bool isOpenned();

    
    private:
	    ros::NodeHandle *n_; /**< Reference to the nodehandle so, the position controller can call services, get parameters, subscribe to and publish topics */
        ros::Publisher range_sensor_pub_;
            
        sensor_msgs::Range range_msg_;

        float range_ = 0.0f;
        bool device_oppened_ = true;

        TFMiniPro* tf_mini_plus_;
};
#endif

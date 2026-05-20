#include "preprocess/cloud_convert/cloud_convert.h"

#include <cmath>
#include <iostream>
#include <ros/time.h>
#include <stdexcept>

namespace
{
void ExpectNear(double actual, double expected, double tolerance, const std::string &label)
{
     if (std::abs(actual - expected) > tolerance)
     {
          throw std::runtime_error(label + " expected " + std::to_string(expected) +
                                   " got " + std::to_string(actual));
     }
}

void TestLivoxCustomMsgTimestamps()
{
     zjloc::CloudConvert convert;
     convert.lidar_type_ = zjloc::CloudConvert::LidarType::AVIA;

     livox_ros_driver2::CustomMsg::Ptr msg(new livox_ros_driver2::CustomMsg());
     msg->header.stamp = ros::Time(100.0);
     msg->timebase = 100000000000ULL;
     msg->point_num = 2;
     msg->points.resize(2);
     msg->points[0].x = 1.0f;
     msg->points[0].y = 0.0f;
     msg->points[0].z = 0.0f;
     msg->points[0].reflectivity = 10;
     msg->points[0].tag = 0;
     msg->points[0].line = 0;
     msg->points[0].offset_time = 0;
     msg->points[1].x = 2.0f;
     msg->points[1].y = 0.0f;
     msg->points[1].z = 0.0f;
     msg->points[1].reflectivity = 20;
     msg->points[1].tag = 0x10;
     msg->points[1].line = 1;
     msg->points[1].offset_time = 100000000;

     std::vector<point3D> output;
     convert.Process(msg, output);

     if (output.size() != 2)
          throw std::runtime_error("Livox conversion should keep two valid points");
     ExpectNear(convert.getTimeSpan(), 0.1, 1.0e-12, "Livox scan time span");
     ExpectNear(output[0].timestamp, 100.0, 1.0e-12, "first point timestamp");
     ExpectNear(output[1].timestamp, 100.1, 1.0e-12, "second point timestamp");
     ExpectNear(output[1].relative_time, 0.1, 1.0e-12, "second point relative time");
     ExpectNear(output[1].alpha_time, 1.0, 1.0e-12, "second point alpha time");
     if (output[1].ring != 1 || output[1].intensity != 20)
          throw std::runtime_error("Livox ring and reflectivity should be preserved");
}
}

int main()
{
     TestLivoxCustomMsgTimestamps();
     std::cout << "test_cloud_convert_livox passed" << std::endl;
     return 0;
}

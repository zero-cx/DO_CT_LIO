// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>
#include <algorithm>
#include <fstream>
#include <iomanip>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>

#include "common/utility.h"
#include "liw/do_ct_diagnostics.h"
#include "preprocess/cloud_convert/cloud_convert.h"
#include "liw/lio/lidarodom.h"

nav_msgs::Path laserOdoPath;

zjloc::lidarodom *lio;
zjloc::CloudConvert *convert;
std::ofstream trajectory_csv;
Eigen::Vector3d output_world_translation_offset = Eigen::Vector3d::Zero();
Eigen::Vector3d output_body_translation_offset = Eigen::Vector3d::Zero();
double imu_acceleration_scale = 1.0;

DEFINE_string(config_yaml, "./config/mapping.yaml", "配置文件");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

namespace
{
std::string ResolveConfigFile(ros::NodeHandle &private_nh)
{
    std::string config_file = FLAGS_config_yaml;
    private_nh.param<std::string>("config_yaml", config_file, config_file);
    if (config_file.empty() || config_file == "./config/mapping.yaml")
        return std::string(ROOT_DIR) + "config/mapping.yaml";
    if (config_file.front() != '/')
        return std::string(ROOT_DIR) + config_file;
    return config_file;
}

void downsampleAndPushCloud(const std::vector<point3D> &cloud_in,
                            double header_stamp,
                            double time_span)
{
    std::vector<point3D> cloud_out = cloud_in;
    zjloc::common::Timer::Evaluate([&]() {
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.05;
        std::mt19937_64 g;
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
    },
                                   "laser ds");

    const double frame_begin_time =
        do_ct_lio::ComputeLidarFrameBeginTime(header_stamp,
                                              time_span,
                                              convert->lidarHeaderStampIsEnd());
    lio->pushData(cloud_out, std::make_pair(frame_begin_time, time_span));
}
} // namespace

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    static int c = 0;
    // if (c % 2 == 0 && use_velodyne)
    {
        std::vector<point3D> cloud_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert->Process(msg, cloud_out); },
                                       "laser convert");

        downsampleAndPushCloud(cloud_out, msg->header.stamp.toSec(), convert->getTimeSpan());
    }
    c++;
}

void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    std::vector<point3D> cloud_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_out); },
                                   "laser convert");

    downsampleAndPushCloud(cloud_out, msg->header.stamp.toSec(), convert->getTimeSpan());
}

void imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.toSec(),
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x * imu_acceleration_scale,
              msg->linear_acceleration.y * imu_acceleration_scale,
              msg->linear_acceleration.z * imu_acceleration_scale));
    lio->pushData(imu);
}

void cmdVelHandler(const geometry_msgs::Twist::ConstPtr &msg)
{
    const double stamp = ros::Time::now().toSec();
    lio->pushCmdVel(stamp, msg->linear.x, msg->angular.z);
}

void updateStatus(const std_msgs::Int32::ConstPtr &msg)
{
    int type = msg->data;
    if (type == 1)
    {
    }
    else if (type == 2)
    {
    }
    else if (type == 3)
        ;
    else if (type == 4)
        ;
    else
        ;
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string trajectory_csv_path;
    private_nh.param<std::string>("trajectory_csv", trajectory_csv_path, "/home/ubuntu20/CT_LIO_ws/downloads/trajectory.csv");
    trajectory_csv.open(trajectory_csv_path, std::ios::out | std::ios::trunc);
    if (trajectory_csv.is_open())
    {
        trajectory_csv << "time,x,y,z,qx,qy,qz,qw\n";
        trajectory_csv.flush();
        ROS_INFO_STREAM("CT-LIO trajectory CSV logging to " << trajectory_csv_path);
    }
    else
    {
        ROS_WARN_STREAM("Failed to open CT-LIO trajectory CSV: " << trajectory_csv_path);
    }

    std::string config_file = ResolveConfigFile(private_nh);
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;
    auto config_yaml_node = YAML::LoadFile(config_file);
    auto load_output_offset = [](const YAML::Node &node,
                                 const std::string &key,
                                 Eigen::Vector3d &target) -> bool
    {
        if (!node || !node[key])
            return false;
        std::vector<double> offset = node[key].as<std::vector<double>>();
        if (offset.size() != 3)
            return false;
        target = Eigen::Vector3d(offset[0], offset[1], offset[2]);
        return true;
    };

    if (config_yaml_node["output"])
    {
        const auto output_node = config_yaml_node["output"];
        if (!load_output_offset(output_node, "world_translation_offset", output_world_translation_offset))
        {
            if (!load_output_offset(output_node, "translation_offset", output_world_translation_offset) &&
                output_node["translation_offset"])
            {
                ROS_WARN_STREAM("Ignoring output.translation_offset because it does not have 3 values");
            }
        }
        if (!load_output_offset(output_node, "body_translation_offset", output_body_translation_offset) &&
            output_node["body_translation_offset"])
        {
            ROS_WARN_STREAM("Ignoring output.body_translation_offset because it does not have 3 values");
        }

        ROS_INFO_STREAM("DO-CT-LIO output world translation offset: "
                        << output_world_translation_offset.transpose());
        ROS_INFO_STREAM("DO-CT-LIO output body translation offset: "
                        << output_body_translation_offset.transpose());
    }

    if (config_yaml_node["imu_initialization"] &&
        config_yaml_node["imu_initialization"]["acceleration_scale"])
    {
        imu_acceleration_scale =
            config_yaml_node["imu_initialization"]["acceleration_scale"].as<double>();
        if (!std::isfinite(imu_acceleration_scale) || imu_acceleration_scale <= 0.0)
        {
            ROS_WARN_STREAM("Invalid imu_initialization.acceleration_scale, using 1.0");
            imu_acceleration_scale = 1.0;
        }
    }
    ROS_INFO_STREAM("DO-CT-LIO IMU acceleration scale: " << imu_acceleration_scale);

    lio = new zjloc::lidarodom();
    if (!lio->init(config_file))
    {
        return -1;
    }

    ros::Publisher pub_scan = nh.advertise<sensor_msgs::PointCloud2>("scan", 10);
    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
        {
            sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ptr_output);

            cloud_ptr_output->header.stamp = ros::Time().fromSec(time);
            cloud_ptr_output->header.frame_id = "map";
            if (topic_name == "laser")
                pub_scan.publish(*cloud_ptr_output);
            else
                ; // publisher_.publish(*cloud_ptr_output);
            return true;
        }

    );

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Publisher pubLaserOdometryPath = nh.advertise<nav_msgs::Path>("/odometry_path", 5);

    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, double stamp)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            Eigen::Quaterniond q_current(pose.so3().matrix());
            const Eigen::Vector3d corrected_translation =
                do_ct_lio::ApplyBodyTranslationOffset(pose.translation(),
                                                      q_current,
                                                      output_body_translation_offset,
                                                      output_world_translation_offset);
            transform.setOrigin(tf::Vector3(corrected_translation.x(),
                                            corrected_translation.y(),
                                            corrected_translation.z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            if (topic_name == "laser")
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "base_link"));

                // publish odometry
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "map";
                laserOdometry.child_frame_id = "base_link";
                laserOdometry.header.stamp = ros::Time().fromSec(stamp);

                laserOdometry.pose.pose.orientation.x = q_current.x();
                laserOdometry.pose.pose.orientation.y = q_current.y();
                laserOdometry.pose.pose.orientation.z = q_current.z();
                laserOdometry.pose.pose.orientation.w = q_current.w();
                laserOdometry.pose.pose.position.x = corrected_translation.x();
                laserOdometry.pose.pose.position.y = corrected_translation.y();
                laserOdometry.pose.pose.position.z = corrected_translation.z();
                pubLaserOdometry.publish(laserOdometry);

                if (trajectory_csv.is_open())
                {
                    trajectory_csv << std::fixed << std::setprecision(9)
                                   << stamp << ","
                                   << corrected_translation.x() << ","
                                   << corrected_translation.y() << ","
                                   << corrected_translation.z() << ","
                                   << q_current.x() << ","
                                   << q_current.y() << ","
                                   << q_current.z() << ","
                                   << q_current.w() << "\n";
                    trajectory_csv.flush();
                }

                //  publish path
                geometry_msgs::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                laserOdoPath.header.stamp = laserOdometry.header.stamp;
                laserOdoPath.poses.push_back(laserPose);
                laserOdoPath.header.frame_id = "/map";
                pubLaserOdometryPath.publish(laserOdoPath);
            }

            return true;
        }

    );

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("/velocity", 1);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("/move_dist", 1);

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string &topic_name, double time1, double time2)
        {
            std_msgs::Float32 time_rviz;

            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub.publish(time_rviz);
            else
                dist_pub.publish(time_rviz);

            return true;
        }

    );

    lio->setFunc(cloud_pub_func);
    lio->setFunc(pose_pub_func);
    lio->setFunc(data_pub_func);

    convert = new zjloc::CloudConvert;
    convert->LoadFromYAML(config_file);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
    std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    std::string cmd_vel_topic = "/cmd_vel";
    if (yaml["common"]["cmd_vel_topic"])
        cmd_vel_topic = yaml["common"]["cmd_vel_topic"].as<std::string>();

    ros::Subscriber subLaserCloud;
    ros::Subscriber subLivoxCloud;
    if (convert->lidar_type_ == zjloc::CloudConvert::LidarType::AVIA)
    {
        subLivoxCloud = nh.subscribe<livox_ros_driver2::CustomMsg>(laser_topic, 100, livox_pcl_cbk);
        ROS_INFO_STREAM("Subscribed Livox CustomMsg LiDAR topic: " << laser_topic);
    }
    else
    {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(laser_topic, 100, standard_pcl_cbk);
        ROS_INFO_STREAM("Subscribed PointCloud2 LiDAR topic: " << laser_topic);
    }

    ros::Subscriber sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, imuHandler);

    ros::Subscriber sub_cmd_vel;
    if (!cmd_vel_topic.empty())
        sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 500, cmdVelHandler);

    ros::Subscriber sub_type = nh.subscribe<std_msgs::Int32>("/change_status", 2, updateStatus);

    std::thread measurement_process(&zjloc::lidarodom::run, lio);

    ros::spin();

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}

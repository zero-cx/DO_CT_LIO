#include "cloud_convert.h"

#include <algorithm>
#include <glog/logging.h>
#include <limits>
#include <yaml-cpp/yaml.h>
// #include <execution>

namespace zjloc
{

    void CloudConvert::Process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                               std::vector<point3D> &pcl_out)
    {
        switch (lidar_type_)
        {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE16:
            RobosenseHandler(msg);
            break;

        case LidarType::PANDAR:
            PandarHandler(msg);
            break;

        case LidarType::AVIA:
            LOG(ERROR) << "AVIA/Livox requires livox_ros_driver2::CustomMsg input";
            cloud_out_.clear();
            timespan_ = 0.0;
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
        }
        pcl_out = cloud_out_;
    }

    void CloudConvert::Process(const livox_ros_driver2::CustomMsg::ConstPtr &msg,
                               std::vector<point3D> &pcl_out)
    {
        if (lidar_type_ != LidarType::AVIA)
        {
            LOG(ERROR) << "Livox CustomMsg input requires preprocess/lidar_type: 1";
            cloud_out_.clear();
            timespan_ = 0.0;
            pcl_out.clear();
            return;
        }

        AviaHandler(msg);
        pcl_out = cloud_out_;
    }

    void CloudConvert::AviaHandler(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        const uint32_t point_num =
            std::min<uint32_t>(msg->point_num, static_cast<uint32_t>(msg->points.size()));
        if (point_num == 0)
        {
            timespan_ = 0.1;
            return;
        }

        cloud_out_.reserve(point_num);

        double max_offset_seconds = 0.0;
        for (uint32_t i = 0; i < point_num; ++i)
        {
            max_offset_seconds =
                std::max(max_offset_seconds, static_cast<double>(msg->points[i].offset_time) * 1.0e-9);
        }
        timespan_ = std::max(1.0e-3, max_offset_seconds);

        const double headertime = msg->header.stamp.toSec();
        const double max_range_sq = max_range_ * max_range_;
        const double blind_sq = blind * blind;

        for (uint32_t i = 0; i < point_num; ++i)
        {
            const auto &src = msg->points[i];
            if (!(std::isfinite(src.x) &&
                  std::isfinite(src.y) &&
                  std::isfinite(src.z)))
                continue;

            if (point_filter_num_ > 1 && i % point_filter_num_ != 0)
                continue;

            if (scan_line_ > 0 && src.line >= scan_line_)
                continue;

            const uint8_t return_tag = src.tag & 0x30;
            if (!(return_tag == 0x00 || return_tag == 0x10))
                continue;

            const double range = src.x * src.x + src.y * src.y + src.z * src.z;
            if (range > max_range_sq || range < blind_sq)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(src.x, src.y, src.z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = static_cast<double>(src.offset_time) * 1.0e-9;
            point_temp.intensity = src.reflectivity;
            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time =
                std::min(1.0, std::max(0.0, point_temp.relative_time / timespan_));
            point_temp.timespan = timespan_;
            point_temp.ring = src.line;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        static double tm_scale = 1e9;

        double headertime = msg->header.stamp.toSec();
        timespan_ = pl_orig.points.back().t / tm_scale;
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
        //           << " , 100: " << pl_orig.points[100].t / tm_scale
        //           << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > max_range_ * max_range_ || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].t / tm_scale; // curvature unit: ms
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();
        pcl::PointCloud<robosense_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();
        //  FIXME:  时间戳大于0.1
        auto time_list_robosense = [&](robosense_ros::Point &point_1, robosense_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_robosense);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }

        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << timespan_ << std::endl;

        // std::cout << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < pl_orig.points.size(); i++)
        {
            // if (i % point_filter_num_ != 0)
            //     continue;
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > max_range_ * max_range_ || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            // point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.timestamp = pl_orig.points[i].timestamp;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;
            if (point_temp.alpha_time > 1 || point_temp.alpha_time < 0)
                std::cout << point_temp.alpha_time << ", this may error." << std::endl;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        if (plsize == 0)
        {
            timespan_ = 0.1;
            return;
        }
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1; //   1e6 - nclt kaist or 1

        //  FIXME:  nclt 及kaist时间戳大于0.1
        auto time_list_velodyne = [&](velodyne_ros::Point &point_1, velodyne_ros::Point &point_2)
        {
            return (point_1.time < point_2.time);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
        while (plsize > 1 && pl_orig.points[plsize - 1].time / tm_scale >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = std::max(1.0e-3, static_cast<double>(pl_orig.points.back().time / tm_scale));
        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale << " , 100: " << pl_orig.points[100].time / tm_scale << std::endl;

        const double scan_begin_time =
            lidar_header_stamp_is_end_ ? headertime - timespan_ : headertime;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > max_range_ * max_range_ || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].time / tm_scale; // curvature unit: s
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = scan_begin_time + point_temp.relative_time;
            point_temp.alpha_time = std::min(1.0, std::max(0.0, point_temp.relative_time / timespan_));
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::PandarHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_out_.clear();
        cloud_full_.clear();

        pcl::PointCloud<pandar_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        cloud_out_.reserve(plsize);

        double headertime = msg->header.stamp.toSec();

        static double tm_scale = 1; //   1e6

        auto time_list_pandar = [&](pandar_ros::Point &point_1, pandar_ros::Point &point_2)
        {
            return (point_1.timestamp < point_2.timestamp);
        };
        sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_pandar);
        while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1)
        {
            plsize--;
            pl_orig.points.pop_back();
        }
        timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

        // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp
        //           << " , 100: " << pl_orig.points[100].timestamp - pl_orig.points[0].timestamp
        //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
        //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

        for (int i = 0; i < plsize; i++)
        {
            if (!(std::isfinite(pl_orig.points[i].x) &&
                  std::isfinite(pl_orig.points[i].y) &&
                  std::isfinite(pl_orig.points[i].z)))
                continue;

            if (i % point_filter_num_ != 0)
                continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range > max_range_ * max_range_ || range < blind * blind)
                continue;

            point3D point_temp;
            point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
            point_temp.point = point_temp.raw_point;
            point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
            point_temp.intensity = pl_orig.points[i].intensity;

            point_temp.timestamp = headertime + point_temp.relative_time;
            point_temp.alpha_time = point_temp.relative_time / timespan_;
            point_temp.timespan = timespan_;
            point_temp.ring = pl_orig.points[i].ring;

            cloud_out_.push_back(point_temp);
        }
    }

    void CloudConvert::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        int lidar_type = yaml["preprocess"]["lidar_type"].as<int>();

        point_filter_num_ = yaml["preprocess"]["point_filter_num"].as<int>();
        blind = yaml["preprocess"]["blind"].as<double>();
        if (yaml["preprocess"]["scan_line"])
            scan_line_ = yaml["preprocess"]["scan_line"].as<int>();
        if (yaml["preprocess"]["max_range"])
            max_range_ = yaml["preprocess"]["max_range"].as<double>();
        if (yaml["preprocess"]["lidar_header_stamp_is_end"])
        {
            lidar_header_stamp_is_end_ =
                yaml["preprocess"]["lidar_header_stamp_is_end"].as<bool>();
        }

        if (lidar_type == 1)
        {
            lidar_type_ = LidarType::AVIA;
            LOG(INFO) << "Using AVIA Lidar";
        }
        else if (lidar_type == 2)
        {
            lidar_type_ = LidarType::VELO32;
            LOG(INFO) << "Using Velodyne 32 Lidar";
        }
        else if (lidar_type == 3)
        {
            lidar_type_ = LidarType::OUST64;
            LOG(INFO) << "Using OUST 64 Lidar";
        }
        else if (lidar_type == 4)
        {
            lidar_type_ = LidarType::ROBOSENSE16;
            LOG(INFO) << "Using Robosense 16 LIdar";
        }
        else if (lidar_type == 5)
        {
            lidar_type_ = LidarType::PANDAR;
            LOG(INFO) << "Using Pandar LIdar";
        }
        else
        {
            LOG(WARNING) << "unknown lidar_type";
        }
    }

} // namespace zjloc

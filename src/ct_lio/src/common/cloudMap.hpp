#ifndef CLOUD_MAP_HPP_
#define CLOUD_MAP_HPP_
// c++
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>

// eigen
#include <Eigen/Core>

// robin_map
#include <tsl/robin_map.h>

struct point3D
{
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     Eigen::Vector3d raw_point; //  raw point
     Eigen::Vector3d point;     //  global frame

     double intensity;           //   intensity
     double alpha_time = 0.0;    //  reference to last point of current frame [0,1]
     double relative_time = 0.0; //  feference to current frame
     double timespan;            //   total time of current frame
     double timestamp = 0.0;     //   global timestamp
     int ring;                   //   ring

     point3D() = default;
};

struct voxel
{

     voxel() = default;

     voxel(short x, short y, short z) : x(x), y(y), z(z) {}

     bool operator==(const voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

     inline bool operator<(const voxel &vox) const
     {
          return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
     }

     inline static voxel coordinates(const Eigen::Vector3d &point, double voxel_size)
     {
          return {short(point.x() / voxel_size),
                  short(point.y() / voxel_size),
                  short(point.z() / voxel_size)};
     }

     short x;
     short y;
     short z;
};

struct voxelBlock
{

     explicit voxelBlock(int num_points_ = 20) : num_points(num_points_) { points.reserve(num_points_); }

     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;

     bool IsFull() const { return num_points == points.size(); }

     void AddPoint(const Eigen::Vector3d &point, int frame_id = -1)
     {
          assert(num_points > points.size());
          points.push_back(point);
          Touch(frame_id);
     }

     inline int NumPoints() const { return points.size(); }

     inline int Capacity() { return num_points; }

     inline int FirstFrameId() const { return first_frame_id; }

     inline int LastFrameId() const { return last_frame_id; }

     inline bool IsStale(int current_frame_id, int max_frame_age) const
     {
          return max_frame_age > 0 && last_frame_id >= 0 &&
                 current_frame_id - last_frame_id > max_frame_age;
     }

     void ResetWithPoint(const Eigen::Vector3d &point, int frame_id)
     {
          points.clear();
          first_frame_id = -1;
          last_frame_id = -1;
          AddPoint(point, frame_id);
     }

     void Touch(int frame_id)
     {
          if (frame_id < 0)
               return;
          if (first_frame_id < 0)
               first_frame_id = frame_id;
          last_frame_id = frame_id;
     }

private:
     int num_points;
     int first_frame_id = -1;
     int last_frame_id = -1;
};

typedef tsl::robin_map<voxel, voxelBlock> voxelHashMap;

namespace std
{

     template <>
     struct hash<voxel>
     {
          std::size_t operator()(const voxel &vox) const
          {
               const size_t kP1 = 73856093;
               const size_t kP2 = 19349669;
               const size_t kP3 = 83492791;
               return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
          }
     };
}

#endif

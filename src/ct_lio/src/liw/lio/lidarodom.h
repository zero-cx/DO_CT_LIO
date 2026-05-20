#ifndef LIDAR_ODOM_H__
#define LIDAR_ODOM_H__
#include "common/timer/timer.h"
#include "liw/do_ct_diagnostics.h"
#include "liw/lidarFactor.h"
#include "liw/poseParameterization.h"

#include "algo/eskf.hpp"
#include "algo/static_imu_init.h"
#include "liw/lio_utils.h"

#include <condition_variable>
#include <fstream>

#include "tools/tool_color_printf.hpp"
#include "common/timer/timer.h"
#include "common/utility.h"
#include "tools/point_types.h"

#include <sys/times.h>
#include <sys/vtimes.h>

namespace zjloc
{

     struct liwOptions
     {
          double surf_res;
          bool log_print;
          int max_num_iteration;
          //   ct_icp
          IcpModel icpmodel;
          double size_voxel_map;
          double min_distance_points;
          int max_num_points_in_voxel;
          double max_distance;
          int map_max_frame_age = 0;
          double weight_alpha;
          double weight_neighborhood;
          double max_dist_to_plane_icp;
          int init_num_frames;
          int voxel_neighborhood;
          int max_number_neighbors;
          int threshold_voxel_occupancy;
          bool estimate_normal_from_neighborhood;
          int min_number_neighbors;
          double power_planarity;
          int num_closest_neighbors;

          double sampling_rate;

          double ratio_of_nonground;
          int max_num_residuals = 0;
          int min_num_residuals = 0;

          MotionCompensation motion_compensation;
          bool point_to_plane_with_distortion = false;
          double beta_location_consistency;    // Constraints on location
          double beta_constant_velocity;       // Constraint on velocity
          double beta_scan_translation = 0.0;  // Constraint on IMU-predicted scan translation
          double beta_small_velocity;          // Constraint on the relative motion
          double beta_orientation_consistency; // Constraint on the orientation consistency
          double beta_relative_orientation = 0.0; // Constraint on scan relative rotation
          double beta_relative_yaw = 0.0;      // Constraint on scan relative yaw from IMU prediction
          double beta_nonholonomic = 0.0;      // Ground vehicle lateral/vertical motion prior
          double beta_frame_nonholonomic = 0.0; // Ground vehicle inter-frame lateral/vertical prior
          double beta_world_z_consistency = 0.0; // World-frame inter-frame height prior
          double beta_world_z_reference = 0.0;   // Fixed world-frame height reference prior
          double beta_command_translation = 0.0; // Commanded body-frame forward motion prior
          double beta_command_lateral = 0.0;     // Commanded planar lateral velocity prior
          double beta_command_yaw = 0.0;         // Commanded scan yaw prior
          double command_motion_max_gap = 0.25;
          int command_motion_min_samples = 2;

          double thres_orientation_norm;
          double thres_translation_norm;

          struct DoCtDiagnosticsOptions
          {
               bool enabled = false;
               bool logging_only = true;
               bool finite_difference_enabled = true;
               bool log_first_iteration_only = true;
               int log_every_n_frames = 50;
               int finite_difference_max_columns = 12;
               int effective_rank_threshold = 8;
               double relative_eigen_floor = 1e-6;
               double rotation_scale_meters = 1.0;
               double finite_difference_epsilon = 1e-5;
               std::string log_path;
          } do_ct_diagnostics;

          struct DegeneracyAwareOptions
          {
               bool enable = false;
               bool diagnostic_6d = false;
               bool log_diagnostics = true;
               bool jacobian_check = true;
               double lambda0 = 0.08;
               double gamma = 0.7;
               double omega_min = 0.15;
               double omega_max = 1.5;
               std::string rotation_scale_mode = "adaptive";
               double rotation_scale = 8.0;
               double rotation_scale_min = 2.0;
               double rotation_scale_max = 20.0;
               double bucket_size = 0.8;
               int bucket_top_k = 6;
               int time_bucket_count = 4;
               int min_candidates = 120;
               int max_candidate_multiplier = 3;
               double degeneracy_threshold = 0.05;
               int effective_rank_threshold = 8;
               double prior_boost = 2.0;
               double prior_boost_max = 4.0;
               double max_weight_delta = 0.25;
               double jacobian_norm_min = 1.0e-8;
               bool post_solve_projection = false;
               double projection_strength = 0.0;
               double projection_min_gate_threshold = 0.1;
               std::string log_path;
          } degeneracy_aware;
     };

     struct DoCtPlaneResidual
     {
          Eigen::Vector3d raw_point = Eigen::Vector3d::Zero();
          Eigen::Vector3d world_point = Eigen::Vector3d::Zero();
          Eigen::Vector3d reference_point = Eigen::Vector3d::Zero();
          Eigen::Vector3d normal = Eigen::Vector3d::Zero();
          double norm_offset = 0.0;
          double alpha_time = 1.0;
          double local_factor_weight = 1.0;
          double local_info_weight = 1.0;
          double point_to_plane_distance = 0.0;
          double hessian_quality = 1.0;
          double final_info_weight = 1.0;
          double final_factor_weight = 1.0;
          int spatial_bucket_x = 0;
          int spatial_bucket_y = 0;
          int spatial_bucket_z = 0;
          int time_bucket = 0;
          int source_index = 0;
     };

     class lidarodom
     {
     public:
          lidarodom(/* args */);
          ~lidarodom();

          bool init(const std::string &config_yaml);

          void pushData(std::vector<point3D>, std::pair<double, double> data);
          void pushData(IMUPtr imu);
          void pushCmdVel(double stamp, double linear_x, double angular_z);

          void run();

          int getIndex() { return index_frame; }

          void setFunc(std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> &fun) { pub_cloud_to_ros = fun; }
          void setFunc(std::function<bool(std::string &topic_name, SE3 &pose, double time)> &fun) { pub_pose_to_ros = fun; }
          void setFunc(std::function<bool(std::string &topic_name, double time1, double time2)> &fun) { pub_data_to_ros = fun; }

     private:
          void loadOptions();
          /// 使用IMU初始化
          void TryInitIMU();

          /// 利用IMU预测状态信息
          /// 这段时间的预测数据会放入imu_states_里
          void Predict();

          /// 对measures_中的点云去畸变
          void Undistort(std::vector<point3D> &points);

          std::vector<MeasureGroup> getMeasureMents();

          /// 处理同步之后的IMU和雷达数据
          void ProcessMeasurements(MeasureGroup &meas);

          void stateInitialization();

          cloudFrame *buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                 double timestamp_begin, double timestamp_end);

          void poseEstimation(cloudFrame *p_frame);

          void optimize(cloudFrame *p_frame);

          void lasermap_fov_segment();

          void map_incremental(cloudFrame *p_frame, int min_num_points = 0);

          void addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                             const double &intensity, double voxel_size,
                             int max_num_points_in_voxel, double min_distance_points,
                             int min_num_points, cloudFrame *p_frame);

          void addSurfCostFactor(std::vector<ceres::CostFunction *> &surf, std::vector<Eigen::Vector3d> &normals,
                                 std::vector<point3D> &keypoints, const cloudFrame *p_frame,
                                 std::vector<DoCtPlaneResidual> *do_ct_plane_residuals = nullptr,
                                 bool defer_cost_creation = false);

          double applyDegeneracyAwareResiduals(std::vector<ceres::CostFunction *> &surf,
                                               std::vector<DoCtPlaneResidual> &candidates,
                                               const Eigen::Vector3d &begin_t,
                                               const Eigen::Quaterniond &begin_quat,
                                               const Eigen::Vector3d &end_t,
                                               const Eigen::Quaterniond &end_quat,
                                               int frame_id,
                                               int iteration);

          do_ct_lio::CommandMotionDelta commandMotionDelta(double begin_time,
                                                           double end_time);

          void logDoCtHessianDiagnostics(ceres::Problem &problem,
                                         const std::vector<double *> &parameter_blocks,
                                         int frame_id,
                                         int iteration,
                                         int surf_num);

          void addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points,
                             const Eigen::Vector3d &point, const double &intensity,
                             cloudFrame *p_frame);

          double checkLocalizability(std::vector<Eigen::Vector3d> planeNormals);

          // search neighbors
          Neighborhood computeNeighborhoodDistribution(
              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                          int nb_voxels_visited, double size_voxel_map, int max_num_neighbors,
                          int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);

          inline Sophus::SO3d r2SO3(const Eigen::Vector3d r)
          {
               return Sophus::SO3d::exp(r);
          }

     private:
          /// @brief 数据
          std::string config_yaml_;
          StaticIMUInit imu_init_;    // IMU静止初始化
          SE3 TIL_;                   //   lidar 转换到 imu
          MeasureGroup measures_;     // sync IMU and lidar scan
          bool imu_need_init_ = true; // 是否需要估计IMU初始零偏
          int index_frame = 1;
          liwOptions options_;

          voxelHashMap voxel_map;
          Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity(); //   lidar 转换到 imu坐标系下
          Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero();     //   need init
          double laser_point_cov = 0.001;
          bool do_ct_projection_valid_ = false;
          bool do_ct_projection_degenerate_ = false;
          double do_ct_projection_min_gate_ = 1.0;
          do_ct_lio::Vector12d do_ct_projection_scale_ = do_ct_lio::Vector12d::Ones();
          do_ct_lio::Vector12d do_ct_projection_gates_ = do_ct_lio::Vector12d::Ones();
          do_ct_lio::Matrix12d do_ct_projection_eigenvectors_ = do_ct_lio::Matrix12d::Identity();
          do_ct_lio::OneShotScalarReference world_z_reference_;

          double PR_begin[6] = {0, 0, 0, 0, 0, 0}; //  p         r
          double PR_end[6] = {0, 0, 0, 0, 0, 0};   //  p         r

          /// @brief 滤波器
          ESKFD eskf_;
          std::vector<NavStated> imu_states_; // ESKF预测期间的状态
          IMUPtr last_imu_ = nullptr;

          double time_curr;
          double delay_time_;
          Vec3d g_{0, 0, -9.8};

          /// @brief 数据管理及同步
          std::deque<std::vector<point3D>> lidar_buffer_;
          std::deque<IMUPtr> imu_buffer_;    // imu数据缓冲
          std::deque<do_ct_lio::CommandMotionSample> cmd_velocity_buffer_;
          double last_timestamp_imu_ = -1.0; // 最近imu时间
          double last_timestamp_lidar_ = 0;  // 最近lidar时间
          std::deque<std::pair<double, double>> time_buffer_;

          /// @brief mutex
          std::mutex mtx_buf;
          std::mutex mtx_state;
          std::mutex mtx_cmd;
          std::condition_variable cond;

          state *current_state;
          std::vector<cloudFrame *> all_cloud_frame; //  cache all frame
          std::vector<state *> all_state_frame;      //   多保留一份state，这样可以不用去缓存all_cloud_frame

          std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> pub_cloud_to_ros;
          std::function<bool(std::string &topic_name, SE3 &pose, double time)> pub_pose_to_ros;
          std::function<bool(std::string &topic_name, double time1, double time2)> pub_data_to_ros;
          pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;

          std::ofstream do_ct_diagnostics_log_;
          bool do_ct_diagnostics_header_written_ = false;
          std::ofstream degeneracy_aware_log_;
          bool degeneracy_aware_header_written_ = false;
     };
}

#endif

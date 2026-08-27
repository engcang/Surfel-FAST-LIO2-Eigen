#pragma once

#include <algorithm>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include "eigen_lie.hpp"
#include "imu_processing.hpp"
#include "ros_converter.hpp"
#include "runtime_profiler.hpp"
#include "surfel_voxel_map.hpp"
#include "tbb_surfel_voxel_map.hpp"

#define LASER_POINT_COV (0.001)

class SurfelFastLioApplication
{
public:
    SurfelFastLioApplication()
    {
        active_application_ = this;
    }

    ~SurfelFastLioApplication()
    {
        if (active_application_ == this)
        {
            active_application_ = nullptr;
        }
    }

private:
    inline static SurfelFastLioApplication *active_application_ = nullptr;
    inline static volatile std::sig_atomic_t exit_requested_ = 0;

    template<bool UseConcurrentHashMap>
    static void measurementJacobianCallback(LioState &_state,
                                            DynamicSharedData &_measurement_data)
    {
        active_application_->buildMeasurementModelJacobianMatrix<UseConcurrentHashMap>(_state, _measurement_data);
    }

    template<typename T>
    T parameter(ros::NodeHandle &_node,
                const std::string &_name,
                const T &_default_value)
    {
        std::string ros_name = _name;
        std::replace(ros_name.begin(), ros_name.end(), '.', '/');
        T value;
        _node.param<T>(ros_name, value, _default_value);
        return value;
    }

    bool path_enabled_ = true;

    std::string lidar_topic_, imu_topic_;
    std::string map_frame_ = "map";
    std::string odometry_frame_ = "odom";

    double last_timestamp_lidar_ = 0, last_timestamp_imu_ = -1.0;
    double gyroscope_covariance_ = 0.1, accelerometer_covariance_ = 0.1, gyroscope_bias_covariance_ = 0.0001, accelerometer_bias_covariance_ = 0.0001;
    double voxel_resolution_ = 0;
    double local_map_box_size_ = 0, lidar_end_time_ = 0;
    double lidar_mean_scantime_ = 0.0;
    int count_lidar_scan_ = 0;
    int num_effective_points_ = 0;
    int num_voxel_points_ = 0, maximum_iterations_ = 0;
    bool point_has_valid_surfel_[100000]{};
    bool lidar_pushed_ = false;
    bool first_synchronized_measurement_ = true;
    bool scan_publish_enabled_ = false, dense_publish_enabled_ = false, body_scan_publish_enabled_ = false;
    int lidar_type_ = LIVOX;

    std::deque<double> time_buffer_;
    std::deque<LidarPointCloud::Ptr> lidar_buffer_;
    std::deque<ImuSample> imu_buffer_;

    LidarPointCloud::Ptr points_undistorted_lidar_ = LidarPointCloud::Ptr(new LidarPointCloud());
    LidarPointCloud::Ptr points_voxel_lidar_ = LidarPointCloud::Ptr(new LidarPointCloud());
    LidarPointCloud::Ptr points_voxel_world_ = LidarPointCloud::Ptr(new LidarPointCloud());
    LidarPointCloud::Ptr point_normal_vectors_ = LidarPointCloud::Ptr(new LidarPointCloud(100000, 1));
    LidarPointCloud::Ptr points_voxel_lidar_effective_ = LidarPointCloud::Ptr(new LidarPointCloud(100000, 1));
    LidarPointCloud::Ptr point_normal_vectors_effective_ = LidarPointCloud::Ptr(new LidarPointCloud(100000, 1));

    pcl::VoxelGrid<LidarPoint> voxel_grid_;
    std::unique_ptr<SurfelVoxelMap> dense_surfel_map_;
    std::unique_ptr<TbbSurfelVoxelMap> tbb_surfel_map_;
    bool use_concurrent_hash_map_ = false;

    /*** EKF inputs and output ***/
    MeasureGroup measurements_;
    RuntimeProfiler runtime_profiler_;
    ErrorStateIterativeKalmanFilter esikf_;
    LioState esikf_state_;

    nav_msgs::Path lio_path_;
    nav_msgs::Odometry mapped_odometry_;
    geometry_msgs::PoseStamped body_pose_message_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    std::shared_ptr<RosConverter> points_preprocessor_ = std::make_shared<RosConverter>();
    std::shared_ptr<ImuProcess> imu_processor_ = std::make_shared<ImuProcess>();

    template<bool UseConcurrentHashMap>
    auto &activeSurfelMap()
    {
        if constexpr (UseConcurrentHashMap)
        {
            return *tbb_surfel_map_;
        }
        else
        {
            return *dense_surfel_map_;
        }
    }

    template<typename MapType>
    void configureSurfelMap(MapType &_map,
                            const float _leaf_voxel_size,
                            const float _map_half_extent,
                            const float _recenter_distance,
                            const float _maximum_flatness,
                            const float _minimum_linearity,
                            const std::size_t _minimum_occupied_leaf_count)
    {
        typename MapType::Parameters parameters;
        parameters.leaf_voxel_size_ = _leaf_voxel_size;
        parameters.map_half_extent_ = _map_half_extent;
        parameters.recenter_distance_ = _recenter_distance;
        parameters.maximum_flatness_ = _maximum_flatness;
        parameters.minimum_linearity_ = _minimum_linearity;
        parameters.minimum_occupied_leaf_count_ = _minimum_occupied_leaf_count;
        _map.configure(parameters);
    }

    inline ros::Time secondsToStamp(const double _seconds)
    {
        ros::Time stamp;
        stamp.fromSec(_seconds);
        return stamp;
    }

    inline double stampToSeconds(const ros::Time &_stamp)
    {
        return _stamp.toSec();
    }

    void pointLidarToWorld(LidarPoint const *const _pi, LidarPoint *const _po)
    {
        const Eigen::Vector3d &lidar_to_imu_translation = imu_processor_->getLidarTranslationWrtImu();
        const Eigen::Matrix3d &lidar_to_imu_rotation = imu_processor_->getLidarRotationWrtImu();
        Eigen::Vector3d p_body(_pi->x, _pi->y, _pi->z);
        Eigen::Vector3d p_global(esikf_state_.rotation_ * (lidar_to_imu_rotation * p_body + lidar_to_imu_translation) + esikf_state_.position_);

        _po->x = p_global(0);
        _po->y = p_global(1);
        _po->z = p_global(2);
        _po->intensity = _pi->intensity;
    }

    void pointLidarToImu(LidarPoint const *const _pi, LidarPoint *const _po)
    {
        const Eigen::Vector3d &lidar_to_imu_translation = imu_processor_->getLidarTranslationWrtImu();
        const Eigen::Matrix3d &lidar_to_imu_rotation = imu_processor_->getLidarRotationWrtImu();
        Eigen::Vector3d p_body_lidar(_pi->x, _pi->y, _pi->z);
        Eigen::Vector3d p_body_imu(lidar_to_imu_rotation * p_body_lidar + lidar_to_imu_translation);

        _po->x = p_body_imu(0);
        _po->y = p_body_imu(1);
        _po->z = p_body_imu(2);
        _po->intensity = _pi->intensity;
    }

    void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr &_msg)
    {
        const double lidar_timestamp = stampToSeconds(_msg->header.stamp);
        if (lidar_timestamp < last_timestamp_lidar_)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer_.clear();
        }
        last_timestamp_lidar_ = lidar_timestamp;

        if (!imu_buffer_.empty() && std::abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0)
        {
            std::printf("IMU and LiDAR not synced, IMU time: %lf, LiDAR time: %lf\n", last_timestamp_imu_, last_timestamp_lidar_);
        }

        LidarPointCloud::Ptr ptr(new LidarPointCloud());
        points_preprocessor_->preProcessPoints(_msg, ptr);
        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(last_timestamp_lidar_);
    }

    void pcdLivoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &_msg)
    {
        const double lidar_timestamp = stampToSeconds(_msg->header.stamp);
        if (lidar_timestamp < last_timestamp_lidar_)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer_.clear();
        }
        last_timestamp_lidar_ = lidar_timestamp;

        if (!imu_buffer_.empty() && std::abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0)
        {
            std::printf("IMU and LiDAR not synced, IMU time: %lf, LiDAR time: %lf\n", last_timestamp_imu_, last_timestamp_lidar_);
        }

        LidarPointCloud::Ptr ptr(new LidarPointCloud());
        points_preprocessor_->preProcessPoints(_msg, ptr);
        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(last_timestamp_lidar_);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &_msg_in)
    {
        const double timestamp = stampToSeconds(_msg_in->header.stamp);
        ImuSample sample;
        sample.timestamp_ = timestamp;
        sample.linear_acceleration_ = {_msg_in->linear_acceleration.x,
                                       _msg_in->linear_acceleration.y,
                                       _msg_in->linear_acceleration.z};
        sample.angular_velocity_ = {_msg_in->angular_velocity.x,
                                    _msg_in->angular_velocity.y,
                                    _msg_in->angular_velocity.z};

        if (timestamp < last_timestamp_imu_)
        {
            ROS_WARN("imu loop back, clear buffer");
            imu_buffer_.clear();
        }

        last_timestamp_imu_ = timestamp;

        imu_buffer_.push_back(sample);
    }

    bool synchronizeMeasurements(MeasureGroup &_meas)
    {
        if (lidar_buffer_.empty() || imu_buffer_.empty())
        {
            return false;
        }

        /*** push a lidar scan ***/
        if (!lidar_pushed_)
        {
            _meas.lidar_measured_ = lidar_buffer_.front();
            _meas.lidar_beg_time_ = time_buffer_.front();


            if (_meas.lidar_measured_->points.size() <= 1) // time too little
            {
                lidar_end_time_ = _meas.lidar_beg_time_ + lidar_mean_scantime_;
                ROS_WARN("Too few input point cloud!");
            }
            else if (_meas.lidar_measured_->points.back().curvature / static_cast<double>(1000) < 0.5 * lidar_mean_scantime_)
            {
                lidar_end_time_ = _meas.lidar_beg_time_ + lidar_mean_scantime_;
            }
            else
            {
                count_lidar_scan_++;
                lidar_end_time_ = _meas.lidar_beg_time_ + _meas.lidar_measured_->points.back().curvature / static_cast<double>(1000);
                lidar_mean_scantime_ += (_meas.lidar_measured_->points.back().curvature / static_cast<double>(1000) - lidar_mean_scantime_) / count_lidar_scan_;
            }
            if (lidar_type_ == MARSIM)
                lidar_end_time_ = _meas.lidar_beg_time_;

            _meas.lidar_end_time_ = lidar_end_time_;

            lidar_pushed_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_)
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = imu_buffer_.front().timestamp_;
        _meas.imu_measured_.clear();
        while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_))
        {
            imu_time = imu_buffer_.front().timestamp_;
            if (imu_time > lidar_end_time_)
                break;
            _meas.imu_measured_.push_back(imu_buffer_.front());
            imu_buffer_.pop_front();
        }

        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
        lidar_pushed_ = false;
        return true;
    }

    template<bool UseConcurrentHashMap>
    void updateMap()
    {
        auto &surfel_map = activeSurfelMap<UseConcurrentHashMap>();
        points_voxel_world_->resize(num_voxel_points_);
        //clang-format off
        tbb::parallel_for(tbb::blocked_range<int>(0, num_voxel_points_),
                          [this](const tbb::blocked_range<int> &_range)
                          {
                              for (int index = _range.begin(); index != _range.end(); ++index)
                              {
                                  pointLidarToWorld(&points_voxel_lidar_->points[index], &points_voxel_world_->points[index]);
                              }
                          });
        //clang-format on

        surfel_map.update(*points_voxel_world_, esikf_state_.position_);
    }

    void publishPointCloudWorld(const ros::Publisher &_pub_laser_cloud_full)
    {
        if (scan_publish_enabled_)
        {
            LidarPointCloud::Ptr laser_cloud_full_res(dense_publish_enabled_ ? points_undistorted_lidar_ : points_voxel_lidar_);
            int size = laser_cloud_full_res->points.size();
            LidarPointCloud::Ptr laser_cloud_world(new LidarPointCloud(size, 1));

            for (int i = 0; i < size; i++)
            {
                pointLidarToWorld(&laser_cloud_full_res->points[i],
                                  &laser_cloud_world->points[i]);
            }

            sensor_msgs::PointCloud2 laser_cloudmsg;
            pcl::toROSMsg(*laser_cloud_world, laser_cloudmsg);
            laser_cloudmsg.header.stamp = secondsToStamp(lidar_end_time_);
            laser_cloudmsg.header.frame_id = odometry_frame_;
            _pub_laser_cloud_full.publish(laser_cloudmsg);
        }
    }

    void publishPointCloudBody(const ros::Publisher &_pub_laser_cloud_full_body)
    {
        int size = points_undistorted_lidar_->points.size();
        LidarPointCloud::Ptr laser_cloud_imu_body(new LidarPointCloud(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointLidarToImu(&points_undistorted_lidar_->points[i],
                            &laser_cloud_imu_body->points[i]);
        }

        sensor_msgs::PointCloud2 laser_cloudmsg;
        pcl::toROSMsg(*laser_cloud_imu_body, laser_cloudmsg);
        laser_cloudmsg.header.stamp = secondsToStamp(lidar_end_time_);
        laser_cloudmsg.header.frame_id = "body";
        _pub_laser_cloud_full_body.publish(laser_cloudmsg);
    }

    template<typename T>
    void setPoseStamp(T &_out)
    {
        _out.pose.position.x = esikf_state_.position_(0);
        _out.pose.position.y = esikf_state_.position_(1);
        _out.pose.position.z = esikf_state_.position_(2);
        _out.pose.orientation.x = esikf_state_.rotation_.x();
        _out.pose.orientation.y = esikf_state_.rotation_.y();
        _out.pose.orientation.z = esikf_state_.rotation_.z();
        _out.pose.orientation.w = esikf_state_.rotation_.w();
    }

    void publishOdometry(const ros::Publisher &_pub_odom_aft_mapped)
    {
        mapped_odometry_.header.frame_id = odometry_frame_;
        mapped_odometry_.child_frame_id = "body";
        mapped_odometry_.header.stamp = secondsToStamp(lidar_end_time_);
        setPoseStamp(mapped_odometry_.pose);
        const auto &p = esikf_.getCovariance();
        for (int row = 0; row < 6; ++row)
        {
            for (int column = 0; column < 6; ++column)
            {
                mapped_odometry_.pose.covariance[row * 6 + column] = p(row, column);
            }
        }
        _pub_odom_aft_mapped.publish(mapped_odometry_);

        geometry_msgs::TransformStamped transform;
        transform.header = mapped_odometry_.header;
        transform.child_frame_id = "body";
        transform.transform.translation.x = mapped_odometry_.pose.pose.position.x;
        transform.transform.translation.y = mapped_odometry_.pose.pose.position.y;
        transform.transform.translation.z = mapped_odometry_.pose.pose.position.z;
        transform.transform.rotation = mapped_odometry_.pose.pose.orientation;
        transform_broadcaster_->sendTransform(transform);

        if (map_frame_ != odometry_frame_)
        {
            const Eigen::Quaterniond &gravity_alignment = imu_processor_->getGravityAlignmentRotation();
            geometry_msgs::TransformStamped gravity_transform;
            gravity_transform.header.stamp = mapped_odometry_.header.stamp;
            gravity_transform.header.frame_id = map_frame_;
            gravity_transform.child_frame_id = odometry_frame_;
            gravity_transform.transform.rotation.x = gravity_alignment.x();
            gravity_transform.transform.rotation.y = gravity_alignment.y();
            gravity_transform.transform.rotation.z = gravity_alignment.z();
            gravity_transform.transform.rotation.w = gravity_alignment.w();
            transform_broadcaster_->sendTransform(gravity_transform);
        }
    }

    void publishPath(const ros::Publisher &_pub_path)
    {
        setPoseStamp(body_pose_message_);
        body_pose_message_.header.stamp = secondsToStamp(lidar_end_time_);
        body_pose_message_.header.frame_id = odometry_frame_;

        /*** if path is too large, the rvis will crash ***/
        static int jjj = 0;
        jjj++;
        if (jjj % 10 == 0)
        {
            lio_path_.poses.push_back(body_pose_message_);
            _pub_path.publish(lio_path_);
        }
    }

    template<bool UseConcurrentHashMap>
    void buildMeasurementModelJacobianMatrix(LioState &_s, DynamicSharedData &_measurement_data)
    {
        auto &surfel_map = activeSurfelMap<UseConcurrentHashMap>();
        const Eigen::Vector3d &lidar_to_imu_translation = imu_processor_->getLidarTranslationWrtImu();
        const Eigen::Matrix3d &lidar_to_imu_rotation = imu_processor_->getLidarRotationWrtImu();
        points_voxel_lidar_effective_->clear();
        point_normal_vectors_effective_->clear();

        //Use direct O(1) surfel lookup for residual computation.
        //clang-format off
        tbb::parallel_for(tbb::blocked_range<int>(0, num_voxel_points_),
                          [this, &_s, &surfel_map, &lidar_to_imu_translation, &lidar_to_imu_rotation](const tbb::blocked_range<int> &_range)
                          {
                              for (int index = _range.begin(); index != _range.end(); ++index)
                              {
                                  const LidarPoint &point_body = points_voxel_lidar_->points[index];
                                  LidarPoint &point_world = points_voxel_world_->points[index];
                                  const Eigen::Vector3d point_lidar(point_body.x, point_body.y, point_body.z);
                                  const Eigen::Vector3d point_global = _s.rotation_ *
                                                                           (lidar_to_imu_rotation * point_lidar + lidar_to_imu_translation) +
                                                                       _s.position_;
                                  point_world.x = point_global.x();
                                  point_world.y = point_global.y();
                                  point_world.z = point_global.z();
                                  point_world.intensity = point_body.intensity;
                                  point_has_valid_surfel_[index] = false;

                                  using SurfelType = std::conditional_t<UseConcurrentHashMap,
                                                                        TbbSurfel,
                                                                        Surfel>;
                                  SurfelType surfel;
                                  if (!surfel_map.findSurfel(point_global, surfel))
                                  {
                                      continue;
                                  }

                                  const float distance = surfel.normal_.dot(point_global.cast<float>() - surfel.centroid_);
                                  const float range_scale = std::sqrt(std::max(static_cast<float>(point_lidar.norm()), 1.0e-6F));
                                  const float score = 1.0F - 0.9F * std::abs(distance) / range_scale;
                                  if (score <= 0.9F)
                                  {
                                      continue;
                                  }

                                  point_has_valid_surfel_[index] = true;
                                  point_normal_vectors_->points[index].x = surfel.normal_.x();
                                  point_normal_vectors_->points[index].y = surfel.normal_.y();
                                  point_normal_vectors_->points[index].z = surfel.normal_.z();
                                  point_normal_vectors_->points[index].intensity = distance;
                              }
                          });
        //clang-format on

        num_effective_points_ = 0;

        for (int i = 0; i < num_voxel_points_; i++)
        {
            if (point_has_valid_surfel_[i])
            {
                points_voxel_lidar_effective_->points[num_effective_points_] = points_voxel_lidar_->points[i];
                point_normal_vectors_effective_->points[num_effective_points_] = point_normal_vectors_->points[i];
                num_effective_points_++;
            }
        }

        if (num_effective_points_ < 1)
        {
            _measurement_data.valid_ = false;
            ROS_WARN("No Effective Points!");
            return;
        }

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        _measurement_data.jacobian_.setZero(num_effective_points_, kMeasurementStateDim);
        _measurement_data.residual_.resize(num_effective_points_);

        //clang-format off
        tbb::parallel_for(tbb::blocked_range<int>(0, num_effective_points_),
                          [this, &_s, &_measurement_data, &lidar_to_imu_translation, &lidar_to_imu_rotation](const tbb::blocked_range<int> &_range)
                          {
                              for (int index = _range.begin(); index != _range.end(); ++index)
                              {
                                  const LidarPoint &laser_point = points_voxel_lidar_effective_->points[index];
                                  const Eigen::Vector3d point_lidar(laser_point.x, laser_point.y, laser_point.z);
                                  const Eigen::Vector3d point_imu = lidar_to_imu_rotation * point_lidar + lidar_to_imu_translation;
                                  const Eigen::Matrix3d point_imu_cross = lie::hat(point_imu);

                                  const LidarPoint &normal_point = point_normal_vectors_effective_->points[index];
                                  const Eigen::Vector3d normal(normal_point.x, normal_point.y, normal_point.z);
                                  const Eigen::Vector3d rotated_normal = _s.rotation_.conjugate() * normal;
                                  const Eigen::Vector3d rotation_jacobian = point_imu_cross * rotated_normal;
                                  _measurement_data.jacobian_.block<1, kMeasurementStateDim>(index, 0) << normal.transpose(), rotation_jacobian.transpose();
                                  _measurement_data.residual_(index) = -normal_point.intensity;
                              }
                          });
        //clang-format on
    }

public:
    static void requestExit()
    {
        exit_requested_ = 1;
    }

    int mainFunction()
    {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");
        std::vector<double> lidar_to_imu_translation_values(3, 0.0);
        std::vector<double> lidar_to_imu_rotation_values(9, 0.0);

        path_enabled_ = parameter<bool>(private_node, "publish.path_en", true);
        scan_publish_enabled_ = parameter<bool>(private_node, "publish.scan_publish_en", true);
        dense_publish_enabled_ = parameter<bool>(private_node, "publish.dense_publish_en", true);
        body_scan_publish_enabled_ = parameter<bool>(private_node, "publish.scan_bodyframe_pub_en", true);
        const bool runtime_enabled = parameter<bool>(private_node, "runtime.enabled", false);
        const std::string runtime_output_path = parameter<std::string>(private_node, "runtime.output_path", "runtime.csv");
        runtime_profiler_.configure(runtime_enabled, runtime_output_path);
        maximum_iterations_ = parameter<int>(private_node, "filter.maximum_iterations", 4);
        lidar_topic_ = parameter<std::string>(private_node, "common.lidar_topic", "/livox/lidar");
        imu_topic_ = parameter<std::string>(private_node, "common.imu_topic", "/livox/imu");
        map_frame_ = parameter<std::string>(private_node, "common.map_frame", "map");
        odometry_frame_ = parameter<std::string>(private_node, "common.odometry_frame", "odom");
        voxel_resolution_ = parameter<double>(private_node, "preprocess.voxel_resolution", 0.5);
        local_map_box_size_ = parameter<double>(private_node, "mapping.local_map_box_size", 1000.0);
        gyroscope_covariance_ = parameter<double>(private_node, "mapping.gyroscope_covariance", 0.1);
        accelerometer_covariance_ = parameter<double>(private_node, "mapping.accelerometer_covariance", 0.1);
        gyroscope_bias_covariance_ = parameter<double>(private_node, "mapping.gyroscope_bias_covariance", 0.0001);
        accelerometer_bias_covariance_ = parameter<double>(private_node, "mapping.accelerometer_bias_covariance", 0.0001);
        points_preprocessor_->minimum_range_ = parameter<double>(private_node, "preprocess.min_range", 0.01);
        lidar_type_ = parameter<int>(private_node, "preprocess.lidar_type", LIVOX);
        points_preprocessor_->scan_channels_ = parameter<int>(private_node, "preprocess.scan_line_count", 16);
        points_preprocessor_->point_timestamp_unit_ = parameter<int>(private_node, "preprocess.point_timestamp_unit", US);
        points_preprocessor_->scan_rate_ = parameter<int>(private_node, "preprocess.scan_rate_hz", 10);
        points_preprocessor_->point_stride_ = parameter<int>(private_node, "preprocess.point_stride", 2);
        lidar_to_imu_translation_values = parameter<std::vector<double>>(private_node, "mapping.lidar_to_imu_translation", lidar_to_imu_translation_values);
        lidar_to_imu_rotation_values = parameter<std::vector<double>>(private_node, "mapping.lidar_to_imu_rotation", lidar_to_imu_rotation_values);
        if (lidar_to_imu_translation_values.size() != 3U || lidar_to_imu_rotation_values.size() != 9U)
        {
            ROS_FATAL("Invalid LiDAR-IMU extrinsic dimensions: lidar_to_imu_translation requires 3 values and lidar_to_imu_rotation requires 9 values (received %zu and %zu)",
                      lidar_to_imu_translation_values.size(),
                      lidar_to_imu_rotation_values.size());
            ros::shutdown();
            return 1;
        }
        use_concurrent_hash_map_ = parameter<bool>(private_node, "surfel.use_concurrent_hash_map", false);
        const float surfel_leaf_voxel_size = static_cast<float>(parameter<double>(private_node,
                                                                                  "surfel.leaf_voxel_size",
                                                                                  voxel_resolution_));
        const float surfel_map_half_extent = static_cast<float>(local_map_box_size_ * 0.5);
        const float surfel_recenter_distance = static_cast<float>(parameter<double>(private_node,
                                                                                    "surfel.recenter_distance",
                                                                                    local_map_box_size_ * 0.25));
        const float surfel_maximum_flatness = static_cast<float>(parameter<double>(private_node,
                                                                                   "surfel.maximum_flatness",
                                                                                   0.03));
        const float surfel_minimum_linearity = static_cast<float>(parameter<double>(private_node,
                                                                                    "surfel.minimum_linearity",
                                                                                    0.3));
        const std::size_t surfel_minimum_occupied_leaf_count = static_cast<std::size_t>(parameter<int>(private_node,
                                                                                                       "surfel.minimum_occupied_leaf_count",
                                                                                                       5));
        if (use_concurrent_hash_map_)
        {
            tbb_surfel_map_ = std::make_unique<TbbSurfelVoxelMap>();
            configureSurfelMap(*tbb_surfel_map_,
                               surfel_leaf_voxel_size,
                               surfel_map_half_extent,
                               surfel_recenter_distance,
                               surfel_maximum_flatness,
                               surfel_minimum_linearity,
                               surfel_minimum_occupied_leaf_count);
        }
        else
        {
            dense_surfel_map_ = std::make_unique<SurfelVoxelMap>();
            configureSurfelMap(*dense_surfel_map_,
                               surfel_leaf_voxel_size,
                               surfel_map_half_extent,
                               surfel_recenter_distance,
                               surfel_maximum_flatness,
                               surfel_minimum_linearity,
                               surfel_minimum_occupied_leaf_count);
        }
        ROS_INFO("Surfel map backend: %s, scan leaf: %.3f m, map leaf: %.3f m, minimum occupied leaves: %zu",
                 use_concurrent_hash_map_ ? "tbb_hash" : "dense",
                 voxel_resolution_,
                 surfel_leaf_voxel_size,
                 surfel_minimum_occupied_leaf_count);

        lio_path_.header.stamp = ros::Time::now();
        lio_path_.header.frame_id = odometry_frame_;
        voxel_grid_.setLeafSize(voxel_resolution_, voxel_resolution_, voxel_resolution_);
        points_preprocessor_->lidar_type_ = lidar_type_;

        Eigen::Vector3d lidar_to_imu_translation;
        Eigen::Matrix3d lidar_to_imu_rotation;
        lidar_to_imu_translation << VEC_FROM_ARRAY(lidar_to_imu_translation_values);
        lidar_to_imu_rotation << MAT_FROM_ARRAY(lidar_to_imu_rotation_values);
        imu_processor_->setExtrinsic(lidar_to_imu_translation, lidar_to_imu_rotation);
        imu_processor_->setGyroCov(Eigen::Vector3d(gyroscope_covariance_, gyroscope_covariance_, gyroscope_covariance_));
        imu_processor_->setAccelCov(Eigen::Vector3d(accelerometer_covariance_, accelerometer_covariance_, accelerometer_covariance_));
        imu_processor_->setGyroBiasCov(Eigen::Vector3d(gyroscope_bias_covariance_, gyroscope_bias_covariance_, gyroscope_bias_covariance_));
        imu_processor_->setAccelBiasCov(Eigen::Vector3d(accelerometer_bias_covariance_, accelerometer_bias_covariance_, accelerometer_bias_covariance_));
        imu_processor_->setLidarType(lidar_type_);
        const ErrorStateVector convergence_limits = ErrorStateVector::Constant(0.001);
        esikf_.initialize(use_concurrent_hash_map_ ? measurementJacobianCallback<true> : measurementJacobianCallback<false>,
                          maximum_iterations_,
                          convergence_limits);


        /*** ROS subscribe initialization ***/
        ros::Subscriber sub_livox;
        ros::Subscriber sub_pcl;
        if (points_preprocessor_->lidar_type_ == LIVOX)
        {
            sub_livox = node.subscribe<livox_ros_driver::CustomMsg>(lidar_topic_,
                                                                    200000,
                                                                    &SurfelFastLioApplication::pcdLivoxCallback,
                                                                    this,
                                                                    ros::TransportHints().tcpNoDelay());
        }
        else
        {
            sub_pcl = node.subscribe<sensor_msgs::PointCloud2>(lidar_topic_,
                                                               200000,
                                                               &SurfelFastLioApplication::pcdCallback,
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());
        }
        auto sub_imu = node.subscribe<sensor_msgs::Imu>(imu_topic_,
                                                        200000,
                                                        &SurfelFastLioApplication::imuCallback,
                                                        this,
                                                        ros::TransportHints().tcpNoDelay());
        auto pub_laser_cloud_full = node.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
        auto pub_laser_cloud_full_body = node.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
        auto pub_odom_aft_mapped = node.advertise<nav_msgs::Odometry>("/Odometry", 100000);
        auto pub_path = node.advertise<nav_msgs::Path>("/path", 100000);
        transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        //------------------------------------------------------------------------------------------------------
        ros::Rate rate(5000.0);
        bool status = ros::ok();
        while (status)
        {
            if (exit_requested_ != 0)
            {
                break;
            }

            ros::spinOnce();
            if (synchronizeMeasurements(measurements_))
            {
                if (first_synchronized_measurement_)
                {
                    first_synchronized_measurement_ = false;
                    continue;
                }

                RuntimeProfiler::Measurement runtime_measurement(runtime_profiler_);
                imu_processor_->forwardBackwardPropagation(measurements_, esikf_, points_undistorted_lidar_);
                esikf_state_ = esikf_.getState();

                if (points_undistorted_lidar_->empty() || (points_undistorted_lidar_ == NULL))
                {
                    ROS_WARN("No point, skip this scan!");
                    continue;
                }

                /*** downsample the feature points in a scan ***/
                voxel_grid_.setInputCloud(points_undistorted_lidar_);
                voxel_grid_.filter(*points_voxel_lidar_);
                num_voxel_points_ = points_voxel_lidar_->points.size();

                /*** initialize the hierarchical surfel map ***/
                const bool surfel_map_empty = use_concurrent_hash_map_ ? tbb_surfel_map_->empty() : dense_surfel_map_->empty();
                if (surfel_map_empty)
                {
                    if (num_voxel_points_ > 5)
                    {
                        if (use_concurrent_hash_map_)
                        {
                            updateMap<true>();
                            ROS_INFO("Initialized TBB-hash surfel map from %d points: %zu leaf voxels, %zu surfels",
                                     num_voxel_points_,
                                     tbb_surfel_map_->voxelCount(),
                                     tbb_surfel_map_->surfelCount());
                        }
                        else
                        {
                            updateMap<false>();
                            ROS_INFO("Initialized dense surfel map from %d points: %zu leaf voxels, %zu surfels",
                                     num_voxel_points_,
                                     dense_surfel_map_->voxelCount(),
                                     dense_surfel_map_->surfelCount());
                        }
                    }
                    continue;
                }

                /*** ICP and iterated Kalman filter update ***/
                if (num_voxel_points_ < 5)
                {
                    ROS_WARN("No point, skip this scan!");
                    continue;
                }

                point_normal_vectors_->resize(num_voxel_points_);
                points_voxel_world_->resize(num_voxel_points_);

                /*** iterated state estimation ***/
                esikf_.updateIterated(LASER_POINT_COV);
                esikf_state_ = esikf_.getState();

                /******* Publish odometry *******/
                publishOdometry(pub_odom_aft_mapped);
                runtime_measurement.finish();

                /*** Update Surfel map ***/
                if (use_concurrent_hash_map_)
                {
                    updateMap<true>();
                }
                else
                {
                    updateMap<false>();
                }

                /******* Publish points *******/
                if (path_enabled_)
                    publishPath(pub_path);
                if (scan_publish_enabled_)
                    publishPointCloudWorld(pub_laser_cloud_full);
                if (scan_publish_enabled_ && body_scan_publish_enabled_)
                    publishPointCloudBody(pub_laser_cloud_full_body);
            }

            status = ros::ok();
            rate.sleep();
        }

        transform_broadcaster_.reset();
        if (runtime_profiler_.enabled())
        {
            ROS_INFO("Runtime: %s", runtime_profiler_.summary().c_str());
            if (!runtime_profiler_.writeCsv())
            {
                ROS_ERROR("Failed to write runtime CSV '%s'.",
                          runtime_profiler_.outputPath().c_str());
            }
        }
        ros::shutdown();
        return 0;
    }
};

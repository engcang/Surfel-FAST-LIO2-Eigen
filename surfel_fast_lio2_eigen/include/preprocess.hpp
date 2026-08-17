#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <cstdint>

#include "common.hpp"

enum TimeUnit
{
    SEC = 0,
    MS = 1,
    US = 2,
    NS = 3
};

namespace velodyne_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity; //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        float time;      //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        uint16_t ring;   //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace velodyne_ros

//clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring))
//clang-format on

namespace ouster_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;       //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        uint32_t t;            //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        uint16_t reflectivity; //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        uint8_t ring;          //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        uint16_t ambient;      //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        uint32_t range;        //Codex: NOLINT(readability-identifier-naming) PCL schema field.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ouster_ros

//clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range))
//clang-format on

class Preprocess
{
public:
    LidarPointCloud livox_working_cloud_;
    LidarPointCloud preprocessed_cloud_;
    float point_timestamp_unit_scale_ = 1.0f;
    int lidar_type_ = LIVOX;
    int point_stride_ = 1;
    int scan_channels_ = 6;
    int scan_rate_ = 10;
    int point_timestamp_unit_ = US;
    double minimum_range_ = 0.01;
    bool if_given_offset_time_ = false;
};

#endif

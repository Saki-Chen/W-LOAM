#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace wloam
{
    struct PointXYZIR
    {
        PCL_ADD_POINT4D

        float intensity;
        uint16_t ring;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned

        PointXYZIR()
        {
        }
        PointXYZIR(float x_, float y_, float z_, float intensity_, uint16_t ring_)
            : x(x_), y(y_), z(z_), intensity(intensity_), ring(ring_) {}
    } EIGEN_ALIGN16;

    struct PointXYZIRT
    {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY
        uint16_t ring;
        float rel_time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;

    struct RsPointXYZIR
    {
        PCL_ADD_POINT4D

        uint8_t intensity;
        uint16_t ring;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned

        RsPointXYZIR()
        {
        }
        RsPointXYZIR(float x_, float y_, float z_, uint8_t intensity_, uint16_t ring_)
            : x(x_), y(y_), z(z_), intensity(intensity_), ring(ring_) {}
    } EIGEN_ALIGN16;

    struct RsPointXYZIRT
    {
        PCL_ADD_POINT4D
        uint8_t intensity;
        uint16_t ring = 0;
        double timestamp = 0;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        RsPointXYZIRT() {}
        RsPointXYZIRT(float x_, float y_, float z_, uint8_t intensity_, uint16_t ring_, double timestamp_)
            : x(x_), y(y_), z(z_), intensity(intensity_), ring(ring_), timestamp(timestamp_) {}
    } EIGEN_ALIGN16;

    using RichPoint = wloam::PointXYZIRT;
    using CloudType = pcl::PointCloud<RichPoint>;
    using VectorType = typename pcl::PointCloud<RichPoint>::VectorType;
    using CloudTypePtr = typename CloudType::Ptr;
    using CloudTypeConstPtr = typename CloudType::ConstPtr;
    using CloudTypePtrList = std::vector<CloudTypePtr>;

} // namespace wloam

POINT_CLOUD_REGISTER_POINT_STRUCT(wloam::PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(wloam::RsPointXYZIR, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(wloam::RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT (wloam::PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, rel_time, time)

)
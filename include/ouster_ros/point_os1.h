#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace ouster_ros {
namespace OS1 {

struct EIGEN_ALIGN16 PointOS1 {
    PCL_ADD_POINT4D;
    float t;
    uint16_t intensity;
    uint16_t reflectivity;
    uint16_t noise;
    uint8_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZIR {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
};

struct EIGEN_ALIGN16 PointXYZIF {
    PCL_ADD_POINT4D;
    uint16_t intensity;
    uint16_t reflectivity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZIRF {
    PCL_ADD_POINT4D;
    uint16_t intensity;
    uint16_t ring;
    uint16_t reflectivity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZIFN {
    PCL_ADD_POINT4D;
    uint16_t intensity;
    uint16_t reflectivity;
    uint16_t noise;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZIRFN {
    PCL_ADD_POINT4D;
    uint16_t intensity;
    uint16_t ring;
    uint16_t reflectivity;
    uint16_t noise;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
}

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointOS1,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, t, t)
    (uint16_t, intensity, intensity)
    (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointXYZIF,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint16_t, intensity, intensity)
    (uint16_t, reflectivity, reflectivity)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointXYZIRF,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint16_t, intensity, intensity)
    (uint16_t, ring, ring)
    (uint16_t, reflectivity, reflectivity)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointXYZIFN,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint16_t, intensity, intensity)
    (uint16_t, reflectivity, reflectivity)
    (uint16_t, noise, noise)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::OS1::PointXYZIRFN,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint16_t, intensity, intensity)
    (uint16_t, ring, ring)
    (uint16_t, reflectivity, reflectivity)
    (uint16_t, noise, noise)
)


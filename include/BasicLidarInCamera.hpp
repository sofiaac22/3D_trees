#ifndef BASICLIDARINCAMERA_HPP
#define BASICLIDARINCAMERA_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <random>

#ifdef USE_CUDA
// Define constants
#define MAX_DISTANCE 3.0f

// CUDA error checking macro
#define cudaCheckError() {                                          \
    cudaError_t e=cudaGetLastError();                               \
    if(e!=cudaSuccess) {                                            \
        printf("CUDA error %s:%d: %s\n", __FILE__, __LINE__,        \
               cudaGetErrorString(e));                              \
        exit(1);                                                    \
    }                                                               \
}
#endif

class BasicLidarInCamera
{
public:
    BasicLidarInCamera(){};
    /**
     * @brief Process a new point cloud and camera image
     *
     * @param lidar_pcl The point cloud from the lidar
     * @param sem_image The segmented image from the camera
     *
     * @return The RGB point cloud in the lidar frame
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processLidarInCamera(const pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pcl,
                                                                const cv::Mat sem_image) const; 

    /**
     * @brief Setup needed parameters
     */
    void configure(const Eigen::Matrix3f &camera_intrinsic_matrix,
                   const Eigen::Matrix4f &lidar_to_camera_transform,
                   float factor = 0.5);

    auto const &getCameraIntrinsicMatrix() const { return camera_intrinsic_matrix_; }
    auto const &getLidarToCameraTransform() const { return lidar_to_camera_transform_; }
    auto const &getFactor() const { return factor_; }
    void setFactor(float factor) { factor_ = factor; }

private:
    Eigen::Matrix3f camera_intrinsic_matrix_;
    Eigen::Matrix4f lidar_to_camera_transform_;

    float sigma_x_, sigma_y_;
    float mean_x_, mean_y_;
    float factor_ = 0.1;

    double gaussian(float x, float y, float sigma_x, float sigma_y, float mean_x = 0, float mean_y = 0);

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> distribution_;

};

#endif // BASICLIDARINCAMERA_HPP
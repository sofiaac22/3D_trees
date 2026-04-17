#include <BasicLidarInCamera.hpp>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr processLidarInCameraCPU(const pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pcl,
                                                               const cv::Mat sem_image,
                                                               const Eigen::Matrix4f &lidar_to_camera_transform,
                                                               const Eigen::Matrix3f &camera_intrinsic_matrix)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_rgb_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
    lidar_rgb_pc->reserve(lidar_pcl->size());

    // Create a matrix of sem_image's size to store which points of the lidar are in this pixel of the image
    // We set it with -1 to indicate that there is no point in this pixel
    Eigen::MatrixXi lidar_in_image = Eigen::MatrixXi::Ones(sem_image.rows, sem_image.cols) * -1;

    for (size_t i = 0; i < lidar_pcl->size(); i++)
    {

        pcl::PointXYZRGB point;
        point.x = lidar_pcl->points[i].x;
        point.y = lidar_pcl->points[i].y;
        point.z = lidar_pcl->points[i].z;
        point.r = 0;
        point.g = 0;
        point.b = 0;

        Eigen::Vector4f lidar_point(point.x, point.y, point.z, 1.0);

        // Skip points that are too close to the lidar. Ego-vehicle is not of interest
        if (lidar_point.norm() < 3)
            continue;

        // Transform lidar point to camera coordinates
        Eigen::Vector4f camera_point = lidar_to_camera_transform * lidar_point;

        if (camera_point.z() < 0)
        {
            continue;
        }

        // Project the camera_point into image coordinates using pinhole camera model
        Eigen::Vector3f projected_point = camera_intrinsic_matrix.block<3, 3>(0, 0) * camera_point.head(3);
        float x_image = projected_point(0) / projected_point(2);
        float y_image = projected_point(1) / projected_point(2);

        // Assuming sem_image is a 3-channel image (BGR)
        if (x_image >= 0 && x_image < sem_image.cols && y_image >= 0 && y_image < sem_image.rows)
        {
            int x_pixel = static_cast<int>(x_image);
            int y_pixel = static_cast<int>(y_image);

            // If there is already a point in this pixel, keep the one with the closest norm
            if (lidar_in_image(y_pixel, x_pixel) != -1)
            {
                Eigen::Vector4f old_lidar_point = lidar_pcl->points[lidar_in_image(y_pixel, x_pixel)].getVector4fMap();
                Eigen::Vector4f old_camera_point = lidar_to_camera_transform * old_lidar_point;
                // If there is already a point in this pixel, keep the one with the closest norm
                if (camera_point.norm() < old_camera_point.norm())
                {
                    lidar_in_image(y_pixel, x_pixel) = i;
                }
            }
            else
            {
                // If there is no point in this pixel, keep the current one
                lidar_in_image(y_pixel, x_pixel) = i;
            }

            lidar_in_image(y_pixel, x_pixel) = i;
        }
    }

    // For each point in the image, project it to the camera image and color it
    for (int y = 0; y < lidar_in_image.rows(); ++y)
    {
        for (int x = 0; x < lidar_in_image.cols(); ++x)
        {
            int index = lidar_in_image(y, x);
            if (index == -1)
            {
                continue;
            }

            pcl::PointXYZRGB point;
            point.x = lidar_pcl->points[index].x;
            point.y = lidar_pcl->points[index].y;
            point.z = lidar_pcl->points[index].z;

            cv::Vec3b intensity = sem_image.at<cv::Vec3b>(y, x);
            point.r = static_cast<int>(intensity[2]); // Red channel
            point.g = static_cast<int>(intensity[1]); // Green channel
            point.b = static_cast<int>(intensity[0]); // Blue channel

            lidar_rgb_pc->push_back(point);
        }
    }

    return lidar_rgb_pc;
}

#ifdef USE_CUDA
#include <cuda_runtime.h>

void vectorProcessLidarInCameraCUDA(const float4 *h_lidar_points, float4 *h_lidar_points_rgb, int num_points,
                                    const uchar3 *h_image, int image_width, int image_height, const float *h_lidar_to_camera,
                                    const float *h_camera_intrinsics);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr processLidarInCameraGPU(const pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pcl,
                                                                                        const cv::Mat sem_image,
                                                                                        const Eigen::Matrix4f &lidar_to_camera_transform,
                                                                                        const Eigen::Matrix3f &camera_intrinsic_matrix)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_rgb_pc(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Number of LiDAR points
    int num_points = lidar_pcl->size();

    // Image dimensions
    int image_width = sem_image.cols;
    int image_height = sem_image.rows;
    int total_pixels = image_width * image_height;

    // Allocate host memory for LiDAR points
    float4 *h_lidar_points = new float4[num_points];
    float4 *h_lidar_points_rgb = new float4[num_points];

    // Prepare LiDAR points data
    for (int i = 0; i < num_points; ++i)
    {
        pcl::PointXYZ point = lidar_pcl->points[i];
        h_lidar_points[i] = make_float4(point.x, point.y, point.z, 1.0f);
        h_lidar_points_rgb[i] = make_float4(point.x, point.y, point.z, 0.0f);
    }

    // Flatten the semantic image into a uchar3 array
    uchar3 *h_image = new uchar3[total_pixels];
    for (int y = 0; y < image_height; ++y)
    {
        for (int x = 0; x < image_width; ++x)
        {
            cv::Vec3b intensity = sem_image.at<cv::Vec3b>(y, x);
            uchar3 color = make_uchar3(intensity[2], intensity[1], intensity[0]); // BGR to RGB
            h_image[y * image_width + x] = color;
        }
    }

    // Prepare transformation matrices
    float h_lidar_to_camera[16];
    float h_camera_intrinsics[9];

    // Flatten lidar_to_camera_transform (4x4 matrix) to a 3x4 matrix
    Eigen::Matrix<float, 4, 4> lidar_to_camera = lidar_to_camera_transform.block<4, 4>(0, 0);
    Eigen::Map<Eigen::Matrix<float, 4, 4>>(h_lidar_to_camera, 4, 4) = lidar_to_camera;

    // Flatten camera_intrinsic_matrix (3x3 matrix) to a 3x3 matrix
    Eigen::Map<Eigen::Matrix<float, 3, 3>>(h_camera_intrinsics, 3, 3) = camera_intrinsic_matrix;
    Eigen::Matrix<float, 3, 3> camera_intrinsics = camera_intrinsic_matrix;

    // Call the CUDA wrapper function
    vectorProcessLidarInCameraCUDA(
        h_lidar_points,
        h_lidar_points_rgb,
        num_points,
        h_image,
        image_width,
        image_height,
        h_lidar_to_camera,
        h_camera_intrinsics);

    // Reconstruct the colored point cloud
    for (int i = 0; i < num_points; ++i)
    {
        float4 point = h_lidar_points_rgb[i];

        // Unpack the RGB color from the w component
        int rgb_packed;
        std::memcpy(&rgb_packed, &point.w, sizeof(int));

        uint8_t r = (rgb_packed >> 16) & 0xFF;
        uint8_t g = (rgb_packed >> 8) & 0xFF;
        uint8_t b = rgb_packed & 0xFF;

        // Only add points that have been assigned a color
        if (rgb_packed != 0)
        {
            pcl::PointXYZRGB pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = point.z;
            pcl_point.r = r;
            pcl_point.g = g;
            pcl_point.b = b;

            lidar_rgb_pc->push_back(pcl_point);
        }
    }

    // Free host memory
    delete[] h_lidar_points;
    delete[] h_lidar_points_rgb;
    delete[] h_image;

    return lidar_rgb_pc;
}
#endif

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BasicLidarInCamera::processLidarInCamera(const pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pcl,
                                                                                const cv::Mat sem_image) const
{
#ifdef USE_CUDA
    // Call the CUDA implementation
    return processLidarInCameraGPU(lidar_pcl,
                                   sem_image,
                                   lidar_to_camera_transform_,
                                   camera_intrinsic_matrix_);
#else
    // Call the CPU implementation
    return processLidarInCameraCPU(lidar_pcl,
                                   sem_image,
                                   lidar_to_camera_transform_,
                                   camera_intrinsic_matrix_);
#endif
}

void BasicLidarInCamera::configure(const Eigen::Matrix3f &camera_intrinsic_matrix,
                                   const Eigen::Matrix4f &lidar_to_camera_transform,
                                   float factor)
{
    camera_intrinsic_matrix_ = camera_intrinsic_matrix;
    lidar_to_camera_transform_ = lidar_to_camera_transform;

    mean_x_ = static_cast<int>(camera_intrinsic_matrix_(0, 2));
    mean_y_ = static_cast<int>(camera_intrinsic_matrix_(1, 2));
    factor_ = factor;

    // Random number generator setup to use as a confident filter
    gen_ = std::mt19937(rd_());
    distribution_ = std::uniform_real_distribution<double>(0.0, 1 / std::sqrt(2 * M_PI));
}

double BasicLidarInCamera::gaussian(float x, float y, float sigma_x, float sigma_y, float mean_x, float mean_y)
{
    double exponent = -((x - mean_x) * (x - mean_x) / (2 * sigma_x * sigma_x) + (y - mean_y) * (y - mean_y) / (2 * sigma_y * sigma_y));
    return exp(exponent);
}
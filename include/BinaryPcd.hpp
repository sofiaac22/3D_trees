#ifndef BASE_BINARYPCD_HPP
#define BASE_BINARYPCD_HPP

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

template <typename PointT>
class BaseBinInterface
{
public:
    static typename pcl::PointCloud<PointT>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<PointT> &cloud);
};


// Implementations of the BaseBinInterface

// Raw point cloud binary interface.
// This kind of .bin files contains this fields: x, y, z, intensity, t, r
/**
 * @brief Read a binary PCD file with raw data.
 *
 * @param filename The path to the binary PCD file
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr The point cloud
 */
class BinaryRawPcdXYZ : public BaseBinInterface<pcl::PointXYZ>
{
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &cloud);
};

class BinaryRawPcdXYZI : public BaseBinInterface<pcl::PointXYZI>
{
public:
    static pcl::PointCloud<pcl::PointXYZI>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZI> &cloud); 
};


// Rectified point cloud binary interface.
// This kind of .bin files contains this fields: x, y, z
/**
 * @brief Read a binary PCD file with rectified data.
 *
 * @param filename The path to the binary PCD file
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr The point cloud
 */
class BinaryRectPcdXYZ : public BaseBinInterface<pcl::PointXYZ>
{
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &cloud);
};

class BinaryRectPcdXYZI : public BaseBinInterface<pcl::PointXYZI>
{
public:
    static pcl::PointCloud<pcl::PointXYZI>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZI> &cloud);
};

class BinaryRectPcdXYZRGB : public BaseBinInterface<pcl::PointXYZRGB>
{
public:
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB> &cloud);
};


// Non rectified point cloud binary interface.
// This kind of .bin files contains this fields: x, y, z, intensity
/**
 * @brief Read a binary PCD file with rectified data.
 *
 * @param filename The path to the binary PCD file
 * @return pcl::PointCloud<pcl::PointXYZI>::Ptr The point cloud
 */
class BinaryNoRectPcdXYZ : public BaseBinInterface<pcl::PointXYZ>
{
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &cloud);
};

class BinaryNoRectPcdXYZI : public BaseBinInterface<pcl::PointXYZI>
{
public:
    static pcl::PointCloud<pcl::PointXYZI>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZI> &cloud);
};

class BinaryNoRectPcdXYZRGB : public BaseBinInterface<pcl::PointXYZRGB>
{
public:
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB> &cloud);
};


// Non rectified point cloud binary interface.
// This kind of .bin files contains this fields: x, y, z, r, g, b
class BinarySemanticPcdXYZRGB : public BaseBinInterface<pcl::PointXYZRGB>
{
public:
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr read(const std::string &filename);

    static void write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB> &cloud);
};

#endif // BASE_BINARYPCD_HPP
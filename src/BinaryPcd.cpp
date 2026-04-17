#include <BinaryPcd.hpp>

// Raw point cloud binary interface.
pcl::PointCloud<pcl::PointXYZ>::Ptr BinaryRawPcdXYZ::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    while (true)
    {
        float x, y, z;
        uint8_t intensity;
        double timestamp;
        uint16_t ring;

        // Read x, y, z (each as float), intensity (as uint8_t), timestamp (as double) and ring (as uint16_t)
        bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&intensity), sizeof(uint8_t));
        bin_file.read(reinterpret_cast<char *>(&timestamp), sizeof(double));
        bin_file.read(reinterpret_cast<char *>(&ring), sizeof(uint16_t));

        // Check if we've reached the end of the file
        if (bin_file.eof())
            break;

        // Add the point to the cloud
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
    }

    return cloud;
}

void BinaryRawPcdXYZ::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;
            uint8_t intensity = 0;
            double t = 0;
            uint16_t r = 0;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
            bin_file.write(reinterpret_cast<char *>(&intensity), sizeof(intensity));
            bin_file.write(reinterpret_cast<char *>(&t), sizeof(t));
            bin_file.write(reinterpret_cast<char *>(&r), sizeof(r));
        }

        bin_file.close();
    }
}

// Raw point cloud binary interface.
pcl::PointCloud<pcl::PointXYZI>::Ptr BinaryRawPcdXYZI::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    while (true)
    {
        float x, y, z;
        uint8_t intensity;
        double timestamp;
        uint16_t ring;

        // Read x, y, z (each as float), intensity (as uint8_t), timestamp (as double) and ring (as uint16_t)
        bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&intensity), sizeof(uint8_t));
        bin_file.read(reinterpret_cast<char *>(&timestamp), sizeof(double));
        bin_file.read(reinterpret_cast<char *>(&ring), sizeof(uint16_t));

        // Check if we've reached the end of the file
        if (bin_file.eof())
            break;

        // Add the point to the cloud
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = static_cast<double>(intensity);
        cloud->push_back(point);
    }

    return cloud;
}

void BinaryRawPcdXYZI::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;
            uint8_t intensity = cloud.points[i].intensity;
            double t = 0;
            uint16_t r = 0;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
            bin_file.write(reinterpret_cast<char *>(&intensity), sizeof(intensity));
            bin_file.write(reinterpret_cast<char *>(&t), sizeof(t));
            bin_file.write(reinterpret_cast<char *>(&r), sizeof(r));
        }

        bin_file.close();
    }
}

// Rectified point cloud binary interface.
pcl::PointCloud<pcl::PointXYZ>::Ptr BinaryRectPcdXYZ::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    while (true)
    {
        float x, y, z;

        // Read x, y, z (each as float)
        bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));

        // Check if we've reached the end of the file
        if (bin_file.eof())
            break;

        // Add the point to the cloud
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
    }

    return cloud;
}

void BinaryRectPcdXYZ::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
        }

        bin_file.close();
    }
}

// Rectified point cloud binary interface.
pcl::PointCloud<pcl::PointXYZI>::Ptr BinaryRectPcdXYZI::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    while (true)
    {
        float x, y, z;

        // Read x, y, z (each as float)
        bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));

        // Check if we've reached the end of the file
        if (bin_file.eof())
            break;

        // Add the point to the cloud
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
    }

    return cloud;
}

void BinaryRectPcdXYZI::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
        }

        bin_file.close();
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BinaryRectPcdXYZRGB::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    while (true)
    {
        float x, y, z;

        // Read x, y, z (each as float)
        bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
        bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));

        // Check if we've reached the end of the file
        if (bin_file.eof())
            break;

        // Add the point to the cloud
        pcl::PointXYZRGB point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
    }

    return cloud;
}

void BinaryRectPcdXYZRGB::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;
            float r = cloud.points[i].r;
            float g = cloud.points[i].g;
            float b = cloud.points[i].b;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
            bin_file.write(reinterpret_cast<char *>(&r), sizeof(r));
            bin_file.write(reinterpret_cast<char *>(&g), sizeof(g));
            bin_file.write(reinterpret_cast<char *>(&b), sizeof(b));
        }

        bin_file.close();
    }
}

// Non rectified point cloud binary interface.
pcl::PointCloud<pcl::PointXYZ>::Ptr BinaryNoRectPcdXYZ::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (bin_file.is_open())
    {
        while (true)
        {
            float x, y, z;
            double intensity;

            // Read x, y, z (each as float) and intensity (as double)
            bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&intensity), sizeof(double));

            // Check if we've reached the end of the file
            if (bin_file.eof())
                break;

            // Add the point to the cloud
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud->push_back(point);
        }

        bin_file.close();
    }

    return cloud;
}

void BinaryNoRectPcdXYZ::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;
            double intensity = 0;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
            bin_file.write(reinterpret_cast<char *>(&intensity), sizeof(intensity));
        }

        bin_file.close();
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr BinaryNoRectPcdXYZI::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (bin_file.is_open())
    {
        while (true)
        {
            float x, y, z;
            double intensity;

            // Read x, y, z (each as float) and intensity (as double)
            bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&intensity), sizeof(double));

            // Check if we've reached the end of the file
            if (bin_file.eof())
                break;

            // Add the point to the cloud
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = intensity;
            cloud->push_back(point);
        }

        bin_file.close();
    }

    return cloud;
}

void BinaryNoRectPcdXYZI::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;
            double intensity = cloud.points[i].intensity;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
            bin_file.write(reinterpret_cast<char *>(&intensity), sizeof(intensity));
        }

        bin_file.close();
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BinaryNoRectPcdXYZRGB::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (bin_file.is_open())
    {
        while (true)
        {
            float x, y, z;
            double intensity;

            // Read x, y, z (each as float) and intensity (as double)
            bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&intensity), sizeof(double));

            // Check if we've reached the end of the file
            if (bin_file.eof())
                break;

            // Add the point to the cloud
            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud->push_back(point);
        }

        bin_file.close();
    }

    return cloud;
}

void BinaryNoRectPcdXYZRGB::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;
            double intensity = 0;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
            bin_file.write(reinterpret_cast<char *>(&intensity), sizeof(intensity));;
        }

        bin_file.close();
    }
}

// Semantic point cloud binary interface.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr BinarySemanticPcdXYZRGB::read(const std::string &filename)
{
    std::ifstream bin_file(filename, std::ios::in | std::ios::binary);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (bin_file.is_open())
    {
        while (true)
        {
            float x, y, z;
            uint8_t r, g, b;

            // Read x, y, z (each as float) and r, g, b (as uint8_t)
            bin_file.read(reinterpret_cast<char *>(&x), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&y), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&z), sizeof(float));
            bin_file.read(reinterpret_cast<char *>(&r), sizeof(uint8_t));
            bin_file.read(reinterpret_cast<char *>(&g), sizeof(uint8_t));
            bin_file.read(reinterpret_cast<char *>(&b), sizeof(uint8_t));

            // Check if we've reached the end of the file
            if (bin_file.eof())
                break;

            // Add the point to the cloud
            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.r = r;
            point.g = g;
            point.b = b;
            cloud->push_back(point);
        }

        bin_file.close();
    }
    return cloud;
}

void BinarySemanticPcdXYZRGB::write(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    // If parent directory does not exist, create it
    boost::filesystem::path dir(filename);
    if (!boost::filesystem::exists(dir.parent_path()))
    {
        boost::filesystem::create_directories(dir.parent_path());
    }

    std::ofstream bin_file;
    bin_file.open(filename, std::ios::out | std::ios::binary);

    if (bin_file.is_open())
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float x = cloud.points[i].x;
            float y = cloud.points[i].y;
            float z = cloud.points[i].z;
            uint8_t r = cloud.points[i].r;
            uint8_t g = cloud.points[i].g;
            uint8_t b = cloud.points[i].b;

            bin_file.write(reinterpret_cast<char *>(&x), sizeof(x));
            bin_file.write(reinterpret_cast<char *>(&y), sizeof(y));
            bin_file.write(reinterpret_cast<char *>(&z), sizeof(z));
            bin_file.write(reinterpret_cast<const char *>(&r), sizeof(uint8_t));
            bin_file.write(reinterpret_cast<const char *>(&g), sizeof(uint8_t));
            bin_file.write(reinterpret_cast<const char *>(&b), sizeof(uint8_t));
        }

        bin_file.close();
    }
}

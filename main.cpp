#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <BinaryPcd.hpp>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <random>
#include <sstream>
#include <vector>
#include <set>
#include <cmath>
#include <algorithm>
#include <limits> 

// ============================================================================
// ESTRUCTURA Y FUNCIONES PARA ANÁLISIS Y ELIMINACIÓN DE OBSTÁCULOS
// ============================================================================

struct ClusterStats {
    float width, depth, height;
    float density;
    float roughness;
    int num_points;
    
    void print() {
        std::cout << "  Dims: " << width << "x" << depth << "x" << height << "m" << std::endl;
        std::cout << "  Puntos: " << num_points << " | dens=" << density << " pts/m3" << std::endl;
        std::cout << "  Rugosidad: " << roughness << std::endl;
    }
};

float computeRoughness(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
    if (cluster->size() < 50) return 1.0f;
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    
    ne.setInputCloud(cluster);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.compute(*normals);
    
    float sum_variation = 0.0f;
    int valid = 0;
    
    for (size_t i = 0; i < normals->size() - 1; ++i) {
        if (!std::isfinite(normals->points[i].normal_x)) continue;
        if (!std::isfinite(normals->points[i+1].normal_x)) continue;
        
        Eigen::Vector3f n1(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
        Eigen::Vector3f n2(normals->points[i+1].normal_x, normals->points[i+1].normal_y, normals->points[i+1].normal_z);
        
        float dot = std::abs(n1.dot(n2));
        sum_variation += (1.0f - dot);
        valid++;
    }
    
    return valid > 0 ? sum_variation / valid : 1.0f;
}

ClusterStats computeStats(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
    ClusterStats stats;
    stats.num_points = cluster->size();
    
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    stats.width = max_pt.x - min_pt.x;
    stats.depth = max_pt.y - min_pt.y;
    stats.height = max_pt.z - min_pt.z;
    
    float volume = std::max(0.001f, stats.width * stats.depth * stats.height);
    stats.density = stats.num_points / volume;
    stats.roughness = computeRoughness(cluster);
    
    return stats;
}

bool testMorphology(const ClusterStats& stats, float& score) {
    std::vector<float> dims = {stats.width, stats.depth, stats.height};
    std::sort(dims.begin(), dims.end());
    
    float min_dim = dims[0];
    float mid_dim = dims[1];
    float max_dim = dims[2];
    
    float thinness = min_dim / std::max(max_dim, 0.01f);
    float elongation = max_dim / std::max(mid_dim, 0.01f);
    
    bool is_thin = thinness < 0.5f;
    bool is_elongated = elongation > 1.0f;
    bool is_tall_enough = max_dim > 1.5f;
    
    bool pass = is_thin && is_elongated && is_tall_enough;
    score = pass ? 1.0f : 0.0f;
    
    return pass;
}

bool testSmoothness(const ClusterStats& stats, float& score) {
    if (stats.roughness < 0.30f) {
        score = 1.0f;
    } else if (stats.roughness < 0.50f) {
        score = 0.8f;
    } else if (stats.roughness < 0.70f) {
        score = 0.5f;
    } else {
        score = 0.2f;
    }
    return stats.roughness < 0.60f;
}

bool testPlanarity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster, float& score) {
    auto remaining = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cluster));
    int vertical_planes = 0;
    float total_coverage = 0.0f;
    
    for (int iter = 0; iter < 3 && remaining->size() > 20; ++iter) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(10.0);
        seg.setMaxIterations(150);
        seg.setInputCloud(remaining);
        seg.segment(*inliers, *coef);
        
        if (inliers->indices.size() < 20) break;
        
        float ratio = static_cast<float>(inliers->indices.size()) / cluster->size();
        float nz = std::abs(coef->values[2]);
        bool is_vertical = nz < 1.0f;
        
        total_coverage += ratio;
        
        if (is_vertical && ratio > 0.15f) {
            vertical_planes++;
        }
        
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*remaining);
    }
    
    bool is_planar = (vertical_planes >= 1 && total_coverage > 0.50f) || (vertical_planes >= 2);
    score = is_planar ? 1.0f : 0.0f;
    
    return is_planar;
}

bool isVerticalFence(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
    if (cluster->size() < 30) return false;
    
    ClusterStats stats = computeStats(cluster);
    
    float morph_score = 0, smooth_score = 0, planar_score = 0;
    bool morph_pass = testMorphology(stats, morph_score);
    bool smooth_pass = testSmoothness(stats, smooth_score);
    bool planar_pass = testPlanarity(cluster, planar_score);
    
    float final_score = morph_score * 0.35f + smooth_score * 0.30f + planar_score * 0.35f;
    
    bool is_fence = false;
    if (final_score > 0.60f) {
        is_fence = true;
    } else if (morph_pass && planar_pass) {
        is_fence = true;
    } else if (smooth_score > 0.9f && planar_pass) {
        is_fence = true;
    }
    
    return is_fence;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeGroundPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(0.2);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    
    std::cout << "  Suelo detectado: " << inliers->indices.size() << " puntos" << std::endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_ground);
    
    return cloud_no_ground;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeVerticalFences(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    
    // Convertir a XYZ para clustering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_xyz);
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_xyz);
    
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(150000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_xyz);
    ec.extract(clusters);
    
    std::cout << "  Clusters detectados: " << clusters.size() << std::endl;
    
    std::vector<pcl::PointIndices> bad_clusters;
    
    for (size_t i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_xyz, clusters[i].indices, *cluster_cloud);
        
        if (isVerticalFence(cluster_cloud)) {
            bad_clusters.push_back(clusters[i]);
        }
    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (!bad_clusters.empty()) {
        pcl::PointIndices::Ptr bad_points(new pcl::PointIndices);
        for (const auto& cluster : bad_clusters) {
            bad_points->indices.insert(bad_points->indices.end(), 
                                      cluster.indices.begin(), cluster.indices.end());
        }
        
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(bad_points);
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        
        std::cout << "  Vallas eliminadas: " << bad_clusters.size() << std::endl;
    } else {
        cloud_filtered = cloud;
    }
    
    return cloud_filtered;
}
namespace cfg {
    constexpr double CLUSTER_TOLERANCE_MIN = 0.3;   // usado para calcular tolerancia por profundidad
    constexpr double CLUSTER_TOLERANCE_MAX = 4.0;
    constexpr float PLANE_INLIER_RATIO = 0.70f;     // RANSAC: proporción para considerar plano aislado
    constexpr int   PLANE_MIN_POINTS = 50;
    constexpr float PLANE_DISTANCE_THRESH = 0.05f;  // distancia RANSAC (m)

    // Nuevas constantes para reintentos de tolerancia
    constexpr double TOLERANCE_REDUCTION_FACTOR = 0.6; // multiplicador al reducir tolerancia
    constexpr double TOLERANCE_MIN_ALLOWED = 0.05;    // tolerancia mínima absoluta permitida
    constexpr int    TOLERANCE_MAX_ATTEMPTS = 5;      // cuantos reintentos máximos
}

// ============================================================================
// ESTRUCTURAS DE DATOS
// ============================================================================

struct BoundingBox {
    float x_min, y_min, x_max, y_max;
    
    BoundingBox(int id = 0, float xmin = 0, float ymin = 0, float xmax = 0, float ymax = 0, float sc = 1.0f) 
        : x_min(xmin), y_min(ymin), x_max(xmax), y_max(ymax) {}
    
    bool contains(float x, float y) const {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
    
    float getCenterX() const { return (x_min + x_max) / 2.0f; }
    float getCenterY() const { return (y_min + y_max) / 2.0f; }
};


struct BBoxProcessingResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tree_cluster;
    std::set<int> tree_point_indices;
    double applied_tolerance;
    float average_depth;
    std::vector<int> candidate_cluster_order;
    std::vector<float> candidate_distances;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> candidate_clouds;
    std::vector<std::set<int>> candidate_global_indices;

    BBoxProcessingResult() 
        : applied_tolerance(0.0), average_depth(0.0f) {
        tree_cluster.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
};

// ============================================================================
// FUNCIONES DE CALIBRACIÓN
// ============================================================================

Eigen::Matrix4f loadExtrinsic(const std::string& filepath) {
    Eigen::Matrix4f extr = Eigen::Matrix4f::Identity();
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "No se pudo abrir: " << filepath << std::endl;
        return extr;
    }
    
    float R[9], T[3];
    std::string line;
    
    if (std::getline(file, line)) {
        std::istringstream iss(line.substr(3));
        for (int i = 0; i < 9; ++i) iss >> R[i];
    }
    
    if (std::getline(file, line)) {
        std::istringstream iss(line.substr(3));
        for (int i = 0; i < 3; ++i) iss >> T[i];
    }
    
    extr(0, 0) = R[0]; extr(0, 1) = R[1]; extr(0, 2) = R[2];
    extr(1, 0) = R[3]; extr(1, 1) = R[4]; extr(1, 2) = R[5];
    extr(2, 0) = R[6]; extr(2, 1) = R[7]; extr(2, 2) = R[8];
    extr(0, 3) = T[0]; extr(1, 3) = T[1]; extr(2, 3) = T[2];
    
    return extr;
}

Eigen::Matrix3f loadIntrinsic(const std::string& filepath) {
    Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "No se pudo abrir: " << filepath << std::endl;
        return K;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("P_rect_02:") != std::string::npos) {
            std::istringstream iss(line.substr(line.find(":") + 1));
            float val[12];
            for (int i = 0; i < 12; ++i) iss >> val[i];
            
            K(0, 0) = val[0]; K(0, 1) = val[1]; K(0, 2) = val[2];
            K(1, 0) = val[4]; K(1, 1) = val[5]; K(1, 2) = val[6];
            K(2, 0) = val[8]; K(2, 1) = val[9]; K(2, 2) = val[10];
            break;
        }
    }
    
    return K;
}

// ============================================================================
// FUNCIONES DE CARGA DE DATOS
// ============================================================================

std::vector<BoundingBox> loadBoundingBoxes(const std::string& filepath) {
    std::vector<BoundingBox> bboxes;
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "No se pudo abrir: " << filepath << std::endl;
        return bboxes;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#' || line.find("---") != std::string::npos) {
            continue;
        }
        
        std::istringstream iss(line);
        int class_id;
        float x1, y1, x2, y2, score;
        
        if (iss >> class_id >> x1 >> y1 >> x2 >> y2 >> score) {
            bboxes.emplace_back(class_id, x1, y1, x2, y2, score);
        }
    }
    
    std::cout << "Bounding boxes cargadas: " << bboxes.size() << std::endl;
    return bboxes;
}

std::vector<std::string> getTimestampsFromDirectory(const std::string& lidar_dir) {
    std::vector<std::string> timestamps;
    
    if (!boost::filesystem::exists(lidar_dir) || !boost::filesystem::is_directory(lidar_dir)) {
        std::cerr << "Directorio no existe: " << lidar_dir << std::endl;
        return timestamps;
    }
    
    boost::filesystem::directory_iterator end;
    for (boost::filesystem::directory_iterator it(lidar_dir); it != end; ++it) {
        if (boost::filesystem::is_regular_file(it->path()) && 
            it->path().extension() == ".bin") {
            timestamps.push_back(it->path().stem().string());
        }
    }
    
    std::sort(timestamps.begin(), timestamps.end());
    return timestamps;
}

// ============================================================================
// FUNCIONES DE PROCESAMIENTO 3D - PROYECCIÓN
// ============================================================================

Eigen::Vector2f projectPointToImage(
    const pcl::PointXYZ& point,
    const Eigen::Matrix3f& intrinsics,
    const Eigen::Matrix4f& extrinsics_inv
) {
    Eigen::Vector4f pt(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f pt_cam = extrinsics_inv * pt;
    
    if (pt_cam.z() <= 0) {
        return Eigen::Vector2f(-1, -1);
    }
    
    Eigen::Vector3f pt_img = intrinsics * pt_cam.head<3>();
    return Eigen::Vector2f(pt_img.x() / pt_img.z(), pt_img.y() / pt_img.z());
}

bool isPointInImageBounds(const Eigen::Vector2f& uv, int cols, int rows) {
    return uv.x() >= 0 && uv.x() < cols && uv.y() >= 0 && uv.y() < rows;
}

// ============================================================================
// FUNCIONES DE TOLERANCIA ADAPTATIVA
// ============================================================================

float calculateAverageDepth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty()) return 0.0f;
    
    float sum_depth = 0.0f;
    for (const auto& point : cloud->points) {
        sum_depth += point.z;
    }
    
    return sum_depth / cloud->size();
}

double calculateClusterToleranceByDepth(
    float average_depth,
    double min_tolerance = 0.5,
    double max_tolerance = 1.0,
    float min_depth = 0.0f,
    float max_depth = 100.0f
) {
    if (max_depth <= min_depth) return min_tolerance;

    float normalized_depth = (average_depth - min_depth) / (max_depth - min_depth);
    normalized_depth = std::max(0.0f, std::min(1.0f, normalized_depth));

    // escala no lineal (hace la tolerancia mucho más pequeña para cercanos)
    const double exponent = 2.0;
    double scaled = std::pow(static_cast<double>(normalized_depth), exponent);

    double tolerance = min_tolerance + (max_tolerance - min_tolerance) * scaled;
    return std::max(min_tolerance, std::min(max_tolerance, tolerance));
}

// ============================================================================
// FUNCIONES DE CLUSTERING Y EXTRACCIÓN
// ============================================================================

std::pair<float, float> calculateClusterCentroidInImage(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointIndices& cluster_indices,
    const Eigen::Matrix3f& intrinsics,
    const Eigen::Matrix4f& extrinsics_inv
) {
    float sum_u = 0.0f, sum_v = 0.0f;
    int valid_points = 0;
    
    for (int idx : cluster_indices.indices) {
        Eigen::Vector2f uv = projectPointToImage(cloud->points[idx], intrinsics, extrinsics_inv);
        
        if (uv.x() >= 0) {
            sum_u += uv.x();
            sum_v += uv.y();
            valid_points++;
        }
    }
    
    if (valid_points > 0) {
        return {sum_u / valid_points, sum_v / valid_points};
    }
    return {0.0f, 0.0f};
}


std::vector<pcl::PointIndices> detectVerticalClusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double cluster_tolerance = 0.5,
    int min_cluster_size = 100,
    int max_cluster_size = 5000,
    double min_height_ratio = 0.5
) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices> vertical_clusters;
    
    for (const auto& indices : cluster_indices) {
        if (indices.indices.empty()) continue;

        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();

        for (int idx : indices.indices) {
            const auto& point = cloud->points[idx];
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }

        float width = std::max(max_x - min_x, max_y - min_y);
        float height = max_z - min_z;

        if (height > 0 && width > 0) {
            double height_width_ratio = height / width;
            if (height_width_ratio >= min_height_ratio) {
                vertical_clusters.push_back(indices);
            }
        }
    }

    return vertical_clusters;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractClusterPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointIndices& cluster_indices
) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    for (int idx : cluster_indices.indices) {
        cluster_cloud->points.push_back(cloud->points[idx]);
    }
    
    cluster_cloud->width = static_cast<uint32_t>(cluster_cloud->points.size());
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = false;
    
    return cluster_cloud;
}

// Nueva función: detectar si un cluster es un plano o plano inclinado
bool isIsolatedPlanarCluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    float inlier_ratio_threshold = 0.70f,
    int min_plane_points = 50,
    float plane_distance_thresh = 0.05f
) {
    if (!cluster || cluster->empty()) return true;
    if (static_cast<int>(cluster->size()) < min_plane_points) {
        // clusters muy pequeños se consideran potencialmente planos aislados
        return true;
    }

    // configurar segmentador RANSAC para plano
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_distance_thresh);
    seg.setMaxIterations(1000);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cluster));

    seg.setInputCloud(temp_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        return false;
    }

    float ratio = static_cast<float>(inliers->indices.size()) / static_cast<float>(temp_cloud->size());
    // Si el mayor plano encontrado cubre la mayoría de los puntos -> plano aislado
    if (ratio >= inlier_ratio_threshold && static_cast<int>(inliers->indices.size()) >= min_plane_points) {
        return true;
    }

    return false;
}

void computeDepthRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float& min_depth, float& max_depth) {
    if (cloud->empty()) {
        min_depth = 0.0f;
        max_depth = 100.0f;
        return;
    }
    min_depth = std::numeric_limits<float>::max();
    max_depth = std::numeric_limits<float>::lowest();
    for (const auto& p : cloud->points) {
        min_depth = std::min(min_depth, p.z);
        max_depth = std::max(max_depth, p.z);
    }
    // evitar rango cero
    if ((max_depth - min_depth) < 1e-3f) max_depth = min_depth + 1.0f;
}


// ============================================================================
// FUNCIONES DE CÁLCULO DE CENTROIDES
// ============================================================================

struct TreeCentroid {
    int tree_id;
    float x, y, z;
    int num_points;
    
    TreeCentroid(int id = -1, float cx = 0, float cy = 0, float cz = 0, int npts = 0)
        : tree_id(id), x(cx), y(cy), z(cz), num_points(npts) {}
};

std::vector<TreeCentroid> calculateTreeCentroids(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::vector<int>& tree_ids,
    int max_tree_id
) {
    std::vector<TreeCentroid> centroids;
    
    if (tree_ids.size() != cloud->size()) {
        std::cerr << "Error: tamaño de tree_ids no coincide con el tamaño de la nube" << std::endl;
        return centroids;
    }
    
    // Estructura para acumular suma y conteo por árbol
    std::vector<Eigen::Vector3f> tree_sums(max_tree_id + 1, Eigen::Vector3f::Zero());
    std::vector<int> tree_counts(max_tree_id + 1, 0);
    
    // Acumular puntos por árbol
    for (size_t i = 0; i < cloud->size(); ++i) {
        int tree_id = tree_ids[i];
        if (tree_id >= 0 && tree_id <= max_tree_id) {
            const auto& pt = cloud->points[i];
            tree_sums[tree_id] += Eigen::Vector3f(pt.x, pt.y, pt.z);
            tree_counts[tree_id]++;
        }
    }
    
    // Calcular centroides
    for (int tree_id = 0; tree_id <= max_tree_id; ++tree_id) {
        if (tree_counts[tree_id] > 0) {
            Eigen::Vector3f centroid = tree_sums[tree_id] / tree_counts[tree_id];
            centroids.emplace_back(tree_id, centroid.x(), centroid.y(), centroid.z(), tree_counts[tree_id]);
        }
    }
    
    return centroids;
}

void saveCentroidsToFile(
    const std::string& filepath,
    const std::vector<TreeCentroid>& centroids
) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "No se pudo abrir archivo para guardar centroides: " << filepath << std::endl;
        return;
    }
    
    file << "# Centroides de arboles\n";
    file << "# x\ty\tz\n";
    
    for (const auto& centroid : centroids) {
        file << centroid.x << "\t"
             << centroid.y << "\t"
             << centroid.z << "\n";
    }
    
    file.close();
    std::cout << "Centroides guardados en: " << filepath << std::endl;
}

// ============================================================================
// FUNCIONES DE VISUALIZACIÓN
// ============================================================================

cv::Scalar generateRandomColor(int seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(50, 255);
    return cv::Scalar(dist(rng), dist(rng), dist(rng));
}

void drawBoundingBoxes(cv::Mat& image, const std::vector<BoundingBox>& bboxes) {
    for (size_t i = 0; i < bboxes.size(); ++i) {
        const auto& bbox = bboxes[i];
        cv::Scalar color = generateRandomColor(static_cast<int>(i) + 500);
        
        cv::rectangle(image, 
                     cv::Point(static_cast<int>(bbox.x_min), static_cast<int>(bbox.y_min)),
                     cv::Point(static_cast<int>(bbox.x_max), static_cast<int>(bbox.y_max)),
                     color, 2);
        
        int center_x = static_cast<int>(bbox.getCenterX());
        int center_y = static_cast<int>(bbox.getCenterY());
        
        cv::line(image, cv::Point(center_x - 10, center_y), cv::Point(center_x + 10, center_y), cv::Scalar(255, 255, 0), 3);
        cv::line(image, cv::Point(center_x, center_y - 10), cv::Point(center_x, center_y + 10), cv::Scalar(255, 255, 0), 3);
        cv::circle(image, cv::Point(center_x, center_y), 5, cv::Scalar(255, 255, 0), -1);
        
        std::string label = "Tree_" + std::to_string(i);
        cv::putText(image, label, cv::Point(static_cast<int>(bbox.x_min), static_cast<int>(bbox.y_min) - 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
}

// ============================================================================
// FUNCIÓN PRINCIPAL DE PROCESAMIENTO
// ============================================================================

BBoxProcessingResult processLidarWithSingleBoundingBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const cv::Mat& image,
    const Eigen::Matrix3f& intrinsics,
    const Eigen::Matrix4f& extrinsics_inv,
    const BoundingBox& bbox,
    int bbox_id
) {
    BBoxProcessingResult result;
    
    // Extraer puntos dentro de la bounding box
    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_points(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<std::pair<int, Eigen::Vector2f>> bbox_point_mapping;
    
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        Eigen::Vector2f uv = projectPointToImage(point, intrinsics, extrinsics_inv);
        
        if (!isPointInImageBounds(uv, image.cols, image.rows) || !bbox.contains(uv.x(), uv.y())) {
            continue;
        }
        
        bbox_points->points.push_back(point);
        bbox_point_mapping.push_back({static_cast<int>(i), uv});
    }
    
    std::cout << "Puntos en bbox " << bbox_id << ": " << bbox_points->size() << std::endl;
    
    if (bbox_points->empty()) {
        return result;
    }

    // Calcular tolerancia basada en profundidad promedio
    if (!bbox_points->empty()) {
        float min_depth, max_depth;
        computeDepthRange(bbox_points, min_depth, max_depth);

        result.average_depth = calculateAverageDepth(bbox_points);
        result.applied_tolerance = calculateClusterToleranceByDepth(
            result.average_depth,
            cfg::CLUSTER_TOLERANCE_MIN,
            cfg::CLUSTER_TOLERANCE_MAX,
            min_depth,
            max_depth
        );

        std::cout << "  Profundidad promedio: " << result.average_depth << " m" << std::endl;
        std::cout << "  Profundidad rango: [" << min_depth << ", " << max_depth << "] m" << std::endl;
        std::cout << "  Tolerancia aplicada: " << result.applied_tolerance << " m" << std::endl;
    }
    
    // Detectar clusters verticales: intentar reducir tolerancia si no se encuentran candidatos válidos
    std::vector<pcl::PointIndices> vertical_clusters;
    std::vector<std::pair<float,int>> dist_idx;

    double current_tolerance = result.applied_tolerance;
    int attempt = 0;
    while (attempt < cfg::TOLERANCE_MAX_ATTEMPTS) {
        vertical_clusters = detectVerticalClusters(bbox_points, current_tolerance);
        dist_idx.clear();

        // Construir lista de candidatos ordenados por distancia al centro (imagen) y filtrar planos aislados
        for (size_t vi = 0; vi < vertical_clusters.size(); ++vi) {
            auto cand_cloud = extractClusterPoints(bbox_points, vertical_clusters[vi]);

            // Descartar solo clusters que sean un plano aislado (plano dominante por RANSAC)
            if (isIsolatedPlanarCluster(cand_cloud,
                                        cfg::PLANE_INLIER_RATIO,
                                        cfg::PLANE_MIN_POINTS,
                                        cfg::PLANE_DISTANCE_THRESH)) {
                continue;
            }

            auto centroid = calculateClusterCentroidInImage(bbox_points, vertical_clusters[vi], intrinsics, extrinsics_inv);
            float centroid_u = centroid.first;
            float centroid_v = centroid.second;

            float bbox_center_u = bbox.getCenterX();
            float bbox_center_v = bbox.getCenterY();
            float distance = std::sqrt(std::pow(centroid_u - bbox_center_u, 2) + std::pow(centroid_v - bbox_center_v, 2));
            dist_idx.emplace_back(distance, static_cast<int>(vi));
        }

        // Si encontramos candidatos ya break; si no, reducir tolerancia y reintentar
        if (!dist_idx.empty()) {
            break;
        }

        // reducir tolerancia y reintentar
        ++attempt;
        double next_tolerance = current_tolerance * cfg::TOLERANCE_REDUCTION_FACTOR;
        if (next_tolerance < cfg::TOLERANCE_MIN_ALLOWED) {
            // ya alcanzamos tolerancia mínima absoluta -> no más reintentos
            current_tolerance = std::max(cfg::TOLERANCE_MIN_ALLOWED, next_tolerance);
            break;
        }
        current_tolerance = next_tolerance;
        std::cout << "    No se encontraron candidatos válidos con tolerancia " << current_tolerance / cfg::TOLERANCE_REDUCTION_FACTOR
                  << ". Reintentando con tolerancia reducida: " << current_tolerance << " (intento " << attempt << ")" << std::endl;
    }

    // Guardar la tolerancia finalmente aplicada en el resultado (la usada en la última búsqueda)
    result.applied_tolerance = current_tolerance;
    std::cout << "  Candidatos filtrados: " << dist_idx.size() << std::endl;

    if (dist_idx.empty()) {
        // No se encontró ningún candidato válido tras los reintentos
        return result;
    }

    std::sort(dist_idx.begin(), dist_idx.end()); // por distancia ascendente

    // Llenar candidatos en el resultado (ordenados)
    for (const auto& di : dist_idx) {
        int vi = di.second;
        float d = di.first;
        result.candidate_cluster_order.push_back(vi);
        result.candidate_distances.push_back(d);
        auto cand_cloud = extractClusterPoints(bbox_points, vertical_clusters[vi]);
        result.candidate_clouds.push_back(cand_cloud);
        // global indices
        std::set<int> global_idx_set;
        for (int local_idx : vertical_clusters[vi].indices) {
            if (local_idx >= 0 && local_idx < static_cast<int>(bbox_point_mapping.size()))
                global_idx_set.insert(bbox_point_mapping[local_idx].first);
        }
        result.candidate_global_indices.push_back(global_idx_set);
    }
    // Seleccionar el mejor candidato por defecto (primero)
    if (!result.candidate_clouds.empty()) {
        result.tree_cluster = result.candidate_clouds[0];
        result.tree_point_indices = result.candidate_global_indices[0];
        std::cout << "  Árbol detectado (mejor candidato): " << result.tree_cluster->size() << " puntos" << std::endl;
    }

    return result;
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::string base_path = "../";
    std::string sequence = "test_sincro";
    std::string camera_name = "camera_0";
    std::string lidar_name = "lidar_0";
    
    // Crear directorios de salida
    std::string output_dir = "../resultados/ejemplo";
    boost::filesystem::create_directories(output_dir);
    boost::filesystem::create_directories(output_dir + "/nubes_coloreadas");
    boost::filesystem::create_directories(output_dir + "/imagenes_con_bbox");
    boost::filesystem::create_directories(output_dir + "/centroides");
    
    // Cargar calibraciones
    std::string calib_ext_path = base_path + "/calibs/calib_" + lidar_name + "_to_" + camera_name + ".txt";
    std::string calib_int_path = base_path + "/calibs/calib_cam_to_" + camera_name + ".txt";
    
    Eigen::Matrix3f intrinsics = loadIntrinsic(calib_int_path);
    Eigen::Matrix4f extrinsics_inv = loadExtrinsic(calib_ext_path);
    
    // Obtener timestamps
    std::string lidar_dir = base_path + "/" + sequence + "/data/" + lidar_name;
    std::vector<std::string> timestamps = getTimestampsFromDirectory(lidar_dir);
    
    if (timestamps.empty()) {
        std::cerr << "No se encontraron archivos .bin" << std::endl;
        return -1;
    }
    
    std::cout << "Procesando " << timestamps.size() << " timestamps..." << std::endl;
    
    // Procesar cada timestamp
    for (const std::string& timestamp : timestamps) {
        std::cout << "\n=== TIMESTAMP: " << timestamp << " ===" << std::endl;
        
        // Construir rutas
        std::string img_path = base_path + "/" + sequence + "/data/" + camera_name + "/" + timestamp + ".png";
        std::string bin_path = base_path + "/" + sequence + "/data/" + lidar_name + "/" + timestamp + ".bin";
        std::string bbox_path = base_path + "/" + sequence + "/data/annotations/" + timestamp + ".txt";
        
        // Verificar archivos
        if (!boost::filesystem::exists(img_path) || !boost::filesystem::exists(bin_path) || !boost::filesystem::exists(bbox_path)) {
            std::cout << "Archivos faltantes, saltando..." << std::endl;
            continue;
        }
        
        // Cargar imagen y nube
        cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);
        auto cloud = BinaryRectPcdXYZ::read(bin_path);
        std::vector<BoundingBox> bboxes = loadBoundingBoxes(bbox_path);
        
        if (image.empty() || cloud->empty() || bboxes.empty()) {
            std::cout << "Datos vacíos, saltando..." << std::endl;
            continue;
        }
        
        // Procesar cada bounding box
        std::vector<BBoxProcessingResult> bbox_results;
        std::map<int, int> point_to_tree_id; // mapeo: índice punto global -> índice árbol
        
        for (size_t bbox_idx = 0; bbox_idx < bboxes.size(); ++bbox_idx) {
            BBoxProcessingResult result = processLidarWithSingleBoundingBox(
                cloud, image, intrinsics, extrinsics_inv, bboxes[bbox_idx], static_cast<int>(bbox_idx));
            
            bbox_results.push_back(result);
            
            // Llenar el mapeo de puntos a árbol
            for (int point_idx : result.tree_point_indices) {
                point_to_tree_id[point_idx] = static_cast<int>(bbox_idx);
            }
        }
        
        // Crear nube coloreada final y mantener mapeo a índices originales
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        std::vector<int> colored_cloud_original_indices;  // mapeo: índice en colored_cloud -> índice en cloud original
        
        // Paleta de colores para cada árbol
        std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> tree_colors = {
            {255, 0, 0},       // Rojo
            {0, 255, 0},       // Verde
            {0, 0, 255},       // Azul
            {255, 255, 0},     // Amarillo
            {255, 0, 255},     // Magenta
            {0, 255, 255},     // Cian
            {255, 128, 0},     // Naranja
            {128, 0, 255},     // Púrpura
            {0, 255, 128},     // Verde claro
            {255, 192, 203},   // Rosa
            {128, 128, 0},     // Oliva
            {0, 128, 128},     // Teal
            {255, 128, 128},   // Rojo claro
            {128, 255, 128},   // Verde claro
            {128, 128, 255},   // Azul claro
        };
        
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& point = cloud->points[i];
            Eigen::Vector2f uv = projectPointToImage(point, intrinsics, extrinsics_inv);
            
            if (!isPointInImageBounds(uv, image.cols, image.rows)) {
                continue;
            }
            
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            
            auto it = point_to_tree_id.find(static_cast<int>(i));
            if (it != point_to_tree_id.end()) {
                // Punto pertenece a un árbol, asignar color del árbol
                int tree_id = it->second;
                auto color = tree_colors[tree_id % tree_colors.size()];
                colored_point.r = std::get<0>(color);
                colored_point.g = std::get<1>(color);
                colored_point.b = std::get<2>(color);
            } else {
                // Punto no pertenece a un árbol, usar color de imagen
                cv::Vec3b pixel = image.at<cv::Vec3b>(static_cast<int>(uv.y()), static_cast<int>(uv.x()));
                colored_point.b = pixel[0];
                colored_point.g = pixel[1];
                colored_point.r = pixel[2];
            }
            
            final_colored_cloud->points.push_back(colored_point);
            colored_cloud_original_indices.push_back(static_cast<int>(i));
        }
        
        final_colored_cloud->width = static_cast<uint32_t>(final_colored_cloud->points.size());
        final_colored_cloud->height = 1;
        final_colored_cloud->is_dense = false;

        
        // Filtrar puntos a más de 30 metros del origen y mantener rastreo de árbol
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        std::vector<int> filtered_tree_ids;  // tree_id para cada punto en filtered_cloud
        const float max_distance = 30.0f;
        
        for (size_t i = 0; i < final_colored_cloud->points.size(); ++i) {
            const auto& point = final_colored_cloud->points[i];
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance <= max_distance) {
                filtered_cloud->points.push_back(point);
                
                // Usar el mapa de índices originales para encontrar el tree_id
                int original_idx = colored_cloud_original_indices[i];
                auto it = point_to_tree_id.find(original_idx);
                int tree_id = (it != point_to_tree_id.end()) ? it->second : -1;
                filtered_tree_ids.push_back(tree_id);
            }
        }
        filtered_cloud->width = static_cast<uint32_t>(filtered_cloud->points.size());
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = false;


        
        // Eliminar suelo y vallas antes de guardar
        std::cout << "\n--- ELIMINANDO SUELO Y VALLAS ---" << std::endl;
        auto cloud_no_ground = removeGroundPlane(filtered_cloud);
        auto cloud_final = removeVerticalFences(cloud_no_ground);
        
        // Calcular centroides de árboles después del filtrado a 30 metros
        std::cout << "\n--- CALCULANDO CENTROIDES DE ÁRBOLES ---" << std::endl;
        int max_tree_id = -1;
        for (int tree_id : filtered_tree_ids) {
            if (tree_id > max_tree_id) {
                max_tree_id = tree_id;
            }
        }
        
        std::vector<TreeCentroid> centroids = calculateTreeCentroids(filtered_cloud, filtered_tree_ids, max_tree_id);
        
        // Imprimir centroides
        std::cout << "Centroides de árboles encontrados: " << centroids.size() << std::endl;
        for (const auto& c : centroids) {
            std::cout << "  Arbol " << c.tree_id << ": (" << c.x << ", " << c.y << ", " << c.z << ") "
                      << "con " << c.num_points << " puntos" << std::endl;
        }
        
        // Guardar centroides en archivo
        std::string centroid_output_file = output_dir + "/centroides/" + timestamp + ".txt";
        saveCentroidsToFile(centroid_output_file, centroids);
        
        // Guardar archivos
        pcl::io::savePCDFileBinary(output_dir + "/nubes_coloreadas/" + timestamp + "_coloreada.pcd", *cloud_final);
        
        cv::Mat image_with_bboxes = image.clone();
        drawBoundingBoxes(image_with_bboxes, bboxes);
        cv::imwrite(output_dir + "/imagenes_con_bbox/" + timestamp + "_con_bbox.png", image_with_bboxes);
        
        // Resumen
        std::cout << "\nResultados:" << std::endl;
        std::cout << "  Puntos totales: " << cloud->size() << std::endl;
        std::cout << "  Puntos filtrados (30m): " << filtered_cloud->size() << std::endl;
        std::cout << "  Puntos finales guardados: " << cloud_final->size() << std::endl;
        std::cout << "  Archivos guardados en: " << output_dir << std::endl;
    }
    
    std::cout << "\n=== PROCESAMIENTO COMPLETADO ===" << std::endl;
    return 0;
}
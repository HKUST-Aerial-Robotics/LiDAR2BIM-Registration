#ifndef CLOUD_DIST_H
#define CLOUD_DIST_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <omp.h>

class CloudDist
{
public:
    // Compute Chamfer distance between two point clouds with truncation
    static float chamferDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
                                 float truncationDistance);

    // Compute overlap between two point clouds
    static float overlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
                         float d = 1.0f);

    // Compute entropy-based distance between two point clouds
    static float entropyDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
                                 float radius, float epsilon = 1e-6);

private:
    // Helper function to compute squared Euclidean distance between two points
    static float squaredEuclideanDistance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

    // Helper function to compute sample covariance matrix for a point
    static Eigen::Matrix3f computeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointXYZ &point, float radius);

    // Helper function to compute sample covariance matrix for a point using a precomputed kdtree
    static Eigen::Matrix3f computeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                                                   const pcl::PointXYZ &point, 
                                                   float radius, 
                                                   pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree);
};

float CloudDist::squaredEuclideanDistance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

Eigen::Matrix3f CloudDist::computeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointXYZ &point, float radius)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 4)
    {
        Eigen::Matrix3f covarianceMatrix = Eigen::Matrix3f::Zero();
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();

        for (const auto &idx : pointIdxRadiusSearch)
        {
            Eigen::Vector3f pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
            mean += pt;
        }
        mean /= pointIdxRadiusSearch.size();

        for (const auto &idx : pointIdxRadiusSearch)
        {
            Eigen::Vector3f pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
            covarianceMatrix += (pt - mean) * (pt - mean).transpose();
        }
        covarianceMatrix /= pointIdxRadiusSearch.size();

        return covarianceMatrix;
    }
    return Eigen::Matrix3f::Zero();
}

Eigen::Matrix3f CloudDist::computeCovarianceMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                                                   const pcl::PointXYZ &point, 
                                                   float radius, 
                                                   pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 4)
    {
        Eigen::Matrix3f covarianceMatrix = Eigen::Matrix3f::Zero();
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();

        for (const auto &idx : pointIdxRadiusSearch)
        {
            Eigen::Vector3f pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
            mean += pt;
        }
        mean /= pointIdxRadiusSearch.size();

        for (const auto &idx : pointIdxRadiusSearch)
        {
            Eigen::Vector3f pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
            covarianceMatrix += (pt - mean) * (pt - mean).transpose();
        }
        covarianceMatrix /= pointIdxRadiusSearch.size();

        return covarianceMatrix;
    }
    return Eigen::Matrix3f::Zero();
}

float CloudDist::chamferDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
                                 float truncationDistance)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);

    float dist1 = 0.0f;
#pragma omp parallel for reduction(+ : dist1)
    for (size_t i = 0; i < cloud1->points.size(); ++i)
    {
        const auto &p1 = cloud1->points[i];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(p1, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            float distance = std::sqrt(pointNKNSquaredDistance[0]);
            dist1 += std::min(distance, truncationDistance);
        }
    }
    dist1 /= cloud1->points.size();

    kdtree.setInputCloud(cloud1);

    float dist2 = 0.0f;
#pragma omp parallel for reduction(+ : dist2)
    for (size_t i = 0; i < cloud2->points.size(); ++i)
    {
        const auto &p2 = cloud2->points[i];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(p2, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            float distance = std::sqrt(pointNKNSquaredDistance[0]);
            dist2 += std::min(distance, truncationDistance);
        }
    }
    dist2 /= cloud2->points.size();

    return (dist1 + dist2) / 2.0f;
}

float CloudDist::overlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, float d)
{

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);

    int recallCount = 0;
#pragma omp parallel for reduction(+ : recallCount)
    for (size_t i = 0; i < cloud1->points.size(); ++i)
    {
        const auto &p1 = cloud1->points[i];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(p1, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            float distance = std::sqrt(pointNKNSquaredDistance[0]);
            if (distance < d)
            {
                recallCount++;
            }
        }
    }

    return static_cast<float>(recallCount) / cloud1->points.size();
}

float CloudDist::entropyDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
                                 float radius, float epsilon)
{
    auto computeEntropy = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree)
    {
        float entropy = 0.0f;
#pragma omp parallel for reduction(+ : entropy)
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            const auto &point = cloud->points[i];
            Eigen::Matrix3f covarianceMatrix = computeCovarianceMatrix(cloud, point, radius, kdtree);
            float det = covarianceMatrix.determinant();
            entropy += 0.5f * std::log(2 * M_PI * M_E * (det + epsilon));
        }
        return entropy;
    };

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1, kdtree2, kdtreeJoint;
    kdtree1.setInputCloud(cloud1);
    kdtree2.setInputCloud(cloud2);

    float Ha = computeEntropy(cloud1, kdtree1);
    float Hb = computeEntropy(cloud2, kdtree2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr jointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *jointCloud = *cloud1 + *cloud2;
    kdtreeJoint.setInputCloud(jointCloud);
    float Hj = computeEntropy(jointCloud, kdtreeJoint);

    float Hsep = (Ha + Hb) / (cloud1->points.size() + cloud2->points.size());
    float Hjoint = Hj / jointCloud->points.size();

    return Hjoint - Hsep;
}

#endif // CLOUD_DIST_H
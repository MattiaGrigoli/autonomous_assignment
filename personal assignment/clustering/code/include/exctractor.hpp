#ifndef EXCTRACTOR
#define EXCTRACTOR

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/search/octree.h>

// flags
#define DEBUG // for debug purposes
// #define VOXEL_GRID
#define APPROXIMATE_VOXEL_GRID
// #define DATASET_1
#define DATASET_2
// #define KDTREE
#define OCTREE

using namespace lidar_obstacle_detection;

class PointCloudProcessor 
{
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f;
#ifdef VOXEL_GRID
    pcl::VoxelGrid<pcl::PointXYZ> sor;
#endif
#ifdef APPROXIMATE_VOXEL_GRID
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
#endif

    pcl::CropBox<pcl::PointXYZ> cb;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr coefficients; //the resultant model coefficients
    pcl::PointIndices::Ptr inliers;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
#ifdef KDTREE
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
#endif
#ifdef OCTREE
    pcl::search::Octree<pcl::PointXYZ>::Ptr tree;
#endif
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchCluster;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};
    // std::vector<int> pointIdxNKNSearch(1);
    // std::vector<float> pointNKNSquaredDistance(1);
    // pcl::PointXYZ origin(0.0f, 0.0f, 0.0f);
public:
    PointCloudProcessor();
    
    void ProcessAndRenderPointCloud(lidar_obstacle_detection::Renderer& renderer, 
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);
};

#endif
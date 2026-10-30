#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include <boost/filesystem.hpp>
#include "../include/exctractor.hpp"
namespace fs = boost::filesystem;
#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

#ifdef DEBUG
    #define LOG_DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
    #define START_TIMER(tag) \
        std::chrono::steady_clock::time_point start_##tag = std::chrono::steady_clock::now()

    // Calcola e stampa la differenza di tempo
    #define STOP_TIMER(tag, name) \
        do { \
            std::chrono::steady_clock::time_point end_##tag = std::chrono::steady_clock::now(); \
            double duration_##tag = std::chrono::duration<double, std::milli>(end_##tag - start_##tag).count(); \
            std::cout << "[TIMER] " << name << ": " << duration_##tag << " ms" << std::endl; \
        } while(0)
#else
    #define LOG_DEBUG(msg) ((void)0)
    #define START_TIMER(tag) ((void)0)
    #define STOP_TIMER(tag, name) ((void)0)
#endif

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
	my_visited_set_t visited{};                //already visited points
	std::vector<pcl::PointIndices> clusters;   //vector of PointIndices that will contain all the clusters
    std::vector<int> cluster;                  //vector of int that is used to store the points that the function proximity will give me back
	//for every point of the cloud
    //  if the point has not been visited (use the function called "find")
    //    find clusters using the proximity function
    //
    //    if we have more clusters than the minimum
    //      Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)   
    //    end if
    //  end if
    //end for
	return clusters;	
}

void 
PointCloudProcessor::ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // TODO: 1) Downsample the dataset 
    START_TIMER(downsample);
    LOG_DEBUG("starting filtering");
    LOG_DEBUG("PointCloud before filtering: " << cloud->width * cloud->height 
    << " data points (" << pcl::getFieldsList (*cloud) << ").");
    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
#ifdef DATASET_1
    sor.setLeafSize (0.3f, 0.3f, 0.3f);
#endif
#ifdef DATASET_2
    sor.setLeafSize (0.3f, 0.3f, 0.3f);
#endif
    sor.filter (*cloud_filtered);
    LOG_DEBUG("PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").");
    STOP_TIMER(downsample, "1. downsample");

    // 2) here we crop the points that are far away from us, in which we are not interested
    // pcl::CropBox<pcl::PointXYZ> cb(true);
    START_TIMER(faraway);
    cb.setInputCloud(cloud_filtered);
#ifdef DATASET_1
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
#endif
#ifdef DATASET_2
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
#endif
    cb.filter(*cloud_filtered); 
    STOP_TIMER(faraway, "2. removing far points");

    // TODO: 4) iterate over the filtered cloud, segment and remove the planar inliers 
    START_TIMER(segmentation);
    int i = 0, nr_points = (int) cloud_filtered->size ();
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        LOG_DEBUG("Could not estimate a planar model for the given dataset.");
    }

    // Extract the planar inliers from the input cloud
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane); // floor
    LOG_DEBUG("PointCloud representing the planar component: " << cloud_plane->size () << " data points.");

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
    STOP_TIMER(segmentation, "3. segmentation");


    #ifdef USE_PCL_LIBRARY

        //PCL functions
        //HERE 6)

        // TODO: 5) Create the KDTree and the vector of PointIndices
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        START_TIMER(clustering);
        tree->setInputCloud (cloud_filtered);
        // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

        // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
        std::vector<pcl::PointIndices> cluster_indices;
#ifdef DATASET_1
        ec.setClusterTolerance (0.35);
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (25000);
#endif
#ifdef DATASET_2
        ec.setClusterTolerance (0.35);
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (500);
#endif
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);
        STOP_TIMER(clustering, "4. clustering");
    #else
        // Optional assignment
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);
        cluster_indices = euclideanCluster(cloud_filtered, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
    #endif

    // std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};


    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    **/
    START_TIMER(rendering);
    int j = 0;
    int clusterId = 0;
    renderer.RenderPointCloud(cloud_plane, "floor", colors[1]);
    if(renderer.getLidarStatus())
    {
        renderer.RenderPointCloud(cloud,"originalCloud",colors[2]);
    }
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    pcl::PointXYZ origin(0.0f, 0.0f, 0.0f);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_filtered)[*pit]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // renderer.RenderPointCloud(cloud,"originalCloud"+std::to_string(clusterId),colors[2]);
        // TODO: 7) render the cluster and plane without rendering the original cloud 
        //<-- here
        renderer.RenderPointCloud(cloud_cluster, "cluster"+std::to_string(clusterId), colors[3]);
        //----------

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        searchCluster->setInputCloud(cloud_cluster);
        searchCluster->nearestKSearch(origin, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        pcl::PointXYZ closest_point = cloud_cluster->points[pointIdxNKNSearch[0]];
        pcl::PointCloud<pcl::PointXYZ>::Ptr single_point_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        single_point_cloud->push_back(closest_point);
        renderer.RenderRays(Eigen::Vector3f{0.0,0.0,0.0}, single_point_cloud); // closest point of the cluster to the origin/ closest point of the obstacle

        //TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        Box box{minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};
        // renderer.RenderRays(Eigen::Vector3f{0.0,0.0,0.0}, minPt);
        //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
        //please take a look at the function RenderBox to see how to color the box
        float min_distance = std::sqrt(pointNKNSquaredDistance[0]);
        if(cloud_cluster->size () > 100)
        {
            if (min_distance < 5)
            {
                renderer.RenderBox(box, j, colors[0]);   
            }else
            {
                renderer.RenderBox(box, j, colors[4]); 
            }
        }else
        {
            renderer.RenderBox(box, j, Color(0.906, 0, 1)); 
        }
        

        ++clusterId;
        j++;
    }
    STOP_TIMER(rendering, "5. rendering frame");

}

PointCloudProcessor::PointCloudProcessor()
    : cb(true)
{
    // Allocazione unica all'avvio del programma
    cloud_filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_plane    = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_f        = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
#ifdef KDTREE
    tree           = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
#endif
#ifdef OCTREE
    tree           = pcl::make_shared<pcl::search::Octree<pcl::PointXYZ>>(0.35f); //0.35 = resolution/tolerance
#endif
    coefficients   = pcl::make_shared<pcl::ModelCoefficients>();
    inliers        = pcl::make_shared<pcl::PointIndices>();
    searchCluster  = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

    // Pre-configurazione parametri fissi
    // TODO: 3) Segmentation and apply RANSAC
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
#ifdef DATASET_1
    seg.setMethodType(pcl::SAC_MSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.25);
#endif
#ifdef DATASET_2
    seg.setMethodType(pcl::SAC_MSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.25);
#endif
}


int main(int argc, char* argv[])
{
    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

#ifdef DATASET_1
    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/mnt/500GB SSD/uniproject/autonomous_driving/autonomous_assignment/personal assignment/clustering/dataset_1"},
                                                boost::filesystem::directory_iterator{});
#endif
#ifdef DATASET_2
    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/mnt/500GB SSD/uniproject/autonomous_driving/autonomous_assignment/personal assignment/clustering/dataset_2"},
                                                boost::filesystem::directory_iterator{});
#endif

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    PointCloudProcessor processor;

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader; // need to optimize?
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        processor.ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
        << input_cloud->points.size() << " data points from " << streamIterator->string() <<  " plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
    }
}

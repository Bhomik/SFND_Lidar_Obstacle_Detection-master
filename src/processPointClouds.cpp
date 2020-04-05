// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}

  /** \brief @b ExtractIndices extracts a set of indices from a point cloud.
    * <br>
    * Usage examples:
    * \code
    * pcl::ExtractIndices<PointType> filter;
    * filter.setInputCloud (cloud_in);
    * filter.setIndices (indices_in);
    * // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
    * filter.filter (*cloud_out);
    * // Retrieve indices to all points in cloud_in except those referenced by indices_in:
    * filter.setNegative (true);
    * filter.filter (*indices_out);
    * // The resulting cloud_out is identical to cloud_in, but all points referenced by indices_in are made NaN:
    * filter.setNegative (true);
    * filter.setKeepOrganized (true);
    * filter.filter (*cloud_out);
    * \endcode
    * \note Does not inherently remove NaNs from results, hence the \a extract_removed_indices_ system is not used.
    * \author Radu Bogdan Rusu
    * \ingroup filters
    */

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    
    typename pcl::PointCloud<PointT>::Ptr segmented_plane_cloud(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT> ());
    typename pcl::ExtractIndices<PointT> filter;

    filter.setInputCloud(cloud);
    filter.setIndices(inliers);
    filter.filter(*segmented_plane_cloud);

    filter.setNegative(true);
    filter.filter(*obstacle_cloud);
    std::cout << "total points in segmented_plane_cloud : " << segmented_plane_cloud->points.size() << std::endl;
    std::cout << "total points in obs cloud : " << obstacle_cloud->points.size() << std::endl;

    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(segmented_plane_cloud, obstacle_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold,
                                                                                                                                 pcl::visualization::PCLVisualizer::Ptr&viewer)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();


    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    typename pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);

    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);

    seg.segment(*inliers, *coefficients);

    std::cout << "Model inliers size is : " << inliers->indices.size() << std::endl;

    std::cout << "model coeffients are : " <<
                 coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

//    std::cout << "Model inliers size is : " << inliers->indices.size() << std::endl;

    for (std::size_t i=0; i < inliers->indices.size(); ++i)
    {
        std::cout << inliers->indices[i] << " " << cloud->points[inliers->indices[i]].x
                                                << " " <<
                                                 cloud->points[inliers->indices[i]].y
                                                  << " " <<
                                                  cloud->points[inliers->indices[i]].z
                                                  << std::endl;
    }

    viewer->addPlane(*coefficients, "plane");

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating a kdtree object for the search method of extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // vector of pointIndices
    std::vector<pcl::PointIndices> cluster_indices;
    // cluster_indices[0] contains all indices of the first cluster in our point cloud.

    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    std::cout << "total number of clusters are : " << cluster_indices.size() << std::endl;

    for (int i=0; i<cluster_indices.size();i++)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT> ());

    	for (int j = 0 ; j < cluster_indices[i].indices.size(); j++)
    	{
    		cloud_cluster->points.push_back(cloud->points[cluster_indices[i].indices[j]]);
    	}

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

    	clusters.push_back(cloud_cluster);

    }


    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

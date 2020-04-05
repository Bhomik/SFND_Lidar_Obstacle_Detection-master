/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = i+scatter*rx;
        point.y = i+scatter*ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = 5*rx;
        point.y = 5*ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0); // (x, y, z coordinate of cam)
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    std::unordered_set<int> candidates;
    std::unordered_set<int> selectedNum;
    srand(time(NULL));

    // To generate random number between 1 and 20
    int lowest=1;
    int highest=cloud->points.size();
    int range=(highest-lowest)+1;

    std::cout << "total points in cloud : " << cloud->points.size() << std::endl;

//    for (std::size_t i = 0 ; i < cloud->points.size(); i++ )
//    {
//        std::cout << cloud->points[i] << std::endl;
//    }

    int num_inliers = 0 ;
    int max_num_inliers = 0;

    while(maxIterations--)
    {

//        std::cout << "maxiter : " << maxIterations << std::endl;
//        std::cout << "selectedNum size : " << selectedNum.size() << std::endl;

        while(1)
        {
            selectedNum.insert(lowest + rand() % range);
            if (selectedNum.size() ==2)
                break;
        }
        std::unordered_set<int> :: iterator itr;

        std::vector<float> line_end_points_x, line_end_points_y;
        // iterator itr loops from begin() till end()
        for (itr = selectedNum.begin(); itr != selectedNum.end(); itr++)
        {
            line_end_points_x.push_back(cloud->points[*itr].x);
            line_end_points_y.push_back(cloud->points[*itr].y);
        }

        float A = line_end_points_y[0]-line_end_points_y[1];
        float B = line_end_points_x[1]-line_end_points_x[0];
        float C = (line_end_points_x[0]*line_end_points_y[1]) - (line_end_points_x[1]*line_end_points_y[0]);

        float denom = std::sqrt(std::pow(A,2) + std::pow(B,2));

        for (std::size_t i = 0; i < cloud->points.size(); i++)
        {
            if (denom != 0.0)
            {
                float distance = std::fabs(A*cloud->points[i].x + B*cloud->points[i].y + C) / (float)denom;
                std::cout << "distance : " << distance << std::endl;

                if (distance < distanceTol)
                {
                    num_inliers++;
                    candidates.insert(i);
                }

            }
        }

        std::cout << "num inliers " << num_inliers << std::endl;

        if (num_inliers > max_num_inliers)
        {
            max_num_inliers = num_inliers;

            inliersResult.erase(inliersResult.begin(), inliersResult.end());
//            for (itr = selectedNum.begin(); itr != selectedNum.end(); itr++)
//            {
//                inliersResult.insert(*itr);
//            }
            for (itr = candidates.begin(); itr != candidates.end(); itr++)
            {
                inliersResult.insert(*itr);
            }
        }


        selectedNum.erase(selectedNum.begin(), selectedNum.end());
        candidates.erase(candidates.begin(), candidates.end());
        num_inliers = 0;

    }

    std::cout << "max inliers : " << max_num_inliers << std::endl;
    std::cout << "size of inliers result is : " << inliersResult.size() << std::endl;


    return inliersResult;

}

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();


    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 10, 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
    else
    {
        renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }

}

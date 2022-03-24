/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "cluster.h"
#include "kdtree.h"
#include "processPointClouds/processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds/processPointClouds.cpp"

#include <unordered_set>



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer);

std::unordered_set<int> newRansacPlane(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    int maxIterations,
    float distanceTol);

void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud);



int main(int argc, char** argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer{
        new pcl::visualization::PCLVisualizer{ "3D Viewer" }
    };
    CameraAngle setAngle{ XY };
    initCamera(setAngle, viewer);


    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI{ new ProcessPointClouds<pcl::PointXYZI>{} };
    std::vector<boost::filesystem::path> stream{ pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1") };
    auto streamIterator{ stream.begin() };
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    while (!viewer->wasStopped()) {

        // Clear viewer.
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process.
        inputCloudI = pointProcessorI->loadPcd(streamIterator->string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        ++streamIterator;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();


        // Controls the frame rate (by default it waits 1 time step).
        viewer->spinOnce();
    }

    delete pointProcessorI;

    return 0;
}







void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // Set camera position and angle
    viewer->initCameraParameters();

    // distance away in meters
    int distance{ 16 };

    switch (setAngle) {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}






std::unordered_set<int> newRansacPlane(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    int maxIterations,
    float distanceTol) {

    
    auto startTime{ std::chrono::steady_clock::now() };
    
    
    srand(time(nullptr));

    std::unordered_set<int> inliersResult;

    while (maxIterations--) {

        // Randomly sample subset and fit plane.

        // Randomly pick three points.
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(rand() % (cloud->points.size()));


        float x1{}, y1{}, z1{}, x2{}, y2{}, z2{}, x3{}, y3{}, z3{};

        auto itr{ inliers.begin() };
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        ++itr;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        ++itr;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // Ax + By + Cz + D = 0
        float a{ (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1) };
        float b{ (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1) };
        float c{ (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1) };
        float d{ -(a * x1 + b * y1 + c * z1) };



        for (int index{ 0 }; index < cloud->points.size(); ++index) {
            if (inliers.count(index) != 0)	    continue;

            pcl::PointXYZI point{ cloud->points[index] };
            float x4{ point.x };
            float y4{ point.y };
            float z4{ point.z };

            // Measure distance between every point and fitted plane.
            float distance{ fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c) };

            // If distance is smaller than threshold, then count it as inlier.
            if (distance <= distanceTol)	inliers.insert(index);
        }

        // Return indices of inliers from fitted line with most inliers.
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }


    auto endTime{ std::chrono::steady_clock::now() };
    auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds." << std::endl;


    return inliersResult;
}






void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {


    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------


    inputCloud = pointProcessor->FilterCloud(inputCloud, 0.3, Eigen::Vector4f{ -10.0f, -5.0f, -2.0f, 1.0f }, Eigen::Vector4f{ 30.0f, 8.0f, 1.0f, 1.0f });

    std::unordered_set<int> inliers{ newRansacPlane(inputCloud, 25, 0.3f) };

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers{  new pcl::PointCloud<pcl::PointXYZI>{} };
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers{ new pcl::PointCloud<pcl::PointXYZI>{} };

    for (int index{ 0 }; index < inputCloud->points.size(); ++index) {
        pcl::PointXYZI point{ inputCloud->points[index] };

        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers,  "inliers",  Color{ 0.0f, 1.0f, 0.0f });
        renderPointCloud(viewer, cloudOutliers, "outliers", Color{ 1.0f, 0.0f, 0.0f });
    }
    else {
        std::cout << "Couldn't estimate a planar model for the given dataset." << std::endl;
        renderPointCloud(viewer, inputCloud, "data");
    }

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud{ 
        cloudInliers, cloudOutliers 
    };



    //pcl::PointXYZI aaa;


    std::vector<std::vector<float>> vectorClusters;
    
    for (int index{ 0 }; index < segmentCloud.first->points.size(); ++index) {
        std::vector<float> temp;
        temp.push_back(segmentCloud.first->points[index].x);
        temp.push_back(segmentCloud.first->points[index].y);
        temp.push_back(segmentCloud.first->points[index].z);

        vectorClusters.push_back(temp);
    }

    KdTree* ptr_tree{ new KdTree{} };
    for (int i{ 0 }; i < vectorClusters.size(); ++i)
        ptr_tree->insert(vectorClusters[i], i);



    //
    // 
    // This line has problem (Memory leak and Too many recursion function)!!!!!
    //  
    //
    std::vector<pcl::Indices> clusterIndices{ euclideanCluster(vectorClusters, ptr_tree, 3.0) };




    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;

    for (std::vector<int> getIndices : clusterIndices) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster{ new pcl::PointCloud<pcl::PointXYZI> {} };

        for (int index : getIndices) {
            cloudCluster->points.push_back(segmentCloud.first->points[index]);
        }

        cloudClusters.push_back(cloudCluster);
    }

    delete ptr_tree;





    int clusterId{ 0 };
    std::vector<Color> colors{ Color{ 1, 0, 0 }, Color{ 1, 1, 0 }, Color{ 0, 0, 1 } };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {

        std::cout << "cluster size : ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box{ pointProcessor->BoundingBox(cluster) };
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}







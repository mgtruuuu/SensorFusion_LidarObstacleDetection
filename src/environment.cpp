/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "cluster.h"
#include "kdtree.h"
#include "segment.h"
#include "processPointClouds/processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds/processPointClouds.cpp"

#include <filesystem>
#include <unordered_set>


void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer);

void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud);



int main(int argc, char** argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer{
        new pcl::visualization::PCLVisualizer{ "3D Viewer" }
    };
    //setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
    CameraAngle setAngle{ CameraAngle::XY };
    initCamera(setAngle, viewer);


    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI{ new ProcessPointClouds<pcl::PointXYZI> };
    std::vector<std::filesystem::path> stream{ pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1") };
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

    viewer->setBackgroundColor(.0, .0, .0);

    // Set camera position and angle
    viewer->initCameraParameters();

    // distance away in meters
    int distance{ 16 };

    switch (setAngle) {
    case CameraAngle::XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1.0, 1.0, .0);
        break;
    case CameraAngle::TopDown:
        viewer->setCameraPosition(.0, .0, distance, 1.0, .0, 1.0);
        break;
    case CameraAngle::Side:
        viewer->setCameraPosition(.0, -distance, .0, .0, .0, 1.0);
        break;
    case CameraAngle::FPS:
        viewer->setCameraPosition(-10.0, .0, .0, .0, .0, 1.0);
    }

    if (setAngle != CameraAngle::FPS)
        viewer->addCoordinateSystem(1.0);
}













void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor,
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {


    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------


    using type_pcl = pcl::PointCloud<pcl::PointXYZI>::Ptr;


    inputCloud = pointProcessor->FilterCloud(inputCloud, 0.3,
        Eigen::Vector4f{ -10.0f, -5.0f, -2.0f, 1.0f }, Eigen::Vector4f{ 30.0f, 8.0f, 1.0f, 1.0f });

    type_pcl planeCloud{ new pcl::PointCloud<pcl::PointXYZI> };
    type_pcl obstCloud{  new pcl::PointCloud<pcl::PointXYZI> };

    std::unordered_set<int> inliers{ newRansacPlane(inputCloud, 25, 0.3f) };


    // obstCloud, planeCloud
    std::pair<type_pcl, type_pcl> segmentCloud{ 
        segmentPointClouds(inliers, inputCloud, planeCloud, obstCloud) 
    };


    if (inliers.size()) {
        renderPointCloud(viewer, planeCloud, "inliers", Color{ 0.0f, 1.0f, 0.0f });
        //renderPointCloud(viewer, cloudOutliers, "outliers", Color{ 0.0f, 0.0f, 1.0f });
    }
    else {
        std::cout << "Couldn't estimate a planar model for the given dataset." << std::endl;
        renderPointCloud(viewer, inputCloud, "data");
    }




    std::vector<std::vector<float>> vectorClusters;
    
    for (int index{ 0 }; index < segmentCloud.first->points.size(); ++index) {
        
        std::vector<float> temp;
        temp.push_back(segmentCloud.first->points[index].x);
        temp.push_back(segmentCloud.first->points[index].y);
        temp.push_back(segmentCloud.first->points[index].z);

        vectorClusters.push_back(temp);
    }

    KdTree* ptr_tree{ new KdTree };
    for (int i{ 0 }; i < vectorClusters.size(); ++i)
        ptr_tree->insert(vectorClusters[i], i);

    //                                                                                      3.0
    std::vector<std::vector<int>> clusterIndices{ euclideanCluster(vectorClusters, ptr_tree, 1.0) };


    std::vector<type_pcl> cloudClusters;

    for (std::vector<int> getIndices : clusterIndices) {

        type_pcl cloudCluster{ new pcl::PointCloud<pcl::PointXYZI> };

        for (auto index : getIndices) {
            cloudCluster->points.push_back(segmentCloud.first->points[index]);
        }

        cloudClusters.push_back(cloudCluster);
    }

    delete ptr_tree;





    int clusterId{ 0 };
    std::vector<Color> colors{ Color{ 1.0f, 1.0f, .0f }, Color{ .0f, 1.0f, 1.0f }, Color{ 1.0f, .0f, 1.0f } };

    for (type_pcl cluster : cloudClusters) {

        std::cout << "cluster size : ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box{ pointProcessor->BoundingBox(cluster) };
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}







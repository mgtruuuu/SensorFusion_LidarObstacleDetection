#ifndef SEGMENT_H_
#define SEGMENT_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <unordered_set>




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
    std::cout << "newRansacPlane took " << elapsedTime.count() << " milliseconds." << std::endl;


    return inliersResult;
}


std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentPointClouds(
    std::unordered_set<int>& inliers,
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud) {

    for (int index{ 0 }; index < inputCloud->points.size(); ++index) {
        pcl::PointXYZI point{ inputCloud->points[index] };

        if (inliers.count(index))
            planeCloud->points.push_back(point);
        else
            obstCloud->points.push_back(point);
    }


    return std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>{
        obstCloud, planeCloud
    };
}





#endif
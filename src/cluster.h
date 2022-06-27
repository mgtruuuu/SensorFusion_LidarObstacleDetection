#ifndef CLUSTER_H_
#define CLUSTER_H_

#include "render/render.h"
#include "render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"


pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);



void clusterHelper(
    const int indice,
    const std::vector<std::vector<float>>& points,
    const float distanceTol,
    KdTree* const ptr_tree,
    std::vector<int>& cluster,
    std::vector<bool>& processed);

std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points,
    KdTree* const ptr_tree, 
    const float distanceTol);











pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom) {

    pcl::visualization::PCLVisualizer::Ptr viewer{ new pcl::visualization::PCLVisualizer{ "2D Viewer" } };
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);

    //viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
    viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, window.z_min, window.z_max, 1, 1, 1, "window");

    return viewer;
}



void clusterHelper(
    const int indice,
    const std::vector<std::vector<float>>& points,
    const float distanceTol,
    KdTree* const ptr_tree,
    std::vector<int>& cluster,
    std::vector<bool>& processed) {

    processed[indice] = true;
    cluster.push_back(indice);

    std::vector<int> nearest{ ptr_tree->search(points[indice], distanceTol) };

    for (const int id : nearest)
        if (processed[id] == false)
            clusterHelper(id, points, distanceTol, ptr_tree, cluster, processed);
}


std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points,
    KdTree* const ptr_tree, 
    const float distanceTol) {

    // TODO: Fill out this function to return list of indices for each cluster.
    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(), false);

    int i{ 0 };
    while (i < points.size()) {
        if (processed[i]) {
            ++i;
            continue;
        }

        std::vector<int> cluster;
        clusterHelper(i, points, distanceTol, ptr_tree, cluster, processed);
        clusters.push_back(cluster);
        ++i;
    }

    return clusters;
}











#endif



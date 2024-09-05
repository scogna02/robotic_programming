#pragma once

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <utility>

namespace fs = boost::filesystem;

class GridMap {
public:
    GridMap();

    void loadImage(const fs::path& folder);
    void setStartGoal(const std::pair<int, int>& start, const std::pair<int, int>& goal);
    void setOccupancy(const nav_msgs::OccupancyGrid& grid);
    void computeDistanceMap();
    std::vector<std::pair<int, int>> findPath();

private:
    int rows;
    int cols;
    std::vector<uint8_t> gridMapArray;
    Eigen::MatrixXf distanceMap;
    double minDist, maxDist;
    std::pair<int, int> start;
    std::pair<int, int> goal;
    nav_msgs::OccupancyGrid gridmapocc;

    fs::path findImage(const fs::path& folder);
    void processImage(const cv::Mat& image);
    bool checkValidStartAndGoal() const;
    std::vector<std::pair<int, int>> reconstructPath(const std::vector<std::vector<std::pair<int, int>>>& parent) const;

    inline int actionCost(int x, int y) const;
    inline bool isValid(int x, int y) const;
    inline int heuristic(int x, int y) const;
};
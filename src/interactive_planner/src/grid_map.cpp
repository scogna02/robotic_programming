#include "/home/gabrielescognamiglio/catkin_ws/src/interactive_planner/include/interactive_planner/grid_map.h"
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <limits>
#include <algorithm>

namespace fs = boost::filesystem;

// Constructor for GridMap class
GridMap::GridMap() : rows(0), cols(0), minDist(0), maxDist(0) {
    std::cout << "Initializing the GridMap..." << std::endl;
}

// Load an image from a specified folder
void GridMap::loadImage(const fs::path& folder) {
    // Find an image file in the given folder
    auto imagePath = findImage(folder);
    if (imagePath.empty()) {
        throw std::runtime_error("No image found in the specified folder");
    }

    // Load the image in grayscale
    cv::Mat loadedImage = cv::imread(imagePath.string(), cv::IMREAD_GRAYSCALE);
    if (loadedImage.empty()) {
        throw std::runtime_error("Error loading the image");
    }

    std::cout << "Image loaded successfully: " << imagePath << std::endl;
    processImage(loadedImage);
}

// Find an image file in the given folder
fs::path GridMap::findImage(const fs::path& folder) {
    const std::vector<std::string> validExtensions = {".png", ".jpeg", ".jpg", ".bmp", ".gif"};
    for (const auto& entry : fs::directory_iterator(folder)) {
        if (fs::is_regular_file(entry.path())) {
            auto ext = entry.path().extension().string();
            if (std::find(validExtensions.begin(), validExtensions.end(), ext) != validExtensions.end()) {
                return entry.path();
            }
        }
    }
    return {};
}

// Process the loaded image and convert it to a binary grid
void GridMap::processImage(const cv::Mat& image) {
    rows = image.rows;
    cols = image.cols;
    gridMapArray.resize(rows * cols);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // Threshold the image: white (>230) becomes 255, everything else becomes 0
            gridMapArray[i * cols + j] = (image.at<uchar>(i, j) > 230) ? 255 : 0;
        }
    }
}

// Set the start and goal positions for path planning
void GridMap::setStartGoal(const std::pair<int, int>& start, const std::pair<int, int>& goal) {
    this->start = start;
    this->goal = goal;
    std::cout << "Start: (" << start.first << ", " << start.second << ")" << std::endl;
    std::cout << "Goal: (" << goal.first << ", " << goal.second << ")" << std::endl;
}

// Set the occupancy grid from a ROS OccupancyGrid message
void GridMap::setOccupancy(const nav_msgs::OccupancyGrid& grid) { 
    gridmapocc = grid;
    rows = grid.info.height;
    cols = grid.info.width;
}

// Compute the distance map using OpenCV's distanceTransform
void GridMap::computeDistanceMap() {
    cv::Mat gridMapCV(rows, cols, CV_8UC1);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            gridMapCV.at<uchar>(i, j) = gridmapocc.data[i * cols + j];
        }
    }

    cv::Mat distance;
    cv::distanceTransform(gridMapCV, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::minMaxLoc(distance, &minDist, &maxDist);

    distanceMap = Eigen::MatrixXf(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            distanceMap(i, j) = distance.at<float>(i, j);
        }
    }
}

// Calculate the cost of moving to a cell based on its distance from obstacles
inline int GridMap::actionCost(int x, int y) const {
    return std::exp(maxDist - distanceMap(x, y)) + 1;
}

// Check if a given cell is valid (within bounds and not occupied)
inline bool GridMap::isValid(int x, int y) const {
    return x >= 0 && x < cols && y >= 0 && y < rows && gridmapocc.data[y * cols + x] == 0;
}

// Calculate the heuristic cost (Manhattan distance) to the goal
inline int GridMap::heuristic(int x, int y) const {
    return std::abs(x - goal.first) + std::abs(y - goal.second);
}

// Check if both start and goal positions are valid
bool GridMap::checkValidStartAndGoal() const {
    auto checkPosition = [this](const std::pair<int, int>& pos, const char* name) {
        if (pos.first < 0 || pos.first >= cols || pos.second < 0 || pos.second >= rows ||
            gridmapocc.data[pos.second * cols + pos.first] == 100) {
            std::cerr << name << " position is not traversable or it is out of bounds." << std::endl;
            return false;
        }
        std::cout << name << " position is valid" << std::endl;
        return true;
    };

    return checkPosition(start, "Start") && checkPosition(goal, "Goal");
}

// Find a path from start to goal using A* algorithm
std::vector<std::pair<int, int>> GridMap::findPath() {
    if (!checkValidStartAndGoal()) {
        return {};
    }

    std::cout << "Starting findPath" << std::endl;

    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));
    std::vector<std::vector<int>> gScore(rows, std::vector<int>(cols, std::numeric_limits<int>::max()));
    
    auto compare = [&](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return gScore[a.second][a.first] + heuristic(a.first, a.second) >
               gScore[b.second][b.first] + heuristic(b.first, b.second);
    };
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, decltype(compare)> openSet(compare);

    gScore[start.second][start.first] = 0;
    openSet.push(start);

    while (!openSet.empty()) {
        auto current = openSet.top();
        openSet.pop();

        if (current == goal) {
            std::cout << "Path found" << std::endl;
            return reconstructPath(parent);
        }

        if (visited[current.second][current.first]) continue;
        visited[current.second][current.first] = true;

        for (const auto& direction : directions) {
            int nx = current.first + direction.first;
            int ny = current.second + direction.second;
            
            if (isValid(nx, ny)) {
                int tentative_gScore = gScore[current.second][current.first] + actionCost(nx, ny);
                if (tentative_gScore < gScore[ny][nx]) {
                    parent[ny][nx] = current;
                    gScore[ny][nx] = tentative_gScore;
                    openSet.push({nx, ny});
                }
            }
        }
    }

    std::cerr << "Path not found" << std::endl;
    return {};
}

// Reconstruct the path from start to goal using the parent information
std::vector<std::pair<int, int>> GridMap::reconstructPath(const std::vector<std::vector<std::pair<int, int>>>& parent) const {
    std::vector<std::pair<int, int>> path;
    auto current = goal;
    while (current != start) {
        path.push_back(current);
        current = parent[current.second][current.first];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

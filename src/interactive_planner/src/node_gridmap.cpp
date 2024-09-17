#include <ros/ros.h>
#include "/home/gabrielescognamiglio/catkin_ws/src/interactive_planner/include/interactive_planner/grid_map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <string>

// Enum to keep track of the current selection state
enum class SelectionState {
    WAITING_FOR_START,
    WAITING_FOR_GOAL
};

class GridMapNode {
public:
    // Constructor: initializes ROS node, publishers, subscribers, and parameters
    GridMapNode() : nh_("~"), selection_state_(SelectionState::WAITING_FOR_START) {
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1, true);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GridMapNode::goalCallback, this);

        nh_.param<std::string>("image_path", image_path_, "");
        nh_.param<double>("publish_rate", publish_rate_, 1.0);
    }

    // Initialize the node: load image, create occupancy grid, and compute distance map
    bool initialize() {
        if (!loadImage()) {
            return false;
        }
        occupancy_grid_ = imageToOccupancyGrid(image_);
        grid_map_.setOccupancy(occupancy_grid_);
        grid_map_.computeDistanceMap();
        return true;
    }

    // Main loop: publish map and path at specified rate
    void run() {
        ros::Rate rate(publish_rate_);
        while (ros::ok()) {
            publishMapAndPath();
            rate.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber goal_sub_;
    GridMap grid_map_;
    std::string image_path_;
    std::pair<int, int> start_;
    std::pair<int, int> goal_;
    cv::Mat image_;
    nav_msgs::OccupancyGrid occupancy_grid_;
    std::vector<std::pair<int, int>> path_points_;
    nav_msgs::Path path_message_;
    double publish_rate_;
    SelectionState selection_state_;

    // Load image from specified path
    bool loadImage() {
        image_ = cv::imread(image_path_, cv::IMREAD_GRAYSCALE);
        if (image_.empty()) {
            ROS_ERROR_STREAM("Failed to load image: " << image_path_);
            return false;
        }
        ROS_INFO("Image loaded successfully");
        return true;
    }

    // Convert OpenCV image to ROS OccupancyGrid message
    nav_msgs::OccupancyGrid imageToOccupancyGrid(const cv::Mat& image) {
        nav_msgs::OccupancyGrid grid;
        grid.header.frame_id = "map";
        grid.info.width = image.cols;
        grid.info.height = image.rows;
        grid.info.resolution = 1.0;
        grid.info.origin.position.x = 0.0;
        grid.info.origin.position.y = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.reserve(image.total());

        for (int i = 0; i < image.rows; ++i) {
            for (int j = 0; j < image.cols; ++j) {
                grid.data.push_back(image.at<uchar>(i, j) == 255 ? 0 : 100);
            }
        }
        return grid;
    }

    // Convert vector of points to ROS Path message
    nav_msgs::Path convertVectorToPath(const std::vector<std::pair<int, int>>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.poses.reserve(path.size());

        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = point.first;
            pose.pose.position.y = point.second;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        return path_msg;
    }

    // Publish occupancy grid and path
    void publishMapAndPath() {
        occupancy_grid_.header.stamp = ros::Time::now();
        map_pub_.publish(occupancy_grid_);

        if (!path_message_.poses.empty()) {
            path_message_.header.stamp = ros::Time::now();
            path_pub_.publish(path_message_);
        }
    }

    // Callback for goal selection
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        int x = static_cast<int>(msg->pose.position.x);
        int y = static_cast<int>(msg->pose.position.y);

        if (selection_state_ == SelectionState::WAITING_FOR_START) {
            start_ = {x, y};
            ROS_INFO("Start point set to (%d, %d)", x, y);
            selection_state_ = SelectionState::WAITING_FOR_GOAL;
        } else {
            goal_ = {x, y};
            ROS_INFO("Goal point set to (%d, %d)", x, y);
            selection_state_ = SelectionState::WAITING_FOR_START;
            calculatePath();
        }
    }

    // Calculate path using GridMap
    void calculatePath() {
        grid_map_.setStartGoal(start_, goal_);
        path_points_ = grid_map_.findPath();
        path_message_ = convertVectorToPath(path_points_);
        ROS_INFO("Path recalculated");
    }
};

int main(int argc, char** argv) {
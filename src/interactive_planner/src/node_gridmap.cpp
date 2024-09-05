#include <ros/ros.h>
#include "/home/gabrielescognamiglio/catkin_ws/src/interactive_planner/include/interactive_planner/grid_map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <string>

class GridMapNode {
public:
    GridMapNode() : nh_("~") {
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1, true);

        nh_.param<std::string>("image_path", image_path_, "");
        nh_.param<int>("start_x", start_.first, 10);
        nh_.param<int>("start_y", start_.second, 10);
        nh_.param<int>("goal_x", goal_.first, 1657);
        nh_.param<int>("goal_y", goal_.second, 1657);
        nh_.param<double>("publish_rate", publish_rate_, 1.0);
    }

    bool initialize() {
        if (!loadImage()) {
            return false;
        }
        grid_map_.setStartGoal(start_, goal_);
        occupancy_grid_ = imageToOccupancyGrid(image_);
        grid_map_.setOccupancy(occupancy_grid_);
        grid_map_.computeDistanceMap();
        path_points_ = grid_map_.findPath();
        path_message_ = convertVectorToPath(path_points_);
        return true;
    }

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
    GridMap grid_map_;
    std::string image_path_;
    std::pair<int, int> start_;
    std::pair<int, int> goal_;
    cv::Mat image_;
    nav_msgs::OccupancyGrid occupancy_grid_;
    std::vector<std::pair<int, int>> path_points_;
    nav_msgs::Path path_message_;
    double publish_rate_;

    bool loadImage() {
        image_ = cv::imread(image_path_, cv::IMREAD_GRAYSCALE);
        if (image_.empty()) {
            ROS_ERROR_STREAM("Failed to load image: " << image_path_);
            return false;
        }
        ROS_INFO("Image loaded successfully");
        return true;
    }

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

    void publishMapAndPath() {
        occupancy_grid_.header.stamp = ros::Time::now();
        map_pub_.publish(occupancy_grid_);

        path_message_.header.stamp = ros::Time::now();
        path_pub_.publish(path_message_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_gridmap");
    GridMapNode node;
    
    if (!node.initialize()) {
        return 1;
    }
    
    node.run();
    return 0;
}

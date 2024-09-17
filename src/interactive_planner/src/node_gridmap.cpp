#include <ros/ros.h>
#include "/home/gabrielescognamiglio/catkin_ws/src/interactive_planner/include/interactive_planner/grid_map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

enum class SelectionState {
    WAITING_FOR_START,
    WAITING_FOR_GOAL
};

class GridMapNode {
public:
    GridMapNode() : nh_("~"), selection_state_(SelectionState::WAITING_FOR_START) {
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1, true);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GridMapNode::goalCallback, this);
        map_sub_ = nh_.subscribe("/map", 1, &GridMapNode::mapCallback, this);
        map_metadata_sub_ = nh_.subscribe("/map_metadata", 1, &GridMapNode::mapMetadataCallback, this);

        nh_.param<double>("publish_rate", publish_rate_, 1.0);
    }

    void run() {
        ros::Rate rate(publish_rate_);
        while (ros::ok()) {
            if (!path_message_.poses.empty()) {
                path_message_.header.stamp = ros::Time::now();
                path_pub_.publish(path_message_);
            }
            rate.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber map_metadata_sub_;
    GridMap grid_map_;
    std::pair<int, int> start_;
    std::pair<int, int> goal_;
    nav_msgs::Path path_message_;
    double publish_rate_;
    double resolution_;
    SelectionState selection_state_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        grid_map_.setOccupancy(*map);
        grid_map_.computeDistanceMap();
        ROS_INFO("Received map and updated GridMap");
    }

    void mapMetadataCallback(const nav_msgs::MapMetaData::ConstPtr& metadata) {
        resolution_ = metadata->resolution;
        ROS_INFO("Received map metadata. Resolution: %f", resolution_);
    }

    nav_msgs::Path convertVectorToPath(const std::vector<std::pair<int, int>>& path) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.poses.reserve(path.size());

        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position = grid2world(point.first, point.second);
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        return path_msg;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        auto grid_pos = world2grid(msg->pose.position);
        int x = grid_pos.first;
        int y = grid_pos.second;

        if (selection_state_ == SelectionState::WAITING_FOR_START) {
            start_ = {x, y};
            selection_state_ = SelectionState::WAITING_FOR_GOAL;
            ROS_INFO("Start point set to grid coordinates: (%d, %d)", x, y);
        } else {
            goal_ = {x, y};
            selection_state_ = SelectionState::WAITING_FOR_START;
            ROS_INFO("Goal point set to grid coordinates: (%d, %d)", x, y);
            calculatePath();
        }
    }

    void calculatePath() {
        grid_map_.setStartGoal(start_, goal_);
        auto path_points = grid_map_.findPath();
        path_message_ = convertVectorToPath(path_points);
        ROS_INFO("Path calculated with %zu points", path_message_.poses.size());
    }

    std::pair<int, int> world2grid(const geometry_msgs::Point& position) {
        int x = static_cast<int>(position.x / resolution_);
        int y = static_cast<int>(position.y / resolution_);
        ROS_INFO("world2grid: (%.2f, %.2f) -> (%d, %d)", position.x, position.y, x, y);
        return {x, y};
    }

    geometry_msgs::Point grid2world(int x, int y) {
        geometry_msgs::Point position;
        position.x = x * resolution_;
        position.y = y * resolution_;
        ROS_INFO("grid2world: (%d, %d) -> (%.2f, %.2f)", x, y, position.x, position.y);
        return position;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_gridmap");
    GridMapNode node;
    node.run();
    return 0;
}
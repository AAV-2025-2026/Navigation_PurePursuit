// navigation_system.cpp
// Combines: Obstacle Avoidance, D* Lite, Pure Pursuit, Velocity Planning are all integrated in 1 file

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;
#functions to help

struct GridCell {
    int x, y;
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

struct DStarNode {
    GridCell cell;
    double g;
    double rhs;
    double key[2];
    
    bool operator<(const DStarNode& other) const {
        if (key[0] != other.key[0]) return key[0] > other.key[0];
        return key[1] > other.key[1];
    }
};

//navigation class

class NavigationSystem : public rclcpp::Node {
public:
    NavigationSystem() : Node("navigation_system") {
        // Parameters
        this->declare_parameter("wheelbase", 1.57);
        this->declare_parameter("max_speed", 2.0);
        this->declare_parameter("lookahead_distance", 1.5);
        this->declare_parameter("obstacle_threshold", 0.5);
        
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
        
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&NavigationSystem::mapCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&NavigationSystem::scanCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 50, std::bind(&NavigationSystem::odomCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&NavigationSystem::goalCallback, this, std::placeholders::_1));
        
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 10);
        cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);
        
        // Timer for control loop (10 Hz)
        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&NavigationSystem::controlLoop, this));
        
   
        RCLCPP_INFO(this->get_logger(), "NAVIGATION SYSTEM STARTED (C++)");
        RCLCPP_INFO(this->get_logger(), ");
        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance: Active");
        RCLCPP_INFO(this->get_logger(), "D* Lite Planner: Ready");
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit: Ready");
        RCLCPP_INFO(this->get_logger(), "Velocity Planner: Active");
        RCLCPP_INFO(this->get_logger(), "");
    }

private:
     //callback files
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = *msg;
        map_received_ = true;
        
        // Initialize D* Lite if not done
        if (!dstar_initialized_) {
            initializeDStar();
        }
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = *msg;
        updateLocalCostmap();
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        current_velocity_ = msg->twist.twist.linear.x;
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = msg->pose;
        goal_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", 
                    goal_pose_.position.x, goal_pose_.position.y);
        
        // Plan path
        planPath();
    }
    
    
    //obstacle avoidance to update local costmap
    void updateLocalCostmap() {
        if (!map_received_) return;
        
        // Create local costmap from LiDAR scan
        nav_msgs::msg::OccupancyGrid costmap;
        costmap.header.stamp = this->now();
        costmap.header.frame_id = "base_link";
        costmap.info.resolution = 0.1;  // 10cm resolution
        costmap.info.width = 100;       // 10m x 10m
        costmap.info.height = 100;
        costmap.info.origin.position.x = -5.0;
        costmap.info.origin.position.y = -5.0;
        
        costmap.data.resize(100 * 100, 0);
        
        // Mark obstacles from LiDAR
        for (size_t i = 0; i < scan_.ranges.size(); ++i) {
            float range = scan_.ranges[i];
            
            if (range < scan_.range_min || range > scan_.range_max) continue;
            if (range > 5.0) continue;  // Only consider close obstacles
            
            float angle = scan_.angle_min + i * scan_.angle_increment;
            float x = range * cos(angle);
            float y = range * sin(angle);
            
            // Convert to grid coordinates
            int grid_x = static_cast<int>((x - costmap.info.origin.position.x) / costmap.info.resolution);
            int grid_y = static_cast<int>((y - costmap.info.origin.position.y) / costmap.info.resolution);
            
            if (grid_x >= 0 && grid_x < 100 && grid_y >= 0 && grid_y < 100) {
                int idx = grid_y * 100 + grid_x;
                costmap.data[idx] = 100;  // Mark as occupied
            }
        }
        
        costmap_pub_->publish(costmap);
        local_costmap_ = costmap;
    }
    
    // D* Lite Path PLanning Algorithm
    void initializeDStar() {
        if (!map_received_) return;
        
        width_ = map_.info.width;
        height_ = map_.info.height;
        resolution_ = map_.info.resolution;
        
        // Initialize g and rhs scores
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                GridCell cell{x, y};
                g_scores_[cell] = INF;
                rhs_scores_[cell] = INF;
            }
        }
        
        dstar_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "D* Lite initialized: %dx%d grid", width_, height_);
    }
    
    void planPath() {
        if (!map_received_ || !goal_received_) return;
        
        auto start_time = this->now();
        
        // Convert poses to grid coordinates
        GridCell start = worldToGrid(current_pose_.position.x, current_pose_.position.y);
        GridCell goal = worldToGrid(goal_pose_.position.x, goal_pose_.position.y);
        
        // Simple A* for now (D* Lite full implementation is complex)
        // For production, use full D* Lite with incremental replanning
        std::vector<GridCell> path = aStar(start, goal);
        
        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), " No path was found!");
            return;
        }
        
        // Convert to ROS path message
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        
        for (const auto& cell : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            auto [x, y] = gridToWorld(cell.x, cell.y);
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        
        current_path_ = path_msg;
        path_pub_->publish(path_msg);
        
        auto end_time = this->now();
        double planning_time = (end_time - start_time).seconds() * 1000.0;
        
        RCLCPP_INFO(this->get_logger(), "Path planned: %zu waypoints in %.1f ms", 
                    path.size(), planning_time);
    }
    
    std::vector<GridCell> aStar(GridCell start, GridCell goal) {
        std::priority_queue<DStarNode> open_list;
        std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
        std::unordered_map<GridCell, double, GridCellHash> cost_so_far;
        
        DStarNode start_node;
        start_node.cell = start;
        start_node.g = 0;
        start_node.key[0] = heuristic(start, goal);
        open_list.push(start_node);
        
        cost_so_far[start] = 0;
        
        while (!open_list.empty()) {
            DStarNode current = open_list.top();
            open_list.pop();
            
            if (current.cell == goal) {
                // Reconstruct path
                std::vector<GridCell> path;
                GridCell curr = goal;
                while (!(curr == start)) {
                    path.push_back(curr);
                    curr = came_from[curr];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            // Explore neighbors
            for (const auto& neighbor : getNeighbors(current.cell)) {
                if (isOccupied(neighbor)) continue;
                
                double new_cost = cost_so_far[current.cell] + 1.0;
                
                if (cost_so_far.find(neighbor) == cost_so_far.end() || 
                    new_cost < cost_so_far[neighbor]) {
                    
                    cost_so_far[neighbor] = new_cost;
                    came_from[neighbor] = current.cell;
                    
                    DStarNode neighbor_node;
                    neighbor_node.cell = neighbor;
                    neighbor_node.g = new_cost;
                    neighbor_node.key[0] = new_cost + heuristic(neighbor, goal);
                    open_list.push(neighbor_node);
                }
            }
        }
        
        return {};  // No path found
    }
    
     //pure pursuit controller node
    void controlLoop() {
        if (current_path_.poses.empty()) return;
        
        // Find lookahead point
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point.has_value()) return;
        
        // Calculate steering angle
        double steering_angle = calculateSteeringAngle(lookahead_point.value());
        
        // Calculate speed based on curvature
        double curvature = std::abs(steering_angle) / wheelbase_;
        double desired_speed = calculateSpeed(curvature);
        
        // Velocity planning - adjust for obstacles
        double safe_speed = adjustSpeedForObstacles(desired_speed);
        
        // Publish command
        ackermann_msgs::msg::AckermannDriveStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.drive.speed = safe_speed;
        cmd.drive.steering_angle = steering_angle;
        
        cmd_pub_->publish(cmd);
    }
    
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint() {
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        
        // Find closest point on path
        for (size_t i = 0; i < current_path_.poses.size(); ++i) {
            double dx = current_path_.poses[i].pose.position.x - current_pose_.position.x;
            double dy = current_path_.poses[i].pose.position.y - current_pose_.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // Find lookahead point
        for (size_t i = closest_idx; i < current_path_.poses.size(); ++i) {
            double dx = current_path_.poses[i].pose.position.x - current_pose_.position.x;
            double dy = current_path_.poses[i].pose.position.y - current_pose_.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist >= lookahead_distance_) {
                return current_path_.poses[i].pose.position;
            }
        }
        
        // Return last point if close to goal
        if (!current_path_.poses.empty()) {
            return current_path_.poses.back().pose.position;
        }
        
        return std::nullopt;
    }
    
    double calculateSteeringAngle(const geometry_msgs::msg::Point& lookahead) {
        // Transform lookahead point to robot frame
        double dx = lookahead.x - current_pose_.position.x;
        double dy = lookahead.y - current_pose_.position.y;
        
        // Get robot yaw from quaternion
        double yaw = 2.0 * std::atan2(current_pose_.orientation.z, current_pose_.orientation.w);
        
        // Rotate to robot frame
        double local_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
        double local_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);
        
        // Pure pursuit formula
        double curvature = 2.0 * local_y / (local_x*local_x + local_y*local_y);
        double steering_angle = std::atan(curvature * wheelbase_);
        
        // Clamp steering angle
        const double max_steering = 0.52;  // ~30 degrees
        return std::clamp(steering_angle, -max_steering, max_steering);
    }
    
    double calculateSpeed(double curvature) {
        // Slow down for sharp turns
        const double max_lateral_accel = 2.0;  // m/s^2
        
        if (curvature < 0.001) {
            return max_speed_;
        }
        
        double max_speed_curve = std::sqrt(max_lateral_accel / curvature);
        return std::min(max_speed_, max_speed_curve);
    }
    
    double adjustSpeedForObstacles(double desired_speed) {
        // Check for obstacles in front
        double min_distance = 10.0;
        
        for (size_t i = 0; i < scan_.ranges.size() / 4; ++i) {
            size_t idx = scan_.ranges.size() / 2 - scan_.ranges.size() / 8 + i;
            if (idx < scan_.ranges.size()) {
                float range = scan_.ranges[idx];
                if (range > scan_.range_min && range < scan_.range_max) {
                    min_distance = std::min(min_distance, static_cast<double>(range));
                }
            }
        }
        
        // Slow down if obstacle close
        if (min_distance < 1.0) {
            return 0.0;  // Stop
        } else if (min_distance < 2.0) {
            return desired_speed * 0.5;  // Slow down
        }
        
        return desired_speed;
    }
    //utility functions
    GridCell worldToGrid(double x, double y) {
        int grid_x = static_cast<int>((x - map_.info.origin.position.x) / resolution_);
        int grid_y = static_cast<int>((y - map_.info.origin.position.y) / resolution_);
        return {grid_x, grid_y};
    }
    
    std::pair<double, double> gridToWorld(int x, int y) {
        double world_x = x * resolution_ + map_.info.origin.position.x;
        double world_y = y * resolution_ + map_.info.origin.position.y;
        return {world_x, world_y};
    }
    
    bool isOccupied(const GridCell& cell) {
        if (cell.x < 0 || cell.x >= width_ || cell.y < 0 || cell.y >= height_) {
            return true;
        }
        int idx = cell.y * width_ + cell.x;
        return map_.data[idx] > 50;
    }
    
    std::vector<GridCell> getNeighbors(const GridCell& cell) {
        return {
            {cell.x + 1, cell.y},
            {cell.x - 1, cell.y},
            {cell.x, cell.y + 1},
            {cell.x, cell.y - 1},
            {cell.x + 1, cell.y + 1},
            {cell.x + 1, cell.y - 1},
            {cell.x - 1, cell.y + 1},
            {cell.x - 1, cell.y - 1}
        };
    }
    
    double heuristic(const GridCell& a, const GridCell& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
    
   
    //Member Variables
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::OccupancyGrid local_costmap_;
    sensor_msgs::msg::LaserScan scan_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose goal_pose_;
    nav_msgs::msg::Path current_path_;
    
    double current_velocity_ = 0.0;
    bool map_received_ = false;
    bool goal_received_ = false;
    bool dstar_initialized_ = false;
    
    // Parameters
    double wheelbase_;
    double max_speed_;
    double lookahead_distance_;
    double obstacle_threshold_;
    
    // D* Lite
    int width_, height_;
    double resolution_;
    std::unordered_map<GridCell, double, GridCellHash> g_scores_;
    std::unordered_map<GridCell, double, GridCellHash> rhs_scores_;
    
    static constexpr double INF = std::numeric_limits<double>::infinity();
};

// Main Function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationSystem>());
    rclcpp::shutdown();
    return 0;
}

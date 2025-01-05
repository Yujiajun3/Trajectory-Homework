#include <ros/ros.h>
#include <utility>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/Path.h"
#include <spdlog/spdlog.h>


struct Node {
    int x, y;        // 节点所在的网格坐标
    double g_cost;   // 从起点到当前节点的代价
    double h_cost;   // 从当前节点到终点的估计代价
    std::shared_ptr<Node> parent;    // 父节点，用于回溯路径

    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {}

    double f() const { return g_cost + h_cost; } // 总代价值

};
// 比较器，用于优先队列
struct cmp{
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b){
        return a->f() > b->f();
    }

};
struct GridMap {
    int width;
    int height;
    double map_max;
    double map_min;
    double grid_resolution;
    int inflate_radius;
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 占用

    GridMap(int w, int h, double map_min_, double map_max_, double res,int inflate_radius_ ) : width(w), height(h), map_min(map_min_), map_max(map_max_),grid_resolution(res), inflate_radius(inflate_radius_), grid(w, std::vector<int>(h, 0)) {}

    void markObstacle(double cx, double cy, double radius) {
        int grid_cx = std::round((cx - map_min) / grid_resolution);
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution);
        // Step 1: 将圆形区域标记为占用，将 grid 中的值设为 1
            // your code
            for (int i = std::max(0, grid_cx - grid_radius); i < std::min(width, grid_cx + grid_radius); i++) {
                for (int j = std::max(0, grid_cy - grid_radius); j < std::min(height, grid_cy + grid_radius); j++) {
                    if (std::pow(i - grid_cx, 2) + std::pow(j - grid_cy, 2) <= std::pow(grid_radius+inflate_radius, 2)) {
                        grid[i][j] = 1;
                    }
                }
            }
        // finish
    }
};

// 圆柱形障碍物结构，包含位置和半径
struct Obstacle {
    int x;  // 圆心
    int y;
    double radius; // 半径
    Obstacle(int _x,int _y, double _radius) : x(_x),y(_y), radius(_radius) {}
};

class AStarPlanner {
public:
    AStarPlanner(int width, int height, double m_min, double m_max, double res,int inflate_radius_) : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res), grid_map_(width, height, map_min_, map_max_, grid_resolution_,inflate_radius_), num_of_obs_(0) {

    }

    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
        if (!obs_flag_) {
            obstacles_.emplace_back(cx, cy, radius);
        }
    }

    void printGridMap(){
        for(int i = 0; i < width_; i++){
            for(int j = 0; j < height_; j++){
                std::cout<<grid_map_.grid[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<"num of obstacles: "<<num_of_obs_<<std::endl;
    }

    auto getObstacles(){
        return obstacles_;
    }

    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal) {
        if(num_of_obs_ == 0){
            return {};
        }
        // 起点和终点转换为网格坐标
        auto gridStart = worldToGrid(start);
        auto gridGoal = worldToGrid(goal);
        spdlog::info("NEW!");
        std::cout<<"start: "<<gridStart.first<<" "<<gridStart.second<<std::endl;
        std::cout<<"goal: "<<gridGoal.first<<" "<<gridGoal.second<<std::endl;
        // 开放列表和关闭列表
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;
        std::unordered_map<int, std::unordered_map<int, std::shared_ptr<Node>>> open_map;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));

        // 起点加入开放列表
        open_list.push(std::make_shared<Node>(Node(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal))));
        open_map[gridStart.first][gridStart.second] = open_list.top();
        // 障碍物加入关闭列表
        for(int i = 0; i < width_; i++){
            for(int j = 0; j < height_; j++){
                if(grid_map_.grid[i][j] == 1){
                    closed_list[i][j] = true;
                }
            }
        }
        std::cout<<"add obstacles to closed list"<<std::endl;
        // Step 3： 实现 A* 算法，搜索结束调用 reconstructPath 返回路径
            // your code
            while(!open_list.empty()) {
                auto current = open_list.top();
                if (current->x == gridGoal.first && current->y == gridGoal.second) {
                    return reconstructPath(current);
                }
                else {
                    open_list.pop();
                    open_map[current->x].erase(current->y);
                    closed_list[current->x][current->y] = true;
                    auto neighbors = getNeighbors(*current);
                    for (const auto& neighbor : neighbors) {
                        if (closed_list[neighbor.x][neighbor.y]) {
                            continue;
                        }
                        else {
                        // 检测neighbor是否在open_list中
                        double g_cost = current->g_cost + distance(*current, neighbor);
                        if (open_map[neighbor.x].find(neighbor.y) != open_map[neighbor.x].end()) {
                            // 如果在open_list中
                            if (g_cost < open_map[neighbor.x][neighbor.y]->g_cost) {
                                open_map[neighbor.x][neighbor.y]->g_cost = g_cost;
                                open_map[neighbor.x][neighbor.y]->parent = current;
                            }
                        }
                        else {
                        double h_cost = heuristic({neighbor.x, neighbor.y}, gridGoal);
                        auto node = std::make_shared<Node>(neighbor.x, neighbor.y, g_cost, h_cost, current);
                        open_list.push(node);
                        open_map[neighbor.x][neighbor.y] = node;
                        }
                        }
                    }
                }
            }
        // finish
        // 如果没有找到路径，返回空路径
        return {};
    }
    void reset(){
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }
    void setObsFlag(bool flag){
        obs_flag_ = flag;
    }
    //**
    // * @brief: 从路径中删除多余的点
    // * @param path: 输入A*搜索出的原路径（冗余点过多）
    // * @param obstacles: 障碍物列表
    // * @param threshold: 距离障碍物的阈值，小于该阈值的点的一个点会被保留，作为新路径的点
    // * @return: 精简后的路径
    // * @athor: yujiajun
    // */
    std::vector<Eigen::Vector2d> shortPath(std::vector<Eigen::Vector2d> path, std::vector<Obstacle> obstacles, double threshold){
        int n = path.size();
        std::vector<Eigen::Vector2d> path_short;
        path_short.push_back(path[0]);
        int last_index = 0;
        while (last_index < n - 1) {
            bool canShortcut = true;
            // 尝试跳过中间点，检查是否可以直接从simplifiedPath.back()到path[i]而不受障碍物影响
            for (int i = last_index + 1; i < n; i++) {
                if ((!isPathVisible(path[last_index], path[i], obstacles, threshold))||i==n-1) {
                    canShortcut = false;
                    last_index = i;
                    break;
                    }
            }
            if (!canShortcut) {
                path_short.push_back(path[last_index]);
                // spdlog::info("add point");
                // std::cout<<"add point: "<<path[last_index].x()<<" "<<path[last_index].y()<<std::endl;
            }
        }
        return path_short;
    }


private:

    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
    }

    // 计算两节点之间的距离（用于邻居代价计算）
    double distance(const Node& a, const Node& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // 从世界坐标转换到栅格坐标
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& position) {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        return {x, y};
    }

    // 从栅格坐标转换到世界坐标（主要用于路径结果显示）
    Eigen::Vector2d gridToWorld(int x, int y) {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    // 获取当前节点的所有邻居节点
    std::vector<Node> getNeighbors(const Node& current) {
        std::vector<Node> neighbors;

        // 八连通邻居
        std::vector<std::pair<int, int>> directions = {
                {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (const auto& dir : directions) {
            // Step 2: 根据当前节点和方向计算邻居节点的坐标，并将其加入 neighbors

                // your code
                int x = current.x + dir.first;
                int y = current.y + dir.second;
                if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                    neighbors.push_back(Node(x, y, 0.0, 0.0));
                }
                
            // finish
        }

        return neighbors;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) {
        std::vector<Eigen::Vector2d> path;
        while (node) {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        reset();
        return path;
    }

    // 检查是否有障碍物与路径相交
    bool isPathVisible(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const std::vector<Obstacle>& obstacles, double threshold) {
        for (const Obstacle& obs : obstacles) {
            int dist = pointToLineSegmentDistance(obs.x,obs.y ,start, end);
            int dist_int = std::round(dist);
            if (dist_int < threshold+obs.radius) {
                return false;  // 如果有任何障碍物与路径相交，认为不可见
            }
        }
        return true;  // 路径没有被障碍物阻挡
    }

    // 计算点到线段的最短距离
    double pointToLineSegmentDistance(const int& O_x, const int& O_y, const Eigen::Vector2d& lineStart, const Eigen::Vector2d& lineEnd) {
    // 计算向量 (lineEnd - lineStart) 和 (p - lineStart)
    double dx = lineEnd.x() - lineStart.x();
    double dy = lineEnd.y() - lineStart.y();
    double lengthSquared = dx * dx + dy * dy;

    if (lengthSquared == 0) return std::sqrt((O_x - lineStart.x()) * (O_x - lineStart.x()) + (O_y - lineStart.y()) * (O_y - lineStart.y()));

    // 向量点积计算投影比例 t
    double t = ((O_x - lineStart.x()) * dx + (O_y - lineStart.y()) * dy) / lengthSquared;

    // 确保 t 在 [0, 1] 范围内，表示投影点在线段上
    if (t < 0) {
        return std::sqrt((O_x - lineStart.x()) * (O_x - lineStart.x()) + (O_y - lineStart.y()) * (O_y - lineStart.y()));
    } else if (t > 1) {
        return std::sqrt((O_x - lineEnd.x()) * (O_x - lineEnd.x()) + (O_y - lineEnd.y()) * (O_y - lineEnd.y()));
    } else {
        // 投影点在线段内，计算投影点到 p 的距离
        double projX = lineStart.x() + t * dx;
        double projY = lineStart.y() + t * dy;
        return std::sqrt((O_x - projX) * (O_x - projX) + (O_y - projY) * (O_y - projY));
    }
    }
    

    // 地图数据
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_; // 栅格地图，0: 空闲，1: 障碍物
    int num_of_obs_;
    std::vector<Obstacle> obstacles_;
    bool obs_flag_ = false;
};

int main(int argc, char** argv) {
    std::cout<<"start"<<std::endl;
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;
    double map_min_, map_max_, grid_resolution_;
    double start_x_, start_y_, goal_x_, goal_y_;
    double short_cut_threshold;
    int inflate_radius_;
    nh.param("astar_planner/map_min", map_min_, -5.0);
    nh.param("astar_planner/map_max", map_max_, 5.0);
    nh.param("astar_planner/grid_resolution", grid_resolution_, 0.1);
    nh.param("astar_planner/start_x", start_x_, -4.5);
    nh.param("astar_planner/start_y", start_y_, -4.5);
    nh.param("astar_planner/goal_x", goal_x_, 4.5);
    nh.param("astar_planner/goal_y", goal_y_, 4.5);
    nh.param("astar_planner/short_cut_threshold", short_cut_threshold,0.01);
    nh.param("astar_planner/inflate_radius", inflate_radius_, 0);
    // 地图参数
    int grid_width = std::round((map_max_ - map_min_) / grid_resolution_);
    int grid_height = grid_width;

    AStarPlanner planner(grid_width, grid_height, map_min_, map_max_, grid_resolution_,inflate_radius_);
    // 障碍物订阅
    ros::Subscriber obstacle_sub = nh.subscribe<visualization_msgs::MarkerArray>("obstacles", 1,
                                                                                 [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr& msg) {
                                                                                     for (const auto& marker : msg->markers) {
                                                                                        planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                                                                                     }
                                                                                        planner.setObsFlag(true);
                                                                                 });



    // 发布路径
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    ros::Publisher path_pub_short = nh.advertise<nav_msgs::Path>("path_short", 1);

    // 起点和终点参数
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);
    while (ros::ok()) {
        planner.reset();
//        // 等待障碍物加载
//        ros::Duration(1.0).sleep();
        ros::spinOnce();
        // 执行路径搜索
        std::vector<Eigen::Vector2d> path = planner.findPath(start, goal);
        // 路径可视化
        if (path.empty()){
            continue;
        }
        // shortcut path
        std::vector<Eigen::Vector2d> path_short = planner.shortPath(path, planner.getObstacles(), short_cut_threshold);
        std::cout<<"path size: "<<path.size()<<std::endl;
        std::cout<<"pathshort size: "<<path_short.size()<<std::endl;
        std::cout<<"inflated radius: "<<inflate_radius_<<std::endl;
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "odom";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        nav_msgs::Path path_msg_short;
        path_msg_short.header.frame_id = "odom";
        path_msg_short.header.stamp = ros::Time::now();
        for (const auto& point : path_short) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg_short.poses.push_back(pose);
        }
        path_pub.publish(path_msg);
        path_pub_short.publish(path_msg_short);
        rate.sleep();
    }

    return 0;
}
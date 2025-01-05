#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <vector>

// 生成圆柱体点云的函数
pcl::PointCloud<pcl::PointXYZ>::Ptr createCylinderPointCloud(double x, double y, double radius, double height, int num_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> angle_dist(0, 2 * M_PI);
    std::uniform_real_distribution<> height_dist(0, height);

    for (int i = 0; i < num_points; ++i) {
        double angle = angle_dist(gen);
        double z = height_dist(gen);
        pcl::PointXYZ point;
        point.x = x + radius * cos(angle);
        point.y = y + radius * sin(angle);
        point.z = z;
        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cylinder_point_cloud_generator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    double map_min_, map_max_, num_obstacles_, min_radius_, max_radius_, height_;
    int num_points_per_cylinder_;
    nh_private.param("map_min", map_min_, -5.0);
    nh_private.param("map_max", map_max_, 5.0);
    nh_private.param("num_obstacles", num_obstacles_, 10.0);
    nh_private.param("min_radius", min_radius_, 0.2);
    nh_private.param("max_radius", max_radius_, 0.5);
    nh_private.param("height", height_, 1.0);
    nh_private.param("num_points_per_cylinder", num_points_per_cylinder_, 1000);

    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cylinder_point_cloud", 1);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> pos_dist(map_min_, map_max_);
    std::uniform_real_distribution<> radius_dist(min_radius_, max_radius_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < num_obstacles_; ++i) {
        double x = pos_dist(gen);
        double y = pos_dist(gen);
        double radius = radius_dist(gen);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud = createCylinderPointCloud(x, y, radius, height_, num_points_per_cylinder_);
        *cloud += *cylinder_cloud;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";

    ros::Rate rate(1);
    while (ros::ok()) {
        cloud_pub.publish(output);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
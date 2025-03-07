#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher cloud_pub;
ros::Publisher path_pub;
ros::Publisher es_pub;
ros::Publisher poly_pub;
sensor_msgs::PointCloud global_cloud;
vec_Vec2f global_path;
bool path_received = false; // 경로를 처음 받았는지 확인

// OccupancyGrid -> PointCloud 변환 함수
sensor_msgs::PointCloud occupancy_grid_to_pointcloud(const nav_msgs::OccupancyGrid& grid) {
    sensor_msgs::PointCloud cloud;
    cloud.header = grid.header;

    int width = grid.info.width;
    int height = grid.info.height;
    float resolution = grid.info.resolution;
    geometry_msgs::Point32 pt;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            if (grid.data[index] > 98) { // 장애물이 있는 셀만 추가
                pt.x = grid.info.origin.position.x + x * resolution;
                pt.y = grid.info.origin.position.y + y * resolution;
                pt.z = 0;
                cloud.points.push_back(pt);
            }
        }
    }
    return cloud;
}

// costmap 콜백 함수 (최신 데이터 저장)
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received costmap data!");
    global_cloud = occupancy_grid_to_pointcloud(*msg);
}

// global_plan 콜백 함수
void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg) {
    ROS_INFO("Received global plan!");
    global_path.clear();

    int offset = 40;  // 10개 건너뛰기
    for (size_t i = 0; i < msg->poses.size(); i += offset) {
        Vec2f point(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y);
        global_path.push_back(point);
    }
    path_received = true; // 첫 경로 수신 여부 설정
}

// 시각화 및 경로 계획 업데이트
void updateVisualization() {
    if (global_cloud.points.empty() || global_path.empty()) return;

    ROS_INFO("Updating visualization!");

    // PointCloud 퍼블리싱
    cloud_pub.publish(global_cloud);

    // 장애물 데이터를 2D로 변환
    vec_Vec3f obs = DecompROS::cloud_to_vec(global_cloud);
    vec_Vec2f obs2d;
    for (const auto& it : obs)
        obs2d.push_back(it.topRows<2>());

    // 경로 퍼블리싱
    nav_msgs::Path path_msg = DecompROS::vec_to_path(global_path);
    path_msg.header.frame_id = "map";
    path_pub.publish(path_msg);

    // Ellipsoid Decomposition 수행
    EllipsoidDecomp2D decomp_util;
    decomp_util.set_obs(obs2d);
    decomp_util.set_local_bbox(Vec2f(1, 2));
    decomp_util.dilate(global_path);

    // 시각화 메시지 생성 및 퍼블리싱
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
    es_msg.header.frame_id = "map";
    es_pub.publish(es_msg);

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
    poly_msg.header.frame_id = "map";
    poly_pub.publish(poly_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");

    cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
    path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
    es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);

    // costmap 및 global_plan 구독
    ros::Subscriber costmap_sub = nh.subscribe("/R_001/move_base/global_costmap/costmap", 1, costmapCallback);
    ros::Subscriber path_sub = nh.subscribe("/R_001/move_base/WaypointsGlobalPlanner/global_plan", 1, globalPlanCallback);

    ros::Rate loop_rate(1.0); // 1Hz 주기로 실행
    while (ros::ok()) {
        ros::spinOnce(); // 콜백 실행 (최신 costmap 및 경로 데이터 업데이트)
        if (path_received) {
            updateVisualization(); // 시각화 업데이트 (경로를 처음 받을 때부터)
        }
        loop_rate.sleep(); // 다음 루프까지 대기
    }

    return 0;
}


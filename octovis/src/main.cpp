#include <iostream>
#include <filesystem>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <QtGui>
#include <QApplication>

#include "Octomap/OcTree.h"
#include "octovis/ViewerGui.h"

float voxel_size = 2.0;  // in meters
constexpr int kSleepUsec = 1e3;  // for usleep()
std::mutex mutex_cloud, mutex_pose, mutex_image;
sensor_msgs::PointCloud2 cloud;
nav_msgs::Odometry pose;
cv::Mat image;
uint64_t image_time = 0;
std::shared_ptr<octomap::ViewerGui> gui;

const Eigen::Matrix4d FRD_wrt_FLU = (Eigen::Matrix4d() <<
    1.0, 0.0, 0.0, 0.0,
    0.0,-1.0, 0.0, 0.0,
    0.0, 0.0,-1.0, 0.0,
    0.0, 0.0, 0.0, 1.0
).finished();

// // Quat --> Euler(Z-Y-X) (pitch [X] range is limited to [-pi/2, pi/2])
// Eigen::Vector3d Quaterniond2EulerAngles(const Eigen::Quaterniond& q) {
//     Eigen::Vector3d angles;
//
//     // roll (x-axis rotation)
//     double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
//     double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
//     angles(2) = std::atan2(sinr_cosp, cosr_cosp);
//
//     // pitch (y-axis rotation)
//     double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
//     if (std::abs(sinp) >= 1)
//         angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//     else
//         angles(1) = std::asin(sinp);
//
//     // yaw (z-axis rotation)
//     double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
//     double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
//     angles(0) = std::atan2(siny_cosp, cosy_cosp);
//
//     return angles;
// }

class PointCloudPoseSubscriber {
public:
    PointCloudPoseSubscriber() {
        // 初始化 ROS 节点
        ros::NodeHandle nh;

        // 创建点云订阅者，订阅 "/cloud_in" 话题
        cloud_sub_ = nh.subscribe("/points2", 1, &PointCloudPoseSubscriber::cloudCallback, this);

        // 创建位姿订阅者，订阅 "/pose" 话题
        pose_sub_ = nh.subscribe("/mavros/local_position/odom", 1, &PointCloudPoseSubscriber::poseCallback, this);

        image_sub_ = nh.subscribe("/zhz/driver/cam0/image_raw", 1, &PointCloudPoseSubscriber::imageCallback, this);
    }

    // 点云回调函数
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        mutex_cloud.lock();
        cloud = *cloud_msg;
        mutex_cloud.unlock();

        // ROS_INFO("Received point cloud with %lu points.", cloud.data.size());
    }

    // 位姿回调函数
    void poseCallback(const nav_msgs::OdometryConstPtr& pose_msg) {
        // ROS_INFO("Received pose: x=%f, y=%f, z=%f",
        //     pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
        mutex_pose.lock();
        pose = *pose_msg;
        mutex_pose.unlock();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
        mutex_image.lock();
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            image = cv_ptr->image;
            image_time = image_msg->header.stamp.toNSec();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        mutex_image.unlock();
    }

private:
    ros::Subscriber cloud_sub_, pose_sub_, image_sub_;
};

// 假设你已经有了以下变量：
// sensor_msgs::PointCloud2 cloud_msg; // 深度相机发布的点云消息
// geometry_msgs::Pose sensor_pose;    // 深度相机的位姿

bool written = false;

void updateOctomap(
    pcl::PointCloud<pcl::PointXYZ>& pcl_cloud,
    const nav_msgs::Odometry& sensor_pose,
    std::shared_ptr<octomap::OcTree>& octree) {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(pcl_cloud, pcl_cloud, indices);

    // 2. 将相机位姿转换为 octomap::point3d (传感器原点)
    const octomap::point3d sensor_origin(sensor_pose.pose.pose.position.x,
                                         sensor_pose.pose.pose.position.y,
                                         sensor_pose.pose.pose.position.z);

    Eigen::Affine3d sensor_pose_eigen = Eigen::Isometry3d::Identity();
    sensor_pose_eigen.translation() = Eigen::Vector3d(sensor_pose.pose.pose.position.x,
                                                      sensor_pose.pose.pose.position.y,
                                                      sensor_pose.pose.pose.position.z);
    sensor_pose_eigen.linear() = Eigen::Quaterniond(sensor_pose.pose.pose.orientation.w,
                                                    sensor_pose.pose.pose.orientation.x,
                                                    sensor_pose.pose.pose.orientation.y,
                                                    sensor_pose.pose.pose.orientation.z).toRotationMatrix();
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensor_pose_eigen);

    // 3. 将 PCL 点云转换为 octomap::Pointcloud
    octomap::Pointcloud octomap_cloud;
    for (const auto& pcl_point : pcl_cloud.points) {
        octomap_cloud.push_back(pcl_point.x, pcl_point.y, pcl_point.z);
    }

    octree->insertPointCloud(octomap_cloud, sensor_origin, -1.0, true, false);

    // 6. 更新 OctoMap 的内部节点
    octree->updateInnerOccupancy();
}

pcl::PointCloud<pcl::PointXYZ> DepthMap2PointCloud(const cv::Mat& depth_map)
{
    // 相机内参（示例值，替换为你的相机参数）
    float fu = 414.770203;  // x方向焦距
    float fv = 388.814423;  // y方向焦距
    float u0 = 324.260956;  // 主点x
    float v0 = 237.812637;  // 主点y

    // 创建PCL点云对象
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 遍历深度图中的每个像素
    cloud.points.reserve(320 * 240);
    for (int v = 0; v < depth_map.rows; v += 2) {
        for (int u = 0; u < depth_map.cols; u += 2) {
            // 获取深度值，假设深度图为16位图像，单位为毫米
            uint16_t depth_value = depth_map.at<uint16_t>(v, u);

            // 将深度值转换为米
            float Z = depth_value / 1000.0f;

            // 如果深度值为零（无效的深度值），则跳过
            if (Z < 0.1 || Z > 15) continue;

            // 使用相机内参将像素转换为3D空间中的点
            float X = (u - u0) * Z / fu;
            float Y = (v - v0) * Z / fv;

            // 将计算出的3D点添加到点云中
            cloud.points.emplace_back(X, Y, Z);
        }
    }
    // downsample cloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud.makeShared());
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(cloud);
    return cloud;
}

double getTime()
{
    return ros::Time::now().toSec();
}

void addPointClouds()
{
    emit gui->m_glwidget->pauseRequested();

    // 创建一个 OctoMap
    ros::Rate loop_rate(100); // 设置循环频率为 10 Hz

    // tf publisher
    tf::TransformBroadcaster br;

    Eigen::Matrix4d T_C_wrt_I = (Eigen::Matrix4d() <<
        0.000000, 0.139173, 0.990268, 0.031976,
        1.000000, 0.000000, 0.000000, -0.055000,
        0.000000, 0.990268, -0.139173, 0.011920,
        0, 0, 0, 1
        ).finished();

    // Preload depth map
    std::unordered_map<uint64_t, cv::Mat> depth_map_cache;
    /*
    int loaded_depth_map_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator("/home/jingye/Downloads/depth_map")) {
        const std::string depth_map_path = entry.path().string();
        if (depth_map_path.find(".tiff") == std::string::npos) {
            continue;
        }
        cv::Mat depth_map = imread(depth_map_path, cv::IMREAD_UNCHANGED);
        if (depth_map.empty()) {
            std::cerr << "Error loading depth image at " << depth_map_path << "\n";
            continue;
        }
        const std::string time_str = depth_map_path.substr(depth_map_path.find_last_of('/') + 1, depth_map_path.find_last_of('.') - depth_map_path.find_last_of('/') - 1);
        const uint64_t time = std::stoull(time_str);
        depth_map_cache[time] = depth_map;
        std::cout << "\rLoaded " << ++loaded_depth_map_count << " depth images" << std::flush;
    }
    std::cout << "\nFinished loading depth maps\n";
    */

    uint64_t last_time = 0;
    std::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(voxel_size));
    gui->addOctree(octree.get(), 0);
    bool printout_mode_toggled = false;
    int i_frame = 0;
    while (ros::ok()) {
        mutex_pose.lock();
        // mutex_cloud.lock();
        if (last_time >= image_time)
        {
            mutex_pose.unlock();
            usleep(kSleepUsec);
            continue;
        }
        last_time = image_time;
        bool updates_octomap = false;
        if (i_frame++ % 4 == 0)  // Lower the update rate of octomap
        {
            updates_octomap = true;
        }

        if (!printout_mode_toggled)
        {
            gui->on_actionPrintout_mode_toggled(true);
            printout_mode_toggled = true;
            gui->m_glwidget->setFPSIsDisplayed(true);
        }

        auto cam_pose = pose;  // Should be a copy to avoid retransform when reusing the same pose
        mutex_pose.unlock();
        auto& pos = cam_pose.pose.pose.position;
        auto& ori = cam_pose.pose.pose.orientation;
        Eigen::Matrix4d T_I_wrt_W = Eigen::Matrix4d::Identity();
        T_I_wrt_W.block<3, 3>(0, 0) = Eigen::Quaterniond(ori.w, ori.x, ori.y, ori.z).toRotationMatrix();
        T_I_wrt_W.block<3, 1>(0, 3) = Eigen::Vector3d(pos.x, pos.y, pos.z);
        T_I_wrt_W = T_I_wrt_W * FRD_wrt_FLU;
        Eigen::Matrix4d T_C_wrt_W = T_I_wrt_W * T_C_wrt_I;
        Eigen::Vector3d pos_C(T_C_wrt_W.block<3, 1>(0, 3));
        pos.x = pos_C.x();
        pos.y = pos_C.y();
        pos.z = pos_C.z();
        Eigen::Quaterniond q_eigen(T_C_wrt_W.block<3, 3>(0, 0));
        ori.x = q_eigen.x();
        ori.y = q_eigen.y();
        ori.z = q_eigen.z();
        ori.w = q_eigen.w();

        // transform.setOrigin(tf::Vector3(pos_C.x(), pos_C.y(), pos_C.z()));
        // transform.setRotation(tf::Quaternion(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w()));
        // br.sendTransform(tf::StampedTransform(transform, pose.header.stamp, "map", "stereo_frame")); // 发布变换 (变换, 时间戳, 父坐标系, 子坐标系)

        double start_time = getTime();
        cv::Mat rgb_img;
        if (!image.empty())
        {
            cv::cvtColor(image, rgb_img, cv::COLOR_GRAY2RGB);
        }

        gui->m_glwidget->img_mutex_.lock();
        gui->m_glwidget->background_img_ = rgb_img;
        gui->m_glwidget->img_mutex_.unlock();

        pcl::PointCloud<pcl::PointXYZ> point_cloud;
        if (updates_octomap)
        {
            cv::Mat depth_map;
            if (depth_map_cache.count(image_time) == 0) {
                const std::string depth_map_path = "/home/jingye/Downloads/depth_map/" +
                    std::to_string(image_time) + ".tiff";
                depth_map = imread(depth_map_path, cv::IMREAD_UNCHANGED);
                if (depth_map.empty()) {
                    std::cerr << "Error loading depth image at " << depth_map_path << "\n";
                    usleep(kSleepUsec);
                    continue;
                }
            } else
            {
                depth_map = depth_map_cache[image_time];
            }
            point_cloud = DepthMap2PointCloud(depth_map);
            // std::cout << (getTime() - start_time) * 1e3 << " ms (depth map loading time)\n";
        }

        emit gui->m_glwidget->pauseRequested();
        while (gui->m_glwidget->painting_) {
            // std::cout << "Waiting for painting to finish...\n";
            usleep(kSleepUsec);
        }
        const Eigen::Quaterniond octovis_cam_q = q_eigen * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
        gui->m_glwidget->camera()->setOrientation(
            {octovis_cam_q.x(), octovis_cam_q.y(), octovis_cam_q.z(), octovis_cam_q.w()});
        gui->m_glwidget->camera()->setPosition({pos.x, pos.y, pos.z});

        if (updates_octomap)
        {
            // start_time = getTime();
            octree->clear();
            // std::cout << (getTime() - start_time) * 1e3 << " ms (clear octomap time)\n";

            // start_time = getTime();
            updateOctomap(point_cloud, cam_pose, octree);
            // std::cout << (getTime() - start_time) * 1e3 << " ms (update octomap time)\n";

            // start_time = getTime();
        }
        gui->showOcTree();
        std::cout << (getTime() - start_time) * 1e3 << " ms (Octomap process time)---------------------------\n"
                  << i_frame << "========================\n";

        emit gui->m_glwidget->resumeRequested();
        // std::cout << "pose: " << pose.pose.pose.position.x << " " << pose.pose.pose.position.y << " "
        //           << pose.pose.pose.position.z << " " << pose.pose.pose.orientation.x << " "
        //           << pose.pose.pose.orientation.y << " " << pose.pose.pose.orientation.z << " "
        //           << pose.pose.pose.orientation.w << "\n";

        // mutex_cloud.unlock();
        // loop_rate.sleep();
        // sleep(1);
        usleep(kSleepUsec);
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ar_obstacle_vis_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    PointCloudPoseSubscriber subscriber;

    QApplication app(argc, argv);

    gui = std::make_shared<octomap::ViewerGui>("", nullptr, 16);
    gui->resize(640, 504);
    gui->show();

    std::thread t(addPointClouds);

    return QApplication::exec();
}

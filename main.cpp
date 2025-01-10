#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>


std::mutex mutex_cloud, mutex_pose;
sensor_msgs::PointCloud2 cloud;
geometry_msgs::PoseStamped pose;

// Quat --> Euler(Z-Y-X) (pitch [X] range is limited to [-pi/2, pi/2])
Eigen::Vector3d Quaterniond2EulerAngles(const Eigen::Quaterniond& q) {
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

class PointCloudPoseSubscriber {
public:
    PointCloudPoseSubscriber() {
        // 初始化 ROS 节点
        ros::NodeHandle nh;

        // 创建点云订阅者，订阅 "/cloud_in" 话题
        cloud_sub_ = nh.subscribe("/iris_D435i/realsense/depth_camera/depth/points", 1, &PointCloudPoseSubscriber::cloudCallback, this);

        // 创建位姿订阅者，订阅 "/pose" 话题
        pose_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &PointCloudPoseSubscriber::poseCallback, this);
    }

    // 点云回调函数
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        mutex_cloud.lock();
        cloud = *cloud_msg;
        mutex_cloud.unlock();

        // ROS_INFO("Received point cloud with %lu points.", cloud.data.size());
    }

    // 位姿回调函数
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
        // ROS_INFO("Received pose: x=%f, y=%f, z=%f",
        //     pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
        mutex_pose.lock();
        pose = *pose_msg;
        mutex_pose.unlock();
    }

private:
    ros::Subscriber cloud_sub_;
    ros::Subscriber pose_sub_;
};

// 假设你已经有了以下变量：
// sensor_msgs::PointCloud2 cloud_msg; // 深度相机发布的点云消息
// geometry_msgs::Pose sensor_pose;    // 深度相机的位姿

void updateOctomap(
    const sensor_msgs::PointCloud2& cloud_msg,
    const geometry_msgs::PoseStamped& sensor_pose,
    std::shared_ptr<octomap::OcTree>& octree) {
      // 1. 将 sensor_msgs::PointCloud2 转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_msg, pcl_cloud);
    // remove nan pts
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(pcl_cloud, pcl_cloud, indices);

    // 2. 将相机位姿转换为 octomap::point3d (传感器原点)
    const octomap::point3d sensor_origin(sensor_pose.pose.position.x,
                                         sensor_pose.pose.position.y,
                                         sensor_pose.pose.position.z);

    Eigen::Affine3d sensor_pose_eigen = Eigen::Isometry3d::Identity();
    sensor_pose_eigen.translation() = Eigen::Vector3d(sensor_pose.pose.position.x,
                                                      sensor_pose.pose.position.y,
                                                      sensor_pose.pose.position.z);
    sensor_pose_eigen.linear() = Eigen::Quaterniond(sensor_pose.pose.orientation.w,
                                                    sensor_pose.pose.orientation.x,
                                                    sensor_pose.pose.orientation.y,
                                                    sensor_pose.pose.orientation.z).toRotationMatrix();
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensor_pose_eigen);

    // 3. 将 PCL 点云转换为 octomap::Pointcloud
    octomap::Pointcloud octomap_cloud;
    for (const auto& pcl_point : pcl_cloud.points) {
        octomap_cloud.push_back(pcl_point.x, pcl_point.y, pcl_point.z);
    }

    // 注意：insertPointCloud 会执行 ray casting, 因此一般不需要再手动调用 castRay
    //      如果只想用点云末端更新, 可以先调用 octomap_cloud.filter(...) 过滤点云, 只保留需要的点
    octree->insertPointCloud(octomap_cloud, sensor_origin, -1.0, true, false);

    // 5. (可选) 手动执行射线投射来更新空闲空间
    //    如果需要显式地标记空闲空间，可以使用 computeRayKeys 或 castRay 进行射线投射。
    //    只有在不使用 insertPointCloud 或者需要更精细地控制空闲空间更新时才需要这样做。
    /*
    for (const auto& pcl_point : pcl_cloud.points) {
    octomap::point3d end_point(pcl_point.x, pcl_point.y, pcl_point.z);
    octomap::KeyRay keyRay;
    octree->computeRayKeys(sensor_origin, end_point, keyRay);
    for (auto key : keyRay) {
      octree->updateNode(key, false); // 标记为 free
    }
    }
    */

    // 6. 更新 OctoMap 的内部节点
    octree->updateInnerOccupancy();

    std::cout << "octree->size(): " << octree->size() << "\n";

    if (octree->size() > 1e6) {
        octree->prune();
        octree->write("/home/jingye/Downloads/octomap.ot");
        exit(0);
    }
}

int main() {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "octomap_example_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    PointCloudPoseSubscriber subscriber;

    // 创建一个 OctoMap
    std::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(0.1)); // 分辨率 0.1 米
    ros::Rate loop_rate(100); // 设置循环频率为 10 Hz

    // tf publisher
    tf::TransformBroadcaster br;

    Eigen::Quaterniond R_I_wrt_C = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());

    double last_time = 0;
    while (ros::ok()) {
        if (last_time >= cloud.header.stamp.toSec() ||
            fabs(pose.header.stamp.toSec() - cloud.header.stamp.toSec()) >= 0.5/30)
        {
            continue;
        }
        mutex_cloud.lock();
        mutex_pose.lock();
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));

        Eigen::Quaterniond q_eigen(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        q_eigen = q_eigen * R_I_wrt_C;
        pose.pose.orientation.x = q_eigen.x();
        pose.pose.orientation.y = q_eigen.y();
        pose.pose.orientation.z = q_eigen.z();
        pose.pose.orientation.w = q_eigen.w();

        transform.setRotation(tf::Quaternion(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w()));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "depth_camera_base")); // 发布变换 (变换, 时间戳, 父坐标系, 子坐标系)
        updateOctomap(cloud, pose, octree);
        last_time = cloud.header.stamp.toSec();
        mutex_pose.unlock();
        mutex_cloud.unlock();
        loop_rate.sleep();
    }

    return 0;
}

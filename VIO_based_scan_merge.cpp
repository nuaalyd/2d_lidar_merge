#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <iostream>
#include <dirent.h>
#include <fstream>
#include <cmath>
#include <chrono>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace chrono;

std::string int2string(int value)
{
    std::stringstream ss;
    ss << value;
    return ss.str();
}

class ScanMerge
{
public:
    ScanMerge()
    {
        ros::NodeHandle nh_param("~");

        nh_param.param<string>("lidar_topic", lidar_topic_, "/scan");
        nh_param.param<string>("odometry_topic", odometry_topic_, "/vins_estimator/imu_propagate");
        nh_param.param<string>("output_frame_id", output_frame_id_, "/merged_cloud");
        nh_param.param<string>("output_path", output_path_, "../");
        //nh_param.param<string>("config_file", config_file_, "../result.yaml");

        nh_param.param<int>("merge_frame_num", merge_frame_num_, 50);
        merge_frame_count_ = 0;
        output_count_ = 0;
        nh_param.param<int>("motion_mode", motion_mode_, 1);
        nh_param.param<int>("motion_dirction", motion_dirction_, 1);
        nh_param.param<double>("delta_x", dx_, 0.01);
        nh_param.param<double>("delta_angle", da_, 0.01);

        nh_param.param<bool>("save_ply", save_ply_, true);

        nh_param.param<bool>("scan_direction_clockwise", scan_direction_clockwise_, true);

        nh_param.param<bool>("use_range_filter", use_range_filter_, false);
        nh_param.param<double>("range_filter_min", range_filter_min_, 0.2);
        nh_param.param<double>("range_filter_max", range_filter_max_, 20.0);

        nh_param.param<bool>("use_angle_filter", use_angle_filter_, false);
        nh_param.param<double>("angle_filter_min", angle_filter_min_, -2.5);
        nh_param.param<double>("angle_filter_max", angle_filter_max_, 2.5);

        nh_param.param<bool>("use_radius_outlier_filter", use_radius_outlier_filter_, false);
        nh_param.param<double>("radius_outlier_filter_search_radius", radius_outlier_filter_search_radius_, 0.1);
        nh_param.param<int>("radius_outlier_filter_min_neighbors", radius_outlier_filter_min_neighbors_, 1);

        T_lc << -1.4944524239259415e-02, 9.2907831706560573e-01, 3.6958103570638529e-01, -8.7347968367012141e-02,
            -6.8336766752516900e-03, 3.6951877667239907e-01, -9.2919813524868422e-01, -4.2630133951408154e-02,
            -9.9986497191288815e-01, -1.6412021358620611e-02, 8.2673850997097337e-04, 4.4079899040471354e-02,
            0., 0., 0., 1.;
        T_bc << -0.00025202 ,-0.99999163 , 0.00408401, 0.00535959,
           0.99996739 ,-0.00021905 , 0.0080725, -0.04407191,
          -0.00807154 , 0.00408591 , 0.99995908, 0.02281941,
          0,0,0,1;
        T_bl = T_bc * T_lc.inverse();

        scan_sub_.subscribe(nh_, lidar_topic_, 5);
        odometry_sub_.subscribe(nh_, odometry_topic_, 100);
        sync.connectInput(scan_sub_, odometry_sub_);
        sync.registerCallback(boost::bind(&ScanMerge::VIO_based_ScanMergeCallback, this, _1, _2));

        merged_Cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged_cloud", 10);
    }
    ~ScanMerge()
    {
    }

    Eigen::Matrix4d combinePose(Eigen::Vector3d _position, Eigen::Quaterniond _orientation)
    {
        Eigen::Matrix4d Trans;
        Trans.setIdentity(); // Set to Identity to make bottom row of Matrix 0,0,0,1
        Trans.block<3, 3>(0, 0) = _orientation.toRotationMatrix();
        Trans.block<3, 1>(0, 3) = _position;
        return Trans;
    }

    void VIO_based_ScanMergeCallback(const sensor_msgs::LaserScanConstPtr &_lidar_msg, const nav_msgs::OdometryConstPtr &_odometry_msg)
    {
        ROS_INFO("time sync sucessfully!");
        // ROS_INFO("lidar time stamp:[%ld]", _lidar_msg->header.stamp.toNSec());
        // ROS_INFO("odometry time stamp:  [%ld]", _odometry_msg->header.stamp.toNSec());
        ROS_INFO("lidar-odometry time stamp: %f ms", ((long int)_lidar_msg->header.stamp.toNSec() - (long int)_odometry_msg->header.stamp.toNSec()) / 1.0e6);
        pcl::PointCloud<pcl::PointXYZI> pointcloud_temp;
        Eigen::Matrix4d T; // translation from lidar to body of first frame

        Eigen::Vector3d frame_position(_odometry_msg->pose.pose.position.x,
                                       _odometry_msg->pose.pose.position.y,
                                       _odometry_msg->pose.pose.position.z);
        Eigen::Quaterniond frame_orientation(_odometry_msg->pose.pose.orientation.w,
                                             _odometry_msg->pose.pose.orientation.x,
                                             _odometry_msg->pose.pose.orientation.y,
                                             _odometry_msg->pose.pose.orientation.z);
        Eigen::Matrix4d frame_T = combinePose(frame_position, frame_orientation);

        // initializaiton at first frame
        if (merge_frame_count_ == 0)
        {
            pointcloud_merged.header.frame_id = output_frame_id_;
            pcl_conversions::toPCL(ros::Time::now(), pointcloud_merged.header.stamp);
            T_bm = frame_T.inverse();
        }

        LaserScanToPointCloud(_lidar_msg, pointcloud_temp);

        //ApplyRangeFilter(pointcloud_temp);
        //ApplyAngleFilter(pointcloud_temp);
        ApplyRadiusOutlierFilter(pointcloud_temp);

        //T = T_bm * frame_T * T_bl; // lidar(last frame)->body(last frame)->world->body(first frame)
	T = frame_T * T_bl;

        pcl::PointCloud<pcl::PointXYZI> pointcloud_transed;
        pcl::transformPointCloud(pointcloud_temp, pointcloud_transed, T);
        pointcloud_merged += pointcloud_transed;
        merge_frame_count_++;

        // save the point cloud and publish it
        if (merge_frame_count_ >= merge_frame_num_)
        {
            time_t t = std::time(0);
            struct tm *now = std::localtime(&t);
            string savePath = output_path_ + "merged_cloud" +
                              '-' + int2string(now->tm_year + 1900) +
                              '-' + int2string(now->tm_mon + 1) +
                              '-' + int2string(now->tm_mday) +
                              '-' + int2string(now->tm_hour) +
                              '-' + int2string(now->tm_min) +
                              '-' + int2string(now->tm_sec) +
                              '-' + int2string(output_count_) + ".ply";
            if (save_ply_)
            {
                pcl::io::savePLYFileASCII<pcl::PointXYZI>(savePath, pointcloud_merged);
                ROS_INFO("merge cloud saved as: %s", savePath);
            }

            pcl::toROSMsg(pointcloud_merged, pointcloud_merged_msg);
            merged_Cloud_pub_.publish(pointcloud_merged_msg);

            pointcloud_merged.clear();
            merge_frame_count_ = 0;
            output_count_++;
        }
    }

    void LaserScanToPointCloud(sensor_msgs::LaserScan::ConstPtr _laser_scan, pcl::PointCloud<pcl::PointXYZI> &_pointcloud)
    {
        _pointcloud.clear();
        pcl::PointXYZI newPoint;
        double newPointAngle;

        int beamNum = _laser_scan->ranges.size();

        if (scan_direction_clockwise_ == true)
        {
            for (int i = beamNum - 1; i >= 0; i--)
            {
                newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;

                if (use_range_filter_)
                    if (_laser_scan->ranges[i] < range_filter_min_ || _laser_scan->ranges[i] > range_filter_max_) 
                        continue;

                if (use_angle_filter_)
                    if (newPointAngle < angle_filter_min_ || newPointAngle > angle_filter_max_)
                        continue;

                newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
                newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);
                newPoint.z = 0.0;
                newPoint.intensity = newPoint.z;
                _pointcloud.push_back(newPoint);
            }
        }
        // work in this case for rplidar s1
        else
        {
            for (int i = 0; i < beamNum; i++)
            {
                newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;

                if (use_range_filter_)
                    if (_laser_scan->ranges[i] < range_filter_min_ || _laser_scan->ranges[i] > range_filter_max_) 
                        continue;

                if (use_angle_filter_)
                    if (newPointAngle < angle_filter_min_ || newPointAngle > angle_filter_max_)
                        continue;
                        
                newPoint.x = -_laser_scan->ranges[i] * cos(newPointAngle);
                newPoint.y = -_laser_scan->ranges[i] * sin(newPointAngle);
                newPoint.z = 0.0;
                newPoint.intensity = newPoint.z;
                _pointcloud.push_back(newPoint);
            }
        }
    }

    void ApplyRangeFilter(pcl::PointCloud<pcl::PointXYZI> &_input)
    {
        if (use_range_filter_ == true)
        {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            for (int i = 0; i < _input.size(); i++)
            {
                float range = hypot(_input[i].x, _input[i].y);
                if (range > range_filter_min_ && range < range_filter_max_)
                {
                    cloud.push_back(_input[i]);
                }
            }
            _input.swap(cloud);
        }
    }

    void ApplyAngleFilter(pcl::PointCloud<pcl::PointXYZI> &_input)
    {
        if (use_angle_filter_ == true)
        {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            for (int i = 0; i < _input.size(); i++)
            {
                float angle = atan2(_input[i].y, _input[i].x);
                if (angle > angle_filter_min_ && angle < angle_filter_max_)
                {
                    cloud.push_back(_input[i]);
                }
            }
            _input.swap(cloud);
        }
    }

    void ApplyRadiusOutlierFilter(pcl::PointCloud<pcl::PointXYZI> &_input)
    {
        if (use_radius_outlier_filter_ == true)
        {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::RadiusOutlierRemoval<pcl::PointXYZI> sor;
            sor.setInputCloud(_input.makeShared());
            sor.setRadiusSearch(radius_outlier_filter_search_radius_);
            sor.setMinNeighborsInRadius(radius_outlier_filter_min_neighbors_);
            sor.setNegative(false);
            sor.filter(cloud);
            _input.swap(cloud);
        }
    }

public:
    ros::NodeHandle nh_;
    ros::Publisher merged_Cloud_pub_;

    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> sync_pol;
    message_filters::Synchronizer<sync_pol> sync{sync_pol(100)};

    Eigen::Vector3d localMap_position_;
    Eigen::Quaterniond localMap_orientation_;
    Eigen::Matrix4d T_bm; // localmap to body
    Eigen::Matrix4d T_bl, T_lc, T_bc;

    ros::Duration frame_duration, delay_duration;
    ros::Time current_output_timestamp;

    string lidar_topic_, odometry_topic_, output_frame_id_, output_path_, config_file_;

    int merge_frame_count_, merge_frame_num_, output_count_;

    int motion_mode_;     // 0 for rotationtranslation, 1 for translation
    int motion_dirction_; //-1 for counterclockwise/-y, 1 for clockwise/+y

    bool use_range_filter_;
    double range_filter_min_;
    double range_filter_max_;
    bool scan_direction_clockwise_;

    bool use_angle_filter_;
    double angle_filter_min_;
    double angle_filter_max_;

    bool use_radius_outlier_filter_;
    double radius_outlier_filter_search_radius_;
    int radius_outlier_filter_min_neighbors_;

    double dx_, da_;
    pcl::PointCloud<pcl::PointXYZI> pointcloud_merged;
    sensor_msgs::PointCloud2 pointcloud_merged_msg;

    bool save_ply_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_merge");

    ScanMerge myScanMerge;

    ros::spin();

    ROS_INFO("start VIO based scan merge, waiting for data......");

    return 0;
}

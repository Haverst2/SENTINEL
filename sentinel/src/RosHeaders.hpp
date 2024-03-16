#include <string>
#include <iostream>
#include <chrono>
#include <queue>
#include <thread>
#include <atomic>
#include <Eigen/Core>

// for responding to ctrl+c
#include <sys/wait.h>
#include <sys/types.h>

// ros basic
#include <ros/ros.h>
#include <ros/callback_queue.h>

// point cloud
#include <pcl_conversions/pcl_conversions.h>

// messages
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

// transform
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

// livox
#include "livox_ros_driver/CustomMsg.h"

// user's
#include "resources/resources.hpp"
#include "../include/util.h"
#include "resources.hpp"

const std::string STATIC_MAP_TOPIC = "/map_static";
const std::string DISTANCE_MAP_TOPIC = "/map_distance";
const std::string POINTCLOUD_TOPIC = "/livox/lidar";
const std::string PLAN_PATH_TOPIC = "/plan_path";
const std::string SET_GOAL_TOPIC = "/goal_point";
const std::string TF_TOPIC = "/tf";
const std::string ODOMETRY_TOPIC = "/odom";
const std::string REPATH_TOPIC = "/serial/path";

const std::string TF_MAP = "map";
const std::string TF_ODOMETRY_ORIGIN = "odom_origin";
const std::string TF_CAMERA_ORIGIN = "camera_origin";
const std::string TF_ODOMETRY_NOW = "base_link";
const std::string TF_CAMERA_NOW = "livox_frame";
#pragma once
#include "../RosHeaders.hpp"
#include "../../include/dynamic_voronoi/dynamicvoronoi.h"
#include <unordered_set>
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class DistanceManagerParams
{
public:
    DistanceManagerParams();
    double ignoreMinDistSQ, ignoreMaxDistSQ; // ignore points from livox_driver by distance_square
    double detectMinHeight, detectMaxHeight; // ignore points by z height
    float lidarTransform[3][4];              // the transform from base to lidar
    double camWatchAngleDeg;
    /*
    在distanceManager里，地图是以一定比例缩小后的（详见dvShrink）
    然后每个点第一次出现在雷达中时，并不会立即被认为是障碍物，而是给distanceManager
    中这个位置的点增加一定生命值HP，只有当生命值HP大于一个阈值才会认为是障碍物，
    同时当这个点并不在地图中时，也不认为它立刻消失了，而是随时间减少生命值HP，
    减小到上文的阈值时消失，从distanceManager中移除并判定为空地
    同时来自staticMap的障碍物的HP固定为hpObstacle，一般取-1
    */

    int addHPPerPoint; // add HP to the cell per point, more details see DistanceManagerParams
    int hpMax;         // the Max HP of the cell, more details see DistanceManagerParams
    /*
    when there are no points in the cell,
    the decreasing HP by time,
    more details see DistanceManagerParams
    */
    int minusHPPerFrame;
    /*
    when the HP of the cell is higher than hpConsiderLive,
    consider there is a obstancle in this cell,
    more details see DistanceManagerParams
    */
    int hpConsiderLive;
    /*
    if there is a obstancle from staticMap, this cell's HP,
    more details see DistanceManagerParams
    */
    int hpObstacle = -1;
    int hpUnknownTempObstacle = -2;
    int hpFree = 0;
    /*
    首先，由于Voronoi图更新有点慢，而且雷达扫出来的点时不时飘一下，
    不能直接用到更新distanceManager中，所以将地图缩小一定比例，存到Voronoi图中。
    缩小比例就是这个dvShrink
    */
    int dvShrink;

    bool publishDistanceMap;  // if publish the distance map, true for debug and publish
    double minThroughRadius;  // the minimum RADIUS for robot to go through, any radius less than this would lead to collision
    double bestThroughRadius; // the best RADIUS for robot to go through, all radius bigger than this is considered the same
};

class DistanceManager
{
public:
    const double INF = 1.145e14;

    DistanceManager();

    int mapWidth;
    int mapHeight;
    int dvWidth;
    int dvHeight;
    int *&distMapSQ;
    int *&unknownDistMapSQ;
    const DistanceManagerParams pm;

    // turn the (i,j) point to the idx for array in DV map
    int dvPoint(const int &i, const int &j);
    // turn the (i,j) point to the idx for array in static map
    int mapPoint(const int &i, const int &j);
    // turn the point to the idx for array in DV map (calls dvPoint(y,x))
    int dvPoint(const IntPoint &a);
    // turn the point to the idx for array in static map (calls dvPoint(y,x))
    int mapPoint(const IntPoint &a);

    // turn realpoints to points used in static map
    void RealPoint2MapPixel(const double &rx, const double &ry, int &mx, int &my);
    // turn static map points to realpoints
    void MapPixel2RealPoint(const int &mx, const int &my, double &rx, double &ry);
    const double minThroughRinDVMap();
    const double minThroughRinMap();
    const double minThroughRinDVMapSQ();
    const double minThroughRinMapSQ();
    bool isObstacle(const int &dvP);
    void setLidarCallBack(std::function<void()> AutoCall);
    bool isVoronoi(const int &i, const int &j);

    nav_msgs::OccupancyGrid staticMap;
    nav_msgs::OccupancyGrid distanceMap;

    enum CellStatus
    {
        OB_STATIC,
        OB_LIVE,
        OB_DYING,
        UNKNOWN,
        FREE
    };
    CellStatus getCellStatus(const int &dvP);

private:
    ros::Publisher distanceMapPub;
    ros::Subscriber staticMapSub;
    ros::Subscriber lidarSub;
    std::function<void()> callbackLidar;

    DynamicVoronoi dv;
    DynamicVoronoi unknownDV;
    std::unique_ptr<tf::TransformListener> tfListener;

    bool staticMapReceived = false;
    int *pointHP = 0;

    pcl::PointCloud<PointType> pointCloud;
    size_t pclSize = 0;
    ros::Time lidarTimeStamp;

    std::vector<IntPoint> obstaclePoints;
    size_t obstaclePointsSize;

    // some initializing after receiving the staticMap
    void initDM();
    // update distanceManager using the points from livox_driver
    void manageDistance();
    /*
    receive static map from map_server, other functions wouldn't start before received.
    also after received one static map, this func will ignore the messages afterwards.
    and calls initDM() after first receiving.
    */
    void subscribeStaticMap(const nav_msgs::OccupancyGrid::ConstPtr msg);
    // receive point cloud from livox_driver straightly, and then start other parts of the program
    void subscribeLidar(const livox_ros_driver::CustomMsgConstPtr &livox_msg_in);
};
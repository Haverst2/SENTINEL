#include "DistanceManager.hpp"
// 重要的代码都在上面哦
DistanceManagerParams::DistanceManagerParams()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    double lidarYaw, lidarRoll, lidarPitch;       // the transform from base to lidar
    double lidarTransX, lidarTransY, lidarTransZ; // the transform from base to lidar
    private_nh.param<double>("lidarTransX", lidarTransX, 0.0450753);
    private_nh.param<double>("lidarTransY", lidarTransY, 0);
    private_nh.param<double>("lidarTransZ", lidarTransZ, -0.5741402);
    private_nh.param<double>("ignoreMinDistSQ", ignoreMinDistSQ, 0.04);
    private_nh.param<double>("ignoreMaxDistSQ", ignoreMaxDistSQ, 9.00);
    private_nh.param<double>("lidarYawDeg", lidarYaw, 0); lidarYaw*=M_PI/180.0;
    private_nh.param<double>("lidarRollDeg", lidarRoll, 0);lidarRoll*=M_PI/180.0;
    private_nh.param<double>("lidarPitchDeg", lidarPitch, 37.0);lidarPitch*=M_PI/180.0;
    private_nh.param<double>("detectMinHeight", detectMinHeight, 0.2);
    private_nh.param<double>("detectMaxHeight", detectMaxHeight, 0.5);
    private_nh.param<double>("camWatchAngleDeg", camWatchAngleDeg, 45);
    private_nh.param<int>("addHPPerPoint", addHPPerPoint, 3);
    private_nh.param<int>("hpMax", hpMax, 20);
    private_nh.param<int>("minusHPPerFrame", minusHPPerFrame, 1);
    private_nh.param<int>("hpConsiderLive", hpConsiderLive, 7);
    private_nh.param<int>("hpObstacle", hpObstacle, -1);
    private_nh.param<int>("hpUnknownTempObstacle", hpUnknownTempObstacle, -2);
    private_nh.param<int>("hpFree", hpFree, 0);
    private_nh.param<int>("dvShrink", dvShrink, 3);
    private_nh.param<double>("minThroughRadius", minThroughRadius, 0.25);
    private_nh.param<double>("bestThroughRadius", bestThroughRadius, 1);
    nh.param<bool>("publishDistanceMap", publishDistanceMap, true);

    Eigen::Matrix3d _lidar_rotation(Eigen::AngleAxisd(lidarYaw, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(lidarPitch, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(lidarRoll, Eigen::Vector3d::UnitX()));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            lidarTransform[i][j] = _lidar_rotation(i, j);
    lidarTransform[0][3] = lidarTransX;
    lidarTransform[1][3] = lidarTransY;
    lidarTransform[2][3] = lidarTransZ;
}
DistanceManager::DistanceManager()
    : pm(), pclSize(0), distMapSQ(dv.distMapSQ),
      unknownDistMapSQ(unknownDV.distMapSQ), tfListener(new tf::TransformListener()),
      obstaclePoints(5e5 + 10), obstaclePointsSize(0)
{
    ros::NodeHandle nh;
    staticMapSub = nh.subscribe(STATIC_MAP_TOPIC, 1, &DistanceManager::subscribeStaticMap, this);
    lidarSub = nh.subscribe<livox_ros_driver::CustomMsg>(POINTCLOUD_TOPIC, 2, &DistanceManager::subscribeLidar, this);
    distanceMapPub = nh.advertise<nav_msgs::OccupancyGrid>(DISTANCE_MAP_TOPIC, 1);

    pointCloud.resize(5e4 + 10);
}
bool DistanceManager::isVoronoi(const int &i, const int &j)
{
    return dv.isVoronoi(i, j);
}

void DistanceManager::subscribeLidar(const livox_ros_driver::CustomMsgConstPtr &livox_msg)
{
    auto tbegin = std::chrono::high_resolution_clock::now();
    if (!staticMapReceived)
    {
        ROS_WARN("Static map not received yet. Skipping Callback.");
        return;
    }
    pclSize = 0;
    float x, y, z;
    for (auto &thisP : livox_msg->points)
    {
        float distSq = thisP.x * thisP.x + thisP.y * thisP.y + thisP.z * thisP.z;
        if (distSq < pm.ignoreMinDistSQ)
            continue;
        if (distSq > pm.ignoreMaxDistSQ)
            continue;
        PointType &pt = pointCloud[pclSize];
        auto &tf = pm.lidarTransform;
        // x = tf[0][0] * thisP.x + tf[0][1] * thisP.y + tf[0][2] * thisP.z - tf[0][3];
        // y = tf[1][0] * thisP.x + tf[1][1] * thisP.y + tf[1][2] * thisP.z - tf[1][3];
        z = tf[2][0] * thisP.x + tf[2][1] * thisP.y + tf[2][2] * thisP.z - tf[2][3];
        if (z > pm.detectMaxHeight)
            continue;
        if (z < pm.detectMinHeight)
            continue;
        pt.x = thisP.x;
        pt.y = thisP.y;
        pt.z = thisP.z;
        pclSize++;
        if (pclSize >= pointCloud.size() * 0.9) // this shouldn't happen, but for safe
            pointCloud.resize(pointCloud.size() + 5e4);
    }
    lidarTimeStamp.fromNSec(livox_msg->timebase);

    manageDistance();
    auto tend = std::chrono::high_resolution_clock::now();
    if (callbackLidar != nullptr)
        callbackLidar();

    ROS_INFO("Cost Time: %f ms", (tend - tbegin).count() / 1000000.0);
}

void DistanceManager::manageDistance()
{
    static tf::StampedTransform tf_Map2CamNow;
    static tf::StampedTransform tf_Map2OdomNow;
    static bool *updated = 0;
    if (updated == NULL)
        updated = new bool[dvHeight * dvWidth];
    memset(updated, 0, dvHeight * dvWidth);
    try
    {
        // ros::Rate wait_rate(10);
        // wait_rate.sleep();
        
        // double shit = lidarTimeStamp.toSec()+0.1;
        // lidarTimeStamp.fromSec(shit);
        tfListener->waitForTransform(TF_MAP, TF_CAMERA_NOW, lidarTimeStamp, ros::Duration(0.1));
        ROS_INFO("Want %.5f",lidarTimeStamp.toSec());
        tfListener->lookupTransform(TF_MAP, TF_CAMERA_NOW, lidarTimeStamp, tf_Map2CamNow);
        tfListener->lookupTransform(TF_MAP, TF_ODOMETRY_NOW, lidarTimeStamp, tf_Map2OdomNow);
        ROS_INFO("Dif %.5f",lidarTimeStamp.toSec() - tf_Map2OdomNow.stamp_.toSec());
        
    }
    catch (const tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    static float tMap[3][4];
    {
        tf::Matrix3x3 rot(tf_Map2CamNow.getRotation());
        tf::Vector3 tra = tf_Map2CamNow.getOrigin();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                tMap[i][j] = rot[i][j];
        for (int i = 0; i < 3; i++)
            tMap[i][3] = tra[i];
    }

    tf::Vector3 nowBasePos = tf_Map2OdomNow.getOrigin();
    tf::Matrix3x3 nowBaseRot(tf_Map2OdomNow.getRotation());
    float pX, pY, pZ;
    double nX = nowBasePos.x(), nY = nowBasePos.y(), nZ = nowBasePos.z();
    double fX = nowBaseRot[0][0], fY = nowBaseRot[0][1], fZ = nowBaseRot[0][2];
    for (int i = 0, mapX, mapY; i < pclSize; i++)
    {

        
        pX = tMap[0][0] * pointCloud[i].x + tMap[0][1] * pointCloud[i].y + tMap[0][2] * pointCloud[i].z + tMap[0][3];
        pY = tMap[1][0] * pointCloud[i].x + tMap[1][1] * pointCloud[i].y + tMap[1][2] * pointCloud[i].z + tMap[1][3];
        pZ = tMap[2][0] * pointCloud[i].x + tMap[2][1] * pointCloud[i].y + tMap[2][2] * pointCloud[i].z + tMap[2][3];
        double lenSQ = SQ(pX - nX) + SQ(pY - nY) + SQ(pZ - nZ);
        if(lenSQ<=SQ(pm.minThroughRadius))
            continue;
        if (pZ < nZ + pm.detectMinHeight || pZ > nZ + pm.detectMaxHeight)
            continue;
        RealPoint2MapPixel(pX, pY, mapX, mapY);
        if (mapX >= 0 && mapX < mapWidth && mapY >= 0 && mapY < mapHeight)
        {
            int dvP = dvPoint(mapY / pm.dvShrink, mapX / pm.dvShrink);
            auto pS = getCellStatus(dvP);
            if (pS == OB_LIVE)
            {
                pointHP[dvP] = std::min(pointHP[dvP] + pm.addHPPerPoint, pm.hpMax);
                updated[dvP] = true;
            }
            else if (pS == OB_DYING || pS == FREE)
            {
                pointHP[dvP] = std::min(pointHP[dvP] + pm.addHPPerPoint, pm.hpMax);
                updated[dvP] = true;
                if (pointHP[dvP] >= pm.hpConsiderLive)
                {
                    dv.occupyCell(mapY / pm.dvShrink, mapX / pm.dvShrink);
                }
                if (pS == FREE)
                    obstaclePoints[obstaclePointsSize++] = IntPoint(mapX / pm.dvShrink, mapY / pm.dvShrink);
            }
            else if (pS == UNKNOWN)
            {
                pointHP[dvP] = pm.addHPPerPoint;
                updated[dvP] = true;
                unknownDV.clearCell(mapY / pm.dvShrink, mapX / pm.dvShrink);
            }
        }
    }
    for (int i = 0; i < obstaclePointsSize; i++)
    {
        int dvP = dvPoint(obstaclePoints[i]);
        bool inWatchArea = false;
        auto pointStatus = getCellStatus(dvP); // 0:normal alive, 1:hp>0 but not alive, 2:hp=0, but not watch
        if (updated[dvP] == false)
        {
            double acX, acY, acZ = 0;
            MapPixel2RealPoint(obstaclePoints[i].x * pm.dvShrink, obstaclePoints[i].y * pm.dvShrink, acX, acY);
            double lenSQ = SQ(acX - nX) + SQ(acY - nY) + SQ(acZ - nZ);
            double angle = acos((fX * (acX - nX) + fY * (acY - nY) + fZ * (acZ - nZ)) / sqrt(lenSQ));

            if (lenSQ <= pm.ignoreMaxDistSQ &&
                angle <= pm.camWatchAngleDeg * M_PI / 180)
                inWatchArea = true;
            switch (pointStatus)
            {
            case OB_LIVE:
                pointHP[dvP] = std::max(0, pointHP[dvP] - pm.minusHPPerFrame);
                if (pointHP[dvP] < pm.hpConsiderLive)
                {
                    dv.clearCell(obstaclePoints[i].y, obstaclePoints[i].x);
                    if (inWatchArea)
                        ;
                    else
                    {
                        pointHP[dvP] = pm.hpUnknownTempObstacle;
                        unknownDV.occupyCell(obstaclePoints[i].y, obstaclePoints[i].x);
                    }
                }
                break;
            case OB_DYING:
                pointHP[dvP] = std::max(0, pointHP[dvP] - pm.minusHPPerFrame);
                if (pointHP[dvP] == 0)
                {
                    std::swap(obstaclePoints[i], obstaclePoints[obstaclePointsSize - 1]);
                    obstaclePointsSize--;
                    i--;
                    pointHP[dvP] = pm.hpFree;
                }
                break;
            case UNKNOWN:
                if (inWatchArea)
                {
                    pointHP[dvP] = pm.hpFree;
                    unknownDV.clearCell(obstaclePoints[i].y, obstaclePoints[i].x);

                    std::swap(obstaclePoints[i], obstaclePoints[obstaclePointsSize - 1]);
                    obstaclePointsSize--;
                    i--;
                }
                else
                {
                    if (lenSQ <= SQ(pm.minThroughRadius))
                    {
                        pointHP[dvP] = pm.hpFree;
                        unknownDV.clearCell(obstaclePoints[i].y, obstaclePoints[i].x);

                        std::swap(obstaclePoints[i], obstaclePoints[obstaclePointsSize - 1]);
                        obstaclePointsSize--;
                        i--;
                    }
                }
                break;
            default:
                break;
            }
        }
    }

    dv.update(false);
    unknownDV.update(false);

    if (pm.publishDistanceMap)
    {
        for (int i = 0; i < dvHeight; i++)
            for (int j = 0; j < dvWidth; j++)
            {
                int dvP = dvPoint(i, j);

                // if (pointHP[dvP] == pm.hpUnknownTempObstacle)
                //     distanceMap.data[dvP] = 255;
                // else
                //     distanceMap.data[dvP] =
                //         100 - std::min((int)unknownDV.distMapSQ[dvP] / (pm.dvShrink * pm.dvShrink), 100);

                if (isObstacle(dvP))
                    distanceMap.data[dvP] = 255;
                else if (getCellStatus(dvP) == UNKNOWN)
                    distanceMap.data[dvP] = 160;
                else
                {
                    distanceMap.data[dvP] =
                        100 - std::min((int)dv.distMapSQ[dvP] / (pm.dvShrink * pm.dvShrink), 100);
                }
            }
        distanceMapPub.publish(distanceMap);
    }
}

void DistanceManager::initDM()
{
    ROS_INFO("DM: initializing.");
    mapWidth = staticMap.info.width;
    mapHeight = staticMap.info.height;
    dvWidth = mapWidth / pm.dvShrink + 1;
    dvHeight = mapHeight / pm.dvShrink + 1;
    static bool *temp = 0;
    static bool **isOccupied = 0;
    if (pointHP != nullptr)
        delete[] pointHP;
    if (temp != nullptr)
        delete[] temp;
    if (isOccupied != nullptr)
        delete[] isOccupied;

    isOccupied = new bool *[dvHeight];
    temp = new bool[dvWidth * dvHeight];
    pointHP = new int[dvWidth * dvHeight];
    for (int i = 0; i < dvHeight; i++)
        isOccupied[i] = temp + dvWidth * i;
    memset(pointHP, 0, dvWidth * dvHeight * sizeof(int));
    memset(temp, 0, dvWidth * dvHeight * sizeof(bool));
    for (int i = 0; i < mapHeight; i++)
        for (int j = 0; j < mapWidth; j++)
            if (staticMap.data[mapPoint(i, j)] > 0)
            {
                int dvP = dvPoint(i / pm.dvShrink, j / pm.dvShrink);
                pointHP[dvP] = pm.hpObstacle;
                temp[dvP] = true;
            }

    for (int j = 0; j < dvWidth; j++)
    {
        pointHP[dvPoint(0, j)] = pm.hpObstacle;
        temp[dvPoint(0, j)] = true;
        pointHP[dvPoint(dvHeight - 1, j)] = pm.hpObstacle;
        temp[dvPoint(dvHeight - 1, j)] = true;
    }
    for (int i = 0; i < dvHeight; i++)
    {
        pointHP[dvPoint(i, 0)] = pm.hpObstacle;
        temp[dvPoint(i, 0)] = true;
        pointHP[dvPoint(i, dvWidth - 1)] = pm.hpObstacle;
        temp[dvPoint(i, dvWidth - 1)] = true;
    }

    dv.initializeMap(dvHeight, dvWidth, isOccupied);
    unknownDV.initializeEmpty(dvHeight, dvWidth);

    distanceMap.header = staticMap.header;
    distanceMap.info = staticMap.info;
    distanceMap.info.height = dvHeight;
    distanceMap.info.width = dvWidth;
    distanceMap.info.resolution *= pm.dvShrink;
    distanceMap.data.resize(dvHeight * dvWidth);

    ROS_INFO("DM: finished init.");
}
void DistanceManager::subscribeStaticMap(const nav_msgs::OccupancyGrid::ConstPtr msg)
{
    ROS_INFO("DM: Received StaticMap");
    if (!staticMapReceived)
    {
        staticMap = *msg; // 接收到静态地图数据
        staticMapReceived = true;
        initDM();
    }
    ROS_INFO("DM: Finished StaticMap");
}
int DistanceManager::dvPoint(const int &i, const int &j)
{
    return i * dvWidth + j;
}
int DistanceManager::mapPoint(const int &i, const int &j)
{
    return i * mapWidth + j;
}
int DistanceManager::dvPoint(const IntPoint &a)
{
    return a.y * dvWidth + a.x;
}
int DistanceManager::mapPoint(const IntPoint &a)
{
    return a.y * mapWidth + a.x;
}
void DistanceManager::RealPoint2MapPixel(const double &rx, const double &ry, int &mx, int &my)
{
    mx = (rx - staticMap.info.origin.position.x) / staticMap.info.resolution;
    my = (ry - staticMap.info.origin.position.y) / staticMap.info.resolution;
}
void DistanceManager::MapPixel2RealPoint(const int &mx, const int &my, double &rx, double &ry)
{
    rx = mx * staticMap.info.resolution + staticMap.info.origin.position.x;
    ry = my * staticMap.info.resolution + staticMap.info.origin.position.y;
}
const double DistanceManager::minThroughRinDVMap()
{
    return pm.minThroughRadius / staticMap.info.resolution / pm.dvShrink;
}
const double DistanceManager::minThroughRinMap()
{
    return pm.minThroughRadius / staticMap.info.resolution;
}
const double DistanceManager::minThroughRinDVMapSQ()
{
    return SQ(pm.minThroughRadius / staticMap.info.resolution / pm.dvShrink);
}
const double DistanceManager::minThroughRinMapSQ()
{
    return SQ(pm.minThroughRadius / staticMap.info.resolution);
}
bool DistanceManager::isObstacle(const int &dvP)
{
    return pointHP[dvP] == pm.hpObstacle || pointHP[dvP] >= pm.hpConsiderLive;
}
void DistanceManager::setLidarCallBack(std::function<void()> AutoCall)
{
    callbackLidar = AutoCall;
}
DistanceManager::CellStatus DistanceManager::getCellStatus(const int &dvP)
{
    if (pointHP[dvP] == pm.hpObstacle)
        return OB_STATIC;
    if (pointHP[dvP] == pm.hpUnknownTempObstacle)
        return UNKNOWN;
    if (pointHP[dvP] == 0)
        return FREE;
    if (pointHP[dvP] >= pm.hpConsiderLive)
        return OB_LIVE;
    else if (pointHP[dvP] > 0)
        return OB_DYING;
}
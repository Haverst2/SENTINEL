#include "../RosHeaders.hpp"
#include "../DistanceManager/DistanceManager.hpp"
#include "../CarStatus/CarStatus.hpp"
#include <queue>

class MinCostPlan
{
public:
    MinCostPlan();

private:
    ros::Publisher pathPub;
    ros::Publisher rePathPub;

    CarStatus carStatus;
    DistanceManager dm;
    void decide();
    void plan();
    void turnPath();
    bool needPlan = false;
    std::vector<IntPoint> dvPath;   // very safe size, hopefully QwQ
    std::vector<FloatPoint> smPath; // very safe size, hopefully QwQ
    nav_msgs::Path planPath;
    IntPoint nowPosInMap; // int point in static map's pixel
    IntPoint tarPosInMap; // int point in static map's pixel
    IntPoint nowPosInDV;  // int point in dv map's pixel
    IntPoint tarPosInDV;  // int point in dv map's pixel
    IntPoint nowNearVPos; // int point in dv map's pixel
    IntPoint tarNearVPos; // int point in dv map's pixel

    int dvSize = 0;
    int nowProgress = 0;
    double distToPath = 0;
    void updateProgress();
    void updateVoronoiNearPos();
    double Fd(const double &real_dist);
    double FdUknown(const double &realDist);
    double routeCost(const IntPoint &dvA, const IntPoint &dvB);

    ros::Publisher distanceMapPub;
};

MinCostPlan::MinCostPlan() : dvPath(5e5 + 10), smPath(5e5 + 10)
{
    ros::NodeHandle nh;
    distanceMapPub = nh.advertise<nav_msgs::OccupancyGrid>("planner/map", 1);

    pathPub = nh.advertise<nav_msgs::Path>(PLAN_PATH_TOPIC, 1);
    rePathPub = nh.advertise<nav_msgs::Path>(REPATH_TOPIC, 1);
    carStatus.setTopic(CarStatus::Odometry, ODOMETRY_TOPIC);
    carStatus.setTopic(CarStatus::Goal, SET_GOAL_TOPIC);
    dm.setLidarCallBack(
        [this]() -> void
        {
            carStatus.tarPos = DoublePoint(6, 0);
            tarPosInMap.x = (carStatus.tarPos.x - dm.staticMap.info.origin.position.x) / dm.staticMap.info.resolution;
            tarPosInMap.y = (carStatus.tarPos.y - dm.staticMap.info.origin.position.y) / dm.staticMap.info.resolution;
            tarPosInDV = tarPosInMap / dm.pm.dvShrink;
            updateVoronoiNearPos();
            decide();
        });
    carStatus.setCallBack(CarStatus::Odometry,
                          [this]() -> void
                          {
                              nowPosInMap.x = (carStatus.nowPos.x - dm.staticMap.info.origin.position.x) / dm.staticMap.info.resolution;
                              nowPosInMap.y = (carStatus.nowPos.y - dm.staticMap.info.origin.position.y) / dm.staticMap.info.resolution;
                              nowPosInDV = nowPosInMap / dm.pm.dvShrink;
                              updateVoronoiNearPos();
                          });
    carStatus.setCallBack(CarStatus::Goal,
                          [this]() -> void
                          {
                              needPlan = true;
                              tarPosInMap.x = (carStatus.tarPos.x - dm.staticMap.info.origin.position.x) / dm.staticMap.info.resolution;
                              tarPosInMap.y = (carStatus.tarPos.y - dm.staticMap.info.origin.position.y) / dm.staticMap.info.resolution;
                              tarPosInDV = tarPosInMap / dm.pm.dvShrink;
                              updateVoronoiNearPos();
                          });
}
void smoothPath(const int &n,
                const std::vector<IntPoint> &inputPath,
                std::vector<FloatPoint> &outputPath,
                // int &outputSize,
                const double &alpha,
                const double &beta,
                const int &steps = 2000)
{
    static std::vector<FloatPoint> nowPath(5e5 + 10);  // very safe size, hopefully QwQ
    static std::vector<FloatPoint> gradient(5e5 + 10); // very safe size, hopefully QwQ
    const double stepSize = 0.001;
    for (int i = 0; i < n; i++)
        nowPath[i].x = inputPath[i].x,
        nowPath[i].y = inputPath[i].y;
    for (int i = 0; i < steps; i++)
    {
        for (int j = 1; j + 1 < n; j++)
            gradient[j] = (nowPath[j] - FloatPoint(inputPath[j])) * alpha + (nowPath[j] * 2 - nowPath[j - 1] - nowPath[j + 1]) * beta;
        for (int j = 1; j + 1 < n; j++)
            nowPath[j] = nowPath[j] - gradient[j] * stepSize;
    }
    for (int i = 0; i < n; i++)
        outputPath[i] = nowPath[i];
}
double MinCostPlan::Fd(const double &realDist) // real-obstacle's cost tells robot to keep distance with it
{
    return 100.0 / (1 + exp(30 * (realDist - dm.pm.minThroughRadius))) + 1;
}
double MinCostPlan::FdUknown(const double &realDist) // unknown-obstacle's cost only happens when going through
{
    // return 10.0 / (1 + exp(10 * (realDist - 0)));
    return 20.0 / (1 + exp(5 * (realDist - 0)));
}
double MinCostPlan::routeCost(const IntPoint &dvA, const IntPoint &dvB)
{
    static DoublePoint A, B;
    static double distMid, distMidUnknown;
    distMid = (sqrt(dm.distMapSQ[dm.dvPoint(dvA)]) + sqrt(dm.distMapSQ[dm.dvPoint(dvB)])) / 2;
    if (dm.getCellStatus(dm.dvPoint(dvA)) == DistanceManager::UNKNOWN || dm.getCellStatus(dm.dvPoint(dvB)) == DistanceManager::UNKNOWN)
        distMidUnknown = 0;
    else
        distMidUnknown = (sqrt(dm.unknownDistMapSQ[dm.dvPoint(dvA)]) + sqrt(dm.unknownDistMapSQ[dm.dvPoint(dvB)])) / 2;
    dm.MapPixel2RealPoint(dvA.x * dm.pm.dvShrink, dvA.y * dm.pm.dvShrink, A.x, A.y);
    dm.MapPixel2RealPoint(dvB.x * dm.pm.dvShrink, dvB.y * dm.pm.dvShrink, B.x, B.y);
    distMid *= dm.pm.dvShrink * dm.staticMap.info.resolution;
    distMidUnknown *= dm.pm.dvShrink * dm.staticMap.info.resolution;
    return (A - B).len() * (Fd(distMid) + FdUknown(distMidUnknown));
}

const int dirX[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
const int dirY[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
struct QUE_HELPER
{
    IntPoint v;
    float dist;
    QUE_HELPER() {}
    QUE_HELPER(const IntPoint &V, const double &D) : v(V), dist(D) {}
    ~QUE_HELPER() {}
    bool operator<(const QUE_HELPER &rhs) const
    {
        return dist > rhs.dist;
    }
};
void MinCostPlan::plan()
{
    static float *mapDist = 0;
    static IntPoint *mapPre = 0;
    if (mapDist == NULL)
        mapDist = new float[dm.dvHeight * dm.dvWidth];
    if (mapPre == NULL)
        mapPre = new IntPoint[dm.dvHeight * dm.dvWidth];
    memset(mapDist, 0, dm.dvHeight * dm.dvWidth * sizeof(float));
    // for (int i = 0; i < dm.dvHeight * dm.dvWidth; i++)
    //     mapDist[i] = 0;
    dvSize = 0;
    const int tarDVP = dm.dvPoint(tarPosInDV);
    const int nowDVP = dm.dvPoint(nowPosInDV);
    if (dm.isObstacle(tarDVP) || dm.distMapSQ[tarDVP] <= dm.minThroughRinDVMapSQ())
    {
        ROS_INFO("MinCostPlanner: Tar unreachable, Now(%d,%d) Tar(%d,%d)",
                 nowPosInMap.x, nowPosInMap.y, tarPosInMap.x, tarPosInMap.y);
        needPlan = true;
        return;
    }
    if (dm.distMapSQ[nowDVP] <= dm.minThroughRinDVMapSQ())
    {
        ROS_INFO("MinCostPlanner: Being To Close with Obstacle, Now(%d,%d) Tar(%d,%d)",
                 nowPosInMap.x, nowPosInMap.y, tarPosInMap.x, tarPosInMap.y);
        DoublePoint dir = nowNearVPos - nowPosInDV;
        double len = dir.len();
        dir = dir / len;
        for (int i = 0; i < len; i++)
            dvPath[dvSize++] = nowPosInDV + dir * i;
        dvPath[dvSize++] = nowNearVPos;
        smoothPath(dvSize, dvPath, smPath, 1, 2);
        needPlan = false;
        nowProgress = 0;
        distToPath = 0;
        return;
    }
    std::priority_queue<QUE_HELPER> que;
    mapPre[tarDVP].x = mapPre[tarDVP].y = 0;
    mapDist[tarDVP] = 1;
    que.push(QUE_HELPER(tarPosInDV, 1));
    int updatedSize = 0;
    int popSize = 0;
    while (!que.empty())
    {
        const QUE_HELPER qh = que.top();
        que.pop();
        popSize++;
        const IntPoint u = qh.v;
        int uP = dm.dvPoint(u);
        if (mapDist[uP] != qh.dist)
            continue;
        updatedSize++;
        IntPoint v;
        int vP;
        double vDist;
        DistanceManager::CellStatus cs;
        for (int i = -1; i < 2; i++)
            for (int j = -1; j < 2; j++)
            {
                if (i == 0 && j == 0)
                    continue;
                v.x = u.x + i;
                v.y = u.y + j;
                vP = dm.dvPoint(v);
                if (v.x < 0 || v.x >= dm.dvWidth || v.y < 0 || v.y >= dm.dvHeight)
                    continue;
                cs = dm.getCellStatus(vP);
                if (cs == DistanceManager::OB_STATIC ||
                    cs == DistanceManager::OB_LIVE ||
                    dm.distMapSQ[vP] <= dm.minThroughRinDVMapSQ())
                    continue;
                if (mapDist[vP] >= 0.5 && mapDist[vP] <= vDist)
                    continue;
                vDist = mapDist[uP] + routeCost(u, v);

                mapDist[vP] = vDist;
                mapPre[vP] = u;
                que.emplace(v, vDist);
                if (v == nowPosInDV)
                    break;
            }
    }
    nav_msgs::OccupancyGrid distMap;
    distMap.header = dm.staticMap.header;
    distMap.info = dm.staticMap.info;
    distMap.info.height = dm.dvHeight;
    distMap.info.width = dm.dvWidth;
    distMap.info.resolution *= dm.pm.dvShrink;
    distMap.data.resize(dm.dvHeight * dm.dvWidth);
    for (int i = 0; i < dm.dvHeight * dm.dvWidth; i++)
        distMap.data[i] = mapDist[i] * 10 + 10;
    distanceMapPub.publish(distMap);
    if (mapDist[dm.dvPoint(nowPosInDV)] == 0) // can't reach target
    {
        ROS_INFO("MinCostPlanner: No Path available, Now(%d,%d) Tar(%d,%d) %d %d",
                 nowPosInMap.x, nowPosInMap.y, tarPosInMap.x, tarPosInMap.y, updatedSize, popSize);
        needPlan = true;
        return;
    }
    else // get planned!
    {
        ROS_INFO("%f", mapDist[dm.dvPoint(nowPosInDV)]);
        dvPath[dvSize++] = nowPosInDV;
        IntPoint nowP = nowPosInDV;
        while (nowP != tarPosInDV)
        {
            nowP = mapPre[dm.dvPoint(nowP)];
            dvPath[dvSize++] = nowP;
        }
        smoothPath(dvSize, dvPath, smPath, 1, 2);
        needPlan = false;
        nowProgress = 0;
        distToPath = 0;
    }
}
void MinCostPlan::decide()
{
    if (!needPlan)
    {
        for (int i = nowProgress; i < dvSize; i++)
        {
            IntPoint &nP = dvPath[i];
            int dvP = dm.dvPoint(nP);
            if (dm.isObstacle(dvP) || dm.distMapSQ[dvP] <= dm.minThroughRinDVMapSQ() * 1.44)
            {
                needPlan = true;
                ROS_INFO("need to replan: Plan Invalid");
                break;
            }
        }
    }
    if (!needPlan)
    {
        updateProgress();
        if (distToPath > dm.pm.minThroughRadius)
        {
            needPlan = true;
            ROS_INFO("need to replan: Dist Away %f", distToPath);
        }
        if (dvSize - nowProgress < 3)
        {
            needPlan = true;
            ROS_INFO("need to replan: About To Reach Tar");
        }
    }
    if (needPlan)
    {
        plan();
        if (!needPlan) // get a reachable path
        {
            geometry_msgs::PoseStamped pose_stamped;
            planPath.header.stamp = pose_stamped.header.stamp = ros::Time::now();
            planPath.header.frame_id = pose_stamped.header.frame_id = TF_MAP;
            planPath.poses.clear();
            pose_stamped.pose.position = geometry_msgs::Point();
            pose_stamped.pose.orientation = geometry_msgs::Quaternion();
            for (int i = 0; i < dvSize; i++)
            {
                dm.MapPixel2RealPoint(smPath[i].x * dm.pm.dvShrink, smPath[i].y * dm.pm.dvShrink,
                                      pose_stamped.pose.position.x, pose_stamped.pose.position.y);
                planPath.poses.push_back(pose_stamped);
            }
            pathPub.publish(planPath);
        }
        else // wandering
        {
        }
    }
    else
    {
    }
    turnPath();
}
void MinCostPlan::updateProgress()
{
    // const DoublePoint &nowPos = carStatus.nowPos;
    const FloatPoint &nowPos = nowPosInDV;
    double minD = 1.145e14;
    int minIdx = 0;
    for (int i = 0; i < dvSize; i++)
    {
        const FloatPoint &nowP = smPath[i];
        double nowD = (nowP - nowPos).lenSQ<double>();
        if (nowD < minD)
        {
            minD = nowD;
            minIdx = i;
        }
    }
    nowProgress = minIdx;
    distToPath = sqrt(minD) * dm.staticMap.info.resolution * dm.pm.dvShrink;
    ROS_INFO("%f", distToPath);
}
void MinCostPlan::turnPath()
{
    const double stepLen = 0.15 / dm.staticMap.info.resolution / dm.pm.dvShrink; // 10cm
    static DoublePoint nowPath[PLAN_SIZE];
    int idx = 0;
    DoublePoint nowPos = nowPosInDV;
    double nowStepLen = 0;
    for (int i = nowProgress; i < dvSize && idx < PLAN_SIZE; i++)
    {
        while (idx < PLAN_SIZE)
        {
            const DoublePoint &tarP = smPath[i];
            DoublePoint dir = tarP - nowPos;
            double len = dir.len();
            dir = dir / len;
            if (len + nowStepLen < stepLen)
            {
                nowStepLen += len;
                nowPos = tarP;
                break;
            }
            else
            {
                double oLen = stepLen - nowStepLen;
                nowPath[idx++] = nowPos + dir * oLen;
                nowPos = nowPos + dir * oLen;
                nowStepLen = 0;
            }
        }
    }
    if (idx == 0)
        nowPath[idx++] = nowPos;
    while (idx < PLAN_SIZE)
    {
        nowPath[idx] = nowPath[idx - 1];
        idx++;
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = TF_MAP;

    nav_msgs::Path newPath;
    geometry_msgs::PoseStamped pose_stamped;
    newPath.header = pose_stamped.header = header;
    pose_stamped.pose.position = geometry_msgs::Point();
    pose_stamped.pose.orientation = geometry_msgs::Quaternion();
    newPath.poses.clear();
    for (int i = 0; i < PLAN_SIZE; i++)
    {
        dm.MapPixel2RealPoint(nowPath[i].x * dm.pm.dvShrink, nowPath[i].y * dm.pm.dvShrink,
                              pose_stamped.pose.position.x, pose_stamped.pose.position.y);
        newPath.poses.push_back(pose_stamped);
    }
    rePathPub.publish(newPath);
}
void MinCostPlan::updateVoronoiNearPos()
{
    int minNowD = 0x7ffffff, minTarD = 0x7ffffff;
    for (int i = 0; i < dm.dvHeight; i++)
        for (int j = 0; j < dm.dvWidth; j++)
        {
            if (dm.isVoronoi(i, j))
            {
                if (dm.distMapSQ[dm.dvPoint(i, j)] < dm.minThroughRinDVMap() * dm.minThroughRinDVMap())
                    continue;
                int nowD = SQ(j - nowPosInDV.x) + SQ(i - nowPosInDV.y);
                int tarD = SQ(j - tarPosInDV.x) + SQ(i - tarPosInDV.y);
                if (nowD < minNowD)
                    minNowD = nowD, nowNearVPos.x = j, nowNearVPos.y = i;
                if (tarD < minTarD)
                    minTarD = tarD, tarNearVPos.x = j, tarNearVPos.y = i;
            }
        }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "minCostPlanner");
    MinCostPlan mp;
    ros::spin();
    return 0;
}
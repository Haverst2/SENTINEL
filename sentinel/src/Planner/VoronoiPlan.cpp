#include "../RosHeaders.hpp"
#include "../DistanceManager/DistanceManager.hpp"
#include "../CarStatus/CarStatus.hpp"

class VoronoiPlanner
{
public:
    VoronoiPlanner();

private:
    ros::Publisher pathPub;
    ros::Publisher rePathPub;

    CarStatus carStatus;
    DistanceManager dm;
    void decide();
    void plan();
    void turnPath();
    bool needPlan = true;
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
    void updateVoronoiNearPos();
    void updateProgress();
};

VoronoiPlanner::VoronoiPlanner() : dvPath(5e5 + 10), smPath(5e5 + 10)
{
    ros::NodeHandle nh;
    pathPub = nh.advertise<nav_msgs::Path>(PLAN_PATH_TOPIC, 1);
    rePathPub = nh.advertise<nav_msgs::Path>(REPATH_TOPIC, 1);
    carStatus.setTopic(CarStatus::Odometry, ODOMETRY_TOPIC);
    carStatus.setTopic(CarStatus::Goal, SET_GOAL_TOPIC);
    dm.setLidarCallBack(
        [this]() -> void
        {
            // carStatus.tarPos = DoublePoint(6, 0);
            // tarPosInMap.x = (carStatus.tarPos.x - dm.staticMap.info.origin.position.x) / dm.staticMap.info.resolution;
            // tarPosInMap.y = (carStatus.tarPos.y - dm.staticMap.info.origin.position.y) / dm.staticMap.info.resolution;
            // tarPosInDV = tarPosInMap / dm.pm.dvShrink;
            updateVoronoiNearPos();
            decide();
        });
    carStatus.setCallBack(CarStatus::Odometry,
                          [this]() -> void
                          {
                              nowPosInMap.x = (carStatus.nowPos.x - dm.staticMap.info.origin.position.x) / dm.staticMap.info.resolution;
                              nowPosInMap.y = (carStatus.nowPos.y - dm.staticMap.info.origin.position.y) / dm.staticMap.info.resolution;
                              nowPosInDV = nowPosInMap / dm.pm.dvShrink;
                          });
    carStatus.setCallBack(CarStatus::Goal,
                          [this]() -> void
                          {
                              needPlan = true;
                              tarPosInMap.x = (carStatus.tarPos.x - dm.staticMap.info.origin.position.x) / dm.staticMap.info.resolution;
                              tarPosInMap.y = (carStatus.tarPos.y - dm.staticMap.info.origin.position.y) / dm.staticMap.info.resolution;
                              tarPosInDV = tarPosInMap / dm.pm.dvShrink;
                          });
}
void VoronoiPlanner::updateVoronoiNearPos()
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
    // int lastPush = 0;
    // int outSize = 0;
    // outputPath[outSize++] = nowPath[0];
    // for (int i = 0; i < n; i++)
    // {
    //     if (i > 0 && i + 1 < n)
    //     {
    //         FloatPoint a = nowPath[i] - nowPath[lastPush], b = nowPath[i] - nowPath[i + 1];
    //         double aLen = a.len(), bLen = b.len();
    //         double abCross = fabs(a.x * b.y - a.y * b.x);
    //         double angle = abCross / aLen / bLen;
    //         if (angle > 120 * M_PI / 180)
    //             continue;
    //     }
    //     outputPath[outSize++] = nowPath[i];
    //     lastPush = i;
    // }
    // outputSize = outSize;
}
void VoronoiPlanner::plan()
{
    static int *mapDist = 0;
    static IntPoint *mapPre = 0;
    if (mapDist == NULL)
        mapDist = new int[dm.dvHeight * dm.dvWidth];
    if (mapPre == NULL)
        mapPre = new IntPoint[dm.dvHeight * dm.dvWidth];
    memset(mapDist, 0, dm.dvHeight * dm.dvWidth * sizeof(int));

    dvSize = 0;
    int tarDVP = dm.dvPoint(tarPosInDV);
    if (dm.isObstacle(tarDVP) || dm.distMapSQ[tarDVP] <= dm.minThroughRinDVMapSQ())
    {
        ROS_INFO("VoronoiPlanner: Tar unreachable, Now(%d,%d) Tar(%d,%d)",
                 nowPosInMap.x, nowPosInMap.y, tarPosInMap.x, tarPosInMap.y);
        needPlan = true;
        return;
    }
    std::queue<IntPoint> que;
    int tarNearP = dm.dvPoint(tarNearVPos);
    que.push(tarNearVPos);
    mapPre[tarNearP].x = mapPre[tarNearP].y = 0;
    mapDist[tarNearP] = 1;
    while (!que.empty())
    {
        const IntPoint u = que.front();
        IntPoint v;
        int uP = dm.dvPoint(u);
        que.pop();
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                v.x = u.x + dx;
                v.y = u.y + dy;
                int vP = dm.dvPoint(v);
                if (v.x < 0 || v.x >= dm.dvWidth || v.y < 0 || v.y >= dm.dvHeight)
                    continue;
                if (!dm.isVoronoi(v.y, v.x) || dm.distMapSQ[vP] <= dm.minThroughRinDVMapSQ() * 1.44)
                    continue;
                if (mapDist[vP] != 0)
                    continue;
                mapDist[vP] = mapDist[uP] + 1;
                mapPre[vP] = u;
                que.push(v);
                if (v == nowNearVPos)
                    break;
            }
    }

    if (mapDist[dm.dvPoint(nowNearVPos)] == 0) // can't reach target
    {
        ROS_INFO("VoronoiPlanner: No Path available, Now(%d,%d) Tar(%d,%d)",
                 nowPosInMap.x, nowPosInMap.y, tarPosInMap.x, tarPosInMap.y);
        needPlan = true;
        return;
    }
    else // get planned!
    {
        dvPath[dvSize++] = nowPosInDV;
        dvPath[dvSize++] = nowNearVPos;
        IntPoint nowP = nowNearVPos;
        while (nowP != tarNearVPos)
        {
            nowP = mapPre[dm.dvPoint(nowP)];
            dvPath[dvSize++] = nowP;
        }
        dvPath[dvSize++] = tarPosInDV;
        smoothPath(dvSize, dvPath, smPath, 1, 2);
        needPlan = false;
        nowProgress = 0;
        distToPath = 0;
    }
}
void VoronoiPlanner::decide()
{
    if (!needPlan)
    {
        for (int i = nowProgress; i < dvSize; i++)
        {
            IntPoint &nP = dvPath[i];
            int dvP = dm.dvPoint(nP);
            if (dm.isObstacle(dvP) || dm.distMapSQ[dvP] <= dm.minThroughRinDVMapSQ())
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
void VoronoiPlanner::updateProgress()
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
void VoronoiPlanner::turnPath()
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voronoiPlanner");
    VoronoiPlanner vp;
    ros::spin();
    return 0;
}
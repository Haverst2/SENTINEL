#include "../RosHeaders.hpp"
#include <functional>
#include <unordered_map>
class CarStatus
{
public:
    DoublePoint nowPos;
    double yaw;
    ros::Time nowPosTimeStamp;
    DoublePoint tarPos;
    ros::Time tarPosTimeStamp;
    DoublePoint velocity;
    ros::Time velocityTimeStamp;
    nav_msgs::Path planPath;
    nav_msgs::Path rePath;
    tf::StampedTransform odomTF;

    CarStatus();

    enum EVENT
    {
        Transform,
        Velocity,
        Path,
        Goal,
        Odometry,
        RePath
    };
    void setCallBack(const EVENT &event, std::function<void()> callBack);
    void setTopic(const EVENT &event, const std::string &topic);

private:
    std::unordered_map<EVENT, std::function<void()>> callbacks;

    void subscribeTransform(const tf::tfMessageConstPtr &tfMsg);
    void subscribeVelocity(const geometry_msgs::Twist::ConstPtr &twist);
    void subscribePath(const nav_msgs::PathConstPtr &path);
    void subscribeGoal(const geometry_msgs::PointConstPtr &goal);
    void subscribeOdom(const nav_msgs::OdometryConstPtr &odom);
    void subscribeRePath(const nav_msgs::PathConstPtr &path);
    void turnPath();
    ros::Subscriber pathSub;
    ros::Subscriber twistSub;
    ros::Subscriber tfSub;
    ros::Subscriber goalSub;
    ros::Subscriber odomSub;
    ros::Subscriber rePathSub;
    std::unique_ptr<tf::TransformListener> tfListener;
};
#include "CarStatus.hpp"

CarStatus::CarStatus()
    : tfListener(new tf::TransformListener()) {}
void CarStatus::setTopic(const EVENT &event, const std::string &topic)
{
    ros::NodeHandle nh;
    switch (event)
    {
    case Transform:
        tfSub = nh.subscribe<tf::tfMessage>(topic, 1, &CarStatus::subscribeTransform, this);
        break;
    case Velocity:
        twistSub = nh.subscribe<geometry_msgs::Twist>(topic, 1, &CarStatus::subscribeVelocity, this);
        break;
    case Path:
        pathSub = nh.subscribe<nav_msgs::Path>(topic, 1, &CarStatus::subscribePath, this);
        break;
    case Odometry:
        odomSub = nh.subscribe<nav_msgs::Odometry>(topic, 1, &CarStatus::subscribeOdom, this);
        break;
    case Goal:
        goalSub = nh.subscribe<geometry_msgs::Point>(topic, 3, &CarStatus::subscribeGoal, this);
        break;
    case RePath:
        rePathSub = nh.subscribe<nav_msgs::Path>(topic, 1, &CarStatus::subscribeRePath, this);
    default:
        break;
    }
}
void CarStatus::setCallBack(const EVENT &event, std::function<void()> callBack)
{
    callbacks[event] = callBack;
}
void CarStatus::subscribeGoal(const geometry_msgs::PointConstPtr &goal)
{
    tarPos.x = goal->x;
    tarPos.y = goal->y;
    if (callbacks.find(EVENT::Goal) != callbacks.end())
        callbacks[EVENT::Goal]();
}
void CarStatus::subscribeOdom(const nav_msgs::OdometryConstPtr &odom)
{
    static tf::StampedTransform tf_Map2OdomOrigin;
    static tf::StampedTransform tf_Odom2Cam;
    static tf::StampedTransform tf_CamOrigin2CamNow;
    static tf::Transform tf_Map2Base;
    static bool init = false;
    if (!init)
    {
        try
        {
            tfListener->lookupTransform(TF_MAP, TF_ODOMETRY_ORIGIN, ros::Time(0), tf_Map2OdomOrigin);
            tfListener->lookupTransform(TF_ODOMETRY_ORIGIN, TF_CAMERA_ORIGIN, ros::Time(0), tf_Odom2Cam);
            init = true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }
    tf_CamOrigin2CamNow.setOrigin(
        tf::Vector3(odom->pose.pose.position.x,
                    odom->pose.pose.position.y,
                    odom->pose.pose.position.z));
    tf_CamOrigin2CamNow.setRotation(
        tf::Quaternion(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                       odom->pose.pose.orientation.z, odom->pose.pose.orientation.w));
    tf_Map2Base = tf_Map2OdomOrigin * tf_Odom2Cam * tf_CamOrigin2CamNow * tf_Odom2Cam.inverse();
    tf::Matrix3x3 rotation(tf_Map2Base.getRotation());
    tf::Vector3 translation = tf_Map2Base.getOrigin();
    double R, P, Y;
    rotation.getRPY(R, P, Y);
    yaw = Y;
    nowPos.x = translation.x();
    nowPos.y = translation.y();
    nowPosTimeStamp = odom->header.stamp;

    tf::Vector3 _vel(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);
    _vel = tf_Map2OdomOrigin * tf_Odom2Cam * _vel;
    velocity.x = _vel.x();
    velocity.y = _vel.y();
    ROS_INFO("Odom Trans(%6.2f, %6.2f, %6.2f), Rot(p=%5.2f, y=%5.2f, r=%5.2f), Vel(%5.2f, %5.2f)",
             translation.x(), translation.y(), translation.z(), P, Y, R, velocity.x, velocity.y);

    if (callbacks.find(EVENT::Odometry) != callbacks.end())
        callbacks[EVENT::Odometry]();

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin(translation + _vel * 0.3);
    // transform.setRotation(tf_Map2Base.getRotation());
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), TF_ODOMETRY_ORIGIN, "tf_vel"));
}
void CarStatus::subscribeTransform(const tf::tfMessageConstPtr &tfMsg)
{
    try
    {
        tfListener->lookupTransform(TF_MAP, TF_ODOMETRY_NOW, ros::Time(0), odomTF);
        tf::Matrix3x3 rotation(odomTF.getRotation());
        tf::Vector3 translation = odomTF.getOrigin();
        double R, P, Y;
        rotation.getRPY(R, P, Y);
        ROS_INFO("Receive Translation: (%6.2f, %6.2f, %6.2f), Rotation: (pitch=%5.2f, yaw=%5.2f, roll=%5.2f) %f",
                 translation.x(), translation.y(), translation.z(), P, Y, R, ros::Time::now().toSec());
        yaw = Y;
        nowPos.x = translation.x();
        nowPos.y = translation.y();
        nowPosTimeStamp = ros::Time::now();
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    if (callbacks.find(EVENT::Transform) != callbacks.end())
        callbacks[EVENT::Transform]();
}
void CarStatus::subscribeVelocity(const geometry_msgs::Twist::ConstPtr &twist)
{
    velocity.x = twist->linear.x;
    velocity.y = twist->linear.y;
    velocityTimeStamp = ros::Time::now();
    if (callbacks.find(EVENT::Velocity) != callbacks.end())
        callbacks[EVENT::Velocity]();
}
void CarStatus::subscribePath(const nav_msgs::PathConstPtr &path)
{
    planPath = *path;
    if (callbacks.find(EVENT::Path) != callbacks.end())
        callbacks[EVENT::Path]();
}
void CarStatus::subscribeRePath(const nav_msgs::PathConstPtr &path)
{
    rePath = *path;
    if (callbacks.find(EVENT::RePath) != callbacks.end())
        callbacks[EVENT::RePath]();
}
#include "../RosHeaders.hpp"
#include "rmSerial.hpp"
#include "../CarStatus/CarStatus.hpp"

std::atomic<bool> endProgram(false);
ros::Time startTime;
bool shouldBeUpdate = false;
Sentinel::Serials rmSerial("/dev/ttyUSB_0", 460800);

std::unique_ptr<CarStatus> carStatus;

void sendSerial()
{
    if (!shouldBeUpdate)
        return;
    if (carStatus->rePath.poses.size() != PLAN_SIZE)
    {
        ROS_WARN("Serial: wrong plan path size! NeedSize %d ActualSize %d", PLAN_SIZE, carStatus->rePath.poses.size());
        return;
    }
    static Sentinel::Serials::SendFrame frame;
    static bool status = false;
    static bool up_floor = false;
    static bool if_up_down = false;
    frame.yaw = carStatus->yaw;
    frame.posX = -carStatus->nowPos.y * 1000;
    frame.posY = carStatus->nowPos.x * 1000;
    if(frame.posX/rmSerial.map_resolution >= 250&&
    frame.posX/rmSerial.map_resolution <= 415 &&
    frame.posX/rmSerial.map_resolution <= 1292 &&
    frame.posX/rmSerial.map_resolution >= 1200)up_floor = true;
    if(frame.posX/rmSerial.map_resolution >= 762&&
    frame.posX/rmSerial.map_resolution <= 791 &&
    frame.posX/rmSerial.map_resolution <= 1333 &&
    frame.posX/rmSerial.map_resolution >= 1240)up_floor = true;
    frame.velX = -carStatus->velocity.y * 1000;
    frame.velY = carStatus->velocity.x * 1000;
    for (int i = 0; i < PLAN_SIZE; i++)
    {
        frame.planX[i] = -carStatus->rePath.poses[i].pose.position.y * 1000;
        frame.planY[i] = carStatus->rePath.poses[i].pose.position.x * 1000;
        if(carStatus->rePath.poses[i].pose.position.x/rmSerial.map_resolution >=250 && 
        carStatus->rePath.poses[i].pose.position.x/rmSerial.map_resolution <= 400 &&
        carStatus->rePath.poses[i].pose.position.y/rmSerial.map_resolution >= 940 &&
        carStatus->rePath.poses[i].pose.position.y/rmSerial.map_resolution <= 1200)if_up_down = true;
        if(carStatus->rePath.poses[i].pose.position.x/rmSerial.map_resolution >=800 && 
        carStatus->rePath.poses[i].pose.position.x/rmSerial.map_resolution <= 833 &&
        carStatus->rePath.poses[i].pose.position.y/rmSerial.map_resolution >= 1250 &&
        carStatus->rePath.poses[i].pose.position.y/rmSerial.map_resolution <= 1334)if_up_down = true;
    }
    frame.time_stamp = (ros::Time::now() - startTime).toNSec() / 1000000;
    status = rmSerial.sendMessage(frame);
    if(if_up_down&&up_floor)
    frame.mode = 2;
    else if (if_up_down)
    frame.mode = 1;
    else frame.mode = 0;
    if (status)
    {
        ROS_INFO("Serial Send Successfully");
        shouldBeUpdate = false;
    }
}

class StatusUpdateWrapper : public ros::CallbackInterface
{
public:
    CallResult call()
    {
        sendSerial();
        return Success;
    }
};

void receiveTarget(ros::Publisher pub)
{
    static Sentinel::Serials::ReceiveFrame frame;
    static bool status = false;
    static geometry_msgs::Point tarPoint;
    while (!endProgram)
    {
        status = rmSerial.recvMessage(frame);
        if (status)
        {
            tarPoint.x = frame.goalX / 1000.0;
            tarPoint.y = frame.goalY / 1000.0;
            tarPoint.z = 0;
            pub.publish(tarPoint);
            ROS_INFO("Serial TarSet Publish: %6.3lf %6.3lf", tarPoint.x, tarPoint.y);
        }
    }
    ROS_INFO("Serial Receive Ended");
}
static void stopHandler(int sig)
{
    endProgram = true;
    exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, stopHandler);
    ros::init(argc, argv, "sentinel");
    auto autoCall = []() -> void
    {
        shouldBeUpdate = true;
        ros::CallbackQueueInterface *queue = ros::getGlobalCallbackQueue();
        ros::CallbackInterfacePtr spt(new StatusUpdateWrapper());
        queue->addCallback(spt);
    };
    carStatus.reset(new CarStatus());
    carStatus->setTopic(CarStatus::Odometry, ODOMETRY_TOPIC);
    carStatus->setTopic(CarStatus::RePath, REPATH_TOPIC);
    carStatus->setCallBack(CarStatus::Odometry, autoCall);
    carStatus->setCallBack(CarStatus::RePath, autoCall);

    ros::NodeHandle nh;
    rmSerial.openSerial();
    ros::Subscriber getStaticMap = nh.subscribe(STATIC_MAP_TOPIC, 1, &Sentinel::Serials::subscribeStaticMap, &rmSerial);
    ros::Publisher setGoalPub = nh.advertise<geometry_msgs::Point>(SET_GOAL_TOPIC, 1);
    std::thread receiveTar(receiveTarget, setGoalPub);

    ros::spin();
    endProgram = true;
    receiveTar.join();

    return 0;
}
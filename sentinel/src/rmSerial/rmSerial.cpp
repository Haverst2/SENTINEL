#include "rmSerial.hpp"

void Sentinel::Serials::subscribeStaticMap(const nav_msgs::OccupancyGrid::ConstPtr msg)
{
    staticMap = *msg;
    map_resolution = staticMap.info.resolution;
}

Sentinel::Serials::Serials(const std::string &serial_device, const uint32_t baudrate)
{
    serial.setPort(serial_device);
    serial.setBaudrate(baudrate);
    serial.setBytesize(serial::eightbits);
    serial.setStopbits(serial::stopbits_one);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial.setTimeout(to);
}

Sentinel::Serials::~Serials()
{
    serial.close();
}

bool Sentinel::Serials::openSerial(void)
{
    if (serial.isOpen())
    {
        serial.close();
    }
    try
    {
        serial.open();
        if (!serial.isOpen())
        {
            std::cerr << "Unable to open port " << serial.getPort() << std::endl;
            return false;
        }
    }
    catch (const serial::IOException &e)
    {
        std::cerr << "Unable to open port " << serial.getPort() << std::endl;
        return false;
    }
    return true;
}
bool Sentinel::Serials::sendMessage(SendFrame sendFrame)
{
    sendFrame._SOF = _SOF;
    sendFrame._EOF = _EOF;
    try
    {
        serial.write((uint8_t *)&sendFrame, sizeof(sendFrame));
        std::cerr << "send success" << std::endl;
    }
    catch (const serial::IOException &e)
    {
        std::cerr << "Unable to send message: " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool Sentinel::Serials::recvMessage(ReceiveFrame &receiveFrame)
{
    uint8_t buffer[sizeof(ReceiveFrame)];
    ReceiveFrame temp;
    serial.flushInput(); // 清理输出缓冲区

    try
    {
        serial.read(buffer, sizeof(receiveFrame));
    }
    catch (const serial::SerialException &e)
    {
        std::cerr << "Unable to receive message: " << e.what() << std::endl;
        return false;
    }

    temp = (*(ReceiveFrame *)buffer);

    // Calibration
    if (_SOF == temp._SOF && _EOF == temp._EOF)
    {
        receiveFrame = temp;
        return true;
    }
    else
    {
        std::cerr << "Receive message failed." << std::endl;
        return false;
    }
}
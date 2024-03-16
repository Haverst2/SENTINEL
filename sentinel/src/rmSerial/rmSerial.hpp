#pragma once

#include <iostream>
#include <string>
#include <ros/ros.h>
#include "../../include/serial/include/serial/serial.h"
#include "../resources/resources.hpp"
#include "../RosHeaders.hpp"
    // struct ReceiveFrame
    //     {
    //         uint8_t _SOF; // Start of frame
    //         uint16_t pH0; // holder, not for use
    //         uint8_t res1;
    //         uint16_t pH1; // holder, not for use
    //         uint8_t mode; // mode
    //         uint8_t res2;
           
    //         float goalX; // goalX = real point in map, mm
    //         float goalY; // goalY = real point in map, mm

    //         // uint32_t time_stamp; // ms
    //         uint16_t res3;
    //         uint8_t pH2;  // holder, not for use
    //         uint8_t _EOF; // End of frame
    //     };                // 16 Byte
namespace Sentinel
{
#undef EOF // Serial frame's EOF may conflict with somewhere else's EOF
    class Serials
    {
    public:
        serial::Serial serial;
        uint8_t buffer[20];
        struct SendFrame
        {
            uint8_t _SOF; // Start of frame
            uint8_t mode; // mode
            int16_t yaw;  // rotation of the body in 2d plane, counterclockwise

            int16_t posX; // real point in map, mm
            int16_t posY; // real point in map, mm

            int16_t velX; // velocity vector's x, mm/s
            int16_t velY; // velocity vector's y, mm/s

            int16_t planX[PLAN_SIZE]; // points of plan path, x, mm
            int16_t planY[PLAN_SIZE]; // points of plan path, y, mm

            uint32_t time_stamp; // ms

            uint16_t pH0; // holder, not for use
            uint8_t pH1;  // holder, not for use
            uint8_t _EOF; // End of frame
        };                // 40 Byte

        struct ReceiveFrame
        {
            uint8_t _SOF; // Start of frame
            uint8_t res1;
            uint16_t pH0; // holder, not for use
            uint16_t pH1; // holder, not for use
            uint8_t mode; // mode
            uint8_t res2;
           
            float goalX; // goalX = real point in map, mm
            float goalY; // goalY = real point in map, mm

            // uint32_t time_stamp; // ms
            uint16_t res3;
            uint8_t pH2;  // holder, not for use
            uint8_t _EOF; // End of frame
        };                // 16 Byte
        
        Serials(const std::string &serial_device = "/dev/ttyUSB_0", const uint32_t baudrate = 460800);
        Serials() = delete;
        Serials(const Serials &) = delete;
        Serials(Serials &&) = delete;
        Serials &operator=(const Serials &) = delete;

        /**
         * @description: open serial
         * @return bool success or not
         */
        bool openSerial(void);

        bool sendMessage(SendFrame sendFrame);

        bool recvMessage(ReceiveFrame &receiveFrame);

        void subscribeStaticMap(const nav_msgs::OccupancyGrid::ConstPtr msg);

        ~Serials();

        nav_msgs::OccupancyGrid staticMap;
        double map_resolution;
    private:
        const uint8_t _SOF = 0x66;
        const uint8_t _EOF = 0x88;

    };

};
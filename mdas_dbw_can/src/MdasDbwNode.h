#ifndef _MDAS_DBW_NODE_H_
#define _MDAS_DBW_NODE_H_

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "can_msgs/Frame.h"

#include "mdas_dbw_msgs/SteeringCmd.h"
#include "mdas_dbw_msgs/SteeringReport.h"
#include "mdas_dbw_msgs/BrakeCmd.h"
#include "mdas_dbw_msgs/BrakeReport.h"
#include "mdas_dbw_msgs/ThrottleReport.h"
#include "mdas_dbw_msgs/ThrottleCmd.h"

#include <cstdint>

#define SRC_ID (0x02)
#define STEER_MULT (3.25f)
#define STEER_ADD (2048)

namespace mdas_dbw_can
{
    class DbwNode
    {
        public:
            DbwNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
            ~DbwNode();

        private:
            // Functions
            inline can_msgs::Frame buildBrakeMsg(std::uint8_t pedal_cmd);
            inline can_msgs::Frame buildThrottleMsg(std::uint8_t pedal_cmd);
            inline can_msgs::Frame buildSteeringMsg(std::float_t wheel_cmd);
            void receivedMessagesCallback(const can_msgs::Frame::ConstPtr& canFrame);
            void receivedBrakeCmdCallback(const mdas_dbw_msgs::BrakeCmd& brakeMsg);
            void receivedSteerCmdCallback(const mdas_dbw_msgs::SteeringCmd& steerMsg);
            void receivedThrottleCmdCallback(const mdas_dbw_msgs::ThrottleCmd& throttleMsg);
            void canSend(const ros::TimerEvent&);

            // Variables
            std::int32_t throttle_id; 
            std::int32_t steer_id;
            std::int32_t brake_id;
            std::int32_t feedback_id;
            ros::Publisher throttlePub;
            ros::Publisher steeringPub;
            ros::Publisher brakePub;
            ros::Publisher canSender;
            mdas_dbw_msgs::BrakeCmd brakeCmd;
            mdas_dbw_msgs::SteeringCmd steerCmd;
            mdas_dbw_msgs::ThrottleCmd throttleCmd;

            // Callback variables
            ros::Subscriber feedbackSub;
            ros::Subscriber brakeSub;
            ros::Subscriber throttleSub;
            ros::Subscriber steeringSub;
            ros::Timer timer;
    };
}

#endif // _MDAS_DBW_NODE_H_
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

namespace mdas_dbw_can
{
    class DbwNode
    {
        public:
            DbwNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
            ~DbwNode();

        private:
            // Functions
            inline can_msgs::Frame BuildBrakeMsg(std::uint8_t pedal_cmd);
            inline can_msgs::Frame BuildThrottleMsg(std::uint8_t pedal_cmd);
            inline can_msgs::Frame BuildSteeringMsg(std::float_t wheel_cmd);
            inline mdas_dbw_msgs::BrakeReport BuildBrakeReport(const can_msgs::Frame::ConstPtr& canFrame);
            inline mdas_dbw_msgs::ThrottleReport BuildThrottleReport(const can_msgs::Frame::ConstPtr& canFrame);
            inline mdas_dbw_msgs::SteeringReport BuildSteeringReport(const can_msgs::Frame::ConstPtr& canFrame);
            void ReceivedMessagesCallback(const can_msgs::Frame::ConstPtr& canFrame);
            void ReceivedBrakeCmdCallback(const mdas_dbw_msgs::BrakeCmd& brakeMsg);
            void ReceivedSteerCmdCallback(const mdas_dbw_msgs::SteeringCmd& steerMsg);
            void ReceivedThrottleCmdCallback(const mdas_dbw_msgs::ThrottleCmd& throttleMsg);
            void CanSend(const ros::TimerEvent&);
            void TimeoutDisable();

            // Constants
            enum STEER_FEEDBACK_IDS:std::uint32_t {
                STEERING_POSITION_CAN_ID = 0x18FF0113,
                STEERING_VELOCITY_CAN_ID = 0x18FF0213,
                STEERING_TORQUE_CAN_ID = 0x18FF0313
            };
            enum SRC_ID:std::uint8_t {
                BRAKE_FEEDBACK_SRC_ID = 0x00,
                THROTTLE_FEEDBACK_SRC_ID = 0x01,
                STEERING_FEEDBACK_SRC_ID = 0x02,
                CONTROL_SRC_ID = 0x02,
            };
            static constexpr std::int32_t TIMEOUT_INTERVAL = 5; // 500 ms command timeout
            static constexpr float STEER_MULT = 3.25f;
            static constexpr float STEER_ADD = 2048.0f;

            // Variables
            std::int32_t throttle_id; 
            std::int32_t steer_id;
            std::int32_t brake_id;
            std::int32_t throttlebrake_feedback_id;
            ros::Publisher throttlePub;
            ros::Publisher steeringPub;
            ros::Publisher brakePub;
            ros::Publisher canSender;
            mdas_dbw_msgs::BrakeCmd brakeCmd;
            mdas_dbw_msgs::SteeringCmd steerCmd;
            mdas_dbw_msgs::ThrottleCmd throttleCmd;
            std::int32_t throttleTimeout = TIMEOUT_INTERVAL + 1;
            std::int32_t brakeTimeout = TIMEOUT_INTERVAL + 1;
            std::int32_t steerTimeout = TIMEOUT_INTERVAL + 1;

            // Callback variables
            ros::Subscriber feedbackSub;
            ros::Subscriber brakeSub;
            ros::Subscriber throttleSub;
            ros::Subscriber steeringSub;
            ros::Timer timer;
    };
}

#endif // _MDAS_DBW_NODE_H_
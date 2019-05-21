#include "MdasDbwNode.h"

namespace mdas_dbw_can
{
    DbwNode::DbwNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh) {
        std::string throttle_id_str, brake_id_str, steer_id_str, feedback_id_str;


        priv_nh.param<std::string>("throttle_can_id", throttle_id_str, "0x1ADB0000");
        priv_nh.param<std::string>("brake_can_id", brake_id_str, "0x18DB0000");
        priv_nh.param<std::string>("steering_can_id", steer_id_str, "0x19DB0000");
        priv_nh.param<std::string>("feedback_can_id", feedback_id_str, "0x1CDBFFFF");

        // Convert hex strings to ints
        throttle_id = std::stoi(throttle_id_str, nullptr, 16);
        brake_id = std::stoi(brake_id_str, nullptr, 16);
        steer_id = std::stoi(steer_id_str, nullptr, 16);
        feedback_id = std::stoi(feedback_id_str, nullptr, 16);

        throttlePub = node.advertise<mdas_dbw_msgs::ThrottleReport>("mdas_dbw/feedback/throttle", 10);
        brakePub = node.advertise<mdas_dbw_msgs::BrakeReport>("mdas_dbw/feedback/brake", 10);
        steeringPub = node.advertise<mdas_dbw_msgs::SteeringReport>("mdas_dbw/feedback/steering", 100);
        canSender = node.advertise<can_msgs::Frame>("sent_messages", 30);

        const ros::TransportHints NODELAY = ros::TransportHints().tcpNoDelay();
        feedbackSub = node.subscribe("received_messages", 100, &DbwNode::receivedMessagesCallback, this, NODELAY);
        brakeSub = node.subscribe("mdas_dbw/cmd/brake", 1, &DbwNode::receivedBrakeCmdCallback, this, NODELAY);
        throttleSub = node.subscribe("mdas_dbw/cmd/throttle", 1, &DbwNode::receivedThrottleCmdCallback, this, NODELAY);
        steeringSub = node.subscribe("mdas_dbw/cmd/steering", 1, &DbwNode::receivedSteerCmdCallback, this, NODELAY);


        timer = node.createTimer(ros::Duration(0.1), &DbwNode::canSend, this);
    }

    DbwNode::~DbwNode() {
    }

    inline can_msgs::Frame DbwNode::buildBrakeMsg(std::uint8_t pedal_cmd) {
        can_msgs::Frame msg;

        msg.is_extended = true;
        msg.is_error = false;
        msg.is_rtr = false;
        msg.data[0] = SRC_ID;
        msg.data[1] = 0;
        msg.data[2] = 0;
        msg.dlc = 4;

        msg.id = brake_id;
        msg.data[3] = pedal_cmd;
        msg.header.stamp = ros::Time::now();

        return msg;
    }

    inline can_msgs::Frame DbwNode::buildThrottleMsg(std::uint8_t pedal_cmd) {
        can_msgs::Frame msg;

        msg.is_extended = true;
        msg.is_error = false;
        msg.is_rtr = false;
        msg.data[0] = SRC_ID;
        msg.data[1] = 0;
        msg.data[2] = 0;
        msg.dlc = 4;

        msg.id = throttle_id;
        msg.data[3] = pedal_cmd;

        msg.header.stamp = ros::Time::now();

        return msg;
    }

    inline can_msgs::Frame DbwNode::buildSteeringMsg(std::float_t wheel_cmd) {
        can_msgs::Frame msg;

        msg.is_extended = true;
        msg.is_error = false;
        msg.is_rtr = false;
        msg.data[0] = 0x01; //TODO: Temporary until finalized with Naso
        msg.data[1] = 0;
        msg.data[2] = 0;
        msg.dlc = 4;

        msg.id = steer_id;

        float steer = STEER_MULT * wheel_cmd + STEER_ADD;
        std::uint16_t steerInt = (uint16_t)steer;

        msg.data[2] = (steerInt >> 8) & 0xff;
        msg.data[3] = steerInt & 0xff;

        msg.header.stamp = ros::Time::now();

        return msg;
    }

    void DbwNode::receivedMessagesCallback(const can_msgs::Frame::ConstPtr& canFrame) {
        if(canFrame->id != feedback_id) {
            return;
        }
        std::uint8_t srcId = canFrame->data[0];

        switch(srcId) {
            case 1:
            mdas_dbw_msgs::ThrottleReport msg;
            msg.enabled = 0;
            msg.pedal_input = canFrame->data[2];
            msg.override = 0;
            msg.header.stamp = ros::Time::now();

            throttlePub.publish(msg);
            if(msg.pedal_input > msg.THROTTLE_MAX) {
                ROS_WARN("Throttle greater than expected: %d%%", msg.pedal_input);
            }
            return;
        };
    }

    void DbwNode::receivedBrakeCmdCallback(const mdas_dbw_msgs::BrakeCmd& brakeMsg) {
        brakeCmd = brakeMsg;

        if(brakeCmd.pedal_cmd > brakeCmd.BRAKE_MAX) { // Clamp input
            brakeCmd.pedal_cmd = brakeCmd.BRAKE_MAX;
            ROS_WARN("Brake Command >%d%%, input clamped", brakeCmd.BRAKE_MAX);
        }
    }

    void DbwNode::receivedSteerCmdCallback(const mdas_dbw_msgs::SteeringCmd& steerMsg) {
        steerCmd = steerMsg;
        const std::int32_t steerNeg = steerCmd.ANGLE_MAX * -1;

        if(steerCmd.steering_wheel_angle_cmd > steerCmd.ANGLE_MAX) { // Clamp input
            steerCmd.steering_wheel_angle_cmd = steerCmd.ANGLE_MAX;
            ROS_WARN("Steering command >%d degrees, input clamped", steerCmd.ANGLE_MAX);
        } else if(steerCmd.steering_wheel_angle_cmd < steerNeg) {
            steerCmd.steering_wheel_angle_cmd = steerNeg;
            ROS_WARN("Steering command <%d degrees, input clamped", steerNeg);
        }
    }

    void DbwNode::receivedThrottleCmdCallback(const mdas_dbw_msgs::ThrottleCmd& throttleMsg) {
        throttleCmd = throttleMsg;

        if(throttleCmd.pedal_cmd > throttleCmd.THROTTLE_MAX) { // Clamp input
            throttleCmd.pedal_cmd = throttleCmd.THROTTLE_MAX;
            ROS_WARN("Steering command >%d%%, input clamped", throttleCmd.THROTTLE_MAX);
        }
    }

    void DbwNode::canSend(const ros::TimerEvent&) {
        static bool brakeState = false, throttleState = false, steerState = false;

        can_msgs::Frame msg;
        msg.is_extended = true;
        msg.is_error = false;
        msg.is_rtr = false;
        msg.data[0] = SRC_ID;
        msg.data[1] = 0;
        msg.data[2] = 0;
        msg.dlc = 4;

        if(brakeCmd.enable) {
            msg = buildBrakeMsg(brakeCmd.pedal_cmd);
            canSender.publish(msg);
        }

        if(throttleCmd.enable) {
            msg = buildThrottleMsg(throttleCmd.pedal_cmd);
            canSender.publish(msg);
        }

        if(steerCmd.enable) {
            msg = buildSteeringMsg(steerCmd.steering_wheel_angle_cmd);
            canSender.publish(msg);
        }


        // Check if any enable states changed to disabled
        // then stop ECU commands
        if((brakeState == true) && (brakeCmd.enable == false)) {
            msg = buildBrakeMsg(0);
            canSender.publish(msg);
        }

        if((throttleState == true) && (throttleCmd.enable == false)) {
            msg = buildThrottleMsg(0);
            canSender.publish(msg);
        }

        if((steerState == true) && (steerCmd.enable == false)) {
            msg = buildSteeringMsg(0);
            canSender.publish(msg);
        }

        // Save enable state
        brakeState = brakeCmd.enable;
        throttleState = throttleCmd.enable;
        steerState = steerCmd.enable;
    }
}
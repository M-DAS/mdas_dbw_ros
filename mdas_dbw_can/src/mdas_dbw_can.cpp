#include "ros/ros.h"

#include "std_msgs/String.h"
#include "can_msgs/Frame.h"

#include "mdas_dbw_msgs/SteeringCmd.h"
#include "mdas_dbw_msgs/SteeringReport.h"
#include "mdas_dbw_msgs/BrakeCmd.h"
#include "mdas_dbw_msgs/BrakeReport.h"
#include "mdas_dbw_msgs/ThrottleReport.h"
#include "mdas_dbw_msgs/ThrottleCmd.h"

#include <sstream>
#include <cstdint>

#define SRC_ID (0x02)
#define STEER_MULT (3.25f)
#define STEER_ADD (2048)

int throttle_id, steer_id, brake_id, feedback_id;
ros::Publisher throttlePub, steeringPub, brakePub, canSender;
mdas_dbw_msgs::BrakeCmd brakeCmd;
mdas_dbw_msgs::SteeringCmd steerCmd;
mdas_dbw_msgs::ThrottleCmd throttleCmd;



void receivedMessagesCallback(const can_msgs::Frame::ConstPtr& canFrame) {
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

void receivedBrakeCmdCallback(const mdas_dbw_msgs::BrakeCmd& brakeMsg) {
  brakeCmd = brakeMsg;

  if(brakeCmd.pedal_cmd > brakeCmd.BRAKE_MAX) { // Clamp input
    brakeCmd.pedal_cmd = brakeCmd.BRAKE_MAX;
    ROS_WARN("Brake Command >%d%%, input clamped", brakeCmd.BRAKE_MAX);
  }
}

void receivedSteerCmdCallback(const mdas_dbw_msgs::SteeringCmd& steerMsg) {
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

void receivedThrottleCmdCallback(const mdas_dbw_msgs::ThrottleCmd& throttleMsg) {
  throttleCmd = throttleMsg;

  if(throttleCmd.pedal_cmd > throttleCmd.THROTTLE_MAX) { // Clamp input
    throttleCmd.pedal_cmd = throttleCmd.THROTTLE_MAX;
    ROS_WARN("Steering command >%d%%, input clamped", throttleCmd.THROTTLE_MAX);
  }
}

void canSend(const ros::TimerEvent&) {

  can_msgs::Frame msg;
  msg.is_extended = true;
  msg.is_error = false;
  msg.is_rtr = false;
  msg.data[0] = SRC_ID;
  msg.data[1] = 0;
  msg.data[2] = 0;
  msg.dlc = 4;

  if(brakeCmd.enable) {
    msg.id = brake_id;
    msg.data[3] = brakeCmd.pedal_cmd;
    msg.header.stamp = ros::Time::now();
    canSender.publish(msg);
  }

  if(throttleCmd.enable) {
    msg.id = throttle_id;
    msg.data[3] = throttleCmd.pedal_cmd;

    msg.header.stamp = ros::Time::now();
    canSender.publish(msg);
  }

  if(steerCmd.enable) {
    msg.id = steer_id;
    msg.data[0] = 0x01; //Temporary until finalized

    float steer = STEER_MULT * steerCmd.steering_wheel_angle_cmd + STEER_ADD;
    std::uint16_t steerInt = (uint16_t)steer;
    ROS_INFO("%d", steerInt);

    msg.data[2] = (steerInt >> 8) & 0xff;
    msg.data[3] = steerInt & 0xff;

    msg.header.stamp = ros::Time::now();
    canSender.publish(msg);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mdas_dbw");

  ros::NodeHandle n, nh_param("~");

  std::string throttle_id_str, brake_id_str, steer_id_str, feedback_id_str;


  nh_param.param<std::string>("throttle_can_id", throttle_id_str, "0x1ADB0000");
  nh_param.param<std::string>("brake_can_id", brake_id_str, "0x18DB0000");
  nh_param.param<std::string>("steering_can_id", steer_id_str, "0x19DB0000");
  nh_param.param<std::string>("feedback_can_id", feedback_id_str, "0x1CDBFFFF");

  // Convert hex strings to ints
  throttle_id = std::stoi(throttle_id_str, nullptr, 16);
  brake_id = std::stoi(brake_id_str, nullptr, 16);
  steer_id = std::stoi(steer_id_str, nullptr, 16);
  feedback_id = std::stoi(feedback_id_str, nullptr, 16);

  throttlePub = n.advertise<mdas_dbw_msgs::ThrottleReport>("mdas_dbw/feedback/throttle", 10);
  brakePub = n.advertise<mdas_dbw_msgs::BrakeReport>("mdas_dbw/feedback/brake", 10);
  steeringPub = n.advertise<mdas_dbw_msgs::SteeringReport>("mdas_dbw/feedback/steering", 100);
  canSender = n.advertise<can_msgs::Frame>("sent_messages", 30);

  ros::Subscriber feedbackSub = n.subscribe("received_messages", 100, receivedMessagesCallback);
  ros::Subscriber brakeSub = n.subscribe("mdas_dbw/cmd/brake", 2, receivedBrakeCmdCallback);
  ros::Subscriber throttleSub = n.subscribe("mdas_dbw/cmd/throttle", 2, receivedThrottleCmdCallback);
  ros::Subscriber steeringSub = n.subscribe("mdas_dbw/cmd/steering", 2, receivedSteerCmdCallback);


  ros::Timer timer = n.createTimer(ros::Duration(0.1), canSend);

  ros::spin();

  return 0;
}

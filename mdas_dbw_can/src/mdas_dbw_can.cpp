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

int throttle_id, steer_id, brake_id, feedback_id;
ros::Publisher throttlePub, steeringPub, brakePub;


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

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  throttlePub = n.advertise<mdas_dbw_msgs::ThrottleReport>("mdas_dbw/throttle_feedback", 2);

  ros::Subscriber feedbackSub = n.subscribe("received_messages", 100, receivedMessagesCallback);

  ros::spin();

  return 0;
}

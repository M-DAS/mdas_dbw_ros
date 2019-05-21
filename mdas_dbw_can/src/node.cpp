#include "ros/ros.h"

#include "MdasDbwNode.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mdas_dbw");

  ros::NodeHandle node, nh_param("~");

  mdas_dbw_can::DbwNode n(node, nh_param);

  ros::spin();

  return 0;
}

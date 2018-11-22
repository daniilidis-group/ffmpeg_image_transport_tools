/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <ros/ros.h>
#include <ros/console.h>
#include "ffmpeg_image_transport_tools/decode_bag.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "decode_bag");
  ros::NodeHandle pnh("~");
  try {
    ffmpeg_image_transport_tools::DecodeBag node(pnh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}

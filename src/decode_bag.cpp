/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport_tools/decode_bag.h"
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>

namespace ffmpeg_image_transport_tools {
  DecodeBag::Session::Session(const std::string &topic) :
    topic_(topic) {
  }

  DecodeBag::DecodeBag(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  DecodeBag::~DecodeBag() {
  }

  bool DecodeBag::initialize() {
    std::string bagFile;
    nh_.param<std::string>("bag_file",  bagFile,  "");
    ROS_INFO_STREAM("decoding bag: " << bagFile << " -> " << bagFile);
    if (!bagFile.empty()) {
      processBag(bagFile);
    } else {
      ROS_ERROR_STREAM("must specfiy bag_file!");
    }
    ros::shutdown();
    return (true);
  }

  void DecodeBag::Session::callback(const ImageConstPtr &img) {
    ROS_INFO_STREAM("got image callback!");
  }

  void DecodeBag::Session::processMessage(const FFMPEGPacketConstPtr &msg) {
    if (!decoder_.isInitialized()) {
      decoder_.initialize(msg, boost::bind(&DecodeBag::Session::callback, this, _1));
      if (!decoder_.isInitialized()) {
        return;
      }
    }
    decoder_.decodePacket(msg);
  }

  void DecodeBag::processBag(const std::string &fname) {
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::string> imageTopics;
    if (!nh_.getParam("image_topics", imageTopics)) {
      ROS_ERROR("no image topics found!");
      return;
    }
    std::vector<std::string> topics = imageTopics;
    std::string all_topics_string;
    for (const auto &topic: topics) {
      rosbag::View cv(bag, rosbag::TopicQuery({topic}));
      if (cv.begin() == cv.end()) {
        ROS_WARN_STREAM("cannot find topic: " << topic);
      }
      all_topics_string += " " + topic;
      if (sessions_.count(topic) == 0) {
        sessions_[topic].reset(new Session(topic));
      } else {
        ROS_WARN_STREAM("duplicate topic: " << topic);
      }
    }
    ROS_INFO_STREAM("using topics: " << all_topics_string);
    ROS_INFO_STREAM("playing from file: " << fname);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    for (const rosbag::MessageInstance &m: view) {
      FFMPEGPacketConstPtr msg = m.instantiate<FFMPEGPacket>();
      if (msg) {
        SessionPtr sess = sessions_[m.getTopic()];
        sess->processMessage(msg);
      }
      if (!ros::ok()) {
        break;
      }
      ros::Duration(0.1).sleep();
    }
    bag.close();
  }
  
}  // namespace

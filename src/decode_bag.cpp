/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport_tools/decode_bag.h"
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <boost/range/irange.hpp>
#include <iomanip>
#include <climits>

namespace ffmpeg_image_transport_tools {
  using boost::irange;
  DecodeBag::Session::Session(const std::string &topic,
                              const ImageSyncPtr &sync) :
    topic_(topic), sync_(sync) {
  }

  DecodeBag::DecodeBag(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  DecodeBag::~DecodeBag() {
  }

  void DecodeBag::packetReady(const FFMPEGPacketConstPtr &pkt) {
    ROS_INFO("writing frame: %5u", frameNum_);
    rawStream_.write((const char *)(&pkt->data[0]), pkt->data.size());
    frameNum_++;
  }


  void DecodeBag::syncCallback(const std::vector<ImageConstPtr> &msgs) {
    if (msgs.size() == 0) {
      ROS_WARN("empty image message!");
      return;
    }
    const auto &msg0 = msgs[0];
    const int w(msg0->width), h(msg0->height);
    if (w == 0 || h == 0) {
      ROS_WARN("image0 not full size!");
      return;
    }
    // arrange individual image into full image
    cv::Mat fullImg(h * numRows_, w * numCols_,  CV_8UC3);
    for (const auto i: irange(0ul, msgs.size())) {
      const auto &msg = msgs[i];
      const int col = cameraToLocation_[i] % numCols_;
      const int row = cameraToLocation_[i] / numCols_;
      cv::Mat dest = fullImg(cv::Rect(col * w, row * h, w, h));
      cv_bridge::toCvShare(msg,
                    sensor_msgs::image_encodings::BGR8)->image.copyTo(dest);
    }
    // convert full image to ros message
    ImageConstPtr imgPtr = cv_bridge::CvImage(msg0->header,
                                              "bgr8", fullImg).toImageMsg();
    // encode full image
    if (!encoder_.isInitialized()) {
      encoder_.initialize(*imgPtr, boost::bind(&DecodeBag::packetReady,
                                               this, _1));
      if (!encoder_.isInitialized()) {
        ROS_ERROR_STREAM("cannot initialize encoder!");
        return;
      }
    }
    // this will trigger a callback to packetReady() when the encoder
    // has produced the first packet
    encoder_.encodeImage(*imgPtr);
  }

  bool DecodeBag::initialize() {
    encoder_.setCodec("hevc_nvenc");
    encoder_.setProfile("main");
    encoder_.setPreset("slow");
    encoder_.setQMax(0);
    encoder_.setBitRate(10*1024*1024);
    encoder_.setGOPSize(20);
    encoder_.setMeasurePerformance(true);

    if (!nh_.getParam("image_topics", imageTopics_)) {
      ROS_ERROR("no image topics found!");
      return (false);
    }

    nh_.param<int>("num_rows",  numRows_, 2);
    nh_.param<int>("num_cols",  numCols_, 4);
    if (!nh_.getParam("camera_to_location", cameraToLocation_)) {
      ROS_WARN("no camera_to_location specified!");
      for (const auto i: irange(0, numRows_ * numCols_)) {
        cameraToLocation_.push_back(i);
      }
    }
    if ((int)cameraToLocation_.size() != numRows_ * numCols_) {
      ROS_ERROR("camera_to_location size mismatches with num_ros * num_cols");
      return (false);
    }
    for (const auto &i: cameraToLocation_) {
      if (i >= numRows_ * numCols_) {
        ROS_ERROR_STREAM("camera_to_location: " << i << " too large!");
        return (false);
      }
    }

    std::string bagFile;
    nh_.param<std::string>("bag_file",  bagFile,  "");
    ROS_INFO_STREAM("decoding bag: " << bagFile << " -> " << bagFile);
    rawStream_.open("raw.h265", std::ios::out | std::ios::binary);
    if (!bagFile.empty()) {
      processBag(bagFile);
    } else {
      ROS_ERROR_STREAM("must specfiy bag_file!");
    }
    rawStream_.close();

    ros::shutdown();
    return (true);
  }

  void DecodeBag::Session::callback(const ImageConstPtr &img) {
    // pipe data into the sync, which will deliver a callback
    // when all images are present
    sync_->process(topic_, img);
  }

  void DecodeBag::Session::processMessage(const FFMPEGPacketConstPtr &msg) {
    if (!decoder_.isInitialized()) {
      decoder_.initialize(msg, boost::bind(&DecodeBag::Session::callback,
                                           this, _1));
      if (!decoder_.isInitialized()) {
        return;
      }
    }
    decoder_.decodePacket(msg);
  }

  void DecodeBag::processBag(const std::string &fname) {
    int maxNumFrames_(INT_MAX);
    nh_.param<int>("max_num_frames",  maxNumFrames_, INT_MAX);

    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::string> topics = imageTopics_;
    std::string all_topics_string;
    ImageSync::Callback cb = std::bind(&DecodeBag::syncCallback,
                                       this, std::placeholders::_1);
    sync_.reset(new ImageSync(topics, cb));
    for (const auto &topic: topics) {
      rosbag::View cv(bag, rosbag::TopicQuery({topic}));
      if (cv.begin() == cv.end()) {
        ROS_WARN_STREAM("cannot find topic: " << topic);
      }
      all_topics_string += " " + topic;
      if (sessions_.count(topic) == 0) {
        sessions_[topic].reset(new Session(topic, sync_));
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
      if (!ros::ok() || (int)frameNum_ > maxNumFrames_) {
        break;
      }
    }
    bag.close();
  }
  
}  // namespace

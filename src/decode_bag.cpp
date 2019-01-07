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
    thread_ = boost::make_shared<boost::thread>(
      &DecodeBag::Session::decodeThread, this);
  }
  DecodeBag::Session::~Session() {
    if (thread_) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        threadStopSignaled_ = true;
        cv_.notify_all();
      }
      thread_->join();
    }
  }

  DecodeBag::DecodeBag(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  DecodeBag::~DecodeBag() {
  }

  void DecodeBag::packetReady(const FFMPEGPacketConstPtr &pkt) {
    //ROS_INFO("writing frame: %5u", frameNum_);
    rawStream_.write((const char *)(&pkt->data[0]), pkt->data.size());
  }

  DecodeBag::CopyThread::CopyThread(const cv::Mat &dest) : dest_(dest) {
    thread_ = boost::make_shared<boost::thread>(
      &DecodeBag::CopyThread::run, this);
  }

  DecodeBag::CopyThread::~CopyThread() {
    if (thread_) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        threadStopSignaled_ = true;
        cv_.notify_all();
      }
      thread_->join();
    }
  }

  void DecodeBag::CopyThread::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!threadStopSignaled_ && ros::ok()) {
      while (!msgToCopy_ && !threadStopSignaled_) {
        cv_.wait(lock);
      }
      if (msgToCopy_) {
        cv_bridge::toCvShare(msgToCopy_,
               sensor_msgs::image_encodings::BGR8)->image.copyTo(dest_);
      }
      msgToCopy_.reset();
      cv_.notify_all();
    }
  }

  void DecodeBag::startCopyThreads(int w, int h, int size) {
    for (int i = 0; i < size; i++) {
      const int col = cameraToLocation_[i] % numCols_;
      const int row = cameraToLocation_[i] / numCols_;
      cv::Mat dst = fullImg_(cv::Rect(col * w, row * h, w, h));
      copyThreads_.emplace_back(new CopyThread(dst));
    }
  }

  static std::string
  make_file_name(const std::string &prefix, const unsigned int counter) {
    std::stringstream ss;
    ss << prefix << "_" << std::setfill('0') << std::setw(6) << counter << ".jpg";
    return (ss.str());
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
    // arrange individual images into full image
    if (fullImg_.rows == 0) {
      fullImg_ = cv::Mat(h * numRows_, w * numCols_,  CV_8UC3);
      startCopyThreads(w, h, (int)msgs.size());
    }
    for (int i = 0; i < (int)msgs.size(); i++) {
      copyThreads_[i]->startCopy(msgs[i]);
    }
    for (int i = 0; i < (int)msgs.size(); i++) {
      copyThreads_[i]->waitForCopyComplete();
    }
    if (writeFrames_ && (frameNum_ % keepRatio_ == 0)) {
      cv::imwrite(make_file_name(baseName_, frameNum_ / keepRatio_), fullImg_);
    }
    frameNum_++;

    if (writeVideo_) {
      // encode full image
      if (!encoder_.isInitialized()) {
        encoder_.initialize(fullImg_.cols, fullImg_.rows,
                            boost::bind(&DecodeBag::packetReady, this, _1));
        if (!encoder_.isInitialized()) {
          ROS_ERROR_STREAM("cannot initialize encoder!");
          return;
        }
      }
      // this will trigger a callback to packetReady() when the encoder
      // has produced the first packet
      encoder_.encodeImage(fullImg_, msg0->header, ros::WallTime::now());
    }
  }

  bool DecodeBag::initialize() {
    nh_.param<std::string>("base_file_name", baseName_, "frame");
    nh_.param<int>("keep_ratio",   keepRatio_, 100);
    nh_.param<bool>("write_video",   writeVideo_, true);
    nh_.param<bool>("write_frames",  writeFrames_, false);
    if (writeVideo_) {
      encoder_.setCodec("hevc_nvenc");
      encoder_.setProfile("main");
      encoder_.setPreset("slow");
      encoder_.setQMax(0);
      encoder_.setBitRate(10*1024*1024);
      encoder_.setGOPSize(20);
      encoder_.setMeasurePerformance(false);
    }

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
    std::string outFile;
    nh_.param<std::string>("output_file",  outFile,  "raw.h265");
    ROS_INFO_STREAM("decoding bag: " << bagFile << " to " << outFile);
    rawStream_.open(outFile, std::ios::out | std::ios::binary);
    if (!bagFile.empty()) {
      processBag(bagFile);
    } else {
      ROS_ERROR_STREAM("must specify bag_file!");
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

  void DecodeBag::Session::decodeThread() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!threadStopSignaled_ && ros::ok()) {
      while (!msgToDecode_ && !threadStopSignaled_) {
        cv_.wait(lock);
      }
      if (msgToDecode_) {
        // this should produce a callback with the decoded image
        decoder_.decodePacket(msgToDecode_);
#ifdef MEASURE_DECODER_PERFORMANCE        
        packetNum_++;
        if (packetNum_ > 50) {
          packetNum_ = 0;
          decoder_.printTimers(topic_);
          decoder_.resetTimers();
        }
#endif        
        msgToDecode_.reset();
        cv_.notify_all();
      }
    }
  }

  void DecodeBag::Session::processMessage(const FFMPEGPacketConstPtr &msg) {
    if (!decoder_.isInitialized()) {
      decoder_.initialize(msg, boost::bind(&DecodeBag::Session::callback,
                                           this, _1));
      if (!decoder_.isInitialized()) {
        return;
      }
    }
    
    std::unique_lock<std::mutex> lock(mutex_);
    // wait for previous message to clear
    while (msgToDecode_ && ros::ok()) {
      cv_.wait(lock);
    }
    msgToDecode_ = msg;
    cv_.notify_all();
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
    auto t0 = ros::WallTime::now();
    int cnt(0), perfInterval(500);
    for (const rosbag::MessageInstance &m: view) {
      FFMPEGPacketConstPtr msg = m.instantiate<FFMPEGPacket>();
      if (msg) {
        SessionPtr sess = sessions_[m.getTopic()];
        sess->processMessage(msg);
      }
      if (!ros::ok() || (int)frameNum_ > maxNumFrames_) {
        break;
      }
      if (cnt++ > perfInterval) {
        const auto t1 = ros::WallTime::now();
        ROS_INFO_STREAM("wrote frames: " << frameNum_ << " fps: " <<
                        perfInterval / (topics.size() * (t1-t0).toSec()));
        cnt = 0;
        t0 = t1;
      }
    }
    bag.close();
    sessions_.clear(); // should collect threads
  }
  
}  // namespace

/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
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

  static std::string
  make_file_name(const std::string &prefix, const unsigned int counter,
                 const std::string &postfix = "") {
    std::stringstream ss;
    ss << prefix << std::setfill('0') << std::setw(6)
       << counter << postfix << ".jpg";
    return (ss.str());
  }


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
    rawStream_.write((const char *)(&pkt->data[0]), pkt->data.size());
  }

  DecodeBag::CopyThread::CopyThread(
    const cv::Mat &dest, int camNum,
    const std::string &baseDir, int keepRatio,
    bool writeFrame) :
    cameraNumber_(camNum),
    writeFrames_(writeFrame),
    keepRatio_(keepRatio),
    dest_(dest) {
    frameBaseName_ = baseDir + "/frame_";
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
        if (writeFrames_ && (frameNum_ % keepRatio_ == 0)) {
          std::string fname =
                 make_file_name(frameBaseName_,
                                frameNum_ / keepRatio_, "_" +
                                std::to_string(cameraNumber_));
          cv::imwrite(fname, dest_);
        }
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
      copyThreads_.emplace_back(
        new CopyThread(dst, i, frameBaseDir_, keepRatio_,
                       writeIndividualFrames_));
    }
  }

  void DecodeBag::writeVideoFrame(const cv::Mat &img, const std_msgs::Header &header) {
    if (!encoder_.isInitialized()) {
      encoder_.initialize(img.cols, img.rows,
                          boost::bind(&DecodeBag::packetReady, this, _1));
      if (!encoder_.isInitialized()) {
        ROS_ERROR_STREAM("cannot initialize encoder!");
        return;
      }
    }
    // if frame rate is specified, fill in missing frames
    if (lastTime_ != ros::Time(0) && framePeriod_ != ros::Duration(0)) {
      while (header.stamp  - lastTime_ > minFrameDiff_) {
        lastTime_ = lastTime_ + framePeriod_;
        std_msgs::Header h = header; // make deep copy
        h.stamp = lastTime_;
        // this will trigger a callback to packetReady() when the encoder
        // has produced the first packet
        encoder_.encodeImage(img, h, ros::WallTime::now());
        frameNum_++;
        numFramesInserted_++;
        ROS_INFO_STREAM("inserting frame " << frameNum_ << " at ROS time "
                        << lastTime_ << " at period " << framePeriod_);
      }
    }
    // this will trigger a callback to packetReady() when the encoder
    // has produced the first packet
    encoder_.encodeImage(img, header, ros::WallTime::now());
    // maintain time stamps
    if (lastTime_ == ros::Time(0)) {
      firstTime_ = header.stamp; // save for later
    }
    lastTime_ = header.stamp;
  }

  void DecodeBag::syncCallback(const std::vector<ImageConstPtr> &msgs) {
    if (msgs.size() == 0) {
      ROS_WARN("empty image message!");
      return;
    }
    if (!outBag_.getFileName().empty()) {
      for (const auto i: irange(0ul, imageTopics_.size())) {
        outBag_.write(imageTopics_[i], msgs[i]->header.stamp, msgs[i]);
      }
    }
    if (writeFrames_ || writeVideo_) {
      const auto &msg0 = msgs[0];
      const int w(msg0->width), h(msg0->height);
      if (w == 0 || h == 0) {
        ROS_WARN("image 0 not full size!");
        return;
      }
      // allocate full image and start copying threads if
      // necessary
      if (fullImg_.rows == 0) {
        fullImg_ = cv::Mat(h * numRows_, w * numCols_,  CV_8UC3);
        startCopyThreads(w, h, (int)msgs.size());
      }
      // copy individual images into the full image
      // This is a surprisingly slow process and therefore done in parallel
      for (int i = 0; i < (int)msgs.size(); i++) {
        copyThreads_[i]->startCopy(msgs[i], frameNum_);
      }
      for (int i = 0; i < (int)msgs.size(); i++) {
        copyThreads_[i]->waitForCopyComplete();
      }
      if (writeFrames_ && (frameNum_ % keepRatio_ == 0)) {
        ROS_INFO_STREAM("wrote frame at time: " << msg0->header.stamp);
        cv::imwrite(make_file_name(frameBaseName_, frameNum_ / keepRatio_), fullImg_);
      }
      frameNum_++;

      if (writeVideo_) {
        writeVideoFrame(fullImg_, msg0->header);
      }
    } else {
      frameNum_++;
    }
  }

  bool DecodeBag::initialize() {
    std::string outBagName;
    nh_.param<std::string>("frame_base_dir", frameBaseDir_, "frames");
    frameBaseName_ = frameBaseDir_ + "/frame_";
    nh_.param<std::string>("video_base_name",  videoBaseName_,  "video_");
    nh_.param<int>("keep_ratio",   keepRatio_, 100);
    nh_.param<std::string>("out_bag_name",   outBagName, "");
    nh_.param<bool>("write_video",   writeVideo_, true);
    nh_.param<bool>("write_frames",  writeFrames_, false);
    nh_.param<bool>("write_individual_frames",
                    writeIndividualFrames_, false);
    nh_.param<int>("max_num_frames",  maxNumFrames_, INT_MAX);
    double frameRate{0};
    nh_.param<double>("frame_rate", frameRate, 0);
    if (frameRate > 0) {
      framePeriod_  = ros::Duration(1.0  / frameRate);
      minFrameDiff_ = ros::Duration(1.25 / frameRate);
    }
    if (!outBagName.empty()) {
      ROS_INFO_STREAM("writing to bag: " << outBagName);
      outBag_.open(outBagName, rosbag::BagMode::Write);
    }

    if (writeVideo_) {
      encoder_.setCodec("hevc_nvenc");
      encoder_.setProfile("main");
      encoder_.setPreset("slow");
      encoder_.setQMax(0);
      encoder_.setBitRate(10*1024*1024);
      encoder_.setGOPSize(20);
      encoder_.setFrameRate(frameRate == 0 ? 30 : (int)frameRate, 1);
      encoder_.setMeasurePerformance(false);
    }
    if (!nh_.getParam("image_topics", imageTopics_)) {
      ROS_ERROR("no image topics found!");
      return (false);
    }

    nh_.param<int>("num_rows",  numRows_, 2);
    nh_.param<int>("num_cols",  numCols_, 4);
    std::vector<int> locationToCamera;
    if (!nh_.getParam("location_to_camera", locationToCamera)) {
      ROS_WARN("no location_to_camera specified!");
      for (const auto i: irange(0, numRows_ * numCols_)) {
        locationToCamera.push_back(i);
      }
    }
    if ((int)locationToCamera.size() != numRows_ * numCols_) {
      ROS_ERROR("location_to_camera size mismatches with num_ros * num_cols");
      return (false);
    }
    cameraToLocation_.resize(numRows_ * numCols_);
    for (const auto loc_idx: irange(0ul, locationToCamera.size())) {
      const auto cam_idx = locationToCamera[loc_idx];
      if (cam_idx < 0 || cam_idx >= (int)cameraToLocation_.size()) {
        ROS_ERROR_STREAM("location_to_camera bad val: " << cam_idx);
        return (false);
      }
      cameraToLocation_[cam_idx] = loc_idx;
    }

    std::string bagFile;
    nh_.param<std::string>("bag_file",  bagFile,  "");
    std::string outFile = videoBaseName_ + ".h265";
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
    msgToDecode_ = msg; // enqueues message
    cv_.notify_all();
  }

  void DecodeBag::writeAuxFile() {
    // aux file has start, stop time, and number of frames
    std::string auxFile = videoBaseName_ + ".aux";
    std::ofstream af(auxFile);
    af << "ros_start_time "   << firstTime_.toNSec() << std::endl;
    af << "ros_end_time "     << lastTime_.toNSec() << std::endl;
    af << "number_of_frames " << frameNum_ << std::endl;
    af << "rate "             << 1.0 / framePeriod_.toSec() << std::endl;
    ROS_INFO_STREAM("wrote aux data to file: " << auxFile);
  }

  void DecodeBag::processBag(const std::string &fname) {
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::string> topics = imageTopics_;
    std::vector<std::vector<std::string>> tpvec = {topics};
    std::string all_topics_string;
    ImageSync::Callback cb = std::bind(&DecodeBag::syncCallback,
                                       this, std::placeholders::_1);
    sync_.reset(new ImageSync(tpvec, cb));
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
    ROS_INFO_STREAM("total frames: " << frameNum_ <<
                    ", inserted: " << numFramesInserted_);
    bag.close();
    writeAuxFile();
    sessions_.clear(); // should collect threads
    if (!outBag_.getFileName().empty()) {
      outBag_.close();
    }
  }
  
}  // namespace

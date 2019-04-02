/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport_tools/decode_frames.h"
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


  DecodeFrames::Session::Session(const std::string &topic,
                              const ImageSyncPtr &sync) :
    topic_(topic), sync_(sync) {
    thread_ = boost::make_shared<boost::thread>(
      &DecodeFrames::Session::decodeThread, this);
  }
  DecodeFrames::Session::~Session() {
    if (thread_) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        threadStopSignaled_ = true;
        cv_.notify_all();
      }
      thread_->join();
    }
  }

  DecodeFrames::DecodeFrames(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  DecodeFrames::~DecodeFrames() {
  }

  DecodeFrames::CopyThread::CopyThread(
    const cv::Mat &dest, int camNum,
    const std::string &baseDir) :
    cameraNumber_(camNum),
    dest_(dest) {
    frameBaseName_ = baseDir + "/frame_";
    thread_ = boost::make_shared<boost::thread>(
      &DecodeFrames::CopyThread::run, this);
  }

  DecodeFrames::CopyThread::~CopyThread() {
    if (thread_) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        threadStopSignaled_ = true;
        cv_.notify_all();
      }
      thread_->join();
    }
  }

  void DecodeFrames::CopyThread::run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!threadStopSignaled_ && ros::ok()) {
      while (!msgToCopy_ && !threadStopSignaled_) {
        cv_.wait(lock);
      }
      if (msgToCopy_) {
        cv_bridge::toCvShare(msgToCopy_,
               sensor_msgs::image_encodings::BGR8)->image.copyTo(dest_);
#if 0        
        std::string fname =
          make_file_name(frameBaseName_,
                         frameNum_ , "_" +
                         std::to_string(cameraNumber_));
        cv::imwrite(fname, dest_);
#endif        
      }
      msgToCopy_.reset();
      cv_.notify_all();
    }
  }

  void DecodeFrames::startCopyThreads(int w, int h, int size) {
    for (int i = 0; i < size; i++) {
      const int col = cameraToLocation_[i] % numCols_;
      const int row = cameraToLocation_[i] / numCols_;
      cv::Mat dst = fullImg_(cv::Rect(col * w, row * h, w, h));
      copyThreads_.emplace_back(
        new CopyThread(dst, i, frameBaseDir_));
    }
  }

  void DecodeFrames::syncCallback(const std::vector<ImageConstPtr> &msgs) {
    if (msgs.size() == 0) {
      ROS_WARN("empty image message!");
      return;
    }
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

    std::unique_lock<std::mutex> lock(mutex_);
    if (msg0->header.stamp > targetTime_) {
      ROS_INFO_STREAM("writing frame " << frameNum_ << " for time: " << msg0->header.stamp);
      cv::imwrite(make_file_name(frameBaseName_, frameNum_), fullImg_);
      frameNum_++;
      // stop all writes immediately until
      // next target time is set
      targetTime_ = ros::TIME_MAX;
    }
  }

  bool DecodeFrames::initialize() {
    nh_.param<std::string>("frame_base_dir", frameBaseDir_, "frames");
    frameBaseName_ = frameBaseDir_ + "/frame_";
    double startTime, endTime;
    nh_.param<double>("start_time", startTime, 0);
    nh_.param<double>("end_time",   endTime, 0);
    nh_.param<double>("delta_time", deltaTime_, 60.0);
    startTime_ = ros::Time(startTime);
    endTime_   = ros::Time(endTime);
   

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
    ROS_INFO_STREAM("decoding bag: " << bagFile);
    if (!bagFile.empty()) {
      processBag(bagFile);
    } else {
      ROS_ERROR_STREAM("must specify bag_file!");
    }
    ros::shutdown();
    return (true);
  }

  void DecodeFrames::Session::callback(const ImageConstPtr &img) {
    // pipe data into the sync, which will deliver a callback
    // when all images are present
    sync_->process(topic_, img);
  }

  void DecodeFrames::Session::decodeThread() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!threadStopSignaled_ && ros::ok()) {
      while (!msgToDecode_ && !threadStopSignaled_) {
        cv_.wait(lock);
      }
      if (msgToDecode_) {
        // this should produce a callback with the decoded image
        if (decoder_.isInitialized()) {
          decoder_.decodePacket(msgToDecode_);
        }
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

  void DecodeFrames::Session::processMessage(const FFMPEGPacketConstPtr &msg) {
    {
      std::unique_lock<std::mutex> lock(decodingMutex_);
      if (msg->flags != 0 && !isDecoding_) {
        isDecoding_ = true;
        if (!decoder_.isInitialized()) {
          decoder_.initialize(msg, boost::bind(&DecodeFrames::Session::callback,
                                               this, _1));
        }
      }
      if (!isDecoding_) {
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

  void DecodeFrames::Session::reset() {
    std::unique_lock<std::mutex> lock(decodingMutex_);
    isDecoding_ = false;
  }

  void DecodeFrames::processBag(const std::string &fname) {
    rosbag::Bag bag;
    bag.open(fname, rosbag::bagmode::Read);
    std::vector<std::string> topics = imageTopics_;
    std::vector<std::vector<std::string>> tpvec = {topics};
    std::string all_topics_string;
    ImageSync::Callback cb = std::bind(&DecodeFrames::syncCallback,
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
    if (startTime_ == ros::Time(0) || endTime_ == ros::Time(0)) {
      rosbag::View testView(bag, rosbag::TopicQuery(topics));
      if (startTime_ == ros::Time(0)) {
        startTime_ = testView.getBeginTime();
      }
      if (endTime_ == ros::Time(0)) {
        endTime_ = testView.getEndTime();
      }
    }
    ROS_INFO_STREAM("playing from file: " << fname << " time: "
                    << startTime_ << " - " << endTime_);

    for (ros::Time t = startTime_; t < endTime_; t = t + ros::Duration(deltaTime_)) {
      ROS_INFO_STREAM("extracting frame at time: " << t);
      rosbag::View view(bag, rosbag::TopicQuery(topics), t, t + ros::Duration(deltaTime_));
      {
        std::unique_lock<std::mutex> lock(mutex_);
        targetTime_ = t;
      }
      for (const rosbag::MessageInstance &m: view) {
        //std::cout << "from bag: " << m.getTime() << " " << m.getTopic() << std::endl;
        FFMPEGPacketConstPtr msg = m.instantiate<FFMPEGPacket>();
        if (msg) {
          SessionPtr sess = sessions_[m.getTopic()];
          sess->processMessage(msg);
        }
        if (!ros::ok()) {
          break;
        }
        {
          std::unique_lock<std::mutex> lock(mutex_);
          if (targetTime_ == ros::TIME_MAX) {
            for (auto &s: sessions_) {
              s.second->reset();
            }
            break;
          }
        }
      }
      if (targetTime_ != ros::TIME_MAX) {
        ROS_WARN_STREAM("no frame found for time " << t);
      }
    }
    bag.close();
    sessions_.clear(); // should collect threads
  }
  
}  // namespace

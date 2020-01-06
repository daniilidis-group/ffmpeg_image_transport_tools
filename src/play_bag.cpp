/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport_tools/play_bag.h"
#include <boost/range/irange.hpp>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <cv_bridge/cv_bridge.h>

namespace ffmpeg_image_transport_tools {
  using boost::irange;

  static std::string add_slash(const std::string &transp) {
    return (transp.empty() ? transp: ("/" + transp));
  }

  PlayBag::Session::Session(const std::string &topic,
                            const std::string &transName,
                            const ImageTransportPtr &transp,
                            const ImageSyncPtr &sync) :
    topic_(topic + add_slash(transName)), sync_(sync) {
    imgPub_ = transp->advertise(topic + "/synced", 1);
    thread_ = std::make_shared<std::thread>(&PlayBag::Session::decodeThread,
                                            this);
  }

  void PlayBag::Session::join() {
    if (thread_) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        keepRunning_ = false;
        cv_.notify_all();
      }
      thread_->join();
    }
  }

  void PlayBag::Session::decodeThread() {
    while (keepRunning_) {
      FFMPEGPacketConstPtr msg;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        while (queue_.empty() && keepRunning_) {
          cv_.wait_for(lock, std::chrono::seconds(1));
        }
        if (!queue_.empty()) {
          msg = queue_.front();
          queue_.pop();
          cv_.notify_all();
        }
      }
      if (msg) {
        // not holding the lock while processing...
        processMessage(msg);
      }
    }
  }

  void PlayBag::Session::addToQueue(FFMPEGPacketConstPtr &msg) {
    std::unique_lock<std::mutex> lock(mutex_);
    const int MAX_QUEUE_SIZE(3);
    // block if too many are in the queue already
    while (queue_.size() >= MAX_QUEUE_SIZE) {
      cv_.wait(lock);
    }
    queue_.push(msg);
    cv_.notify_all();
  }

  void PlayBag::Session::publish(const ImageConstPtr &msg) {
    imgPub_.publish(msg);
  }

  void PlayBag::Session::callback(const ImageConstPtr &img) {
    // pipe data into the sync, which will deliver a callback
    // when all images are present
    sync_->process(topic_, img);
  }

  void PlayBag::Session::processMessage(const FFMPEGPacketConstPtr &msg) {
    if (!decoder_.isInitialized()) {
      decoder_.initialize(msg, boost::bind(&PlayBag::Session::callback,
                                           this, _1));
      if (!decoder_.isInitialized()) {
        return;
      }
    }
    // this will produce a callback to callback() when
    // decoding has been successful
    decoder_.decodePacket(msg);
  }
  
  void PlayBag::Session::processMessage(const ImageConstPtr &msg) {
    sync_->process(topic_, msg);
  }

  void PlayBag::Session::processMessage(const CompressedImageConstPtr &msg) {
    // first convert to Mono8, i.e. just extract the raw bytes.
    cv::Mat im1 = cv_bridge::toCvCopy(
      msg, sensor_msgs::image_encodings::MONO8)->image;
    cv::Mat im;
    // now debayer
    cv::cvtColor(im1, im, cv::COLOR_BayerBG2BGR);
    ImageConstPtr img =
      cv_bridge::CvImage(msg->header, "bgr8", im).toImageMsg();
    sync_->process(topic_, img);
  }

  PlayBag::PlayBag(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  void PlayBag::syncCallback(const std::vector<ImageConstPtr> &msgs) {
    if (msgs.size() != topics_.size()) {
      ROS_WARN("invalid image message!");
      return;
    }
    const ros::Time t = msgs[0]->header.stamp;
    // wait for previous message to clear
    if (waitForAck_) {
      std::unique_lock<std::mutex> lock(mutex_);
      if (ackTime_ != ros::Time(0)) {
        cv_.wait_for(lock, retryTimeout_);
      }
      if (ackTime_ != ros::Time(0)) {
        ROS_WARN_STREAM("play_bag timed out w/o ack, sending anyways");
      }
    }
    for (const auto i: irange(0ul, msgs.size())) {
      sessions_[topics_[i]]->publish(msgs[i]);
    }
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = t;
    clockPub_.publish(clockMsg);

    if (waitForAck_) {
      std::unique_lock<std::mutex> lock(mutex_);
      ackTime_ = msgs[0]->header.stamp;
    }
    frameNum_++;
  }

  bool PlayBag::initialize() {
    std::string bagFile;
    nh_.param<std::string>("transport", transportName_, "ffmpeg");
    nh_.param<std::string>("bag_file",  bagFile,  "");
    nh_.param<int>("max_num_frames",  maxNumFrames_,  1000000000);
    double startTime, endTime;
    nh_.param<double>("start_time", startTime, -1.0);
    nh_.param<double>("end_time", endTime, -1.0);
    startTime_ = startTime < 0 ? ros::TIME_MIN : ros::Time(startTime);
    endTime_ = endTime < 0 ? ros::TIME_MAX : ros::Time(endTime);
    imgTrans_.reset(new image_transport::ImageTransport(nh_));
    if (!nh_.getParam("image_topics", imageTopics_)) {
      ROS_ERROR("no image topics found!");
      return (false);
    }
    nh_.param<bool>("wait_for_ack", waitForAck_, true);
    int retrySeconds;
    nh_.param<int>("retry_timeout", retrySeconds, 5);
    retryTimeout_ = std::chrono::seconds(retrySeconds);
    sub_ = nh_.subscribe("ack", 2, &PlayBag::ackCallback, this);
    clockPub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1000);

    ROS_INFO_STREAM("initialized, " << (waitForAck_ ? "" : " NOT ")
                    << "waiting for ack!");
    if (!bagFile.empty()) {
      openBag(bagFile);
      playBagThread_ = std::make_shared<std::thread>(&PlayBag::play, this);
    } else {
      ROS_ERROR_STREAM("must specify bag_file!");
    }
    return (true);
  }
  
  void PlayBag::ackCallback(const std_msgs::Header::ConstPtr &ackMsg) {
    if (waitForAck_) {
      std::unique_lock<std::mutex> lock(mutex_);
      ackTime_ = ros::Time(0);
      cv_.notify_all();
      if (ackMsg->frame_id == "FINISHED!") {
        maxNumFrames_ = 0; // will cause playback to stop
      }
    }
  }

  void PlayBag::closeBag() {
    ROS_INFO_STREAM("total frames: " << frameNum_ );
    bag_.close();
    cv_.notify_all(); // try to shake loose any threads stuck in sync
    for (auto &s: sessions_) {
      s.second->join();
    }
    sessions_.clear();
  }

  void PlayBag::openBag(const std::string &fname) {
    ROS_INFO_STREAM("opening bag (this could take a while): " << fname);
    bag_.open(fname, rosbag::bagmode::Read);
    ROS_INFO_STREAM("bag opened!");

    for (const auto tp: imageTopics_) {
      topics_.push_back(tp + add_slash(transportName_));
    }
    std::vector<std::vector<std::string>> tpvec = {topics_};
    ImageSync::Callback cb = std::bind(&PlayBag::syncCallback,
                                       this, std::placeholders::_1);
    sync_.reset(new ImageSync(tpvec, cb, 30));
    for (const auto i: irange(0ul, topics_.size())) {
      const auto &topic = topics_[i];
      rosbag::View cv(bag_, rosbag::TopicQuery({topic}),
                      startTime_, endTime_);
      if (cv.begin() == cv.end()) {
        ROS_WARN_STREAM("cannot find topic: " << topic);
      }
      if (sessions_.count(topic) == 0) {
        sessions_[topic].reset(
          new Session(imageTopics_[i], transportName_,
                      imgTrans_, sync_));
        ROS_INFO_STREAM("play_bag topic: " << topic);
      } else {
        ROS_WARN_STREAM("play_bag dup topic: " << topic);
      }
    }
  }

  void PlayBag::play() {
    rosbag::View view(bag_, rosbag::TopicQuery(topics_),
                      startTime_, endTime_);
    auto t0 = ros::WallTime::now();
    int cnt(0), perfInterval(500);
    for (const rosbag::MessageInstance &m: view) {
      FFMPEGPacketConstPtr    msgFF   = m.instantiate<FFMPEGPacket>();
      ImageConstPtr           msgImg  = m.instantiate<Image>();
      CompressedImageConstPtr msgComp = m.instantiate<CompressedImage>();
      if (msgFF || msgImg || msgComp) {
        SessionPtr sess = sessions_[m.getTopic()];
        if (msgFF)   sess->addToQueue(msgFF);
        if (msgImg)  sess->processMessage(msgImg);
        if (msgComp) {
          sess->processMessage(msgComp);
        }
        if (cnt++ > perfInterval) {
          const auto t1 = ros::WallTime::now();
          ROS_INFO_STREAM("played frames: " << frameNum_ << " fps: " <<
                          perfInterval / (topics_.size() * (t1-t0).toSec()));
          cnt = 0;
          t0 = t1;
        }
      }
      if (!ros::ok()) {
        ROS_INFO_STREAM("play_bag quiting due to ros signal");
        break;
      }
      if ((int)frameNum_ > maxNumFrames_) {
        ROS_INFO_STREAM("play_bag reached max num frames: " << maxNumFrames_ );
        break;
      }
    }
    closeBag();
    ROS_INFO_STREAM("finished playing bag!");
  }

}  // namespace

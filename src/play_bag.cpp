/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport_tools/play_bag.h"
#include <rosbag/view.h>
#include <boost/range/irange.hpp>

namespace ffmpeg_image_transport_tools {
  using boost::irange;
  
  PlayBag::Session::Session(const std::string &topic,
                            const ImageTransportPtr &transp,
                            const ImageSyncPtr &sync) :
    topic_(topic), sync_(sync) {
    pub_ = transp->advertise(topic + "/synced", 1);
  }

  void PlayBag::Session::publish(const ImageConstPtr &msg) {
    pub_.publish(msg);
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

  PlayBag::PlayBag(const ros::NodeHandle& pnh) :  nh_(pnh) {
  }

  void PlayBag::syncCallback(const std::vector<ImageConstPtr> &msgs) {
    if (msgs.size() != imageTopics_.size()) {
      ROS_WARN("invalid image message!");
      return;
    }
    for (const auto i: irange(0ul, msgs.size())) {
      sessions_[imageTopics_[i]]->publish(msgs[i]);
    }
    // TODO: wait here for ack!
    frameNum_++;
  }

  bool PlayBag::initialize() {
    std::string bagFile;
    nh_.param<std::string>("bag_file",  bagFile,  "");
    nh_.param<int>("max_num_frames",  maxNumFrames_,  1000000000);
    imgTrans_.reset(new image_transport::ImageTransport(nh_));
    if (!nh_.getParam("image_topics", imageTopics_)) {
      ROS_ERROR("no image topics found!");
      return (false);
    }
    if (!bagFile.empty()) {
      processBag(bagFile);
    } else {
      ROS_ERROR_STREAM("must specify bag_file!");
    }
    return (true);
  }

  void PlayBag::processBag(const std::string &fname) {
    ROS_INFO_STREAM("opening bag: " << fname);
    rosbag::Bag bag(fname, rosbag::bagmode::Read);
    ROS_INFO_STREAM("opening bag (this could take a while): " << fname);
    ROS_INFO_STREAM("bag opened!");
    std::vector<std::string> topics = imageTopics_;
    std::vector<std::vector<std::string>> tpvec = {topics};
    ImageSync::Callback cb = std::bind(&PlayBag::syncCallback,
                                       this, std::placeholders::_1);
    sync_.reset(new ImageSync(tpvec, cb));
    for (const auto &topic: topics) {
      rosbag::View cv(bag, rosbag::TopicQuery({topic}));
      if (cv.begin() == cv.end()) {
        ROS_WARN_STREAM("cannot find topic: " << topic);
      }
      if (sessions_.count(topic) == 0) {
        sessions_[topic].reset(new Session(topic, imgTrans_, sync_));
        ROS_INFO_STREAM("using topic: " << topic);
      } else {
        ROS_WARN_STREAM("duplicate topic: " << topic);
      }
    }
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
    ROS_INFO_STREAM("total frames: " << frameNum_ );
    bag.close();
    sessions_.clear();
  }
  
}  // namespace

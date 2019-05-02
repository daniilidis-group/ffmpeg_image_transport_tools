/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <ffmpeg_image_transport/ffmpeg_decoder.h>
#include <ffmpeg_image_transport/ffmpeg_encoder.h>
#include <image_transport/image_transport.h>
#include <flex_sync/sync.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

#include <boost/thread.hpp>

#include <fstream>
#include <vector>
#include <thread>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <condition_variable>

namespace ffmpeg_image_transport_tools {
  class PlayBag {
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using FFMPEGPacket  = ffmpeg_image_transport_msgs::FFMPEGPacket;
    using FFMPEGPacketConstPtr = FFMPEGPacket::ConstPtr;
    using ThreadPtr = std::shared_ptr<std::thread>;
    typedef flex_sync::Sync<sensor_msgs::Image> ImageSync;
    typedef std::shared_ptr<ImageSync> ImageSyncPtr;
    typedef std::shared_ptr<image_transport::ImageTransport> ImageTransportPtr;
  public:
    class Session {
    public:
      Session(const std::string &topic,
              const ImageTransportPtr &trans,
              const ImageSyncPtr &sync);
      void callback(const ImageConstPtr &img);
      void processMessage(const FFMPEGPacketConstPtr &msg);
      void publish(const ImageConstPtr &msg);
    private:
      // --------- variables
      std::string             topic_;
      ImageSyncPtr            sync_;
      ffmpeg_image_transport::FFMPEGDecoder decoder_;
      image_transport::Publisher   imgPub_;
    };
    
    PlayBag(const ros::NodeHandle& pnh);

    bool initialize();
    void syncCallback(const std::vector<ImageConstPtr> &msgs);
  private:
    void play();
    void ackCallback(const std_msgs::Header::ConstPtr &ackMsg);
    void openBag(const std::string &fname);
    void closeBag();

    // ------------------------ variables --------
    ros::NodeHandle   nh_;
    typedef std::shared_ptr<Session> SessionPtr;
    typedef std::unordered_map<std::string, SessionPtr> SessionMap;
    rosbag::Bag   bag_;
    ImageSyncPtr  sync_;
    SessionMap    sessions_;
    unsigned int  frameNum_{0};
    int           maxNumFrames_;
    ros::Time     ackTime_{0};
    bool          waitForAck_{true};
    std::mutex    mutex_;
    std::condition_variable  cv_;
    std::chrono::seconds     retryTimeout_;
    ros::Subscriber          sub_;
    ros::Publisher           clockPub_;
    std::vector<std::string> imageTopics_;
    std::vector<std::string> topics_;
    std::shared_ptr<image_transport::ImageTransport> imgTrans_;
    ThreadPtr     playBagThread_;
  };

}


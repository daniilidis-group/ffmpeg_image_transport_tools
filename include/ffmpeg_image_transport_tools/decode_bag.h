/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <ffmpeg_image_transport/ffmpeg_decoder.h>
#include <ffmpeg_image_transport/ffmpeg_encoder.h>
#include <flex_sync/sync.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

#include <fstream>
#include <vector>
#include <unordered_map>
#include <memory>

namespace ffmpeg_image_transport_tools {
  class DecodeBag {
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using FFMPEGPacket  = ffmpeg_image_transport_msgs::FFMPEGPacket;
    using FFMPEGPacketConstPtr = FFMPEGPacket::ConstPtr;
    typedef flex_sync::Sync<sensor_msgs::Image> ImageSync;
    typedef std::shared_ptr<ImageSync> ImageSyncPtr;
  public:
    class Session {
    public:
      Session(const std::string &topic, const ImageSyncPtr &sync);
      void callback(const ImageConstPtr &img);
      void processMessage(const FFMPEGPacketConstPtr &msg);
    private:
      // --------- variables
      std::string   topic_;
      ImageSyncPtr  sync_;
      ffmpeg_image_transport::FFMPEGDecoder decoder_;
    };
    
    DecodeBag(const ros::NodeHandle& pnh);
    ~DecodeBag();
    bool initialize();
    void syncCallback(const std::vector<ImageConstPtr> &msgs);
  private:
    void processBag(const std::string &fname);
    void packetReady(const FFMPEGPacketConstPtr &pkt);
    // ------------------------ variables --------
    ros::NodeHandle   nh_;
    typedef std::shared_ptr<Session> SessionPtr;
    typedef std::unordered_map<std::string, SessionPtr> SessionMap;
    ImageSyncPtr  sync_;
    ffmpeg_image_transport::FFMPEGEncoder encoder_;
    SessionMap    sessions_;
    int           numRows_;
    int           numCols_;
    unsigned int  frameNum_{0};
    int           maxNumFrames_;
    std::vector<int>  cameraToLocation_;
    std::vector<std::string> imageTopics_;
    std::ofstream rawStream_;
  };

}


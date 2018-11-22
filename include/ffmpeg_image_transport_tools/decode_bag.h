/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <ffmpeg_image_transport/ffmpeg_decoder.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <memory>
#include <unordered_map>


namespace ffmpeg_image_transport_tools {
  class DecodeBag {
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using FFMPEGPacket  = ffmpeg_image_transport_msgs::FFMPEGPacket;
    using FFMPEGPacketConstPtr = FFMPEGPacket::ConstPtr;
  public:
    class Session {
    public:
      Session(const std::string &topic);
      void callback(const ImageConstPtr &img);
      void processMessage(const FFMPEGPacketConstPtr &msg);
    private:
      // --------- variables
      std::string   topic_;
      ffmpeg_image_transport::FFMPEGDecoder decoder_;
    };
    
    DecodeBag(const ros::NodeHandle& pnh);
    ~DecodeBag();
    bool initialize();
  private:
    void processBag(const std::string &fname);
    // ------------------------ variables --------
    ros::NodeHandle   nh_;
    typedef std::shared_ptr<Session> SessionPtr;
    typedef std::unordered_map<std::string, SessionPtr> SessionMap;
    SessionMap  sessions_;
  };

}


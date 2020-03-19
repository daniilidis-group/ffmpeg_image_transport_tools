/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <ffmpeg_image_transport_msgs/FFMPEGPacket.h>

#include <rosbag/bag.h>
#include <ros/ros.h>

#include <fstream>
#include <vector>
#include <memory>
#include <unordered_map>

namespace ffmpeg_image_transport_tools {
  class SplitBag {
    using FFMPEGPacket  = ffmpeg_image_transport_msgs::FFMPEGPacket;
    using FFMPEGPacketConstPtr = FFMPEGPacket::ConstPtr;
  public:
    class Session {
    public:
      Session(const std::string &topic, const std::string &base,
              bool writeTimeStamp, unsigned int idx);
      ~Session();
      void process(const FFMPEGPacketConstPtr &msg);
    private:
      // --------- variables
      std::string             topic_;
      std::ofstream           rawStream_;
      std::ofstream           ts_;
      unsigned int            frameCnt_{0};
    };
    
    SplitBag(const ros::NodeHandle& pnh);
    ~SplitBag();
    bool initialize();
  private:
    void makeSessions(rosbag::Bag *bag,
                      const ros::Time &t_start,
                      const ros::Time &t_end);

    void processBag(const std::string &fname,
                    double start_time, double duration);

    void convertToMP4() const;
    // ------------------------ variables --------
    ros::NodeHandle   nh_;
    typedef std::shared_ptr<Session> SessionPtr;
    typedef std::unordered_map<std::string, SessionPtr> SessionMap;
    SessionMap    sessions_;
    std::string   outFileBase_;
    unsigned int  frameNum_{0};
    int           maxNumFrames_;
    bool          writeTimeStamps_;
    int           videoRate_;
    std::vector<std::string> imageTopics_;
  };

}


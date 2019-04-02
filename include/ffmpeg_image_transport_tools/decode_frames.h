/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include <ffmpeg_image_transport/ffmpeg_decoder.h>
#include <flex_sync/sync.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

#include <boost/thread.hpp>

#include <fstream>
#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <condition_variable>

namespace ffmpeg_image_transport_tools {
  class DecodeFrames {
    using ImageConstPtr = sensor_msgs::ImageConstPtr;
    using FFMPEGPacket  = ffmpeg_image_transport_msgs::FFMPEGPacket;
    using FFMPEGPacketConstPtr = FFMPEGPacket::ConstPtr;
    typedef flex_sync::Sync<sensor_msgs::Image> ImageSync;
    typedef std::shared_ptr<ImageSync> ImageSyncPtr;
  public:
    class CopyThread {
    public:
      CopyThread(const cv::Mat &dest, int camNum,
                 const std::string &baseDir);
      ~CopyThread();
      inline void startCopy(const ImageConstPtr &msg,
                            unsigned int fnum) {
        std::unique_lock<std::mutex> lock(mutex_);
        msgToCopy_ = msg;
        frameNum_  = fnum;
        cv_.notify_all();
      }
      inline void waitForCopyComplete() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (msgToCopy_ && !threadStopSignaled_) {
          cv_.wait(lock);
        }
        msgToCopy_.reset();
      }
      void run();
    private:
      std::mutex              mutex_;
      std::condition_variable cv_;
      bool                    threadStopSignaled_{false};
      ImageConstPtr           msgToCopy_;
      std::string             frameBaseName_;
      int                     cameraNumber_{0};
      unsigned int            frameNum_{0};
      cv::Mat                 dest_;
      boost::shared_ptr<boost::thread> thread_;
    };
    class Session {
    public:
      Session(const std::string &topic, const ImageSyncPtr &sync);
      ~Session();
      void callback(const ImageConstPtr &img);
      void processMessage(const FFMPEGPacketConstPtr &msg);
      void reset();
    private:
      void decodeThread();
      // --------- variables
      std::string             topic_;
      ImageSyncPtr            sync_;
      unsigned int            packetNum_{0};
      std::mutex              mutex_;
      std::condition_variable cv_;
      ffmpeg_image_transport::FFMPEGDecoder decoder_;
      FFMPEGPacketConstPtr    msgToDecode_;
      boost::shared_ptr<boost::thread> thread_;
      bool                    threadStopSignaled_{false};
      bool                    isDecoding_{false};
      std::mutex              decodingMutex_;
    };
    
    DecodeFrames(const ros::NodeHandle& pnh);
    ~DecodeFrames();
    bool initialize();
    void syncCallback(const std::vector<ImageConstPtr> &msgs);
  private:
    void processBag(const std::string &fname);
    void startCopyThreads(int w, int h, int size);

    // ------------------------ variables --------
    ros::NodeHandle   nh_;
    typedef std::shared_ptr<Session> SessionPtr;
    typedef std::unordered_map<std::string, SessionPtr> SessionMap;
    std::vector<std::shared_ptr<CopyThread>> copyThreads_;
    ImageSyncPtr  sync_;
    SessionMap    sessions_;
    int           numRows_;
    int           numCols_;
    cv::Mat       fullImg_;
    unsigned int  frameNum_{0};
    int           maxNumFrames_;
    std::vector<int>  cameraToLocation_;
    std::vector<std::string> imageTopics_;
    ros::Time     startTime_{0};
    ros::Time     endTime_{0};
    double        deltaTime_;
    std::string   frameBaseDir_;
    std::string   frameBaseName_;
    bool          frameFound_{false};
    std::mutex    mutex_;
    ros::Time     targetTime_{ros::TIME_MAX};
  };

}


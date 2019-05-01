/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport_tools/play_bag.h"
#include <nodelet/nodelet.h>

#include <memory>

namespace ffmpeg_image_transport_tools {
  class PlayBagNodelet: public nodelet::Nodelet {

  public:
    void onInit() override {
      nh_ = getPrivateNodeHandle();
      instance_.reset(new PlayBag(nh_));
      ROS_INFO_STREAM("created PlayBag");
      instance_->initialize();
    }
  private:
    ros::NodeHandle nh_;
    std::shared_ptr<PlayBag> instance_;
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ffmpeg_image_transport_tools::PlayBagNodelet,
                       nodelet::Nodelet)

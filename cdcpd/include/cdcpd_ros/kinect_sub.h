#ifndef KINECT_SUB_H
#define KINECT_SUB_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <memory>
#include <string>

class KinectSub {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  struct SubscriptionOptions {
    rclcpp::Node::SharedPtr node;
    int queue_size;
    std::string topic_prefix;
    std::string rgb_topic;
    std::string depth_topic;
    std::string cam_topic;

    explicit SubscriptionOptions(const std::string& prefix = "kinect2_victor_head/hd")
        : node(nullptr),
          queue_size(10),
          topic_prefix(prefix),
          rgb_topic(topic_prefix + "/image_color_rect"),
          depth_topic(topic_prefix + "/image_depth_rect"),
          cam_topic(topic_prefix + "/camera_info") {}
  };

  std::shared_ptr<image_transport::ImageTransport> it;
  std::shared_ptr<image_transport::SubscriberFilter> rgb_sub;
  std::shared_ptr<image_transport::SubscriberFilter> depth_sub;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> cam_sub;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

  SubscriptionOptions options;

  std::function<void(cv::Mat, cv::Mat, cv::Matx33d)> externCallback;

  // Callback is in the form (rgb, depth, cameraIntrinsics)
  explicit KinectSub(const std::function<void(cv::Mat, cv::Mat, cv::Matx33d)>& _externCallback,
                     const SubscriptionOptions _options = SubscriptionOptions());

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg, 
               const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_msg);
};

#endif  // KINECT_SUB_H

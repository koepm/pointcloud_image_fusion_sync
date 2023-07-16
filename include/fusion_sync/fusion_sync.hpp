#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

// image type
using StampedImageMsg = sensor_msgs::msg::Image;
using StampedImageMsgSubscriber = message_filters::Subscriber<StampedImageMsg>;

// pointcloud type
using StampedPclMsg = sensor_msgs::msg::PointCloud2;
using StampedPclMsgSubscriber = message_filters::Subscriber<StampedPclMsg>;

using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<StampedImageMsg, StampedPclMsg>;
using ApproximateSync = message_filters::Synchronizer<ApproximateSyncPolicy>;


class FusionSync : public rclcpp::Node
{
public:
  FusionSync();

private:
  std::shared_ptr<StampedImageMsgSubscriber> img_sub_;
  std::shared_ptr<StampedPclMsgSubscriber> pcl_sub_;
  //std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<StampedImageMsg, StampedPclMsgSubscriber>>> approximate_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;
  
  void approximateSyncCallback(
    const std::shared_ptr<const sensor_msgs::msg::Image>& msg1,
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg2
    );
  
  //variable
  int img_height = 480;
  int img_width = 640;
  cv::Mat concatedMat;
  
  // CameraExtrinsicMat
  cv::Mat CameraExtrinsicMat = (cv::Mat_<double>(4, 4) << 
      0.01357418, -0.99990202, -0.00341862,  0.01607133,
      0.0375878 ,  0.00392679, -0.99928561,  0.04019551,
      0.99920113,  0.01343598,  0.03763742, -0.33372581,
      0.         ,  0.         ,  0.         ,  1.        );

  // CameraMat
  cv::Mat CameraMat = (cv::Mat_<double>(3, 3) << 
      496.7298126215774,   0.,   322.3650797270366,
      0.,  496.986275559556,  231.3556675479293, 
      0., 0.,   1. );

  // DistCoeff
  cv::Mat DistCoeff = (cv::Mat_<double>(1, 5) << 
      0.069668722452185, -0.180233541236036, 0.,   0.,   0.);

  cv::Mat transformMat;
  cv_bridge::CvImagePtr cv_ptr_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  
  

  float max_FOV = CV_PI/4;    // 카메라의 최대 화각(라디안)
};
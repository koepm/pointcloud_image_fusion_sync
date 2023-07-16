#include "fusion_sync/fusion_sync.hpp"

FusionSync::FusionSync()
: Node("fusion_test")
{
  img_sub_ = std::make_shared<StampedImageMsgSubscriber>(this, "video1", rmw_qos_profile_sensor_data);
  pcl_sub_ = std::make_shared<StampedPclMsgSubscriber>(this, "velodyne_points", rmw_qos_profile_sensor_data);
  pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cropped_roi", 10);

  approximate_sync_ = std::make_shared<ApproximateSync>(ApproximateSyncPolicy(10), *img_sub_, *pcl_sub_);
  approximate_sync_->registerCallback(std::bind(&FusionSync::approximateSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void FusionSync::approximateSyncCallback(
  const std::shared_ptr<const sensor_msgs::msg::Image>& msg1,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg2)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // ROS2 메세지를 PCL 포인트 클라우드로 변환
    pcl::fromROSMsg(*msg2, *point_cloud);

    
    // crop 영역설정
    pcl::CropBox<pcl::PointXYZI> cropped_pcl;
    cropped_pcl.setInputCloud(point_cloud);
    //cropped_pcl.setMin(Eigen::Vector4f(-0, -7, -0.3, 0)); // x, y, z, min (m)
    //cropped_pcl.setMax(Eigen::Vector4f(15, 0, 1.0, 0));    // (-0,20) (-15,15) (-0.5,1.0)...............8/4 data
    cropped_pcl.setMax(Eigen::Vector4f(20.0, 5.0, 6.0, 1.0));    // 상자의 최대 좌표 설정
    cropped_pcl.setMin(Eigen::Vector4f(0.0, -5.0, -6.0, 1.0));  // 상자의 최소 좌표 설정    
    cropped_pcl.filter(*point_cloud);

    try
    {
      cv_bridge::CvImagePtr cv_ptr_;
      cv_ptr_ = cv_bridge::toCvCopy(msg1, "bgr8");
      cv::Mat undistorted_image = cv_ptr_->image;
      ///cv::undistort(cv_ptr_->image, undistorted_image, CameraMat, DistCoeff);
      //cv::resize(cv_ptr_->image, cv_ptr_->image, cv::Size(img_width,img_height));

      int size = point_cloud->points.size(); 

      //std::cout<<"point cloud size: "<<size<< std::endl;;

      for(int i =0; i <size; i++)
      {
          // project get the photo coordinate
          pcl::PointXYZI temp;
          
          temp.x = point_cloud->points[i].x;
          temp.y = point_cloud->points[i].y;
          temp.z = point_cloud->points[i].z;
          temp.intensity = point_cloud->points[i].intensity;

          printf("intensity : %f\n", temp.intensity);

          //float R_ = sqrt(pow(temp.x,2)+pow(temp.y,2)); // 수평거리
          float azimuth_ = abs(atan2(temp.y, temp.z)); // 각도
          //float elevation_ = abs(atan2(R_, temp.z));  // 수직방향

          //if(azimuth_ > max_FOV )
          //{
              // Construct transformMat
              cv::Mat Rlc = CameraExtrinsicMat(cv::Rect(0, 0, 3, 3));
              cv::Mat Tlc = CameraExtrinsicMat(cv::Rect(3, 0, 1, 3));
              cv::Mat concatedMat;
              
              cv::hconcat(Rlc, Tlc, concatedMat);
              cv::Mat transformMat = CameraMat * concatedMat;

              double a_[4] = { temp.x, temp.y, temp.z, 1.0 };
              cv::Mat pos(4, 1, CV_64F, a_);

              cv::Mat newpos(transformMat * pos);

              float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
              float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));

              if (x >= 0 && x < 640 && y >= 0 && y < 480)
              {
                  //printf("x : %f y : %f\n", x, y);
                  if (temp.intensity > 30)
                  {
                      cv::circle(undistorted_image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
                  }
                  else
                  {
                      cv::circle(undistorted_image, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
                  }
              }
      }

      sensor_msgs::msg::PointCloud2 cropped_point_cloud;
      pcl::toROSMsg(*point_cloud, cropped_point_cloud);
      pcl_pub_ -> publish(cropped_point_cloud);

      cv::imshow("combined_img", undistorted_image);
      cv::waitKey(1);

    }
    catch(cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionSync>());
  rclcpp::shutdown();
  return 0;
}
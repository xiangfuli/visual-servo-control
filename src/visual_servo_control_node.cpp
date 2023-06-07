#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "visual_servo_control/SurfaceNorm.h"
#include "vision_process/planes/plane_recognizer.hpp"
#include "vision_process/utils/image_utils.hpp"

uint16_t region_width = 100, region_height = 100;
ros::Publisher surface_norm_pub, surface_norm_filtered_pub, white_region_threshold_pub;
ros::Subscriber depth_image_sub, rgb_image_sub;
sensor_msgs::CameraInfo camera_info;

void deprojection(const cv::Mat image, const sensor_msgs::CameraInfo camera_info, std::vector<Eigen::Vector3d> &points) {
  uint16_t height, width;
  height = image.rows;
  width = image.cols;

  float32_t Kx = camera_info.K[0];
  float32_t Ky = camera_info.K[4];

  for (int i=0; i < height; i++) {
    for (int j=0; j < width; j++) {
      float32_t z = image.at<float32_t>(i, j);
      if (z < 1) {
        points.push_back(Eigen::Vector3d((i - height / 2.0) / Kx * z, (j - width / 2.0) / Ky * z, z));
      }
    }
  }
}

void norm_calc(const sensor_msgs::Image depth_image, visual_servo_control::SurfaceNorm &norm) {
  // extract the central part of the depth image
  uint32_t height, width; 
  height = depth_image.height;
  width = depth_image.width;

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(depth_image, depth_image.encoding);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat cv_cropped_image = cv_ptr->image(cv::Rect(
     width / 2 - region_width / 2, height / 2 - region_height / 2,
     region_width, region_height 
  ));
  
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d norm_vector;
  norm_vector.setZero();

  deprojection(cv_cropped_image, camera_info, points);

  if (points.size() > 100) {
    PlaneRecognizer::norm(points, norm_vector, PlaneRecognizationMethod::PLANE_SVD);
    norm.detected = true;
  } else {
    ROS_INFO_DELAYED_THROTTLE(10, "Surface not close enough to get the norm.");
    norm.detected = false;
  }

  norm.x = norm_vector.x();
  norm.y = norm_vector.y();
  norm.z = norm_vector.z();
}

void depth_callback(sensor_msgs::Image depth_image) {
  visual_servo_control::SurfaceNorm norm;
  norm_calc(depth_image, norm);

  surface_norm_pub.publish(norm);
}

void rgb_callback(sensor_msgs::Image rgb_image) {
  std_msgs::Header header;
  sensor_msgs::Image image_msg;

  cv::Mat grayscale_image, white_region_threshold_image;
  
  cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(rgb_image, rgb_image.encoding);
  cv::cvtColor(ptr->image, grayscale_image, cv::COLOR_BGR2GRAY);
  ImageUtils::extract_white_region(grayscale_image, white_region_threshold_image);
  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, 
    white_region_threshold_image);
  img_bridge.toImageMsg(image_msg);
  white_region_threshold_pub.publish(image_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_servo_control");

  ros::NodeHandle n;

  std::string camera_depth_info_topic_name = argv[1];
  std::string camera_depth_topic_name = argv[2];
  std::string camera_rgb_topic_name = argv[3];

  // Get the camera info from the topic
  camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_depth_info_topic_name.c_str());

  depth_image_sub = n.subscribe(camera_depth_topic_name, 1, depth_callback);
  rgb_image_sub = n.subscribe(camera_rgb_topic_name, 1, rgb_callback);


  white_region_threshold_pub = n.advertise<sensor_msgs::Image>("white_region_threshold/raw", 1);
  surface_norm_pub = n.advertise<visual_servo_control::SurfaceNorm>("surface_norm/raw", 1);
  surface_norm_filtered_pub = n.advertise<visual_servo_control::SurfaceNorm>("surface_norm/filtered", 1);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

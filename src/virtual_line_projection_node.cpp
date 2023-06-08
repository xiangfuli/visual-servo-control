#include <iostream>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "visual_servo_control/SurfaceNorm.h"
#include "vision_process/planes/plane_recognizer.hpp"
#include "vision_process/utils/image_utils.hpp"
#include "vision_process/utils/so3_utils.hpp"
#include "vision_lines/line.h"

ros::Publisher rgb_virtual_image_pub, virtual_line_measure_pub;
ros::Subscriber rgb_image_sub, vehicle_position_sub, depth_image_sub;
sensor_msgs::CameraInfo camera_info;

Eigen::Vector3f virtual_line_start_point, virtual_line_end_point;
Eigen::Vector2i virtual_line_left_pixel, virtual_line_right_pixel;

Eigen::Vector3f vehicle_position;
Eigen::Vector4f vehicle_pose;
Eigen::Matrix<float, 3, 3> K_matrix;

float rho, last_rho, pitch_radiant, last_pitch_radiant;
float depth_to_the_line, last_depth_to_the_line;
float pixel_distance_to_the_camera_center, last_pixel_distance_to_the_camera_center;
float velocity_filtered = 0;
float pitch_velocity;
ros::Time last_ts_get_depth_image = ros::Time(1.0), ts_get_depth_image;

bool ready_to_compute_velocity_y = false;
bool already_prejected_lines = false;

// publish the virtual line's properties: rho and theta
vision_lines::line virtual_line_msg;

bool already_received_odom = false;

void vehicle_odom_callback(nav_msgs::Odometry odom) {
  vehicle_position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
  vehicle_pose << odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, 
                  // in the simulation, the pitch angle is incorrect
                  odom.pose.pose.orientation.y, odom.pose.pose.orientation.z;
  pitch_velocity = odom.twist.twist.angular.y;
  already_received_odom = true;  
}

void get_the_depth_to_the_line(Eigen::Vector2i virtual_line_left_pixel,
  Eigen::Vector2i virtual_line_right_pixel,
  int image_width,
  int image_height,
  int image_row_step,
  uchar *pointer,
  Eigen::Matrix<float, 3, 3> K_matrix,
  float &pixel_distance_to_camera_center,
  float &depth_to_the_line) {
  // get the distance to the detected line
  // first get the pixel point in the middle of the image
  float A = virtual_line_right_pixel.y() - virtual_line_left_pixel.y();
  float B = virtual_line_right_pixel.x() - virtual_line_left_pixel.x();
  float C = virtual_line_left_pixel.x() * virtual_line_right_pixel.y() 
            - virtual_line_right_pixel.x() *  virtual_line_left_pixel.y();
  pixel_distance_to_camera_center = (virtual_line_right_pixel.y() + virtual_line_left_pixel.y()) / 2.0 - image_height/2;
  if (abs(pixel_distance_to_camera_center) > (image_height / 2)) {
    ROS_ERROR_THROTTLE(1, "Line is no longer in vision. line position: %f", pixel_distance_to_camera_center);
    return;
  }

  depth_to_the_line = *(float *)(pointer + image_row_step * int(K_matrix(1, 2) - pixel_distance_to_camera_center) + image_row_step / 2);
  // ROS_INFO_THROTTLE(1, "Get the distance to the line: %f.", depth_to_the_line);
}

/**
 * @brief  This function is used to calculate the velocity of the vehicle in z direction
 * @note   
 * @param  depth_image: 
 * @retval None
 */
void depth_image_callback(sensor_msgs::Image depth_image) {
  if (!already_received_odom || !already_prejected_lines) {
    return;
  }

  // get the distance to the center of the camera
  std::vector<uchar>::iterator it = depth_image.data.begin();
  uchar *pointer = (uchar *)&(*it);
  // distance_to_the_camera_center = *(float *)(pointer + depth_image.step * depth_image.height / 2 + depth_image.step / 2);
  // ROS_INFO_THROTTLE(1, "Get the distance to the camera center pixel: %f.", distance_to_the_camera_center);

  get_the_depth_to_the_line(virtual_line_left_pixel, virtual_line_right_pixel, 
    depth_image.width, depth_image.height, depth_image.step, pointer, K_matrix, pixel_distance_to_the_camera_center, depth_to_the_line);

  // Get the pitch angle
  // Eigen::Vector3f rpy;
  // SO3Utils::quaternion_2_RPY(vehicle_pose, rpy);
  // pitch_radiant = rpy.y();

  // initialize all the required variables
  if (!ready_to_compute_velocity_y) {
    last_ts_get_depth_image = depth_image.header.stamp;
    last_pitch_radiant = pitch_radiant;
    last_depth_to_the_line = depth_to_the_line;
    last_pixel_distance_to_the_camera_center = pixel_distance_to_the_camera_center;
    ready_to_compute_velocity_y = true;
    return;
  }

  // depending on the velocity of the line pixels motion, we can directly estimate the velocity of the vehicle

  // compute the required velocity
  ts_get_depth_image = depth_image.header.stamp;
  float dt = ts_get_depth_image.toSec() - last_ts_get_depth_image.toSec();
  // float pitch_velocity = (pitch_radiant - last_pitch_radiant) / dt;
  float depth_velocity = (depth_to_the_line - last_depth_to_the_line) / dt;
  float pixel_velocity = (pixel_distance_to_the_camera_center - last_pixel_distance_to_the_camera_center) / dt;

  float fy = K_matrix(1, 1);
  float position_y = pixel_distance_to_the_camera_center / fy * depth_to_the_line;
  float velocity = pixel_velocity * depth_to_the_line / fy - depth_velocity / depth_to_the_line * position_y;

  velocity = -velocity + (-pitch_velocity) * depth_to_the_line;

  // transform velocity from image frame to vehicle frame
  velocity = -velocity;

  // use low-pass filter to get the state velocity
  velocity_filtered = 0.2f * velocity_filtered + 0.8f * velocity;
  virtual_line_msg.Bottom.x = velocity_filtered;

  last_depth_to_the_line = depth_to_the_line;
  // last_pitch_radiant = pitch_radiant;
  last_pixel_distance_to_the_camera_center = pixel_distance_to_the_camera_center;
  last_ts_get_depth_image = depth_image.header.stamp;
}

void rgb_callback(sensor_msgs::Image rgb_image) {
  if (!already_received_odom) {
    return;
  }

  // transform the position and pose into camera frame
  Eigen::Vector3f vehicle_position_transformed;
  Eigen::Vector4f transfrom_quaternion, vehicle_pose_transformed;
  transfrom_quaternion << 0.5, -0.5, 0.5, -0.5;

  SO3Utils::quaternion_transform(vehicle_position, transfrom_quaternion, vehicle_position_transformed);
  SO3Utils::quaternion_transform(vehicle_pose, transfrom_quaternion, vehicle_pose_transformed);

  cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(rgb_image, rgb_image.encoding);
  ImageUtils::virtual_line_projection(
    ptr->image, vehicle_position_transformed, vehicle_pose_transformed,
    K_matrix, 
    virtual_line_start_point, 
    virtual_line_end_point,
    virtual_line_left_pixel, virtual_line_right_pixel);
  
  float theta = atan2(virtual_line_right_pixel.y() - virtual_line_left_pixel.y(),
    virtual_line_right_pixel.x() - virtual_line_left_pixel.x());
  
  float A = virtual_line_right_pixel.y() - virtual_line_left_pixel.y();
  float B = virtual_line_right_pixel.x() - virtual_line_left_pixel.x();
  float C = virtual_line_left_pixel.x() * virtual_line_right_pixel.y() 
            - virtual_line_right_pixel.x() *  virtual_line_left_pixel.y();

  // publish the velocity of the property rho
  // first, detect the line
  rho = (A * rgb_image.width / 2 + B * rgb_image.height / 2  + C) / sqrt(A*A + B*B) 
                              / rgb_image.height * 2;

  // here we assume our vehicle body is level
  virtual_line_msg.header.stamp = ros::Time::now();
  virtual_line_msg.State_bottom.data = 1;  
  virtual_line_msg.Bottom.y = rho;
  virtual_line_msg.Bottom.theta = theta;
  if (last_rho == -10) {
    virtual_line_msg.Bottom.x = 0;
  } else {
    // virtual_line_msg.Bottom.x = (last_rho - rho);
  }
  last_rho = rho;

  std::string text = "rho: " + std::to_string(virtual_line_msg.Bottom.y) +", theta: " + std::to_string(theta);
  cv::putText(ptr->image, text, cv::Point(50, 50), 0, 1.0, cv::Scalar(255, 100, 30), 2);

  virtual_line_measure_pub.publish(virtual_line_msg);

  std_msgs::Header header;
  sensor_msgs::Image image_msg;

  cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3, ptr->image);
  img_bridge.toImageMsg(image_msg);

  rgb_virtual_image_pub.publish(image_msg);

  already_prejected_lines = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_line_projection");

  ros::NodeHandle n;

  std::string camera_depth_info_topic_name = argv[1];
  std::string camera_rgb_topic_name = argv[2];
  std::string vehicle_position_topic_name = argv[3];
  std::string camera_depth_image_topic_name = argv[4];

  // Get the camera info from the topic
  camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_depth_info_topic_name.c_str());
  K_matrix << camera_info.K[0], camera_info.K[1], camera_info.K[2],
              camera_info.K[3], camera_info.K[4], camera_info.K[5], 
              camera_info.K[6], camera_info.K[7], camera_info.K[8];

  std::cout << K_matrix << std::endl;

  // specify the virtual line position in camera frame
  virtual_line_start_point << -4, -0.85 + 0.14, 9.4;
  virtual_line_end_point << 4, -0.85 + 0.14, 9.4;

  rgb_image_sub = n.subscribe(camera_rgb_topic_name, 1, rgb_callback);
  vehicle_position_sub = n.subscribe(vehicle_position_topic_name, 1, vehicle_odom_callback);
  depth_image_sub = n.subscribe(camera_depth_image_topic_name, 1, depth_image_callback);

  rgb_virtual_image_pub = n.advertise<sensor_msgs::Image>("virtual_camera_image/raw", 1);
  virtual_line_measure_pub = n.advertise<vision_lines::line>("virtual_line_measure", 1);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
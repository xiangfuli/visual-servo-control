#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <Eigen/Core>
#include "stdint.h"
#include <math.h>

#include "visual_servo_control/line.h"
#include "visual_servo_control/VelocityEstimation.h"


ros::Publisher velocity_estimation_pub;
ros::Subscriber visual_measure_sub, vehicle_orientation_sub, depth_image_sub;
ros::Time last_ts_get_visual_measure = ros::Time(1.0);
float pitch_angular_velocity;
float last_rho=0, rho=0, rho_velocity;
bool rho_velocity_computed=false;

ros::Time last_ts_get_depth_image = ros::Time(1.0);
float last_depth_to_the_line = -1;
Eigen::Matrix<float, 3, 3> K_matrix;

float velocity_filtered;

void imu_data_callback(sensor_msgs::Imu imu) {
	pitch_angular_velocity = imu.angular_velocity.y;
}

void get_the_depth_to_the_line(int pixel_distance_to_camera_center, sensor_msgs::Image depth_image, float &depth_to_the_line) {
  std::vector<u_char>::iterator it = depth_image.data.begin();
  u_char *pointer = (u_char *)&(*it);

  int image_row_step = depth_image.step;
  depth_to_the_line = *(float *)(pointer + image_row_step 
    * int(K_matrix(1, 2) + pixel_distance_to_camera_center) + image_row_step / 2);
}

void depth_image_callback(sensor_msgs::Image depth_image) {
  visual_servo_control::VelocityEstimation ve;
  ve.header.stamp = ros::Time::now();

  float depth_to_the_line;
  int pixel_distance_to_camera_center = rho * depth_image.height / 2;
  get_the_depth_to_the_line(pixel_distance_to_camera_center, depth_image, depth_to_the_line);

  if (last_depth_to_the_line == -1 || !rho_velocity_computed) {
    last_depth_to_the_line = depth_to_the_line;
    last_ts_get_depth_image = depth_image.header.stamp;
    return;
  } else {
    float dt = (depth_image.header.stamp - last_ts_get_depth_image).toSec();
    float depth_velocity = (depth_to_the_line - last_depth_to_the_line) / dt;

    // compute the z axis velocity estimation
    float fy = K_matrix(1, 1);
    float position_y = pixel_distance_to_camera_center / fy * depth_to_the_line;
    float velocity = rho_velocity * depth_image.height / 2 * depth_to_the_line / fy 
                      - depth_velocity / depth_to_the_line * position_y;

    velocity = -velocity + (-pitch_angular_velocity) * depth_to_the_line;

    // transform velocity from image frame to vehicle frame
    velocity = -velocity;

    // use low-pass filter to get the state velocity
    velocity_filtered = 0.2f * velocity_filtered + 0.8f * velocity;
    ve.z_velocity = velocity_filtered;

    velocity_estimation_pub.publish(ve);
  }

  last_ts_get_depth_image = depth_image.header.stamp;
  last_depth_to_the_line = depth_to_the_line;
}

void visual_measure_callback(visual_servo_control::line visual_measure) {
  if (visual_measure.State_bottom.data == 1) {
    float dt = (visual_measure.header.stamp - last_ts_get_visual_measure).toSec();

    // satisfy with positive y axis direction
    rho = -visual_measure.Bottom.y;
    rho_velocity = (rho - last_rho) / dt;
    rho_velocity_computed = true;
  } 

  last_ts_get_visual_measure = visual_measure.header.stamp;
  last_rho = rho;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_estimation");

  ros::NodeHandle n, n_param("~/velocity_estimation");

	std::string visual_measure_topic_name = n_param.param<std::string>("visual_measure_topic_name", "");
	std::string vehicle_imu_topic_name = n_param.param<std::string>("vehicle_imu_topic_name", "");
  std::string depth_topic_name = n_param.param<std::string>("depth_topic_name", "");
  std::string camera_info_topic_name = n_param.param<std::string>("camera_info_topic_name", "");

  sensor_msgs::CameraInfo camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_name.c_str());
  K_matrix << camera_info.K[0], camera_info.K[1], camera_info.K[2],
              camera_info.K[3], camera_info.K[4], camera_info.K[5], 
              camera_info.K[6], camera_info.K[7], camera_info.K[8];

  ROS_INFO("[velocity_estimation] Get camera K matrix, fx: %f, fy: %f, cx: %f, cy: %f", camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5]);

	vehicle_orientation_sub = n.subscribe(vehicle_imu_topic_name, 1, imu_data_callback);
  visual_measure_sub = n.subscribe(visual_measure_topic_name, 1, visual_measure_callback);
  depth_image_sub = n.subscribe(depth_topic_name, 1, depth_image_callback);

  velocity_estimation_pub = n.advertise<visual_servo_control::VelocityEstimation>("velocity_estimation", 1);

	ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
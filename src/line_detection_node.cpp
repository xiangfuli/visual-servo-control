#include "visual_servo_control/line_detection_node.hpp"

uint16_t region_width = 100, region_height = 100;
ros::Publisher surface_norm_pub, surface_norm_filtered_pub, 
               white_region_threshold_pub, image_with_lines_pub, line_measure_pub;
ros::Subscriber depth_image_sub, rgb_image_sub;
Eigen::Matrix<float, 3, 3> K_matrix;
sensor_msgs::CameraInfo camera_info;
uint8_t white_region_threshold;
bool line_detected = false;

void deprojection(const cv::Mat image, const sensor_msgs::CameraInfo camera_info, std::vector<Eigen::Vector3d> &points) {
  uint16_t height, width;
  height = image.rows;
  width = image.cols;

  float Kx = camera_info.K[0];
  float Ky = camera_info.K[4];

  for (int i=0; i < height; i++) {
    for (int j=0; j < width; j++) {
      float z = image.at<float>(i, j);
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
    norm.x = norm_vector.x();
    norm.y = norm_vector.y();
    norm.z = norm_vector.z();
  } else {
    ROS_INFO_THROTTLE(10, "Surface not close enough to get the norm.");
    norm.detected = false;
    norm.x = 0;
    norm.y = 0;
    norm.z = 0;
  }
}

void depth_callback(sensor_msgs::Image depth_image) {
  visual_servo_control::SurfaceNorm norm;
  norm.header.stamp = ros::Time::now();

  // instead of use the fixed region of image around the camera center
  // we should extract the depth of the white board region and compute the norm vector
  norm_calc(depth_image, norm);

  surface_norm_pub.publish(norm);
}

void hough_property(int point1_x, int point1_y, int point2_x, int point2_y, int image_height, int image_width,
  float &theta, float &rho) {
  theta = atan2(point2_y - point1_y, point2_x - point1_x);
  
  float A = point2_y - point1_y;
  float B = point2_x - point1_x;
  float C = point1_x * point2_y - point2_x * point1_y;

  // publish the velocity of the property rho
  // right now, the coordinate of the camera center is (0, 0)
  rho = (A * 0 + B * 0  + C) / sqrt(A*A + B*B) / (image_height / 2);
}


/**
 * @brief filter the lines detected by the hough transfrom, this function will use following constraints to 
 *        treat the lines as the bottom line or the left/right boarder lines.
 *        
 *        Before further processing the lines, all lines coordinate will be transformed to image plane frame.
 *        
 *        For the bottom lines:
 *        1. All the lines whose abs(theta) is less than 0.34(around 20 degrees) will be regarded as the bottom line candidate.
 *        2. Then among these lines, the line cloest to the camera center will be thought as the bottom line.
 * 
 *        For the left/right boarder lines:
 *        1. First the absolute value line's x pixel coordinate must be less than cx(camera center loc in K matrix)
 *        2. After matching the first constraint, the vertical line closest to the camera center will be treated as the 
 *          left/right boarder line. 
 * 
 * @param lines 
 * @param line_result return the detected vertical and horizontal lines by the 
 */
void filter_out_lines(std::vector<cv::Vec4i> lines, Eigen::Matrix<float, 3, 3> K_matrix, 
  visual_servo_control::line &line_result, int &bottom_line_index) {

  float angle_threshold = 0.34;
  
  float cx = K_matrix(0, 2);
  float cy = K_matrix(1, 2);
  float height = cy * 2;
  float width = cx * 2;

  // transform the pixel coordinates to location in image frame
  std::vector<Eigen::Vector3f> horizon_lines, vertical_lines;
  for( size_t i = 0; i < lines.size(); i++ ) {
    cv::Vec4i l = lines[i];
    float theta, rho;
    Eigen::Vector2f houth_property;
    hough_property(l[0] - cx, l[1] - cy, l[2] - cx, l[3] - cy, height, width, theta, rho);

    if (abs(theta) < angle_threshold) {
      horizon_lines.push_back(Eigen::Vector3f(theta, rho, i));
    }

    // for the vertical lines, we have to ignore the lines detected bu the edge of the manipulator
    if (abs(theta - M_PI) < angle_threshold ||  abs(theta + M_PI) > angle_threshold) {
      vertical_lines.push_back(Eigen::Vector3f(theta, rho, i));
    }
  }

  // retrieve the lines closet to the camera center
  float cloest_rho = -100.0f, filtered_theta;
  for( size_t i = 0; i < horizon_lines.size(); i++ ) {
    if (abs(cloest_rho) > abs(horizon_lines[i].y())) {
      cloest_rho = horizon_lines[i].y();
      filtered_theta = horizon_lines[i].x();
      bottom_line_index = horizon_lines[i].z();
    }
  }

  if (cloest_rho > -10) {
    line_result.State_bottom.data = 1;  
    line_result.Bottom.y = cloest_rho;
    line_result.Bottom.theta = filtered_theta;
    if (!line_detected) {
      line_detected = true;
      ROS_INFO("Just detected the bottom line.");
    }
  } else {
    line_result.State_bottom.data = 0;  
    line_result.Bottom.y = -100.0f;
    line_result.Bottom.theta = -100.0f;
    ROS_WARN_THROTTLE(0.5, "Fail to detect the lines we want, system will return back to position control mode.");
    line_detected = false;
  }
}

void rgb_callback(sensor_msgs::Image rgb_image) {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  sensor_msgs::Image image_msg, image_with_lines_msg;
  cv::Mat grayscale_image, white_region_threshold_image, edge_image;
  
	// convert sensor msgs to opencv Mat object
  cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(rgb_image, rgb_image.encoding);
  cv::cvtColor(ptr->image, grayscale_image, cv::COLOR_BGR2GRAY);

	// extract the white region in the image
  ImageUtils::extract_white_region(grayscale_image, white_region_threshold_image, white_region_threshold);
  // In-place filtering is supported
  cv::blur(white_region_threshold_image, white_region_threshold_image, cv::Size(5, 5));
  
	// convert sensor msgs to opencv Mat object
	cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, 
    white_region_threshold_image);
  img_bridge.toImageMsg(image_msg);
  
	white_region_threshold_pub.publish(image_msg);

	// detect the lines from the threshold image
	// use opencv hough transform to get the line pixels
  cv::Canny(white_region_threshold_image, edge_image, 50, 200, 3);

	std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edge_image, lines, 10, CV_PI/180, 100, 100, 100);

  visual_servo_control::line line_result;
  line_result.header.stamp = ros::Time::now();
  int bottom_line_index;
  filter_out_lines(lines, K_matrix, line_result, bottom_line_index);
  line_measure_pub.publish(line_result);

  if (line_result.State_bottom.data == 1) {
    cv::Vec4i l = lines[bottom_line_index];
    cv::line(ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(128), 3, cv::LINE_AA);
  }

  std::string text = "rho: " + std::to_string(line_result.Bottom.y) +", theta: " + std::to_string(line_result.Bottom.theta);
  cv::putText(ptr->image, text, cv::Point(50, 50), 0, 1.0, cv::Scalar(255, 100, 30), 2);

  cv_bridge::CvImage img_with_lines_bridge = cv_bridge::CvImage(header, rgb_image.encoding, ptr->image);
  img_with_lines_bridge.toImageMsg(image_with_lines_msg);
  image_with_lines_pub.publish(image_with_lines_msg);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "line_detection_node");

  ros::NodeHandle n, n_param("~/line_detection_node");

  std::string camera_depth_info_topic_name = n_param.param<std::string>("camera_info_topic_name", "");
  std::string camera_depth_topic_name = n_param.param<std::string>("depth_topic_name", "");
  std::string camera_rgb_topic_name = n_param.param<std::string>("rgb_topic_name", "");
  white_region_threshold = n_param.param("white_region_threshold", 180);
  
  // Get the camera info from the topic
  camera_info = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_depth_info_topic_name.c_str());
  K_matrix << camera_info.K[0], camera_info.K[1], camera_info.K[2],
              camera_info.K[3], camera_info.K[4], camera_info.K[5], 
              camera_info.K[6], camera_info.K[7], camera_info.K[8];

  ROS_INFO("Get camera K matrix, fx: %f, fy: %f, cx: %f, cy: %f", camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5]);

  depth_image_sub = n.subscribe(camera_depth_topic_name, 1, depth_callback);
  rgb_image_sub = n.subscribe(camera_rgb_topic_name, 1, rgb_callback);

  white_region_threshold_pub = n.advertise<sensor_msgs::Image>("white_region_threshold/raw", 1);
  image_with_lines_pub = n.advertise<sensor_msgs::Image>("image_with_lines/raw", 1);
  surface_norm_pub = n.advertise<visual_servo_control::SurfaceNorm>("surface_norm/raw", 1);
  surface_norm_filtered_pub = n.advertise<visual_servo_control::SurfaceNorm>("surface_norm/filtered", 1);
  line_measure_pub = n.advertise<visual_servo_control::line>("lines", 1);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
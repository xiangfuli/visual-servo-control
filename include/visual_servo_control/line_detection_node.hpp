#ifndef LINE_DETECTION_NODE_H
#define LINE_DETECTION_NODE_H

/**
 * @brief  This class is used to extract the norm vector of the plane and detect the region of the white board.
 * 
 * 				 The white board region is extracted with color imformation. There are several situations would occur with
 * 				 different aerial vehicle pose.
 * 				 1. at the right-down corner
 *         2. between the left and right board
 *         3. at the left-down corner
 * 				 So we have to define the state transition between different situations to control the vehicle's behavior.
 * 				 So in this node, we can deliver the state/situation information and let control decide what to do.
 * 				
 * 				 The norm vector is computed from the depth information given from the RGBD camera. Each pixel of the
 * 				 plane detected will be deprojected to get its 3D position. Then the norm vector will be retrived using
 * 				 min linear least square algorithm.
 * 
 * @note   
 * @retval None
 */

#include <iostream>
#include <vector>
#include <cstdint>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "visual_servo_control/SurfaceNorm.h"
#include "visual_servo_control/line.h"
#include "vision_process/planes/plane_recognizer.hpp"
#include "vision_process/utils/image_utils.hpp"

enum LineDetectionState {
	BOTTOM_LINE_NOT_DETECTED,
	RIGHT_DOWN,
	BETWEEN_BORDERS,
	LEFT_DOWN
};

#endif /* LINE_DETECTION_NODE_H */

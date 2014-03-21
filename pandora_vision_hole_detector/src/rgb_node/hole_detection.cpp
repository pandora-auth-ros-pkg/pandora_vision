/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Despoina Paschalidou
*********************************************************************/

#include "rgb_node/hole_detection.h"

namespace pandora_vision
{
  /**
    @brief Constructor
  **/
  HoleDetection::HoleDetection(): _nh(), holeNowON(false)
  {
    //!< Get general parameters for image processing
    getGeneralParams();

    //!< Convert field of view from degrees to rads
    hfov = hfov * CV_PI / 180;
    vfov = vfov * CV_PI / 180;

    ratioX = hfov / frameWidth;
    ratioY = vfov / frameHeight;

    //!< Memory will allocated in the imageCallback
    _holeFrame= cv::Mat::zeros(frameWidth, frameHeight, CV_8UC3);

    //!< Subscribe to the RGB image published by the
    //!< rgb_depth_synchronizer node
    _frameSubscriber = _nh.subscribe(
      "/synchronized/camera/rgb/image_raw", 1,
      &HoleDetection::imageCallback, this);

    //!< Advertise the candidate holes found by the depth node
    rgbCandidateHolesPublisher_ = _nh.advertise
      <vision_communications::RgbCandidateHolesVectorMsg>(
      "/synchronized/camera/rgb/candidate_holes", 1000);

    //!< The dynamic reconfigure (RGB) parameter's callback
    server.setCallback(boost::bind(&HoleDetection::parametersCallback,
        this, _1, _2));

    ROS_INFO("[rgb_node] : Created Hole Detection instance");
  }

   /**
    @brief Destructor
  */
  HoleDetection::~HoleDetection()
  {
    ROS_DEBUG("[rgb_node] : Destroying Hole Detection instance");
  }

  /**
    @brief Get parameters referring to the view and
    frame characteristics
    @return void
  **/
  void HoleDetection::getGeneralParams()
  {
    packagePath = ros::package::getPath("pandora_vision_hole_detector");

    //!< Get the camera to be used by hole node;
    if (_nh.hasParam("camera_name"))
    {
      _nh.getParam("camera_name", cameraName);
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    }
    else
    {
      cameraName = "camera";
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    }

    //!< Get the Height parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_height"))
    {
      _nh.getParam("/" + cameraName + "/image_height", frameHeight);
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    else
    {
      frameHeight = _holeFrame.rows;
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }

    //!< Get the Width parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_width"))
    {
      _nh.getParam("/" + cameraName + "/image_width", frameWidth);
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    else
    {
      frameWidth = _holeFrame.cols;
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }

    //!< Get the images's topic;
    if (_nh.hasParam("/" + cameraName + "/topic_name"))
    {
      _nh.getParam("/" + cameraName + "/topic_name", imageTopic);
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
    }
    else
    {
      imageTopic = "/camera/rgb/image_color";
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
    }

    //!< Get the images's frame_id;
    if (_nh.hasParam("/" + cameraName + "/camera_frame_id"))
    {
      _nh.getParam("/" + cameraName + "/camera_frame_id", cameraFrameId);
      ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
    }
    else
    {
      cameraFrameId = "/camera";
      ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
    }
  }

  /**
    @brief Function called when new ROS message appears, for camera
    @param msg [const sensor_msgs::ImageConstPtr&] The message
    @return void
  */
  void HoleDetection::imageCallback(const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    _holeFrame = in_msg->image.clone();
    _holeFrameTimestamp = msg.header.stamp;

    if (_holeFrame.empty() )
    {
      ROS_ERROR("[rgb_node] : No more Frames");
      return;
    }


    HolesConveyor conveyor = _holeDetector.findHoles(_holeFrame);

    vision_communications::RgbCandidateHolesVectorMsg rgbCandidateHolesMsg;

    createCandidateHolesMessage(conveyor, msg, &rgbCandidateHolesMsg,
      sensor_msgs::image_encodings::TYPE_32FC1);

    rgbCandidateHolesPublisher_.publish(rgbCandidateHolesMsg);
  }

  void HoleDetection::createCandidateHolesMessage(
    const HolesConveyor& conveyor,
    const sensor_msgs::Image& rgbImage,
    vision_communications::RgbCandidateHolesVectorMsg* rgbCandidateHolesMsg,
    const std::string& encoding)
  {
    //!< Fill the vision_communications::RgbCandidateHolesVectorMsg's
    //!< candidateHoles vector
    for (unsigned int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      vision_communications::CandidateHoleMsg holeMsg;

      //!< Push back the keypoint
      holeMsg.keypointX = conveyor.keyPoints[i].pt.x;
      holeMsg.keypointY = conveyor.keyPoints[i].pt.y;

      //!< Push back the bounding rectangle's vertices
      for (int v = 0; v < conveyor.rectangles[i].size(); v++)
      {
        holeMsg.verticesX.push_back(conveyor.rectangles[i][v].x);
        holeMsg.verticesY.push_back(conveyor.rectangles[i][v].y);
      }

      //!< Push back the blob's outline points
      for (int o = 0; o < conveyor.outlines[i].size(); o++)
      {
        holeMsg.outlineX.push_back(conveyor.outlines[i][o].x);
        holeMsg.outlineY.push_back(conveyor.outlines[i][o].y);
      }

      //!< Push back one hole to the holes vector message
      rgbCandidateHolesMsg->candidateHoles.push_back(holeMsg);
    }

    //!< Fill vision_communications::DepthCandidateHolesVectorMsg's
    //!< sensor_msgs/Image interpolatedDepthImage
    rgbCandidateHolesMsg->rgbImage = rgbImage;
  }



  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_hole_detector::rgb_cfgConfig&]
    @param[in] level [const uint32_t] The level (?)
    @return void
   **/
  void HoleDetection::parametersCallback(
    const pandora_vision_hole_detector::rgb_cfgConfig& config,
    const uint32_t& level)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Parameters callback called");
    #endif

    Parameters::kanny_ratio = config.kanny_ratio;
    Parameters::kanny_kernel_size = config.kanny_kernel_size;
    Parameters::kanny_low_threshold = config.kanny_low_threshold;
    Parameters::kanny_blur_noise_kernel_size =
      config.kanny_blur_noise_kernel_size;
    Parameters::contrast_enhance_beta = config.contrast_enhance_beta;
    Parameters::contrast_enhance_alpha = config.contrast_enhance_alpha;
    Parameters::threshold_lower_value = config.threshold_lower_value;
    Parameters::adaptive_max_value = config.adaptive_max_value;
    Parameters::adaptive_method = config.adaptive_method;
    Parameters::adaptive_block_size = config.adaptive_block_size;
    Parameters::adaptive_c_value = config.adaptive_c_value;
    Parameters::blob_min_threshold = config.blob_min_threshold;
    Parameters::blob_max_threshold = config.blob_max_threshold;
    Parameters::blob_threshold_step = config.blob_threshold_step;
    Parameters::blob_min_area = config.blob_min_area;
    Parameters::blob_max_area = config.blob_max_area;
    Parameters::blob_min_convexity = config.blob_min_convexity;
    Parameters::blob_max_convexity = config.blob_max_convexity;
    Parameters::blob_min_inertia_ratio = config.blob_min_inertia_ratio;
    Parameters::blob_max_circularity = config.blob_max_circularity;
    Parameters::blob_min_circularity = config.blob_min_circularity;
    Parameters::blob_filter_by_color = config.blob_filter_by_color;
    Parameters::blob_filter_by_circularity =
      config.blob_filter_by_circularity;
    Parameters::bounding_box_min_area_threshold =
      config.bounding_box_min_area_threshold;
    Parameters::bounding_box_detection_method =
      config.bounding_box_detection_method;
    Parameters::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;
    Parameters::AB_to_MO_ratio = config.AB_to_MO_ratio;
    Parameters::interpolation_method = config.interpolation_method;
    Parameters::run_checker_depth_diff = config.run_checker_depth_diff;
    Parameters::run_checker_outline_of_rectangle =
      config.run_checker_outline_of_rectangle;
    Parameters::run_checker_depth_area = config.run_checker_depth_area;
    Parameters::run_checker_brushfire_outline_to_rectangle =
      config.run_checker_brushfire_outline_to_rectangle;
    Parameters::rectangle_inflation_size = config.rectangle_inflation_size;
    Parameters::depth_difference = config.depth_difference;
    Parameters::segmentation_method = config.segmentation_method;
    Parameters::max_iterations = config.max_iterations;
    Parameters::num_points_to_exclude = config.num_points_to_exclude;
    Parameters::point_to_plane_distance_threshold =
      config.point_to_plane_distance_threshold;
    Parameters::scale_method = config.scale_method;
    Parameters::debug_show_find_holes = config.debug_show_find_holes;
    Parameters::debug_show_find_holes_size =
      config.debug_show_find_holes_size;
    Parameters::debug_time_find_holes = config.debug_time_find_holes;
    Parameters::debug_show_denoise_edges = config.debug_show_denoise_edges;
    Parameters::debug_show_denoise_edges_size =
      config.debug_show_denoise_edges_size;
    Parameters::debug_show_connect_pairs = config.debug_show_connect_pairs;
    Parameters::debug_show_connect_pairs_size =
      config.debug_show_connect_pairs_size;

    Parameters::debug_show_get_shapes_clear_border  =
      config.debug_show_get_shapes_clear_border;
    Parameters::debug_show_get_shapes_clear_border_size =
      config.debug_show_get_shapes_clear_border_size;

    Parameters::debug_show_check_holes = config.debug_show_check_holes;
    Parameters::debug_show_check_holes_size =
      config.debug_show_check_holes_size;

    Parameters::minimum_curve_points = config.minimum_curve_points;
  }

}// namespace pandora_vision

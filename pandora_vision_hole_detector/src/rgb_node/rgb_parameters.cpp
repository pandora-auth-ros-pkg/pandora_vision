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
* Authors: Alexandros Filotheou, Manos Tsardoulias
*********************************************************************/

#include "rgb_node/rgb_parameters.h"

namespace pandora_vision
{
  //!< Kanny parameters
  int RgbParameters::kanny_ratio = 3;
  int RgbParameters::kanny_kernel_size = 3;
  int RgbParameters::kanny_low_threshold = 50;
  int RgbParameters::kanny_blur_noise_kernel_size = 3;

  float RgbParameters::contrast_enhance_alpha = 2;
  float RgbParameters::contrast_enhance_beta = 2;


  //!< Threshold parameters
  int RgbParameters::threshold_lower_value = 10;
  int RgbParameters::adaptive_max_value = 255;
  int RgbParameters::adaptive_method = 0;
  int RgbParameters::adaptive_block_size = 7;
  int RgbParameters::adaptive_c_value = 7;

  //!< Blob detection parameters
  int RgbParameters::blob_min_threshold = 0;
  int RgbParameters::blob_max_threshold = 1255;
  int RgbParameters::blob_threshold_step = 5;
  int RgbParameters::blob_min_area = 550;
  int RgbParameters::blob_max_area = 300000;
  double RgbParameters::blob_min_convexity = 0;
  double RgbParameters::blob_max_convexity = 100;
  double RgbParameters::blob_min_inertia_ratio = 0;
  double RgbParameters::blob_max_circularity = 1.0;
  double RgbParameters::blob_min_circularity = 0.3;
  bool RgbParameters::blob_filter_by_color = 0;
  bool RgbParameters::blob_filter_by_circularity = 1;

  //!< Bounding boxes parameters
  int RgbParameters::bounding_box_min_area_threshold = 550;
  int RgbParameters::bounding_box_detection_method = 0;
  int RgbParameters::raycast_keypoint_partitions = 8;

  //<! Loose ends connection parameters
  int RgbParameters::AB_to_MO_ratio = 2;


  //!< Interpolation parameters
  int RgbParameters::interpolation_method = 0;

  //!< Hole checkers
  int RgbParameters::run_checker_depth_diff = 1;
  int RgbParameters::run_checker_depth_area = 3;
  int RgbParameters::run_checker_brushfire_outline_to_rectangle = 4;
  int RgbParameters::run_checker_outline_of_rectangle = 2;
  int RgbParameters::rectangle_inflation_size = 20;
  float RgbParameters::depth_difference = 0.4;

  //!< Plane detection
  int RgbParameters::segmentation_method = 0;
  int RgbParameters::max_iterations = 1000;
  double RgbParameters::num_points_to_exclude = 0.1;
  double RgbParameters::point_to_plane_distance_threshold = 0.01;

  //!< Method to scale the CV_32FC1 image to CV_8UC1
  int RgbParameters::scale_method = 0;

  //!< Debug
  bool RgbParameters::debug_show_find_holes = false;
  int RgbParameters::debug_show_find_holes_size = 1000;
  bool RgbParameters::debug_time_find_holes = false;

  bool RgbParameters::debug_show_denoise_edges = false;
  int RgbParameters::debug_show_denoise_edges_size = 900;

  bool RgbParameters::debug_show_connect_pairs = false;
  int RgbParameters::debug_show_connect_pairs_size = 1200;
  bool RgbParameters::debug_print_connect_pairs = false;

  bool RgbParameters::debug_show_get_shapes_clear_border = false;
  int RgbParameters::debug_show_get_shapes_clear_border_size  = 1200;

  bool RgbParameters::debug_show_check_holes = false;
  int RgbParameters::debug_show_check_holes_size = 1200;

  int RgbParameters::minimum_curve_points = 1200;
  //!<----------------------------------------------------------------------!>//



  RgbParameters::RgbParameters(void)
  {
    ros::NodeHandle _nodeHandle;

    server.setCallback(boost::bind(&RgbParameters::callback, this,_1, _2));
  }



  /**
    @brief The function called when a parateter is changed
    @param[in] config [const pandora_vision_hole_detector::depth_cfgConfig&]
    @param[in] level [const uint32_t] The level (?)
    @return void
   **/
  void RgbParameters::callback(
    const pandora_vision_hole_detector::rgb_cfgConfig& config,
    const uint32_t& level)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("RgbParameters callback called");
    #endif

    RgbParameters::kanny_ratio = config.kanny_ratio;
    RgbParameters::kanny_kernel_size = config.kanny_kernel_size;
    RgbParameters::kanny_low_threshold = config.kanny_low_threshold;
    RgbParameters::kanny_blur_noise_kernel_size =
      config.kanny_blur_noise_kernel_size;
    RgbParameters::contrast_enhance_beta = config.contrast_enhance_beta;
    RgbParameters::contrast_enhance_alpha = config.contrast_enhance_alpha;
    RgbParameters::threshold_lower_value = config.threshold_lower_value;
    RgbParameters::adaptive_max_value = config.adaptive_max_value;
    RgbParameters::adaptive_method = config.adaptive_method;
    RgbParameters::adaptive_block_size = config.adaptive_block_size;
    RgbParameters::adaptive_c_value = config.adaptive_c_value;
    RgbParameters::blob_min_threshold = config.blob_min_threshold;
    RgbParameters::blob_max_threshold = config.blob_max_threshold;
    RgbParameters::blob_threshold_step = config.blob_threshold_step;
    RgbParameters::blob_min_area = config.blob_min_area;
    RgbParameters::blob_max_area = config.blob_max_area;
    RgbParameters::blob_min_convexity = config.blob_min_convexity;
    RgbParameters::blob_max_convexity = config.blob_max_convexity;
    RgbParameters::blob_min_inertia_ratio = config.blob_min_inertia_ratio;
    RgbParameters::blob_max_circularity = config.blob_max_circularity;
    RgbParameters::blob_min_circularity = config.blob_min_circularity;
    RgbParameters::blob_filter_by_color = config.blob_filter_by_color;
    RgbParameters::blob_filter_by_circularity =
      config.blob_filter_by_circularity;
    RgbParameters::bounding_box_min_area_threshold =
      config.bounding_box_min_area_threshold;
    RgbParameters::bounding_box_detection_method =
      config.bounding_box_detection_method;
    RgbParameters::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;
    RgbParameters::AB_to_MO_ratio = config.AB_to_MO_ratio;
    RgbParameters::interpolation_method = config.interpolation_method;
    RgbParameters::run_checker_depth_diff = config.run_checker_depth_diff;
    RgbParameters::run_checker_outline_of_rectangle =
      config.run_checker_outline_of_rectangle;
    RgbParameters::run_checker_depth_area = config.run_checker_depth_area;
    RgbParameters::run_checker_brushfire_outline_to_rectangle =
      config.run_checker_brushfire_outline_to_rectangle;
    RgbParameters::rectangle_inflation_size = config.rectangle_inflation_size;
    RgbParameters::depth_difference = config.depth_difference;
    RgbParameters::segmentation_method = config.segmentation_method;
    RgbParameters::max_iterations = config.max_iterations;
    RgbParameters::num_points_to_exclude = config.num_points_to_exclude;
    RgbParameters::point_to_plane_distance_threshold =
      config.point_to_plane_distance_threshold;
    RgbParameters::scale_method = config.scale_method;
    RgbParameters::debug_show_find_holes = config.debug_show_find_holes;
    RgbParameters::debug_show_find_holes_size =
      config.debug_show_find_holes_size;
    RgbParameters::debug_time_find_holes = config.debug_time_find_holes;
    RgbParameters::debug_show_denoise_edges = config.debug_show_denoise_edges;
    RgbParameters::debug_show_denoise_edges_size =
      config.debug_show_denoise_edges_size;
    RgbParameters::debug_show_connect_pairs = config.debug_show_connect_pairs;
    RgbParameters::debug_show_connect_pairs_size =
      config.debug_show_connect_pairs_size;

    RgbParameters::debug_show_get_shapes_clear_border  =
      config.debug_show_get_shapes_clear_border;
    RgbParameters::debug_show_get_shapes_clear_border_size =
      config.debug_show_get_shapes_clear_border_size;

    RgbParameters::debug_show_check_holes = config.debug_show_check_holes;
    RgbParameters::debug_show_check_holes_size =
      config.debug_show_check_holes_size;

    RgbParameters::minimum_curve_points = config.minimum_curve_points;
  }



  /**
    @brief Retrieve the interpolation method
    @return int The interpolation method
   **/
  int RgbParameters::getInterpolationMethod()
  {
    return RgbParameters::interpolation_method;
  }
}

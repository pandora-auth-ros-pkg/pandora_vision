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

#include "hole_fusion_node/hole_fusion_parameters.h"

namespace vision
{
  //!< Kanny parameters
  int HoleFusionParameters::kanny_ratio = 3;
  int HoleFusionParameters::kanny_kernel_size = 3;
  int HoleFusionParameters::kanny_low_threshold = 50;
  int HoleFusionParameters::kanny_blur_noise_kernel_size = 3;

  float HoleFusionParameters::contrast_enhance_alpha = 2;
  float HoleFusionParameters::contrast_enhance_beta = 2;


  //!< Threshold parameters
  int HoleFusionParameters::threshold_lower_value = 10;
  int HoleFusionParameters::adaptive_max_value = 255;
  int HoleFusionParameters::adaptive_method = 0;
  int HoleFusionParameters::adaptive_block_size = 7;
  int HoleFusionParameters::adaptive_c_value = 7;

  //!< Blob detection parameters
  int HoleFusionParameters::blob_min_threshold = 0;
  int HoleFusionParameters::blob_max_threshold = 1255;
  int HoleFusionParameters::blob_threshold_step = 5;
  int HoleFusionParameters::blob_min_area = 550;
  int HoleFusionParameters::blob_max_area = 300000;
  double HoleFusionParameters::blob_min_convexity = 0;
  double HoleFusionParameters::blob_max_convexity = 100;
  double HoleFusionParameters::blob_min_inertia_ratio = 0;
  double HoleFusionParameters::blob_max_circularity = 1.0;
  double HoleFusionParameters::blob_min_circularity = 0.3;
  bool HoleFusionParameters::blob_filter_by_color = 0;
  bool HoleFusionParameters::blob_filter_by_circularity = 1;

  //!< Bounding boxes parameters
  int HoleFusionParameters::bounding_box_min_area_threshold = 550;
  int HoleFusionParameters::bounding_box_detection_method = 0;
  int HoleFusionParameters::raycast_keypoint_partitions = 8;

  //<! Loose ends connection parameters
  int HoleFusionParameters::AB_to_MO_ratio = 2;


  //!< Interpolation parameters
  int HoleFusionParameters::interpolation_method = 0;

  //!< Hole checkers
  int HoleFusionParameters::run_checker_depth_diff = 1;
  int HoleFusionParameters::run_checker_depth_area = 3;
  int HoleFusionParameters::run_checker_brushfire_outline_to_rectangle = 4;
  int HoleFusionParameters::run_checker_outline_of_rectangle = 2;
  int HoleFusionParameters::rectangle_inflation_size = 20;
  float HoleFusionParameters::depth_difference = 0.4;

  //!< Plane detection
  int HoleFusionParameters::segmentation_method = 0;
  int HoleFusionParameters::max_iterations = 1000;
  double HoleFusionParameters::num_points_to_exclude = 0.1;
  double HoleFusionParameters::point_to_plane_distance_threshold = 0.01;

  //!< Method to scale the CV_32FC1 image to CV_8UC1
  int HoleFusionParameters::scale_method = 0;

  //!< Debug
  bool HoleFusionParameters::debug_show_find_holes = false;
  int HoleFusionParameters::debug_show_find_holes_size = 1000;
  bool HoleFusionParameters::debug_time_find_holes = false;

  bool HoleFusionParameters::debug_show_denoise_edges = false;
  int HoleFusionParameters::debug_show_denoise_edges_size = 900;

  bool HoleFusionParameters::debug_show_connect_pairs = false;
  int HoleFusionParameters::debug_show_connect_pairs_size = 1200;
  bool HoleFusionParameters::debug_print_connect_pairs = false;

  bool HoleFusionParameters::debug_show_get_shapes_clear_border = false;
  int HoleFusionParameters::debug_show_get_shapes_clear_border_size  = 1200;

  bool HoleFusionParameters::debug_show_check_holes = false;
  int HoleFusionParameters::debug_show_check_holes_size = 1200;

  int HoleFusionParameters::minimum_curve_points = 1200;
  //!<----------------------------------------------------------------------!>//

  HoleFusionParameters::HoleFusionParameters(void)
  {
    ros::NodeHandle _nodeHandle;

    server.setCallback(boost::bind(&HoleFusionParameters::callback,
        this,_1, _2));
  }

  void HoleFusionParameters::callback(pandora_vision_hole_detector::
    hole_fusion_cfgConfig &config, uint32_t level)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("HoleFusionParameters callback called");
    #endif
    HoleFusionParameters::kanny_ratio =
      config.kanny_ratio;
    HoleFusionParameters::kanny_kernel_size =
      config.kanny_kernel_size;
    HoleFusionParameters::kanny_low_threshold =
      config.kanny_low_threshold;
    HoleFusionParameters::kanny_blur_noise_kernel_size =
      config.kanny_blur_noise_kernel_size;
    HoleFusionParameters::contrast_enhance_beta =
      config.contrast_enhance_beta;
    HoleFusionParameters::contrast_enhance_alpha =
      config.contrast_enhance_alpha;
    HoleFusionParameters::threshold_lower_value =
      config.threshold_lower_value;
    HoleFusionParameters::adaptive_max_value =
      config.adaptive_max_value;
    HoleFusionParameters::adaptive_method =
      config.adaptive_method;
    HoleFusionParameters::adaptive_block_size =
      config.adaptive_block_size;
    HoleFusionParameters::adaptive_c_value =
      config.adaptive_c_value;
    HoleFusionParameters::blob_min_threshold =
      config.blob_min_threshold;
    HoleFusionParameters::blob_max_threshold =
      config.blob_max_threshold;
    HoleFusionParameters::blob_threshold_step =
      config.blob_threshold_step;
    HoleFusionParameters::blob_min_area =
      config.blob_min_area;
    HoleFusionParameters::blob_max_area =
      config.blob_max_area;
    HoleFusionParameters::blob_min_convexity =
      config.blob_min_convexity;
    HoleFusionParameters::blob_max_convexity =
      config.blob_max_convexity;
    HoleFusionParameters::blob_min_inertia_ratio =
      config.blob_min_inertia_ratio;
    HoleFusionParameters::blob_max_circularity =
      config.blob_max_circularity;
    HoleFusionParameters::blob_min_circularity =
      config.blob_min_circularity;
    HoleFusionParameters::blob_filter_by_color =
      config.blob_filter_by_color;
    HoleFusionParameters::blob_filter_by_circularity =
      config.blob_filter_by_circularity;
    HoleFusionParameters::bounding_box_min_area_threshold =
      config.bounding_box_min_area_threshold;
    HoleFusionParameters::bounding_box_detection_method =
      config.bounding_box_detection_method;
    HoleFusionParameters::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;
    HoleFusionParameters::AB_to_MO_ratio =
      config.AB_to_MO_ratio;
    HoleFusionParameters::interpolation_method =
      config.interpolation_method;
    HoleFusionParameters::run_checker_depth_diff =
      config.run_checker_depth_diff;
    HoleFusionParameters::run_checker_outline_of_rectangle =
      config.run_checker_outline_of_rectangle;
    HoleFusionParameters::run_checker_depth_area =
      config.run_checker_depth_area;
    HoleFusionParameters::run_checker_brushfire_outline_to_rectangle =
      config.run_checker_brushfire_outline_to_rectangle;
    HoleFusionParameters::rectangle_inflation_size =
      config.rectangle_inflation_size;
    HoleFusionParameters::depth_difference =
      config.depth_difference;
    HoleFusionParameters::segmentation_method =
      config.segmentation_method;
    HoleFusionParameters::max_iterations =
      config.max_iterations;
    HoleFusionParameters::num_points_to_exclude =
      config.num_points_to_exclude;
    HoleFusionParameters::point_to_plane_distance_threshold =
      config.point_to_plane_distance_threshold;
    HoleFusionParameters::scale_method =
      config.scale_method;
    HoleFusionParameters::debug_show_find_holes =
      config.debug_show_find_holes;
    HoleFusionParameters::debug_show_find_holes_size =
      config.debug_show_find_holes_size;
    HoleFusionParameters::debug_time_find_holes =
      config.debug_time_find_holes;
    HoleFusionParameters::debug_show_denoise_edges =
      config.debug_show_denoise_edges;
    HoleFusionParameters::debug_show_denoise_edges_size =
      config.debug_show_denoise_edges_size;
    HoleFusionParameters::debug_show_connect_pairs =
      config.debug_show_connect_pairs;
    HoleFusionParameters::debug_show_connect_pairs_size =
      config.debug_show_connect_pairs_size;

    HoleFusionParameters::debug_show_get_shapes_clear_border  =
      config.debug_show_get_shapes_clear_border;
    HoleFusionParameters::debug_show_get_shapes_clear_border_size =
      config.debug_show_get_shapes_clear_border_size;

    HoleFusionParameters::debug_show_check_holes =
      config.debug_show_check_holes;
    HoleFusionParameters::debug_show_check_holes_size =
      config.debug_show_check_holes_size;

    HoleFusionParameters::minimum_curve_points =
      config.minimum_curve_points;
  }
}

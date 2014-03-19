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

#ifndef DEPTH_NODE_DEPTH_PARAMETERS_H
#define DEPTH_NODE_DEPTH_PARAMETERS_H

#include "depth_node/defines.h"
#include <dynamic_reconfigure/server.h>
#include <pandora_vision_hole_detector/depth_cfgConfig.h>

namespace pandora_vision
{

  struct DepthParameters
  {

    /**
      @brief The DepthParameters constructor
     **/
    DepthParameters(void);

    dynamic_reconfigure::Server<pandora_vision_hole_detector::depth_cfgConfig>
      server;
    dynamic_reconfigure::Server<pandora_vision_hole_detector::depth_cfgConfig>::
      CallbackType f;

    /**
      @brief The function called when a parateter is changed
      @param[in] config [const pandora_vision_hole_detector::depth_cfgConfig&]
      @param[in] level [const uint32_t] The level (?)
      @return void
     **/
    void callback(const pandora_vision_hole_detector::depth_cfgConfig& config,
      const uint32_t& level);

    /**
      @brief Retrieve the interpolation method
      @return int The interpolation method
     **/
    int getInterpolationMethod();

    //!< -------------------DepthParameters-----------------------------//

    //!< Subscriber of kinect parameter changing
    ros::Subscriber     _parameterChangingSubscriber;

    //!< Kanny parameters
    static int kanny_ratio;
    static int kanny_kernel_size;
    static int kanny_low_threshold;
    static int kanny_blur_noise_kernel_size;

    static float contrast_enhance_alpha;
    static float contrast_enhance_beta;

    //!< Threshold parameters
    static int threshold_lower_value;
    static int adaptive_max_value;
    static int adaptive_method;
    static int adaptive_block_size;
    static int adaptive_c_value;

    //!< Blob detection parameters
    static int blob_min_threshold;
    static int blob_max_threshold;
    static int blob_threshold_step;
    static int blob_min_area;
    static int blob_max_area;
    static double blob_min_convexity;
    static double blob_max_convexity;
    static double blob_min_inertia_ratio;
    static double blob_max_circularity;
    static double blob_min_circularity;
    static bool blob_filter_by_color;
    static bool blob_filter_by_circularity;

    //!< Bounding boxes parameters
    static int bounding_box_min_area_threshold;
    //!< The bounding box detection method
    //!< 0 for detecting by means of brushfire starting\
    //from the keypoint of the blob
    //!< 1 for detecting by means of contours around the edges of the blob
    static int bounding_box_detection_method;
    //!< When using raycast instead of brushfire to find the (approximate here)
    //!< outline of blobs, raycast_keypoint_partitions dictates the number of
    //!< rays, or equivalently, the number of partitions in which the blob is
    //!< partitioned in search of the blob's borders
    static int raycast_keypoint_partitions;

    //<! Loose ends connection parameters
    static int AB_to_MO_ratio;
    //!< Interpolation parameters
    //
    //!< The interpolation method for noise removal
    //!< 0 for averaging the pixel's neighbor values
    //!< 1 for brushfire near
    //!< 2 for brushfire far
    static int interpolation_method;

    //!< Hole checkers
    static int run_checker_depth_diff;
    static int run_checker_depth_area;
    static int run_checker_brushfire_outline_to_rectangle;
    static int run_checker_outline_of_rectangle;
    static int rectangle_inflation_size;
    static float depth_difference;

    //!< Plane detection
    static int segmentation_method;
    static int max_iterations;
    static double num_points_to_exclude;
    static double point_to_plane_distance_threshold;

    //!< Method to scale the CV_32FC1 image to CV_8UC1
    static int scale_method;

    //!< Debug
    static bool debug_show_find_holes;
    static int debug_show_find_holes_size;
    static bool debug_time_find_holes;

    static bool debug_show_denoise_edges;
    static int debug_show_denoise_edges_size;

    static bool debug_print_connect_pairs;
    static bool debug_show_connect_pairs;
    static int debug_show_connect_pairs_size;

    static bool debug_show_get_shapes_clear_border;
    static int debug_show_get_shapes_clear_border_size;

    static bool debug_show_check_holes;
    static int debug_show_check_holes_size;

    static int minimum_curve_points;

  };
} // namespace pandora_vision

#endif  // DEPTH_NODE_DEPTH_PARAMETERS_H

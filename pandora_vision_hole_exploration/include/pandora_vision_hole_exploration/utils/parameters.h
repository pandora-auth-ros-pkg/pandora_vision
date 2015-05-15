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
 * Authors: Alexandros Philotheou, Manos Tsardoulias, Vasilis Bosdelekidis
 *********************************************************************/

#ifndef UTILS_PARAMETERS_H
#define UTILS_PARAMETERS_H

//#include "utils/defines.h"
#include "utils/parameters.h"
#include <dynamic_reconfigure/server.h>
#include <pandora_vision_hole_exploration/depth_cfgConfig.h>
#include <pandora_vision_hole_exploration/rgb_cfgConfig.h>
#include <pandora_vision_hole_exploration/hole_fusion_cfgConfig.h>
#include <pandora_vision_hole_exploration/debug_cfgConfig.h>
//#include <pandora_vision_hole/filters_priority_cfgConfig.h>
//#include <pandora_vision_hole/filters_thresholds_cfgConfig.h>
#include <pandora_vision_hole_exploration/general_cfgConfig.h>
#include <pandora_vision_hole_exploration/validity_cfgConfig.h>

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  class ParametersHandler
  {
    public:
      //the constructor
      ParametersHandler();

    private:
      // The dynamic reconfigure (RGB) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::rgb_cfgConfig> serverRgb;

      // The dynamic reconfigure (RGB) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::rgb_cfgConfig>::CallbackType fRgb;

      // The dynamic reconfigure (Depth) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::depth_cfgConfig> serverDepth;

      // The dynamic reconfigure (Depth) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::depth_cfgConfig>::CallbackType fDepth;

      /**
        @brief The function called when a parameter is changed
        @param[in] configRgb [const pandora_vision_hole::rgb_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackRgb(
          const pandora_vision_hole_exploration::rgb_cfgConfig& configRgb,
          const uint32_t& level);
      /**
        @brief The function called when a parameter is changed
        @param[in] configDepth [const pandora_vision_hole::depth_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackDepth(
          const pandora_vision_hole_exploration::depth_cfgConfig& configDepth,
          const uint32_t& level);

  };


  /**
    @struct Parameters
    @brief Provides flexibility by parameterizing variables needed by the
    hole detector package
   **/
  struct Parameters
  {
    struct Debug
    {
      // Show the depth image that arrives in the depth node
      static bool show_depth_image;

      //Show the rgb image that arrives in the rgb node
      static bool show_rgb_image;
      //Show the std variance image after rgb image processing
      static bool show_std_variance_image;
      //
      //      // Show the holes that each of the depth and RGB nodes transmit to the
      // hole fusion node, on top of their respective origin images
      static bool show_respective_holes;
      //
      // Show all valid holes, from either the Depth or RGB source, or
      // the merges between them
      static bool show_valid_holes;
      // In the terminal's window, show the probabilities of candidate holes
      static bool show_probabilities;
      //
      //      // Show the texture's watersheded backprojection
      //      static bool show_texture;
      //
      static bool show_find_holes;
      static int show_find_holes_size;
    };
    

    //! Parameters specific to the Depth node
    struct Depth
    {
      static float intensity_threshold;
      static int morphology_open_kernel_size;
      static int morphology_close_kernel_size;
      static int depth_thresh;
      static int border_thresh;
      static int dilation_kernel_size;
      static int rect_diff_thresh;
      static int huge_contour_thresh;
      static int tiny_contour_thresh;
      static int small_contour_thresh;
      static int neighbor_thresh;
      static int neighbor_value_thresh;
      static int depth_similarity_rect_dims_thresh;
      static float merge_thresh;
      static int canny_low_threshold;
      static int canny_ratio;
      static int canny_kernel_size;
      static int filtering_type;
      static float min_valid_depth;
      static int shape_validation;
      static float one_direction_rectangle_contour_overlap_thresh;
      static int max_intersections_thresh;
    };


    struct HoleFusion
    {

      static int bin_to_find_mergable_size;
      static float valid_strong_probability;
      static float valid_medium_probability;
      static float valid_light_probability;
      static float max_depth_to_test_small_thresh;
      static float min_depth_to_test_big_thresh;
      static int small_rect_thresh;
      static int big_rect_thresh;
      static float rgb_distance_variance_thresh;
      static float rgb_small_distance_variance_thresh;
      static int hole_border_thresh;
      static float depth_difference_thresh;
      static int remove_unstuffed_holes;
      static int unstuffed_removal_method;
      static float difference_scanline_thresh;
    };


    //    //! Image representation specific parameters
    struct Image
    {
      // The depth sensor's horizontal field of view
      static float horizontal_field_of_view;
      //
      // The depth sensor's vertical field of view
      static float vertical_field_of_view;
      //
      //      // Fallback values. See the input point cloud callback of the
      //      // synchronizer node
      static int HEIGHT;
      static int WIDTH;

      // Method to scale the CV_32F images to CV_8UC1
      static int scale_method;
      //
    };


    //! Parameters specific to the RGB node
    struct Rgb
    {
      static int original_image_gaussian_blur;
      static int std_variance_kernel_size;
      static int std_variance_threshold;
      static int std_variance_morphology_close_size;
      static int std_variance_morphology_open_size;
      static int contour_erode_kernel_size;
      static int lower_contour_number_to_test_huge;
      static int huge_contour_thresh;
      static int tiny_contour_thresh;
      static int border_thresh;
      static int small_contour_thresh;
      static int neighbor_thresh;
      static int homog_rect_dims_thresh;
      static int neighbor_value_thresh;
      static float homogenity_thresh;
      static int neighbor_tiny_distance_thresh;
      static int rect_diff_thresh;
      static int shape_validation;
      static float one_direction_rectangle_contour_overlap_thresh;
      static int max_intersections_thresh;
    };

  };

} // namespace pandora_vision

#endif  // UTILS_PARAMETERS_H

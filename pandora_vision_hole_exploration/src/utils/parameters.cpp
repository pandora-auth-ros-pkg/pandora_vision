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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#include "utils/parameters.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  ParametersHandler::ParametersHandler()
  {
    serverRgb.setCallback(boost::bind(&ParametersHandler::parametersCallbackRgb, this, _1, _2));
    serverDepth.setCallback(boost::bind(&ParametersHandler::parametersCallbackDepth, this, _1, _2));
  }

  /**
    @brief The function called when a parameter is changed
    @param[in] configRgb [const pandora_vision_hole::rgb_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void ParametersHandler::parametersCallbackRgb(
      const pandora_vision_hole_exploration::rgb_cfgConfig& configRgb,
      const uint32_t& level)
  {

    //////////////////////////////// Debug parameters ////////////////////////////

    //// Show the rgb image that arrives in the rgb node
    Parameters::Debug::show_rgb_image =
      configRgb.show_rgb_image;

    Parameters::Debug::show_std_variance_image =
      configRgb.show_std_variance_image;

    Parameters::Debug::show_find_holes =
      configRgb.show_find_holes;
    Parameters::Debug::show_find_holes_size =
      configRgb.show_find_holes_size;

    ////////////////////// Parameters specific to the RGB node ///////////////////

    // Std variance, morphology extraction, holes validation thresholds, holes merging thresholds.
    Parameters::Rgb::original_image_gaussian_blur =
      configRgb.original_image_gaussian_blur;
    Parameters::Rgb::std_variance_kernel_size =
      configRgb.std_variance_kernel_size;
    Parameters::Rgb::std_variance_threshold =
      configRgb.std_variance_threshold;
    Parameters::Rgb::std_variance_morphology_close_size =
      configRgb.std_variance_morphology_close_size;
    Parameters::Rgb::std_variance_morphology_open_size =
      configRgb.std_variance_morphology_open_size;
    Parameters::Rgb::contour_erode_kernel_size =
      configRgb.contour_erode_kernel_size;
    Parameters::Rgb::lower_contour_number_to_test_huge =
      configRgb.lower_contour_number_to_test_huge;
    Parameters::Rgb::huge_contour_thresh =
      configRgb.huge_contour_thresh;
    Parameters::Rgb::tiny_contour_thresh =
      configRgb.tiny_contour_thresh;
    Parameters::Rgb::border_thresh =
      configRgb.border_thresh;
    Parameters::Rgb::small_contour_thresh =
      configRgb.small_contour_thresh;
    Parameters::Rgb::neighbor_thresh =
      configRgb.neighbor_thresh;
    Parameters::Rgb::homog_rect_dims_thresh =
      configRgb.homog_rect_dims_thresh;
    Parameters::Rgb::neighbor_value_thresh =
      configRgb.neighbor_value_thresh;
    Parameters::Rgb::homogenity_thresh =
      configRgb.homogenity_thresh;
    Parameters::Rgb::neighbor_tiny_distance_thresh =
      configRgb.neighbor_tiny_distance_thresh;
    Parameters::Rgb::shape_validation =
      configRgb.shape_validation;
    Parameters::Rgb::one_direction_rectangle_contour_overlap_thresh =
      configRgb.one_direction_rectangle_contour_overlap_thresh;
    Parameters::Rgb::max_intersections_thresh =
      configRgb.max_intersections_thresh;
  }


  /**
    @brief The function called when a parameter is changed
    @param[in] configDepth [const pandora_vision_hole::depth_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void ParametersHandler::parametersCallbackDepth(
      const pandora_vision_hole_exploration::depth_cfgConfig& configDepth,
      const uint32_t& level)
  {
    //  //////////////////////////////// Debug parameters ////////////////////////////

    // Show the depth image that arrives in the depth node
    Parameters::Debug::show_depth_image =
      configDepth.show_depth_image;

    Parameters::Debug::show_find_holes =
      configDepth.show_find_holes;
    Parameters::Debug::show_find_holes_size =
      configDepth.show_find_holes_size;

    //  Parameters::Rgb::neighbor_tiny_distance_thresh 
    //    configDepth.neighbor_tiny_distance_thresh;
    Parameters::Depth::intensity_threshold =
      configDepth.intensity_threshold;
    Parameters::Depth::morphology_open_kernel_size =
      configDepth.morphology_open_kernel_size;
    Parameters::Depth::morphology_close_kernel_size =
      configDepth.morphology_close_kernel_size;
    Parameters::Depth::border_thresh =
      configDepth.border_thresh;
    Parameters::Depth::dilation_kernel_size =
      configDepth.dilation_kernel_size;
    Parameters::Depth::rect_diff_thresh =
      configDepth.rect_diff_thresh;
    Parameters::Depth::huge_contour_thresh =
      configDepth.huge_contour_thresh;
    Parameters::Depth::tiny_contour_thresh =
      configDepth.tiny_contour_thresh;
    Parameters::Depth::small_contour_thresh =
      configDepth.small_contour_thresh;
    Parameters::Depth::neighbor_thresh =
      configDepth.neighbor_thresh;
    Parameters::Depth::neighbor_value_thresh =
      configDepth.neighbor_value_thresh;
    Parameters::Depth::depth_similarity_rect_dims_thresh =
      configDepth.depth_similarity_rect_dims_thresh;
    Parameters::Depth::merge_thresh =
      configDepth.merge_thresh;
    Parameters::Depth::canny_low_threshold =
      configDepth.canny_low_threshold;
    Parameters::Depth::canny_ratio =
      configDepth.canny_ratio;
    Parameters::Depth::canny_kernel_size =
      configDepth.canny_kernel_size;
    Parameters::Depth::filtering_type =
      configDepth.filtering_type;
    Parameters::Depth::min_valid_depth =
      configDepth.min_valid_depth;
    Parameters::Depth::shape_validation =
      configDepth.shape_validation;
    Parameters::Depth::one_direction_rectangle_contour_overlap_thresh =
      configDepth.one_direction_rectangle_contour_overlap_thresh;
    Parameters::Depth::max_intersections_thresh =
      configDepth.max_intersections_thresh;
  }


  // Show the depth image that arrives in the depth node
  bool Parameters::Debug::show_depth_image = false;
  //
  // Show the rgb image that arrives in the rgb node
  bool Parameters::Debug::show_rgb_image = false;
  // Show the std variance image after processing rgb image
  bool Parameters::Debug::show_std_variance_image = false;
  //
  // Show the holes that each of the depth and RGB nodes transmit to the
  // hole fusion node, on top of their respective origin images
  bool Parameters::Debug::show_respective_holes = false;
  //
  // Show all valid holes, from either the Depth or RGB source, or
  // the merges between them
  bool Parameters::Debug::show_valid_holes = false;
  //
  //  // The product of this package: unique, valid holes
  //  bool Parameters::Debug::show_final_holes = false;
  //
  // In the terminal's window, show the probabilities of candidate holes
  bool Parameters::Debug::show_probabilities = false;
  //
  //  // Show the texture's watersheded backprojection
  //  bool Parameters::Debug::show_texture = false;
  //
  bool Parameters::Debug::show_find_holes = false;
  int Parameters::Debug::show_find_holes_size = 1000;


  float Parameters::Depth::intensity_threshold = 0.1;
  int Parameters::Depth::morphology_open_kernel_size = 2;
  int Parameters::Depth::morphology_close_kernel_size = 12;
  int Parameters::Depth::border_thresh = 20;
  int Parameters::Depth::dilation_kernel_size = 12;
  int Parameters::Depth::rect_diff_thresh = 3;
  int Parameters::Depth::huge_contour_thresh = 20000;
  int Parameters::Depth::tiny_contour_thresh = 800;
  int Parameters::Depth::small_contour_thresh = 2500;
  int Parameters::Depth::neighbor_thresh = 50;
  int Parameters::Depth::neighbor_value_thresh = 30;
  int Parameters::Depth::depth_similarity_rect_dims_thresh = 50;
  float Parameters::Depth::merge_thresh = 50.0;
  int Parameters::Depth::canny_low_threshold = 50;
  int Parameters::Depth::canny_ratio = 3;
  int Parameters::Depth::canny_kernel_size = 3;
  int Parameters::Depth::filtering_type = 1;
  float Parameters::Depth::min_valid_depth = 0.92;
  int Parameters::Depth::shape_validation = 0;
  float Parameters::Depth::one_direction_rectangle_contour_overlap_thresh = 40.0;
  int Parameters::Depth::max_intersections_thresh = 4;


  int Parameters::HoleFusion::bin_to_find_mergable_size = 60;
  float Parameters::HoleFusion::valid_strong_probability = 1.0;
  float Parameters::HoleFusion::valid_medium_probability = 0.6;
  float Parameters::HoleFusion::valid_light_probability = 0.4;
  float Parameters::HoleFusion::max_depth_to_test_small_thresh = 2.5;
  float Parameters::HoleFusion::min_depth_to_test_big_thresh = 2.6;
  int Parameters::HoleFusion::small_rect_thresh = 900;
  int Parameters::HoleFusion::big_rect_thresh = 6400;
  float Parameters::HoleFusion::rgb_distance_variance_thresh = 0.5;
  float Parameters::HoleFusion::rgb_small_distance_variance_thresh = 0.2;
  int Parameters::HoleFusion::hole_border_thresh = 10;
  float Parameters::HoleFusion::depth_difference_thresh = 1.2;
  int Parameters::HoleFusion::remove_unstuffed_holes = 1;
  int Parameters::HoleFusion::unstuffed_removal_method = 1;
  float Parameters::HoleFusion::difference_scanline_thresh = 1.0;


  //  ////////////////// Image representation specific parameters //////////////////
  // The depth sensor's horizontal field of view in rads
  float Parameters::Image::horizontal_field_of_view =
    static_cast<float>(58) / 180 * M_PI;
  //
  // The depth sensor's vertical field of view in rads
  float Parameters::Image::vertical_field_of_view =
    static_cast<float>(45) / 180 * M_PI;
  //
  //  // Fallback values. See the input point cloud callback of the
  //  // synchronizer node
  int Parameters::Image::HEIGHT = 480;
  int Parameters::Image::WIDTH = 640;
  //
  //  // Depth and RGB images' representation method.
  //  // 0 if image used is used as obtained from the image sensor
  //  // 1 through wavelet analysis
  //  int Parameters::Image::image_representation_method = 1;
  //
  // Method to scale the CV_32F image to CV_8UC1
  int Parameters::Image::scale_method = 0;
  //


  int Parameters::Rgb::original_image_gaussian_blur = 3;
  int Parameters::Rgb::std_variance_kernel_size = 5;
  int Parameters::Rgb::std_variance_threshold = 48;
  int Parameters::Rgb::std_variance_morphology_close_size = 8; 
  int Parameters::Rgb::std_variance_morphology_open_size = 8;
  int Parameters::Rgb::contour_erode_kernel_size = 4;
  int Parameters::Rgb::lower_contour_number_to_test_huge = 2;
  int Parameters::Rgb::huge_contour_thresh = 8000;
  int Parameters::Rgb::tiny_contour_thresh = 500;
  int Parameters::Rgb::border_thresh = 100;
  int Parameters::Rgb::small_contour_thresh = 100; 
  int Parameters::Rgb::neighbor_thresh = 100;
  int Parameters::Rgb::homog_rect_dims_thresh = 50;
  int Parameters::Rgb::neighbor_value_thresh = 50;
  float Parameters::Rgb::homogenity_thresh = 0.8;
  int Parameters::Rgb::neighbor_tiny_distance_thresh = 50;
  int Parameters::Rgb::rect_diff_thresh = 2;
  int Parameters::Rgb::shape_validation = 0;
  float Parameters::Rgb::one_direction_rectangle_contour_overlap_thresh = 40.0;
  int Parameters::Rgb::max_intersections_thresh = 4;
} // namespace pandora_vision

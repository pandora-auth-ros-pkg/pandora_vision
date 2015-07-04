/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include "depth_node/depth_parameters.h"

namespace pandora_vision
{
namespace pandora_vision_hole
{

  int DepthParameters::Blob::min_threshold = 0;
  int DepthParameters::Blob::max_threshold = 200;
  int DepthParameters::Blob::threshold_step = 100;
  int DepthParameters::Blob::min_area = 550;
  int DepthParameters::Blob::max_area = 300000;
  double DepthParameters::Blob::min_convexity = 0;
  double DepthParameters::Blob::max_convexity = 100;
  double DepthParameters::Blob::min_inertia_ratio = 0;
  double DepthParameters::Blob::max_circularity = 1.0;
  double DepthParameters::Blob::min_circularity = 0.3;
  bool DepthParameters::Blob::filter_by_color = 0;
  bool DepthParameters::Blob::filter_by_circularity = 1;


  ///////////////////////// Debug-specific parameters //////////////////////////

  // Publish the enhanced Images
  bool DepthParameters::Debug::publish_enhanced_Images = false;

  // Show the depth image that arrives in the depth node
  bool DepthParameters::Debug::show_depth_image = false;

  // Show the thermal image that arrives in the thermal node
  bool DepthParameters::Debug::show_thermal_image = false;

  // Show the rgb image that arrives in the rgb node
  bool DepthParameters::Debug::show_rgb_image = false;

  // Show the holes that each of the depth and RGB nodes transmit to the
  // hole fusion node, on top of their respective origin images
  bool DepthParameters::Debug::show_respective_holes = false;

  // Show all valid holes, from either the Depth or RGB source, or
  // the merges between them
  bool DepthParameters::Debug::show_valid_holes = false;

  // The product of this package: unique, valid holes
  bool DepthParameters::Debug::show_final_holes = false;

  // In the terminal's window, show the probabilities of candidate holes
  bool DepthParameters::Debug::show_probabilities = false;

  // Show the texture's watersheded backprojection
  bool DepthParameters::Debug::show_texture = false;

  bool DepthParameters::Debug::show_find_holes = false;
  int DepthParameters::Debug::show_find_holes_size = 1000;

  bool DepthParameters::Debug::show_produce_edges = false;
  int DepthParameters::Debug::show_produce_edges_size = 900;

  bool DepthParameters::Debug::show_denoise_edges = false;
  int DepthParameters::Debug::show_denoise_edges_size = 900;

  bool DepthParameters::Debug::show_connect_pairs = false;
  int DepthParameters::Debug::show_connect_pairs_size = 1200;
  bool DepthParameters::Debug::print_connect_pairs = false;

  bool DepthParameters::Debug::show_get_shapes_clear_border = false;
  int DepthParameters::Debug::show_get_shapes_clear_border_size  = 1200;

  bool DepthParameters::Debug::show_check_holes = false;
  int DepthParameters::Debug::show_check_holes_size = 1200;

  bool DepthParameters::Debug::show_merge_holes = false;
  int DepthParameters::Debug::show_merge_holes_size = 1200;


  /////////////////// DepthParameters specific to the Depth node ////////////////////

  // The interpolation method for noise removal
  // 0 for averaging the pixel's neighbor values
  // 1 for brushfire near
  // 2 for brushfire far
  int DepthParameters::Depth::interpolation_method = 0;

  ////////////////// DepthParameters pecific to the Thermal node ////////////////////

  // The thermal detection method
  // If set to 0 process the binary image acquired from temperatures MultiArray
  // If set to 1 process the sensor/Image from thermal sensor
  int DepthParameters::Thermal::detection_method = 0;

  // The probability extraction method
  // 0 for Gaussian function
  // 1 for Logistic function

  int DepthParameters::Thermal::probability_method = 1;
  float DepthParameters::Thermal::min_thermal_probability = 0.3;

  // Gausian variables
  float DepthParameters::Thermal::optimal_temperature = 35;
  float DepthParameters::Thermal::tolerance = 10;

  // Logistic variables
  float DepthParameters::Thermal::low_acceptable_temperature = 32;
  float DepthParameters::Thermal::high_acceptable_temperature = 38;

  float DepthParameters::Thermal::left_tolerance = 4;
  float DepthParameters::Thermal::right_tolerance = 8;

  ///////////// DepthParameters of acceptable temperature for threshold /////////////
  float DepthParameters::Thermal::low_temperature = 28;
  float DepthParameters::Thermal::high_temperature = 40;

  ////////////////////// DepthParameters of the thermal image ///////////////////////
  int DepthParameters::ThermalImage::WIDTH = 80;
  int DepthParameters::ThermalImage::HEIGHT =60;

  ///////////////////// Edge detection specific parameters /////////////////////

  // canny parameters
  int DepthParameters::Edge::canny_ratio = 3;
  int DepthParameters::Edge::canny_kernel_size = 3;
  int DepthParameters::Edge::canny_low_threshold = 50;
  int DepthParameters::Edge::canny_blur_noise_kernel_size = 3;

  // The opencv edge detection method:
  // 0 for the Canny edge detector
  // 1 for the Scharr edge detector
  // 2 for the Sobel edge detector
  // 3 for the Laplacian edge detector
  // 4 for mixed Scharr / Sobel edge detection
  int DepthParameters::Edge::edge_detection_method = 2;

  // Threshold parameters
  int DepthParameters::Edge::denoised_edges_threshold = 10;

  // When mixed edge detection is selected, this toggle switch
  // is needed in order to shift execution from one edge detector
  // to the other.
  // 1 for the Scharr edge detector,
  // 2 for the Sobel edge detector
  int DepthParameters::Edge::mixed_edges_toggle_switch = 1;


  ///////////////////////  Histogram related parameters ////////////////////////

  int DepthParameters::Histogram::number_of_hue_bins = 30;
  int DepthParameters::Histogram::number_of_saturation_bins = 32;
  int DepthParameters::Histogram::number_of_value_bins = 32;
  int DepthParameters::Histogram::secondary_channel = 2;



  ////////////////////////// Filters-related parameters ////////////////////////

  // DepthDiff
  int DepthParameters::Filters::DepthDiff::priority = 5;
  float DepthParameters::Filters::DepthDiff::threshold = 0.7;

  // 0 for binary probability assignment on positive depth difference
  // 1 for gaussian probability assignment on positive depth difference
  int DepthParameters::Filters::DepthDiff::probability_assignment_method = 1;

  // The mean stardard deviation for the normal distribution
  // incorporated in the depth diff filter.
  float DepthParameters::Filters::DepthDiff::gaussian_mean = 0.15;

  float DepthParameters::Filters::DepthDiff::gaussian_stddev = 0.2;

  // Min difference in depth between the inside and the outside of a hole
  float DepthParameters::Filters::DepthDiff::min_depth_cutoff = 0.02;

  // Max difference in depth between the inside and the outside of a hole
  float DepthParameters::Filters::DepthDiff::max_depth_cutoff = 0.5;


  // DepthArea
  int DepthParameters::Filters::DepthArea::priority = 5;
  float DepthParameters::Filters::DepthArea::threshold = 0.7;

  // DepthHomogeneity
  int DepthParameters::Filters::DepthHomogeneity::priority = 5;
  float DepthParameters::Filters::DepthHomogeneity::threshold = 0.7;

  // RectanglePlaneConstitution
  int DepthParameters::Filters::RectanglePlaneConstitution::priority = 5;
  float DepthParameters::Filters::RectanglePlaneConstitution::threshold = 0.7;

  // IntermediatePointsPlaneConstitution
  int DepthParameters::Filters::IntermediatePointsPlaneConstitution::priority = 5;
  float DepthParameters::Filters::IntermediatePointsPlaneConstitution::threshold = 0.7;

  // ColourHomogeneity
  int DepthParameters::Filters::ColourHomogeneity::rgbd_priority = 5;
  int DepthParameters::Filters::ColourHomogeneity::rgb_priority = 5;

  float DepthParameters::Filters::ColourHomogeneity::rgbd_threshold = 0.7;
  float DepthParameters::Filters::ColourHomogeneity::rgb_threshold = 0.7;

  // LuminosityDiff
  int DepthParameters::Filters::LuminosityDiff::rgbd_priority = 5;
  int DepthParameters::Filters::LuminosityDiff::rgb_priority = 5;

  float DepthParameters::Filters::LuminosityDiff::rgbd_threshold = 0.7;
  float DepthParameters::Filters::LuminosityDiff::rgb_threshold = 0.7;

  // TextureDiff
  int DepthParameters::Filters::TextureDiff::rgbd_priority = 5;
  int DepthParameters::Filters::TextureDiff::rgb_priority = 5;

  float DepthParameters::Filters::TextureDiff::rgbd_threshold = 0.7;
  float DepthParameters::Filters::TextureDiff::rgb_threshold = 0.7;

  // The threshold for texture matching regarding the intermediate points
  float DepthParameters::Filters::TextureDiff::match_texture_threshold = 0.85;

  // The threshold for texture matching reagrding the points inside the hole
  float DepthParameters::Filters::TextureDiff::mismatch_texture_threshold = 0.8;

  // TextureBackprojection
  int DepthParameters::Filters::TextureBackprojection::rgbd_priority = 5;
  int DepthParameters::Filters::TextureBackprojection::rgb_priority = 5;

  float DepthParameters::Filters::TextureBackprojection::rgbd_threshold = 0.7;
  float DepthParameters::Filters::TextureBackprojection::rgb_threshold = 0.7;


  /////////////////////// HoleFusion-specific parameters ///////////////////////

  //-------------------------------- Validation --------------------------------

  // The holes' validation process identifier
  int DepthParameters::HoleFusion::Validation::validation_process = 0;

  // When depth analysis is applicable
  float DepthParameters::HoleFusion::Validation::rgbd_validity_threshold= 0.54;

  // When depth analysis is not applicable, we can only rely
  // on RGB analysis
  float DepthParameters::HoleFusion::Validation::rgb_validity_threshold = 0.40;


  // Plane detection parameters
  float DepthParameters::HoleFusion::Planes::filter_leaf_size = 0.1;
  int DepthParameters::HoleFusion::Planes::max_iterations = 1000;
  double DepthParameters::HoleFusion::Planes::num_points_to_exclude = 0.2;
  double DepthParameters::HoleFusion::Planes::point_to_plane_distance_threshold = 0.08;

  // Option to enable or disable the merging of holes
  bool DepthParameters::HoleFusion::Merger::merge_holes = true;

  // Holes connection - merger
  float DepthParameters::HoleFusion::Merger::connect_holes_min_distance = 0.1;
  float DepthParameters::HoleFusion::Merger::connect_holes_max_distance = 0.2;

  // Merger parameters
  float DepthParameters::HoleFusion::Merger::depth_diff_threshold = 0.3;
  float DepthParameters::HoleFusion::Merger::depth_area_threshold = 1.0;

  // The inflation size of holes' bounding rectangles
  int DepthParameters::HoleFusion::rectangle_inflation_size = 10;

  // Method to scale the CV_32F image to CV_8UC1
  int DepthParameters::Image::scale_method = 0;

  /////////////////// Outline discovery specific parameters ////////////////////

  // The detection method used to obtain the outline of a blob
  // 0 for detecting by means of brushfire
  // 1 for detecting by means of raycasting
  int DepthParameters::Outline::outline_detection_method = 0;

  // When using raycast instead of brushfire to find the (approximate here)
  // outline of blobs, raycast_keypoint_partitions dictates the number of
  // rays, or equivalently, the number of partitions in which the blob is
  // partitioned in search of the blob's borders
  int DepthParameters::Outline::raycast_keypoint_partitions = 32;

  // Loose ends connection parameters
  int DepthParameters::Outline::AB_to_MO_ratio = 4;
  int DepthParameters::Outline::minimum_curve_points = 50;

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

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

#include "thermal_node/thermal_parameters.h"

namespace pandora_vision
{
namespace pandora_vision_hole
{
  //////////////////// Blob detection - specific parameters ////////////////////

  int ThermalParameters::Blob::min_threshold = 0;
  int ThermalParameters::Blob::max_threshold = 200;
  int ThermalParameters::Blob::threshold_step = 100;
  int ThermalParameters::Blob::min_area = 550;
  int ThermalParameters::Blob::max_area = 300000;
  double ThermalParameters::Blob::min_convexity = 0;
  double ThermalParameters::Blob::max_convexity = 100;
  double ThermalParameters::Blob::min_inertia_ratio = 0;
  double ThermalParameters::Blob::max_circularity = 1.0;
  double ThermalParameters::Blob::min_circularity = 0.3;
  bool ThermalParameters::Blob::filter_by_color = 0;
  bool ThermalParameters::Blob::filter_by_circularity = 1;


  ///////////////////////// Debug-specific parameters //////////////////////////

  // Publish the enhanced Images

  // Show the thermal image that arrives in the thermal node
  bool ThermalParameters::Debug::show_thermal_image = false;

  bool ThermalParameters::Debug::show_find_holes = false;
  int ThermalParameters::Debug::show_find_holes_size = 1000;

  bool ThermalParameters::Debug::show_denoise_edges = false;
  int ThermalParameters::Debug::show_denoise_edges_size = 900;

  bool ThermalParameters::Debug::show_connect_pairs = false;
  int ThermalParameters::Debug::show_connect_pairs_size = 1200;

  bool ThermalParameters::Debug::show_get_shapes_clear_border = false;
  int ThermalParameters::Debug::show_get_shapes_clear_border_size  = 1200;

  ////////////////// ThermalParameters pecific to the Thermal node ////////////////////

  // The thermal detection method
  // If set to 0 process the binary image acquired from temperatures MultiArray
  // If set to 1 process the sensor/Image from thermal sensor
  int ThermalParameters::Detection::detection_method = 0;

  // The probability extraction method
  // 0 for Gaussian function
  // 1 for Logistic function

  int ThermalParameters::Detection::probability_method = 1;
  float ThermalParameters::Detection::min_thermal_probability = 0.3;

  // Gausian variables
  float ThermalParameters::Detection::optimal_temperature = 35;
  float ThermalParameters::Detection::tolerance = 10;

  // Logistic variables
  float ThermalParameters::Detection::low_acceptable_temperature = 32;
  float ThermalParameters::Detection::high_acceptable_temperature = 38;

  float ThermalParameters::Detection::left_tolerance = 4;
  float ThermalParameters::Detection::right_tolerance = 8;

  ///////////// ThermalParameters of acceptable temperature for threshold /////////////
  float ThermalParameters::Detection::low_temperature = 28;
  float ThermalParameters::Detection::high_temperature = 40;

  ////////////////////// ThermalParameters of the thermal image ///////////////////////
  int ThermalParameters::ThermalImage::WIDTH = 80;
  int ThermalParameters::ThermalImage::HEIGHT = 60;
  // Method to scale the CV_32F image to CV_8UC1
  int ThermalParameters::ThermalImage::scale_method = 0;


  ///////////////////// Edge detection specific parameters /////////////////////

  // canny parameters
  int ThermalParameters::Edge::canny_ratio = 3;
  int ThermalParameters::Edge::canny_kernel_size = 3;
  int ThermalParameters::Edge::canny_low_threshold = 50;
  int ThermalParameters::Edge::canny_blur_noise_kernel_size = 3;

  // The opencv edge detection method:
  // 0 for the Canny edge detector
  // 1 for the Scharr edge detector
  // 2 for the Sobel edge detector
  // 3 for the Laplacian edge detector
  // 4 for mixed Scharr / Sobel edge detection
  int ThermalParameters::Edge::edge_detection_method = 2;

  // Threshold parameters
  int ThermalParameters::Edge::denoised_edges_threshold = 10;

  // When mixed edge detection is selected, this toggle switch
  // is needed in order to shift execution from one edge detector
  // to the other.
  // 1 for the Scharr edge detector,
  // 2 for the Sobel edge detector
  int ThermalParameters::Edge::mixed_edges_toggle_switch = 1;

  /////////////////// Outline discovery specific parameters ////////////////////

  // The detection method used to obtain the outline of a blob
  // 0 for detecting by means of brushfire
  // 1 for detecting by means of raycasting
  int ThermalParameters::Outline::outline_detection_method = 0;

  // When using raycast instead of brushfire to find the (approximate here)
  // outline of blobs, raycast_keypoint_partitions dictates the number of
  // rays, or equivalently, the number of partitions in which the blob is
  // partitioned in search of the blob's borders
  int ThermalParameters::Outline::raycast_keypoint_partitions = 32;

  // Loose ends connection parameters
  int ThermalParameters::Outline::AB_to_MO_ratio = 4;
  int ThermalParameters::Outline::minimum_curve_points = 50;

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

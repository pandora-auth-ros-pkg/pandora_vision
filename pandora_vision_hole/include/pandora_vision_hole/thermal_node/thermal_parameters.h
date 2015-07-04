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

#ifndef THERMAL_NODE_THERMAL_PARAMETERS_H
#define THERMAL_NODE_THERMAL_PARAMETERS_H

namespace pandora_vision
{
namespace pandora_vision_hole
{

  struct ThermalParameters
  {
    struct Detection
    {
      // The thermal detection method
      // If set to 0 process the binary image acquired from temperatures MultiArray
      // If set to 1 process the sensor/Image from thermal sensor
      static int detection_method;

      //  The probability extraction method
      //  0 for Gaussian function
      //  1 for Logistic function
      static int probability_method;
      static float min_thermal_probability;

      //  Gausian variables
      static float optimal_temperature;
      static float tolerance;

      //  Logistic variables
      static float low_acceptable_temperature;
      static float high_acceptable_temperature;

      static float left_tolerance;
      static float right_tolerance;

      //  Low and High acceptable temperatures for thresholding
      static float low_temperature;
      static float high_temperature;
    };

    //  Thermal image parameters
    struct ThermalImage
    {
      //  Thermal image width and height
      static int WIDTH;
      static int HEIGHT;

      static int scale_method;
    };

    struct Debug
    {
      //  Show the thermal image that arrives in the thermal node
      static bool show_thermal_image;

      static bool show_find_holes;
      static int show_find_holes_size;

      static bool show_denoise_edges;
      static int show_denoise_edges_size;

      static bool show_connect_pairs;
      static int show_connect_pairs_size;

      static bool show_get_shapes_clear_border;
      static int show_get_shapes_clear_border_size;
    };

    struct Edge
    {
      //  canny parameters
      static int canny_ratio;
      static int canny_kernel_size;
      static int canny_low_threshold;
      static int canny_blur_noise_kernel_size;

      //  The opencv edge detection method:
      //  0 for the Canny edge detector
      //  1 for the Scharr edge detector
      //  2 for the Sobel edge detector
      //  3 for the Laplacian edge detector
      //  4 for mixed Scharr / Sobel edge detection
      static int edge_detection_method;

      //  Threshold parameters
      static int denoised_edges_threshold;

      //  When mixed edge detection is selected, this toggle switch
      //  is needed in order to shift execution from one edge detector
      //  to the other.
      //  1 for the Scharr edge detector,
      //  2 for the Sobel edge detector
      static int mixed_edges_toggle_switch;
    };

    struct Blob
    {
      static int min_threshold;
      static int max_threshold;
      static int threshold_step;
      static int min_area;
      static int max_area;
      static double min_convexity;
      static double max_convexity;
      static double min_inertia_ratio;
      static double max_circularity;
      static double min_circularity;
      static bool filter_by_color;
      static bool filter_by_circularity;
    };

    struct Outline
    {
      //  The detection method used to obtain the outline of a blob
      //  0 for detecting by means of brushfire
      //  1 for detecting by means of raycasting
      static int outline_detection_method;

      //  When using raycast instead of brushfire to find the (approximate here)
      //  outline of blobs, raycast_keypoint_partitions dictates the number of
      //  rays, or equivalently, the number of partitions in which the blob is
      //  partitioned in search of the blob's borders
      static int raycast_keypoint_partitions;

      //  Loose ends connection parameters
      static int AB_to_MO_ratio;
      static int minimum_curve_points;
    };
  };

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // THERMAL_NODE_THERMAL_PARAMETERS_H

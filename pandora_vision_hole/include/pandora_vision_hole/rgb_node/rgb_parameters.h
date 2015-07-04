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

#ifndef THERMAL_NODE_RGB_PARAMETERS_H
#define THERMAL_NODE_RGB_PARAMETERS_H

namespace pandora_vision
{
namespace pandora_vision_hole
{

  struct RgbParameters
  {
    struct Detection
    {
      //  Selects the method for extracting a RGB image's edges.
      //  Choices are via segmentation and via backprojection
      static int edges_extraction_method;

      //  The threshold applied to the backprojection of the RGB image
      //  captured by the image sensor
      static int backprojection_threshold;

      //  Parameters specific to the pyrMeanShiftFiltering method
      static int spatial_window_radius;
      static int color_window_radius;
      static int maximum_level_pyramid_segmentation;

      //  True to posterize the product of the segmentation
      static bool posterize_after_segmentation;

      //  FloodFill options regarding minimum and maximum colour difference
      static int floodfill_lower_colour_difference;
      static int floodfill_upper_colour_difference;

      //  Watershed-specific parameters
      static int watershed_foreground_dilation_factor;
      static int watershed_foreground_erosion_factor;
      static int watershed_background_dilation_factor;
      static int watershed_background_erosion_factor;
    };

    struct Debug
    {
      //  Show the rgb image that arrives in the rgb node
      static bool show_rgb_image;

      static bool show_find_holes;
      static int show_find_holes_size;

      static bool show_produce_edges;
      static int show_produce_edges_size;

      static bool show_denoise_edges;
      static int show_denoise_edges_size;

      static bool show_connect_pairs;
      static int show_connect_pairs_size;

      static bool show_get_shapes_clear_border;
      static int show_get_shapes_clear_border_size;

      static bool show_check_holes;
      static int show_check_holes_size;
    };

    struct RgbImage
    {
      static int scale_method;

      static int term_criteria_max_iterations;
      static double term_criteria_max_epsilon;
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

    struct Histogram
    {
      static int number_of_hue_bins;
      static int number_of_saturation_bins;
      static int number_of_value_bins;
      static int secondary_channel;
    };
  };

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // THERMAL_NODE_RGB_PARAMETERS_H

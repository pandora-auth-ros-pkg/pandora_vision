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
 * Authors: Vasilis Bosdelekidis, Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#include "depth_node/depth.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Constructor
   **/
  Depth::Depth()
  {
    // Acquire the names of topics which the depth node will be having
    // transactionary affairs with
    getTopicNames();

    // Subscribe to the Depth image published by the
    // rgb_depth_synchronizer node
    depthImageSubscriber_= nodeHandle_.subscribe( depthImageTopic_, 1,
        &Depth::inputDepthImageCallback, this);

    // Advertise the candidate holes found by the depth node
    candidateHolesPublisher_ = nodeHandle_.advertise
      <pandora_vision_msgs::ExplorerCandidateHolesVectorMsg>(
          candidateHolesTopic_, 1000);

    // The dynamic reconfigure (Depth) parameter's callback
    server.setCallback(boost::bind(&Depth::parametersCallback, this, _1, _2));

    ROS_INFO_NAMED(PKG_NAME, "[Depth node] Initiated");
  }



  /**
    @brief Destructor
   **/
  Depth::~Depth()
  {
    ROS_INFO_NAMED(PKG_NAME, "[Depth node] Terminated");
  }



  /**
    @brief Callback for the depth image received by the synchronizer node.

    The depth image message received by the synchronizer node is unpacked
    in a cv::Mat image. Holes are then located inside this image and
    information about them, along with the depth image, is then sent to the
    hole fusion node
    @param msg [const sensor_msgs::Image&] The depth image message
    @return void
   **/
  void Depth::inputDepthImageCallback(const sensor_msgs::Image& msg)
  {
    ROS_INFO_NAMED(PKG_NAME, "Depth node callback");

#ifdef DEBUG_TIME
    Timer::start("inputDepthImageCallback", "", true);
#endif

    // Obtain the depth image. Since the image is in a format of
    // sensor_msgs::Image, it has to be transformed into a cv format in order
    // to be processed. Its cv format will be TYPE_32FC1.
    cv::Mat depthImage;
    MessageConversions::extractImageFromMessage(msg, &depthImage,
        sensor_msgs::image_encodings::TYPE_32FC1);

#ifdef DEBUG_SHOW
    if (Parameters::Debug::show_depth_image)
    {
      Visualization::showScaled("Depth image", depthImage, 1);
    }
#endif

    // Regardless of the image representation method, the depth node
    // will publish the RGB image of original size to the Hole Fusion node
    cv::Mat depthImageSent;
    depthImage.copyTo(depthImageSent);

    // Locate potential holes in the depth image
    HolesConveyor conveyor = findHoles(depthImage);

    // Create the candidate holes message
    pandora_vision_msgs::ExplorerCandidateHolesVectorMsg depthCandidateHolesMsg;

    // Pack information about holes found and the interpolated depth image
    // inside a message.
    // This message will be published to and received by the hole fusion node
    MessageConversions::createCandidateHolesVectorMessage(conveyor,
        depthImageSent,
        &depthCandidateHolesMsg,
        sensor_msgs::image_encodings::TYPE_32FC1,
        msg);

    // Publish the candidate holes message
    candidateHolesPublisher_.publish(depthCandidateHolesMsg);

#ifdef DEBUG_TIME
    Timer::tick("inputDepthImageCallback");
    Timer::printAllMeansTree();
#endif
  }



  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the depth node
    @param void
    @return void
   **/
  void Depth::getTopicNames ()
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the depth node acquires the
    // unadulterated depth image and store it in a private member variable
    if (nodeHandle_.getParam(
          ns + "/depth_node/subscribed_topics/depth_image_topic",
          depthImageTopic_ ))
    {
      // Make the topic's name absolute
      depthImageTopic_ = ns + "/" + depthImageTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Depth Node] Subscribed to the input depth image");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
          "[Depth Node] Could not find topic depth_image_topic");
    }

    // Read the name of the topic to which the depth node will be publishing
    // information about the candidate holes found and store it in a private
    // member variable
    if (nodeHandle_.getParam(
          ns + "/depth_node/published_topics/candidate_holes_topic",
          candidateHolesTopic_))
    {
      // Make the topic's name absolute
      candidateHolesTopic_ = ns + "/" + candidateHolesTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Depth Node] Advertising to the candidate holes topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
          "[Depth Node] Could not find topic candidate_holes_topic");
    }
  }



  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_hole_exploration::depth_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void Depth::parametersCallback(
      const pandora_vision_hole_exploration::depth_cfgConfig& config,
      const uint32_t& level)
  {
    //  ROS_INFO_NAMED(PKG_NAME, "[RGB node] Parameters callback called");

    //  //////////////////// Blob detection - specific parameters //////////////////

    //  //Parameters::Blob::min_threshold =
    //  //  config.min_threshold;

    //  //Parameters::Blob::max_threshold =
    //  //  config.max_threshold;

    //  //Parameters::Blob::threshold_step =
    //  //  config.threshold_step;

    //  //// In wavelet mode, the image shrinks by a factor of 4
    //  //if (Parameters::Image::image_representation_method == 0)
    //  //{
    //  //  Parameters::Blob::min_area =
    //  //    config.min_area;

    //  //  Parameters::Blob::max_area =
    //  //    config.max_area;
    //  //}
    //  //else if (Parameters::Image::image_representation_method == 1)
    //  //{
    //  //  Parameters::Blob::min_area =
    //  //    static_cast<int>(config.min_area / 4);

    //  //  Parameters::Blob::max_area =
    //  //    static_cast<int>(config.max_area / 4);
    //  //}

    //  //Parameters::Blob::min_convexity =
    //  //  config.min_convexity;

    //  //Parameters::Blob::max_convexity =
    //  //  config.max_convexity;

    //  //Parameters::Blob::min_inertia_ratio =
    //  //  config.min_inertia_ratio;

    //  //Parameters::Blob::max_circularity =
    //  //  config.max_circularity;

    //  //Parameters::Blob::min_circularity =
    //  //  config.min_circularity;

    //  //Parameters::Blob::filter_by_color =
    //  //  config.filter_by_color;

    //  //Parameters::Blob::filter_by_circularity =
    //  //  config.filter_by_circularity;


    //  //////////////////////////////// Debug parameters ////////////////////////////

    // Show the rgb image that arrives in the rgb node
    Parameters::Debug::show_depth_image =
      config.show_depth_image;

    Parameters::Debug::show_find_holes =
      config.show_find_holes;
    Parameters::Debug::show_find_holes_size =
      config.show_find_holes_size;

    //  //Parameters::Debug::show_produce_edges =
    //  //  config.show_produce_edges;
    //  //Parameters::Debug::show_produce_edges_size =
    //  //  config.show_produce_edges_size;

    //  //Parameters::Debug::show_denoise_edges =
    //  //  config.show_denoise_edges;
    //  //Parameters::Debug::show_denoise_edges_size =
    //  //  config.show_denoise_edges_size;

    //  //Parameters::Debug::show_connect_pairs =
    //  //  config.show_connect_pairs;
    //  //Parameters::Debug::show_connect_pairs_size =
    //  //  config.show_connect_pairs_size;

    //  //Parameters::Debug::show_get_shapes_clear_border  =
    //  //  config.show_get_shapes_clear_border;
    //  //Parameters::Debug::show_get_shapes_clear_border_size =
    //  //  config.show_get_shapes_clear_border_size;


    //  ////////////////////// Parameters specific to the RGB node ///////////////////


    //  ////------------------- Edge detection specific parameters -------------------

    //  //// The opencv edge detection method:
    //  //// 0 for the Canny edge detector
    //  //// 1 for the Scharr edge detector
    //  //// 2 for the Sobel edge detector
    //  //// 3 for the Laplacian edge detector
    //  //// 4 for mixed Scharr / Sobel edge detection
    //  //Parameters::Edge::edge_detection_method =
    //  //  config.edge_detection_method;

    //  //Parameters::Edge::denoised_edges_threshold =
    //  //  config.denoised_edges_threshold;

    //  //// Canny parameters
    //  //Parameters::Edge::canny_ratio =
    //  //  config.canny_ratio;

    //  //Parameters::Edge::canny_kernel_size =
    //  //  config.canny_kernel_size;

    //  //Parameters::Edge::canny_low_threshold =
    //  //  config.canny_low_threshold;

    //  //Parameters::Edge::canny_blur_noise_kernel_size =
    //  //  config.canny_blur_noise_kernel_size;


    //  ////------------- Parameters needed for histogram calculation ----------------

    //  //Parameters::Histogram::number_of_hue_bins =
    //  //  config.number_of_hue_bins;

    //  //Parameters::Histogram::number_of_saturation_bins =
    //  //  config.number_of_saturation_bins;

    //  //Parameters::Histogram::number_of_value_bins =
    //  //  config.number_of_value_bins;

    //  //Parameters::Histogram::secondary_channel =
    //  //  config.secondary_channel;


    //  ////----------------- Outline discovery specific parameters ------------------

    //  //// The detection method used to obtain the outline of a blob
    //  //// 0 for detecting by means of brushfire
    //  //// 1 for detecting by means of raycasting
    //  //Parameters::Outline::outline_detection_method =
    //  //  config.outline_detection_method;

    //  //// When using raycast instead of brushfire to find the (approximate here)
    //  //// outline of blobs, raycast_keypoint_partitions dictates the number of
    //  //// rays, or equivalently, the number of partitions in which the blob is
    //  //// partitioned in search of the blob's borders
    //  //Parameters::Outline::raycast_keypoint_partitions =
    //  //  config.raycast_keypoint_partitions;


    //  ////------------------- Loose ends connection parameters ---------------------

    //  //Parameters::Outline::AB_to_MO_ratio = config.AB_to_MO_ratio;

    //  //// In wavelet mode, the image shrinks by a factor of 4
    //  //if (Parameters::Image::image_representation_method == 0)
    //  //{
    //  //  Parameters::Outline::minimum_curve_points =
    //  //    config.minimum_curve_points;
    //  //}
    //  //else if (Parameters::Image::image_representation_method == 1)
    //  //{
    //  //  Parameters::Outline::minimum_curve_points =
    //  //    static_cast<int>(config.minimum_curve_points / 4);
    //  //}


    //  //// Selects the method for extracting a RGB image's edges.
    //  //// Choices are via segmentation and via backprojection
    //  //Parameters::Rgb::edges_extraction_method =
    //  //  config.edges_extraction_method;

    //  ////------------------- RGB image segmentation parameters --------------------

    //  //// Parameters specific to the pyrMeanShiftFiltering method
    //  //Parameters::Rgb::spatial_window_radius =
    //  //  config.spatial_window_radius;
    //  //Parameters::Rgb::color_window_radius =
    //  //  config.color_window_radius;
    //  //Parameters::Rgb::maximum_level_pyramid_segmentation =
    //  //  config.maximum_level_pyramid_segmentation;

    //  //// Term criteria for the pyrMeanShiftFiltering method
    //  //Parameters::Image::term_criteria_max_iterations =
    //  //  config.term_criteria_max_iterations;
    //  //Parameters::Image::term_criteria_max_epsilon =
    //  //  config.term_criteria_max_epsilon;

    //  //// True to posterize the product of the segmentation
    //  //Parameters::Rgb::posterize_after_segmentation =
    //  //  config.posterize_after_segmentation;

    //  //// FloodFill options regarding minimum and maximum colour difference
    //  //Parameters::Rgb::floodfill_lower_colour_difference =
    //  //  config.floodfill_lower_colour_difference;
    //  //Parameters::Rgb::floodfill_upper_colour_difference =
    //  //  config.floodfill_upper_colour_difference;

    //  ////------------ RGB image edges via backprojection parameters ---------------

    //  //// The threshold applied to the backprojection of the RGB image
    //  //// captured by the image sensor
    //  //Parameters::Rgb::backprojection_threshold =
    //  //  config.backprojection_threshold;

    //  //// Watershed-specific parameters
    //  //Parameters::Rgb::watershed_foreground_dilation_factor =
    //  //  config.watershed_foreground_dilation_factor;
    //  //Parameters::Rgb::watershed_foreground_erosion_factor =
    //  //  config.watershed_foreground_erosion_factor;
    //  //Parameters::Rgb::watershed_background_dilation_factor =
    //  //  config.watershed_background_dilation_factor;
    //  //Parameters::Rgb::watershed_background_erosion_factor =
    //  //  config.watershed_background_erosion_factor;
    //  // Std variance, morphology extraction, holes validation thresholds, holes merging thresholds.
    //  Parameters::Rgb::original_image_gaussian_blur 
    //    config.original_image_gaussian_blur;
    //  Parameters::Rgb::std_variance_kernel_size 
    //    config.std_variance_kernel_size;
    //  Parameters::Rgb::std_variance_threshold 
    //    config.std_variance_threshold;
    //  Parameters::Rgb::std_variance_morphology_close_size 
    //    config.std_variance_morphology_close_size;
    //  Parameters::Rgb::std_variance_morphology_open_size 
    //    config.std_variance_morphology_open_size;
    //  Parameters::Rgb::contour_erode_kernel_size 
    //    config.contour_erode_kernel_size;
    //  Parameters::Rgb::lower_contour_number_to_test_huge 
    //    config.lower_contour_number_to_test_huge;
    //  Parameters::Rgb::huge_contour_thresh 
    //    config.huge_contour_thresh;
    //  Parameters::Rgb::tiny_contour_thresh 
    //    config.tiny_contour_thresh;
    //  Parameters::Rgb::border_thresh 
    //    config.border_thresh;
    //  Parameters::Rgb::small_contour_thresh 
    //    config.small_contour_thresh;
    //  Parameters::Rgb::neighbor_thresh 
    //    config.neighbor_thresh;
    //  Parameters::Rgb::homog_rect_dims_thresh 
    //    config.homog_rect_dims_thresh;
    //  Parameters::Rgb::neighbor_value_thresh 
    //    config.neighbor_value_thresh;
    //  Parameters::Rgb::homogenity_thresh 
    //    config.homogenity_thresh;
    //  Parameters::Rgb::neighbor_tiny_distance_thresh 
    //    config.neighbor_tiny_distance_thresh;
    Parameters::Depth::intensity_threshold =
      config.intensity_threshold;
    Parameters::Depth::morphology_open_kernel_size =
      config.morphology_open_kernel_size;
    Parameters::Depth::morphology_close_kernel_size =
      config.morphology_close_kernel_size;
    Parameters::Depth::border_thresh =
      config.border_thresh;
    Parameters::Depth::dilation_kernel_size =
      config.dilation_kernel_size;
    Parameters::Depth::rect_diff_thresh =
      config.rect_diff_thresh;
    Parameters::Depth::huge_contour_thresh =
      config.huge_contour_thresh;
    Parameters::Depth::tiny_contour_thresh =
      config.tiny_contour_thresh;
    Parameters::Depth::small_contour_thresh =
      config.small_contour_thresh;
    Parameters::Depth::neighbor_thresh =
      config.neighbor_thresh;
    Parameters::Depth::neighbor_value_thresh =
      config.neighbor_value_thresh;
    Parameters::Depth::depth_similarity_rect_dims_thresh =
      config.depth_similarity_rect_dims_thresh;
    Parameters::Depth::merge_thresh =
      config.merge_thresh;
    Parameters::Depth::canny_low_threshold =
      config.canny_low_threshold;
    Parameters::Depth::canny_ratio =
      config.canny_ratio;
    Parameters::Depth::canny_kernel_size =
      config.canny_kernel_size;
    Parameters::Depth::filtering_type =
      config.filtering_type;
    Parameters::Depth::min_valid_depth =
      config.min_valid_depth;
  }


  /**
    @brief Finds holes, provided a depth image in CV_32FC1 format.

    1. some basic eroding and dilation to eliminate isolated noise pixels.
    2. eliminate huge contours and tiny contours without filtering. There is no problem with the huge and no problem with the tiny for distances less than 2.5m (in any case, in such distances holes are not obvious at all).
    3. Eliminate contours which have an over 4x bounding box height/ width fraction or the inverse. There was no TP loss by this progress, instead wall edges were eliminated.
    4. Merge contours. Check only in small distance. Firstly label contours to merge together with a probability. The probability consists of a weighted sum of some features inside a box between the two contours' keypoints. Currently, features are the inverse of the euclidean distance between keypoints and the sum of blacks in the original image. 
    5. The contours checked at each step are those that were not eliminated by previous steps.
    @param[in] depthImage [const cv::Mat&] The depth image in CV_32FC1 format
    @return HolesConveyor The struct that contains the holes found
   **/
  HolesConveyor Depth::findHoles(const cv::Mat& depthImage) 
  {
    //#ifdef DEBUG_TIME
    //    Timer::start("findHoles", "inputRgbImageCallback");
    //#endif
    //
#ifdef DEBUG_SHOW
    std::string msg;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
#endif

#ifdef DEBUG_SHOW
    if(Parameters::Debug::show_find_holes) // Debug
    {
      cv::Mat tmp;
      depthImage.copyTo(tmp);
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial depth image";
      msgs.push_back(msg);
      tmp = Visualization::scaleImageForVisualization(
          depthImage, 0);
      imgs.push_back(tmp);
    }
#endif

    HolesConveyor conveyor;
    // do not even try to find holes if the distance is too small or equivalently the frame is too noisy
    float sum = 0;
    for(int row = 0; row < depthImage.rows; row ++)
      for(int col = 0; col < depthImage.cols; col ++)
      {
        sum += depthImage.at<float>(row, col);
      }
    float avg = sum / (depthImage.rows * depthImage.cols);
    if(avg > Parameters::Depth::min_valid_depth)
    {

      // Initial filtering of contours variant from the background
      cv::Mat filteredImage;
      filterImage(
          depthImage,
          &filteredImage);

#ifdef DEBUG_SHOW
      if(Parameters::Debug::show_find_holes) // Debug
      {
        cv::Mat tmp;
        filteredImage.copyTo(tmp);
        std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
        msg += STR(" : Edges after filtering");
        msgs.push_back(msg);
        imgs.push_back(tmp);
      }
#endif

      std::vector<std::vector<cv::Point> > contours;
      // Find contours in the variance image.
      detectContours(filteredImage, &contours);
      // center of mass of each contour
      std::vector<cv::Point2f> mc(contours.size());
      std::vector<cv::Rect> boundRect(contours.size());
      // Get center of mass and bounding box of each contour.
      getContourInfo(contours , &mc, &boundRect);
      std::vector<bool> realContours(contours.size(), true);
      // True contour sizes after possible merging
      std::vector<int> contourWidth(contours.size());
      std::vector<int> contourHeight(contours.size());
      // Validate contours found. The product is a vector with a flag for each contour.
      validateContours(depthImage, contours , &mc, &contourHeight, &contourWidth, &realContours, boundRect);
      // The final vectors of keypoints, and rectangles.
      std::vector<cv::Point2f> keypoints;
      std::vector<cv::Rect> rectangles;
      for(int i = 0; i < contours.size(); i++)
        if(realContours.at(i))
        {
          keypoints.push_back(mc[i]);
          cv::Rect temp(
              mc[i].x - contourWidth[i] / 2, 
              mc[i].y - contourHeight[i] / 2, 
              contourWidth[i], 
              contourHeight[i]);
          rectangles.push_back(temp);
        }

      conveyor.keypoint = keypoints;
      conveyor.rectangle = rectangles;

#ifdef DEBUG_SHOW
      if(Parameters::Debug::show_find_holes) // Debug
      {
        std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
        msg += STR(" : Initial keypoints");
        msgs.push_back(msg);
        imgs.push_back(Visualization::showKeypoints(msg, filteredImage, -1, mc));
      }
#endif


#ifdef DEBUG_SHOW
      if (Parameters::Debug::show_find_holes)
      {
        msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
        msg += STR(" : Blobs");
        msgs.push_back(msg);
        imgs.push_back(
            Visualization::showHoles(
              msg,
              depthImage,
              conveyor,
              -1,
              std::vector<std::string>())
            );
      }
#endif

#ifdef DEBUG_SHOW
      if (Parameters::Debug::show_find_holes)
      {
        Visualization::multipleShow("Depth node", imgs, msgs,
            Parameters::Debug::show_find_holes_size, 1);
      }
#endif
    }
    else 
    {
      conveyor.rectangle.clear();
      conveyor.keypoint.clear();
    }

    //#ifdef DEBUG_TIME
    //    Timer::tick("findHoles");
    //#endif

    return conveyor;
  }


  /**
    @brief The function called to filter the depth image
    @details Based on filtering_type param applying simple thresholding at 0, simple morhology transformations and to eliminate (make dark) the region at borders or simple edge detection method.
    @param[in] depthImage [const cv::Mat&] The Depth image to be processed,
    in CV_8UC3 format
    @param[in] filteredImage [cv::Mat* filteredImage] The output filtered binary image.
    @return void
   **/
  void Depth::filterImage(
      const cv::Mat& depthImage, 
      cv::Mat* filteredImage)
  {
    // The input depth image, in CV_8UC1 format
    cv::Mat visualizableDepthImage = 
      Visualization::scaleImageForVisualization(
          depthImage,
          0);
    if(Parameters::Depth::filtering_type == 0)
    {
      cv::threshold(
          visualizableDepthImage, 
          (*filteredImage), 
          Parameters::Depth::intensity_threshold, 
          1.0, 
          CV_THRESH_BINARY);
      int morphologyKernel = Parameters::Depth::morphology_open_kernel_size;
      cv::Mat structuringElement = 
        cv::getStructuringElement(
            cv::MORPH_ELLIPSE, 
            cv::Size(morphologyKernel, morphologyKernel));
      cv::morphologyEx( 
          (*filteredImage), 
          (*filteredImage), 
          cv::MORPH_OPEN, 
          structuringElement );
      morphologyKernel = Parameters::Depth::morphology_close_kernel_size;
      structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morphologyKernel, morphologyKernel));
      cv::morphologyEx((*filteredImage), (*filteredImage), cv::MORPH_CLOSE, structuringElement);
    }
    else
    {
      cv::Mat temp; 
      cv::Canny(
          visualizableDepthImage, 
          temp, 
          Parameters::Depth::canny_low_threshold, 
          Parameters::Depth::canny_low_threshold * Parameters::Depth::canny_ratio, 
          Parameters::Depth::canny_kernel_size);
      //apply canny mask
      visualizableDepthImage.copyTo((*filteredImage), temp);
    }
    for(int i = 0; i < Parameters::Depth::border_thresh; i ++)
      for(int j = 0; j < (*filteredImage).cols; j ++)
        (*filteredImage).at<uchar>(i, j) = 0;
    for(int i = (*filteredImage).rows - Parameters::Depth::border_thresh; i < (*filteredImage).rows; i ++)
      for(int j = 0; j < (*filteredImage).cols; j ++)
        (*filteredImage).at<uchar>(i, j) = 0;
    for(int i = 0; i < (*filteredImage).rows; i ++)
      for(int j = 0; j < Parameters::Depth::border_thresh; j ++)
        (*filteredImage).at<uchar>(i, j) = 0;
    for(int i = 0; i < (*filteredImage).rows; i ++)
      for(int j = (*filteredImage).cols - Parameters::Depth::border_thresh; j < (*filteredImage).cols; j ++)
        (*filteredImage).at<uchar>(i, j) = 0;
  }


  /**
    @brief The function called to extract contours that were not ignored by the initial filtering held in filterImage function.
    @param[in] filteredImage [cv::Mat&] The filtered image represented as white edges where there was a small depth. Remember that there were only edges left, which were black at the original depth image and were not random black pixels.
    @param[in] contours [std::vector<std::vector<cv::Point>>*] The contours found.
    @return void
   **/
  void Depth::detectContours(
      const cv::Mat& filteredImage, 
      std::vector<std::vector<cv::Point> >* contours)
  {
    cv::Mat temp;
    int dilationKernel = Parameters::Depth::dilation_kernel_size;
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilationKernel, dilationKernel));
    cv::dilate(filteredImage, filteredImage, structuringElement);
    cv::vector<cv::Vec4i> hierarchy;
    //cv::Mat temp1;
    //if (filteredImage.type() != CV_8UC1 && filteredImage.type() != CV_8UC3)
    //  filteredImage.convertTo(temp1, CV_8UC1);
    //else
    //  filteredImage.copyTo(temp1);
    findContours(filteredImage, (*contours), hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  }


  /**
    @brief The function called to estimate center of mass for each contour found and bounding boxes.
    @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
    @param[in] mc [std::vector<cv::Point2f>*] Center of mass of each contour as x, y coordinates..
    @return void
   **/
  void Depth::getContourInfo(
      const std::vector<std::vector<cv::Point> >& contours, 
      std::vector<cv::Point2f>* mc, 
      std::vector<cv::Rect>* boundRect)
  {
    std::vector<std::vector<cv::Point> > contoursPoly(contours.size());
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());
    std::vector<cv::Moments> mu(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {
      mu[i] = moments(contours[i], false);
      (*mc)[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00); 
      cv::approxPolyDP(cv::Mat(contours[i]), contoursPoly[i], 3, true);
      (*boundRect)[i] = cv::boundingRect(cv::Mat(contoursPoly[i]));
      cv::minEnclosingCircle((cv::Mat)contoursPoly[i], center[i], radius[i]);
    }
  }


  /**
    @brief The function called to make validation of found contours
    @param[in] image [cv::Mat&] The original image. Used to extract some features to evaluate contour size, similarity etc.
    @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
    @param[in] mc [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
    @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
    @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
    @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
    @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
    @return void
   **/
  void Depth::validateContours(
      const cv::Mat& image, 
      const std::vector<std::vector<cv::Point> >& contours, 
      std::vector<cv::Point2f>* mc, 
      std::vector<int>* contourHeight, 
      std::vector<int>* contourWidth, 
      std::vector<bool>* realContours, 
      const std::vector<cv::Rect>& boundRect)
  {
    std::vector<int> numLabels(contours.size(), 0);
    // for merging, contour index, other contour index, possibility
    std::map<std::pair<int, int>, float> contourLabel;
    for(int i = 0; i < contours.size(); i ++)
    {
      (*contourHeight)[i] = boundRect[i].height;
      (*contourWidth)[i] = boundRect[i].width;
      numLabels[i] = 0;
      if((*realContours)[i])
      {
        (*realContours)[i] = 
          validateContour(
              image, 
              i, 
              mc, 
              contourHeight, 
              contourWidth, 
              &contourLabel, 
              &numLabels, 
              boundRect, 
              contours, 
              realContours);
        if((*realContours).at(i))
          if((*contourWidth)[i] > Parameters::Depth::rect_diff_thresh * (*contourHeight)[i] 
              || (*contourHeight)[i] > Parameters::Depth::rect_diff_thresh * (*contourWidth)[i])
            (*realContours)[i] = false;
      }
    }
    for( int i = 0; i < contours.size(); i++ )
    {
      if((*realContours)[i] && numLabels[i] > 0)
        mergeContours(i, mc, contourHeight, contourWidth, &contourLabel, &numLabels, realContours, contours);
    }
  }


  /**
    @brief The function called by validateContours to make validation of a single contour
    @param[in] image [const cv::Mat&] The depth image 
    @param[in] ci [int] Current contour index in contours vector 
    @param[in] mcv [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
    @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
    @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
    @param[in] contourLabel [std::map<std::pair<int, int>, float>*] A map of contours relationship represented as current contour & some other contour as key and a probability of shame contour calculated via the values of some features between the two contours.
    @param[in] numLabels [std::vector<int>*] For each contour a counter of how many times it appears in the abovementioned map. In the map are strored only pairs of contours whose merging probability is above some threshold.
    @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
    @param[in] contours [std::vector<std::vector<cv::Point> >&] All contours found. 
    @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. 
    @return void
   **/
  bool Depth::validateContour(
      const cv::Mat& image, 
      int ci, std::vector<cv::Point2f>* mcv, 
      std::vector<int>* contourHeight, 
      std::vector<int>* contourWidth, 
      std::map<std::pair<int, int>, float>* contourLabel, 
      std::vector<int>* numLabels, 
      const std::vector<cv::Rect>& boundRect, 
      const std::vector<std::vector<cv::Point> >& contours, 
      std::vector<bool>* realContours)
  {
    int sumWhites = 0;
    if(cv::contourArea(contours[ci]) > Parameters::Depth::huge_contour_thresh 
        || cv::contourArea(contours[ci]) < Parameters::Depth::tiny_contour_thresh)
      return false;
    else
    {
      for(int i = 0; i < contours.size(); i ++)
      {
        if(i != ci)
        {
          if((*realContours)[i] 
              && (std::abs((*mcv)[ci].x - (*mcv)[i].x) < Parameters::Depth::neighbor_thresh) 
              && (std::abs((*mcv)[ci].y - (*mcv)[i].y) < Parameters::Depth::neighbor_thresh) 
              && (std::abs(static_cast<int>(image.at<uchar>((*mcv)[ci].y, (*mcv)[ci].x)) 
                  - static_cast<int>(image.at<uchar>((*mcv)[i].y, (*mcv)[i].x))) 
                < Parameters::Depth::neighbor_value_thresh))
          {
            int upperX;
            int upperY;
            int lowerX;
            int lowerY;
            if((*mcv)[ci].x > (*mcv)[i].x)
              upperX = (*mcv)[i].x;
            else
              upperX = (*mcv)[ci].x;
            if((*mcv)[ci].y> (*mcv)[i].y)
              upperY = (*mcv)[i].y;
            else
              upperY = (*mcv)[ci].y;
            if((*mcv)[ci].x > (*mcv)[i].x)
              lowerX = (*mcv)[ci].x;
            else
              lowerX = (*mcv)[i].x;
            if((*mcv)[ci].y > (*mcv)[i].y)
              lowerY = (*mcv)[ci].y;
            else
              lowerY = (*mcv)[i].y;
            cv::Mat ROI = image(boundRect[i]);
            cv::Scalar otherAvg = cv::mean(ROI);
            int homogRectWidth = std::abs(lowerX - upperX);
            if(homogRectWidth < Parameters::Depth::depth_similarity_rect_dims_thresh)
              homogRectWidth = Parameters::Depth::depth_similarity_rect_dims_thresh;
            int homogRectHeight = std::abs(lowerY - upperY);
            if(homogRectHeight < Parameters::Depth::depth_similarity_rect_dims_thresh)
              homogRectHeight = Parameters::Depth::depth_similarity_rect_dims_thresh;
            if(upperX + homogRectWidth > image.cols)
              homogRectWidth = image.cols - upperX - 1;
            if(upperY + homogRectHeight > image.rows)
              homogRectHeight = image.rows - upperY - 1;
            if(upperX < 0)
              upperX = 0;
            if(upperY < 0)
              upperY = 0;
            if(lowerX > image.cols)
              lowerX = image.cols;
            if(lowerY > image.rows)
              lowerY = image.rows;
            //cout << upperX << ", " << upperY << ", " << homogRectWidth << ", " << homogRectHeight << "\n";
            ROI = image(cv::Rect(upperX, upperY, homogRectWidth, homogRectHeight));
            int sumBlacks = 0;
            for(int r = 0; r < ROI.rows; r ++)
              for(int c = 0; c < ROI.cols; c ++)
                if(static_cast<int>(image.at<uchar>(r, c)) == 0)
                  sumBlacks++;
            float euclideanDistance = 
              1 / (sqrt(pow((*mcv)[i].x - (*mcv)[ci].x, 2) 
                    + pow((*mcv)[i].x - (*mcv)[ci].x, 2)));
            float mergeProbability = sumBlacks * 0.5 + euclideanDistance * 0.5;
            //std::cout << mergeProbability << "\n";
            if(mergeProbability > Parameters::Depth::merge_thresh)
            {
              if((*contourLabel).find(std::make_pair(i, ci)) == (*contourLabel).end())
              {
                (*contourLabel)[std::make_pair(ci, i)] = mergeProbability;
                (*numLabels)[ci]++;
                (*numLabels)[i]++;
              }
            }
          }
        }
      }
      int newIndX = (*mcv)[ci].x;
      int newIndY = (*mcv)[ci].y;
    }
    return true;
  }


  /**
    @brief The function called to make validation of found contours
    @param[in] image [cv::Mat&] The original image. Used to extract some features to evaluate contour size, similarity etc.
    @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
    @param[in] mc [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
    @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
    @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
    @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
    @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
    @return void
   **/
  void Depth::mergeContours(
      int ci, 
      std::vector<cv::Point2f>* mcv, 
      std::vector<int>* contourHeight, 
      std::vector<int>* contourWidth, 
      std::map<std::pair<int, int>, float>* contourLabel, 
      std::vector<int>* numLabels, 
      std::vector<bool>* realContours, 
      const std::vector<std::vector<cv::Point> >& contours)
  {
    int maxProbability = 0;
    double sumX = (*mcv)[ci].x;
    double sumY = (*mcv)[ci].y;
    double sum = 1;
    for(int i = 0; i < (*contourLabel).size(); i++)
    {
      for(int j = 0; j < contours.size(); j ++)
      {
        if((*realContours).at(j) && j != ci)
          if((*contourLabel).find(std::make_pair(ci, j)) != 
              (*contourLabel).end() 
              || (*contourLabel).find(std::make_pair(j, ci)) != 
              (*contourLabel).end())
          {
            //if((*contourLabel)[ci][j] > max)
            //{
            //    maxProbability = (*contourLabel)[ci][j];
            //}
            (*realContours)[j] = false;
            (*contourWidth)[ci] += (*contourWidth)[j];
            (*contourHeight)[ci] += (*contourHeight)[j];
            sum++;
            sumX += (*mcv)[j].x;
            sumY += (*mcv)[j].y;
            for(int k = 0; k < (*contourLabel).size(); k++)
              for(int l = 0; l < contours.size(); l ++)
                if((*realContours)[l] && l != ci && l != j)
                  if((*contourLabel).find(std::make_pair(j, l)) != 
                      (*contourLabel).end() 
                      || (*contourLabel).find(std::make_pair(l, j)) != 
                      (*contourLabel).end())
                  {
                    //if((*contourLabel)[ci][j] > max)
                    //{
                    //    maxProbability = (*contourLabel)[ci][j];
                    //}
                    (*realContours)[l] = false;
                    (*contourWidth)[ci] += (*contourWidth)[l];
                    (*contourHeight)[ci] += (*contourHeight)[l];
                    sum++;
                    sumX += (*mcv)[l].x;
                    sumY += (*mcv)[l].y;
                  }
          }
      }
    }
    (*mcv)[ci].x = sumX / sum;
    (*mcv)[ci].y = sumY / sum;
  }

} // namespace pandora_vision

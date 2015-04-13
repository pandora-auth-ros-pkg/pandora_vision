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
 * Authors: Vasilis Bosdelekidis, Despoina Paschalidou, Alexandros Philotheou
 *********************************************************************/

#include "rgb_node/rgb.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Constructor
   **/
  Rgb::Rgb()
  {
    // Acquire the names of topics which the rgb node will be having
    // transactionary affairs with
    getTopicNames();

    // Subscribe to the RGB image published by the
    // rgb_depth_synchronizer node
    rgbImageSubscriber_= nodeHandle_.subscribe( rgbImageTopic_, 1,
        &Rgb::inputRgbImageCallback, this);

    // Advertise the candidate holes found by the rgb node
    candidateHolesPublisher_ = nodeHandle_.advertise
      <pandora_vision_msgs::CandidateHolesVectorMsg>(
          candidateHolesTopic_, 1000);

    // The dynamic reconfigure (RGB) parameter's callback
    server.setCallback(boost::bind(&Rgb::parametersCallback, this, _1, _2));

    ROS_INFO_NAMED(PKG_NAME, "[RGB node] Initiated");
  }



  /**
    @brief Destructor
   **/
  Rgb::~Rgb()
  {
    ROS_INFO_NAMED(PKG_NAME, "[RGB node] Terminated");
  }



  /**
    @brief Callback for the rgb image received by the synchronizer node.

    The rgb image message received by the synchronizer node is unpacked
    in a cv::Mat image. Holes are then located inside this image and
    information about them, along with the rgb image, is then sent to the
    hole fusion node
    @param msg [const sensor_msgs::Image&] The rgb image message
    @return void
   **/
  void Rgb::inputRgbImageCallback(const sensor_msgs::Image& msg)
  {
    ROS_INFO_NAMED(PKG_NAME, "RGB node callback");

#ifdef DEBUG_TIME
    Timer::start("inputRgbImageCallback", "", true);
#endif

    // Obtain the rgb image. Since the image is in a format of
    // sensor_msgs::Image, it has to be transformed into a cv format in order
    // to be processed. Its cv format will be CV_8UC3.
    cv::Mat rgbImage;
    MessageConversions::extractImageFromMessage(msg, &rgbImage,
        sensor_msgs::image_encodings::BGR8);

    //#ifdef DEBUG_SHOW
    //    if (Parameters::Debug::show_rgb_image)
    //    {
    //      Visualization::show("RGB image", rgbImage, 1);
    //    }
    //#endif

    // Regardless of the image representation method, the RGB node
    // will publish the RGB image of original size to the Hole Fusion node
    cv::Mat rgbImageSent;
    rgbImage.copyTo(rgbImageSent);

    // Locate potential holes in the rgb image
    HolesConveyor conveyor = findHoles(rgbImage);

    // Create the candidate holes message
    pandora_vision_msgs::ExplorerCandidateHolesVectorMsg rgbCandidateHolesMsg;

    // Pack information about holes found and the rgb image inside a message.
    // This message will be published to and received by the hole fusion node
    MessageConversions::createCandidateHolesVectorMessage(conveyor,
        rgbImageSent,
        &rgbCandidateHolesMsg,
        sensor_msgs::image_encodings::TYPE_8UC3, msg);

    // Publish the candidate holes message
    candidateHolesPublisher_.publish(rgbCandidateHolesMsg);

#ifdef DEBUG_TIME
    Timer::tick("inputRgbImageCallback");
    Timer::printAllMeansTree();
#endif
  }



  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the rgb node
    @param void
    @return void
   **/
  void Rgb::getTopicNames()
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the rgb node acquires the
    // rgb image and store it in a private member variable
    if (nodeHandle_.getParam(
          ns + "/rgb_node/subscribed_topics/rgb_image_topic",
          rgbImageTopic_))
    {
      // Make the topic's name absolute
      rgbImageTopic_ = ns + "/" + rgbImageTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[RGB Node] Subscribed to the input RGB image");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
          "[RGB Node] Could not find topic rgb_image_topic");
    }

    // Read the name of the topic to which the rgb node will be publishing
    // information about the candidate holes found and store it in a private
    // member variable
    if (nodeHandle_.getParam(
          ns + "/rgb_node/published_topics/candidate_holes_topic",
          candidateHolesTopic_))
    {
      // Make the topic's name absolute
      candidateHolesTopic_ = ns + "/" + candidateHolesTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[RGB Node] Advertising to the candidate holes topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
          "[RGB Node] Could not find topic candidate_holes_topic");
    }
  }



  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_hole::rgb_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void Rgb::parametersCallback(
      const pandora_vision_hole_exploration::rgb_cfgConfig& config,
      const uint32_t& level)
  {
    ROS_INFO_NAMED(PKG_NAME, "[RGB node] Parameters callback called");

    //////////////////// Blob detection - specific parameters //////////////////

    //Parameters::Blob::min_threshold =
    //  config.min_threshold;

    //Parameters::Blob::max_threshold =
    //  config.max_threshold;

    //Parameters::Blob::threshold_step =
    //  config.threshold_step;

    //// In wavelet mode, the image shrinks by a factor of 4
    //if (Parameters::Image::image_representation_method == 0)
    //{
    //  Parameters::Blob::min_area =
    //    config.min_area;

    //  Parameters::Blob::max_area =
    //    config.max_area;
    //}
    //else if (Parameters::Image::image_representation_method == 1)
    //{
    //  Parameters::Blob::min_area =
    //    static_cast<int>(config.min_area / 4);

    //  Parameters::Blob::max_area =
    //    static_cast<int>(config.max_area / 4);
    //}

    //Parameters::Blob::min_convexity =
    //  config.min_convexity;

    //Parameters::Blob::max_convexity =
    //  config.max_convexity;

    //Parameters::Blob::min_inertia_ratio =
    //  config.min_inertia_ratio;

    //Parameters::Blob::max_circularity =
    //  config.max_circularity;

    //Parameters::Blob::min_circularity =
    //  config.min_circularity;

    //Parameters::Blob::filter_by_color =
    //  config.filter_by_color;

    //Parameters::Blob::filter_by_circularity =
    //  config.filter_by_circularity;


    //////////////////////////////// Debug parameters ////////////////////////////

    //// Show the rgb image that arrives in the rgb node
    //Parameters::Debug::show_rgb_image =
    //  config.show_rgb_image;

    //Parameters::Debug::show_find_holes =
    //  config.show_find_holes;
    //Parameters::Debug::show_find_holes_size =
    //  config.show_find_holes_size;

    //Parameters::Debug::show_produce_edges =
    //  config.show_produce_edges;
    //Parameters::Debug::show_produce_edges_size =
    //  config.show_produce_edges_size;

    //Parameters::Debug::show_denoise_edges =
    //  config.show_denoise_edges;
    //Parameters::Debug::show_denoise_edges_size =
    //  config.show_denoise_edges_size;

    //Parameters::Debug::show_connect_pairs =
    //  config.show_connect_pairs;
    //Parameters::Debug::show_connect_pairs_size =
    //  config.show_connect_pairs_size;

    //Parameters::Debug::show_get_shapes_clear_border  =
    //  config.show_get_shapes_clear_border;
    //Parameters::Debug::show_get_shapes_clear_border_size =
    //  config.show_get_shapes_clear_border_size;


    ////////////////////// Parameters specific to the RGB node ///////////////////


    ////------------------- Edge detection specific parameters -------------------

    //// The opencv edge detection method:
    //// 0 for the Canny edge detector
    //// 1 for the Scharr edge detector
    //// 2 for the Sobel edge detector
    //// 3 for the Laplacian edge detector
    //// 4 for mixed Scharr / Sobel edge detection
    //Parameters::Edge::edge_detection_method =
    //  config.edge_detection_method;

    //Parameters::Edge::denoised_edges_threshold =
    //  config.denoised_edges_threshold;

    //// Canny parameters
    //Parameters::Edge::canny_ratio =
    //  config.canny_ratio;

    //Parameters::Edge::canny_kernel_size =
    //  config.canny_kernel_size;

    //Parameters::Edge::canny_low_threshold =
    //  config.canny_low_threshold;

    //Parameters::Edge::canny_blur_noise_kernel_size =
    //  config.canny_blur_noise_kernel_size;


    ////------------- Parameters needed for histogram calculation ----------------

    //Parameters::Histogram::number_of_hue_bins =
    //  config.number_of_hue_bins;

    //Parameters::Histogram::number_of_saturation_bins =
    //  config.number_of_saturation_bins;

    //Parameters::Histogram::number_of_value_bins =
    //  config.number_of_value_bins;

    //Parameters::Histogram::secondary_channel =
    //  config.secondary_channel;


    ////----------------- Outline discovery specific parameters ------------------

    //// The detection method used to obtain the outline of a blob
    //// 0 for detecting by means of brushfire
    //// 1 for detecting by means of raycasting
    //Parameters::Outline::outline_detection_method =
    //  config.outline_detection_method;

    //// When using raycast instead of brushfire to find the (approximate here)
    //// outline of blobs, raycast_keypoint_partitions dictates the number of
    //// rays, or equivalently, the number of partitions in which the blob is
    //// partitioned in search of the blob's borders
    //Parameters::Outline::raycast_keypoint_partitions =
    //  config.raycast_keypoint_partitions;


    ////------------------- Loose ends connection parameters ---------------------

    //Parameters::Outline::AB_to_MO_ratio = config.AB_to_MO_ratio;

    //// In wavelet mode, the image shrinks by a factor of 4
    //if (Parameters::Image::image_representation_method == 0)
    //{
    //  Parameters::Outline::minimum_curve_points =
    //    config.minimum_curve_points;
    //}
    //else if (Parameters::Image::image_representation_method == 1)
    //{
    //  Parameters::Outline::minimum_curve_points =
    //    static_cast<int>(config.minimum_curve_points / 4);
    //}


    //// Selects the method for extracting a RGB image's edges.
    //// Choices are via segmentation and via backprojection
    //Parameters::Rgb::edges_extraction_method =
    //  config.edges_extraction_method;

    ////------------------- RGB image segmentation parameters --------------------

    //// Parameters specific to the pyrMeanShiftFiltering method
    //Parameters::Rgb::spatial_window_radius =
    //  config.spatial_window_radius;
    //Parameters::Rgb::color_window_radius =
    //  config.color_window_radius;
    //Parameters::Rgb::maximum_level_pyramid_segmentation =
    //  config.maximum_level_pyramid_segmentation;

    //// Term criteria for the pyrMeanShiftFiltering method
    //Parameters::Image::term_criteria_max_iterations =
    //  config.term_criteria_max_iterations;
    //Parameters::Image::term_criteria_max_epsilon =
    //  config.term_criteria_max_epsilon;

    //// True to posterize the product of the segmentation
    //Parameters::Rgb::posterize_after_segmentation =
    //  config.posterize_after_segmentation;

    //// FloodFill options regarding minimum and maximum colour difference
    //Parameters::Rgb::floodfill_lower_colour_difference =
    //  config.floodfill_lower_colour_difference;
    //Parameters::Rgb::floodfill_upper_colour_difference =
    //  config.floodfill_upper_colour_difference;

    ////------------ RGB image edges via backprojection parameters ---------------

    //// The threshold applied to the backprojection of the RGB image
    //// captured by the image sensor
    //Parameters::Rgb::backprojection_threshold =
    //  config.backprojection_threshold;

    //// Watershed-specific parameters
    //Parameters::Rgb::watershed_foreground_dilation_factor =
    //  config.watershed_foreground_dilation_factor;
    //Parameters::Rgb::watershed_foreground_erosion_factor =
    //  config.watershed_foreground_erosion_factor;
    //Parameters::Rgb::watershed_background_dilation_factor =
    //  config.watershed_background_dilation_factor;
    //Parameters::Rgb::watershed_background_erosion_factor =
    //  config.watershed_background_erosion_factor;
    // Std variance, morphology extraction, holes validation thresholds, holes merging thresholds.
    Parameters::Rgb::original_image_gaussian_blur =
      config.original_image_gaussian_blur;
    Parameters::Rgb::std_variance_kernel_size =
      config.std_variance_kernel_size;
    Parameters::Rgb::std_variance_threshold =
      config.std_variance_threshold;
    Parameters::Rgb::std_variance_morphology_close_size =
      config.std_variance_morphology_close_size;
    Parameters::Rgb::std_variance_morphology_open_size =
      config.std_variance_morphology_open_size;
    Parameters::Rgb::contour_erode_kernel_size =
      config.contour_erode_kernel_size;
    Parameters::Rgb::lower_contour_number_to_test_huge =
      config.lower_contour_number_to_test_huge;
    Parameters::Rgb::huge_contour_thresh =
      config.huge_contour_thresh;
    Parameters::Rgb::tiny_contour_thresh =
      config.tiny_contour_thresh;
    Parameters::Rgb::border_thresh =
      config.border_thresh;
    Parameters::Rgb::small_contour_thresh =
      config.small_contour_thresh;
    Parameters::Rgb::neighbor_thresh =
      config.neighbor_thresh;
    Parameters::Rgb::homog_rect_dims_thresh =
      config.homog_rect_dims_thresh;
    Parameters::Rgb::neighbor_value_thresh =
      config.neighbor_value_thresh;
    Parameters::Rgb::homogenity_thresh =
      config.homogenity_thresh;
    Parameters::Rgb::neighbor_tiny_distance_thresh =
      config.neighbor_tiny_distance_thresh;
  }


  /**
    @brief Finds holes, provided a RGB image in CV_8UC3 format.

    First apply a variance filter to extract only RsOI with big variance from background. After this validate them, find keypoints and bounding boxes.
    @param[in] rgbImage [const cv::Mat&] The RGB image to be processed,
    in CV_8UC3 format
    @return HolesConveyor The struct that contains the holes found
   **/
  HolesConveyor Rgb::findHoles(const cv::Mat& rgbImage) 
  {
    //#ifdef DEBUG_TIME
    //    Timer::start("findHoles", "inputRgbImageCallback");
    //#endif
    //
    //#ifdef DEBUG_SHOW
    //    std::string msg;
    //    std::vector<cv::Mat> imgs;
    //    std::vector<std::string> msgs;
    //#endif
    //
    //#ifdef DEBUG_SHOW
    //    if(Parameters::Debug::show_find_holes) // Debug
    //    {
    //      cv::Mat tmp;
    //      rgbImage.copyTo(tmp);
    //      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    //      msg += " : Initial RGB image";
    //      msgs.push_back(msg);
    //      imgs.push_back(tmp);
    //    }
    //#endif

    // Initial filtering of contours variant from the background
    cv::Mat bigVarianceContours;
    computeVarianceImage(
        rgbImage,
        &bigVarianceContours);

    //#ifdef DEBUG_SHOW
    //    if(Parameters::Debug::show_find_holes) // Debug
    //    {
    //      cv::Mat tmp;
    //      bigVarianceContours.copyTo(tmp);
    //      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    //      msg += STR(" : Edges after denoise");
    //      msgs.push_back(msg);
    //      imgs.push_back(tmp);
    //    }
    //#endif

    std::vector<std::vector<cv::Point> > contours;
    // Find contours in the variance image.
    detectContours(bigVarianceContours, &contours);
    // center of mass of each contour
    std::vector<cv::Point2f> mc(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    // Get center of mass and bounding box of each contour.
    getContourInfo(contours ,&mc, &boundRect);
    std::vector<bool> realContours;
    // True contour sizes after possible merging
    std::vector<int> contourWidth(contours.size());
    std::vector<int> contourHeight(contours.size());
    // Validate contours found. The product is a vector with a flag for each contour.
    validateContours(rgbImage, contours , &mc, &contourHeight, &contourWidth, &realContours, boundRect);

    //#ifdef DEBUG_SHOW
    //    if(Parameters::Debug::show_find_holes) // Debug
    //    {
    //      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    //      msg += STR(" : Initial keypoints");
    //      msgs.push_back(msg);
    //      imgs.push_back(Visualization::showKeypoints(msg, edges, -1, keyPoints));
    //    }
    //#endif

    // The final vectors of keypoints, and rectangles.
    HolesConveyor conveyor;

    //#ifdef DEBUG_SHOW
    //    if (Parameters::Debug::show_find_holes)
    //    {
    //      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    //      msg += STR(" : Blobs");
    //      msgs.push_back(msg);
    //      imgs.push_back(
    //          Visualization::showHoles(
    //            msg,
    //            rgbImage,
    //            conveyor,
    //            -1,
    //            std::vector<std::string>())
    //          );
    //    }
    //#endif
    //
    //#ifdef DEBUG_SHOW
    //    if (Parameters::Debug::show_find_holes)
    //    {
    //      Visualization::multipleShow("RGB node", imgs, msgs,
    //          Parameters::Debug::show_find_holes_size, 1);
    //    }
    //#endif
    //
    //#ifdef DEBUG_TIME
    //    Timer::tick("findHoles");
    //#endif
    std::vector<cv::Point2f> keypoints;
    std::vector<cv::Rect> rectangles;
    for(int i = 0; i < contours.size(); i++)
      if(realContours.at(i))
      {
        keypoints.push_back(mc[i]);
        rectangles.push_back(boundRect[i]);
      }

    conveyor.keypoint = keypoints;
    conveyor.rectangle = rectangles;

    return conveyor;
  }


  void Rgb::computeVarianceImage(const cv::Mat& rgbImage, cv::Mat* bigVarianceContours)
  {
    cv::GaussianBlur(rgbImage, rgbImage, cv::Size(0, 0), Parameters::Rgb::original_image_gaussian_blur);

    cv::Mat image32f;
    rgbImage.convertTo(image32f, CV_32F);

    cv::Mat mu;
    int windowSize = Parameters::Rgb::std_variance_kernel_size;
    cv::blur(image32f, mu, cv::Size(windowSize, windowSize));

    cv::Mat mu2;
    cv::blur(image32f.mul(image32f), mu2, cv::Size(windowSize, windowSize));

    cv::sqrt(mu2 - mu.mul(mu), (*bigVarianceContours));

    cv::normalize((*bigVarianceContours), (*bigVarianceContours), 0.0, 1.0, cv::NORM_MINMAX);
    cv::cvtColor((*bigVarianceContours), (*bigVarianceContours), CV_BGR2GRAY);

    double minVal, maxVal;
    cv::minMaxLoc((*bigVarianceContours), &minVal, &maxVal);
    (*bigVarianceContours).convertTo((*bigVarianceContours), CV_8UC1, 255.0 / (maxVal - minVal));
    cv::minMaxLoc((*bigVarianceContours), &minVal, &maxVal);

    // keep contours with the biggest variance, aka. the most white
    cv::threshold((*bigVarianceContours), (*bigVarianceContours), Parameters::Rgb::std_variance_threshold, 255, CV_THRESH_BINARY);

    int morphologyKernel = Parameters::Rgb::std_variance_morphology_close_size;
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morphologyKernel, morphologyKernel));
    cv::morphologyEx((*bigVarianceContours), (*bigVarianceContours), cv::MORPH_CLOSE, structuringElement);
    morphologyKernel = Parameters::Rgb::std_variance_morphology_open_size;
    structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morphologyKernel, morphologyKernel));
    cv::morphologyEx((*bigVarianceContours), (*bigVarianceContours), cv::MORPH_OPEN, structuringElement );
  }


  void Rgb::detectContours(const cv::Mat& bigVarianceContours, std::vector<std::vector<cv::Point> >* contours)
  {
    int erodeKernel = Parameters::Rgb::contour_erode_kernel_size;
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeKernel, erodeKernel));
    cv::erode(bigVarianceContours, bigVarianceContours, structuringElement);
    cv::vector<cv::Vec4i> hierarchy;
    findContours(bigVarianceContours, (*contours), hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  }


  void Rgb::getContourInfo(std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Point2f>* mc, std::vector<cv::Rect>* boundRect)
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


  void Rgb::validateContours(const cv::Mat& image, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Point2f>* mc, std::vector<int>* contourHeight, std::vector<int>* contourWidth, std::vector<bool>* realContours, std::vector<cv::Rect>& boundRect)
  {
    for(int ci = 0; ci < contours.size(); ci ++)
    {
      if((*realContours)[ci])

        if((contours.size() > Parameters::Rgb::lower_contour_number_to_test_huge && cv::contourArea(contours[ci]) > Parameters::Rgb::huge_contour_thresh))
        {
          (*realContours)[ci] = false;
          continue;
        }
        else if(cv::contourArea(contours[ci]) < Parameters::Rgb::tiny_contour_thresh)
        {
          if((*mc)[ci].x < Parameters::Rgb::border_thresh || (*mc)[ci].y < Parameters::Rgb::border_thresh || (image.cols - (*mc)[ci].x) < Parameters::Rgb::border_thresh || (image.rows - (*mc)[ci].y) < Parameters::Rgb::border_thresh)
          {
            (*realContours)[ci] = false;
            continue;
          }
        }
        else
        {
          if(cv::contourArea(contours[ci]) > Parameters::Rgb::small_contour_thresh)
          {
            cv::Mat ROI = image(boundRect[ci]);
            cv::Scalar curAvg = cv::mean(ROI);
            for(int i = 0; i < contours.size(); i ++)
            {
              if(i != ci)
              {
                if((*realContours)[i] && (std::abs((*mc)[ci].x - (*mc)[i].x) < Parameters::Rgb::neighbor_thresh) && (std::abs((*mc)[ci].y - (*mc)[i].y) < Parameters::Rgb::neighbor_thresh))
                {
                  int upperX;
                  int upperY;
                  int lowerX;
                  int lowerY;
                  if((*mc)[ci].x - (boundRect[ci].width / 2) > (*mc)[i].x - (boundRect[i].width / 2))
                    upperX = (*mc)[i].x - (boundRect[i].width / 2);
                  else
                    upperX = (*mc)[ci].x - (boundRect[ci].width / 2);
                  if((*mc)[ci].y - (boundRect[ci].height / 2) > (*mc)[i].y - (boundRect[i].height / 2))
                    upperY = (*mc)[i].y - (boundRect[i].height / 2);
                  else
                    upperY = (*mc)[ci].y - (boundRect[ci].height / 2);
                  if((*mc)[ci].x + (boundRect[ci].width / 2) > (*mc)[i].x + (boundRect[i].width / 2))
                    lowerX = (*mc)[ci].x + (boundRect[ci].width / 2);
                  else
                    lowerX = (*mc)[i].x + (boundRect[i].width / 2);
                  if((*mc)[ci].y + (boundRect[ci].height / 2) > (*mc)[i].y + (boundRect[i].height / 2))
                    lowerY = (*mc)[ci].y + (boundRect[ci].height / 2);
                  else
                    lowerY = (*mc)[i].y + (boundRect[i].height / 2);
                  cv::Mat ROI = image(boundRect[i]);
                  cv::Scalar otherAvg = cv::mean(ROI);
                  int homogRectWidth = std::abs(lowerX - upperX);
                  if(homogRectWidth < Parameters::Rgb::homog_rect_dims_thresh)
                    homogRectWidth = Parameters::Rgb::homog_rect_dims_thresh;
                  int homogRectHeight = abs(lowerY - upperY);
                  if(homogRectHeight < Parameters::Rgb::homog_rect_dims_thresh)
                    homogRectHeight = Parameters::Rgb::homog_rect_dims_thresh;
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
                  ROI = image(cv::Rect(upperX, upperY, homogRectWidth, homogRectHeight));
                  HaralickFeaturesExtractor haralickFeaturesDetector_;
                  haralickFeaturesDetector_.findHaralickFeatures(ROI);
                  std::vector<double> haralickFeatures = haralickFeaturesDetector_.getFeatures();
                  if((((std::abs(curAvg[0] - otherAvg[0]) < Parameters::Rgb::neighbor_value_thresh) && haralickFeatures[0] > Parameters::Rgb::homogenity_thresh)) || ((std::abs((*mc)[ci].x - (*mc)[i].x) < Parameters::Rgb::neighbor_tiny_distance_thresh) && (std::abs((*mc)[ci].y - (*mc)[i].y) < Parameters::Rgb::neighbor_tiny_distance_thresh)))
                  {
                    if(cv::contourArea(contours[i]) > cv::contourArea(contours[ci]))
                    {
                      (*mc)[i].x = 0.5 * (*mc)[i].x + 0.5 * (*mc)[ci].x;
                      (*mc)[i].y = 0.5 * (*mc)[i].y + 0.5 * (*mc)[ci].y;
                      (*contourHeight)[i] = (*contourHeight)[i] + (*contourHeight)[ci] + std::abs((*mc)[ci].y - (*mc)[i].y);
                      (*contourWidth)[i] = (*contourWidth)[i] + (*contourWidth)[ci] + std::abs((*mc)[ci].x - (*mc)[i].x);
                      (*realContours)[ci] = false;
                      continue;
                    }
                    else
                    {
                      (*mc)[ci].x = 0.5 * (*mc)[i].x + 0.5 * (*mc)[ci].x;
                      (*mc)[ci].y = 0.5 * (*mc)[i].y + 0.5 * (*mc)[ci].x;
                      (*contourHeight)[ci] = (*contourHeight)[ci] + (*contourHeight)[i] + std::abs((*mc)[ci].y - (*mc)[i].y);
                      (*contourWidth)[ci] = (*contourWidth)[ci] + (*contourWidth)[i] + std::abs((*mc)[ci].x - (*mc)[i].x);
                      (*realContours)[i] = false;
                    }
                  }
                }
              }
            }
          }
        }
    }
    for( int i = 0; i < contours.size(); i++ )
    {
      if((*realContours)[i])
        if((*contourWidth)[i] > Parameters::Rgb::rect_diff_thresh * (*contourHeight)[i] || (*contourHeight)[i] > Parameters::Rgb::rect_diff_thresh * (*contourWidth)[i])
          (*realContours)[i] = false;
    }
  }

} // namespace pandora_vision

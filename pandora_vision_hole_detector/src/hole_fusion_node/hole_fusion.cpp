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

#include "hole_fusion_node/hole_fusion.h"

namespace pandora_vision
{
  /**
    @brief The HoleFusion constructor
   **/
  HoleFusion::HoleFusion(void) : pointCloudXYZ_(new PointCloudXYZ)
  {
    #ifdef DEBUG_TIME
    Timer::start("HoleFusion");
    #endif

    ros::Duration(0.5).sleep();

    //!< Calculate the histogram cv::MatND needed for texture comparing
    getWallsHistogram();

    //!< Initialize the numNodesReady variable
    numNodesReady_ = 0;

    //!< Advertise the topic that the rgb_depth_synchronizer will be
    //!< subscribed to in order for the hole_fusion_node to unlock it
    unlockPublisher_ = nodeHandle_.advertise <std_msgs::Empty>
      ("/vision/hole_fusion/unlock_rgb_depth_synchronizer", 1000, true);

    //!< Subscribe to the topic where the depth node publishes
    //!< candidate holes
    depthCandidateHolesSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/depth/candidate_holes", 1,
      &HoleFusion::depthCandidateHolesCallback, this);

    //!< Subscribe to the topic where the rgb node publishes
    //!< candidate holes
    rgbCandidateHolesSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/rgb/candidate_holes", 1,
      &HoleFusion::rgbCandidateHolesCallback, this);

    //!< The dynamic reconfigure (hole fusion's) parameter's callback
    server.setCallback(boost::bind(&HoleFusion::parametersCallback,
        this, _1, _2));



    ROS_INFO("HoleFusion node initiated");

    //!< Start the synchronizer
    unlockSynchronizer();

    #ifdef DEBUG_TIME
    Timer::tick("HoleFusion");
    #endif
  }



  /**
    @brief The HoleFusion deconstructor
   **/
  HoleFusion::~HoleFusion(void)
  {
    ROS_INFO("HoleFusion node terminated");
  }



  /**
    @brief Callback for the candidate holes via the depth node
    @param[in] depthCandidateHolesVector
    [const vision_communications::DepthCandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the depth node
    @return void
   **/
  void HoleFusion::depthCandidateHolesCallback(
    const vision_communications::DepthCandidateHolesVectorMsg&
    depthCandidateHolesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("depthCandidateHolesCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Hole Fusion Depth callback");
    #endif

    //!< Clear the current depthHolesConveyor struct
    //!< (or else keyPoints, rectangles and outlines accumulate)
    depthHolesConveyor_.keyPoints.clear();
    depthHolesConveyor_.rectangles.clear();
    depthHolesConveyor_.outlines.clear();

    //!< Unpack the message
    unpackDepthMessage(depthCandidateHolesVector,
      &depthHolesConveyor_,
      &pointCloudXYZ_,
      &interpolatedDepthImage_);

    #ifdef DEBUG_SHOW
    if (Parameters::debug_show_find_holes)
    {
      Visualization::showScaled(
        "interpolated depth image arrived in Hole Fusion node",
        interpolatedDepthImage_, 1);
    }
    #endif

    numNodesReady_++;

    //!< If both the RGB and the depth nodes are ready
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 2)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("depthCandidateHolesCallback");
    #endif
  }



  /**
    @brief Callback for the candidate holes via the rgb node
    @param[in] depthCandidateHolesVector
    [const vision_communications::RgbCandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the rgb node
    @return void
   **/
  void HoleFusion::rgbCandidateHolesCallback(
    const vision_communications::RgbCandidateHolesVectorMsg&
    rgbCandidateHolesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("rgbCandidateHolesCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Hole Fusion RGB callback");
    #endif

    //!< Clear the current rgbHolesConveyor struct
    //!< (or else keyPoints, rectangles and outlines accumulate)
    rgbHolesConveyor_.keyPoints.clear();
    rgbHolesConveyor_.rectangles.clear();
    rgbHolesConveyor_.outlines.clear();

    //!< Unpack the message
    unpackRgbMessage(rgbCandidateHolesVector,
      &rgbHolesConveyor_,
      &rgbImage_);

    #ifdef DEBUG_SHOW
    if (Parameters::debug_show_find_holes)
    {
      Visualization::showScaled(
        "RGB image arrived in Hole Fusion node", rgbImage_, 1);
    }
    #endif

    numNodesReady_++;

    //!< If both the RGB and the depth nodes are ready
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 2)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("rgbCandidateHolesCallback");
    #endif
  }



  /**
    @brief Recreates the HoleFilters::HolesConveyor struct for the
    candidate holes from the
    vision_communications::CandidateHolerMsg message
    @param[in]candidateHolesVector
    [const std::vector<vision_communications::CandidateHoleMsg>&]
    The input candidate holes
    @param[out] conveyor [HoleFilters::HolesConveyor*] The output conveyor
    struct
    @return void
   **/
  void HoleFusion::fromCandidateHoleMsgToConveyor(
    const std::vector<vision_communications::CandidateHoleMsg>&
    candidateHolesVector,
    HoleFilters::HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("fromCandidateHoleMsgToConveyor");
    #endif

    for (unsigned int i = 0; i < candidateHolesVector.size(); i++)
    {
      //!< Recreate conveyor.keypoints
      cv::KeyPoint holeKeypoint;
      holeKeypoint.pt.x = candidateHolesVector[i].keypointX;
      holeKeypoint.pt.y = candidateHolesVector[i].keypointY;
      conveyor->keyPoints.push_back(holeKeypoint);

      //!< Recreate conveyor.rectangles
      std::vector<cv::Point2f> renctangleVertices;
      for (unsigned int v = 0;
        v < candidateHolesVector[i].verticesX.size(); v++)
      {
        cv::Point2f vertex;
        vertex.x = candidateHolesVector[i].verticesX[v];
        vertex.y = candidateHolesVector[i].verticesY[v];
        renctangleVertices.push_back(vertex);
      }
      conveyor->rectangles.push_back(renctangleVertices);

      //!< Recreate conveyor.outlines
      std::vector<cv::Point> outlinePoints;
      for (unsigned int o = 0;
        o < candidateHolesVector[i].outlineX.size(); o++)
      {
        cv::Point outlinePoint;
        outlinePoint.x = candidateHolesVector[i].outlineX[o];
        outlinePoint.y = candidateHolesVector[i].outlineY[o];
        outlinePoints.push_back(outlinePoint);
      }
      conveyor->outlines.push_back(outlinePoints);
    }

    #ifdef DEBUG_TIME
    Timer::tick("fromCandidateHoleMsgToConveyor");
    #endif
  }



  /**
    @brief Computes a cv::MatND histogram from images loaded in directory
    ${pandora_vision_hole_detector}/src/wall_pictures and stores it in
    a private member variable so as to be used in texture comparing
    @parameters void
    @return void
   **/
  void HoleFusion::getWallsHistogram()
  {
    //!< The path to the package where the wall pictures directory lies in
    std::string packagePath =
      ros::package::getPath("pandora_vision_hole_detector");

    //!< The actual wall pictures directory
    std::string wallPicturesPath = packagePath + "/walls/";

    int fileLength;

    //!< The number of wall picture files inside the wallPicturesPath directory
    int numPictures = 0;

    struct dirent* result = NULL;

    int nameMax = pathconf(wallPicturesPath.c_str(), _PC_NAME_MAX);
    int len = offsetof(struct dirent, d_name) + nameMax + 1;
    struct dirent *theDir = static_cast<struct dirent*>(malloc(len));

    DIR *directory;

    directory = opendir(wallPicturesPath.c_str());
    if (theDir != NULL)
    {
      while ((readdir_r(directory, theDir, &result)) == 0 && result!= NULL)
      {
        fileLength = strlen(theDir->d_name);
        if (strcmp (".png", &(theDir->d_name[fileLength - 4])) == 0)
        {
          numPictures++;
        }
      }
      closedir (directory);
    }

    //!< Read the pictures inside the wallPicturesPath, convert them to HSV
    //!< and calculate their histogram
    cv::Mat* wallImagesHSV = new cv::Mat[numPictures];
    for(int i = 0; i < numPictures; i++)
    {
      char temp_name[250];

      std::string temp = wallPicturesPath +"%d.png";

      sprintf(temp_name, temp.c_str(), i);

      cv::cvtColor(
        Visualization::scaleImageForVisualization(cv::imread(temp_name),
          Parameters::scale_method),
        wallImagesHSV[i], cv::COLOR_BGR2HSV);
    }

    //!< Histogram-related parameters
    int h_bins = Parameters::number_of_hue_bins;
    int s_bins = Parameters::number_of_saturation_bins;
    int histSize[] = { h_bins, s_bins };

    //!< hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    //!< Use the 0-th and 1-st channels
    int channels[] = { 0, 2 };

    //!< Calculate the histogram for the walls and store it in the class's
    //!< private member wallsHistogram_
    cv::calcHist(wallImagesHSV, numPictures, channels, cv::Mat(),
      wallsHistogram_, 2, histSize, ranges, true, false);

    delete[] wallImagesHSV;
  }



  /**
    @brief Implements a strategy to combine
    information from both sources in order to accurately find valid holes
    @return void
   **/
  void HoleFusion::processCandidateHoles()
  {
    #ifdef DEBUG_SHOW
    ROS_INFO ("numNodesReady: %d", numNodesReady_);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Processing candidate holes");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("processCandidateHoles", "depthCandidateHolesCallback");
    #endif

    //!< Do some processing

    //!< Initialize the probabilities 2D vector. But first we need to know
    //!< how many rows the vector will accomodate
    int depthActiveFilters = 0;

    if (Parameters::run_checker_depth_diff > 0)
    {
      depthActiveFilters++;
    }
    if (Parameters::run_checker_outline_of_rectangle > 0)
    {
      depthActiveFilters++;
    }
    if (Parameters::run_checker_depth_area > 0)
    {
      depthActiveFilters++;
    }
    if (Parameters::run_checker_brushfire_outline_to_rectangle > 0)
    {
      depthActiveFilters++;
    }
    if (Parameters::run_checker_depth_homogenity > 0)
    {
      depthActiveFilters++;
    }

    std::vector<std::vector<float> > depthProbabilitiesVector2D(
      depthActiveFilters,
      std::vector<float>(depthHolesConveyor_.keyPoints.size(), 0.0));

    //!< check holes for debugging purposes
    DepthFilters::checkHoles(
      interpolatedDepthImage_,
      pointCloudXYZ_,
      &depthHolesConveyor_,
      &depthProbabilitiesVector2D);

    //!< Initialize the probabilities 2D vector. But first we need to know
    //!< how many rows the vector will accomodate
    int rgbActiveFilters = 0;

    if (Parameters::run_checker_color_homogenity > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::run_checker_luminosity_diff > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::run_checker_texture_diff > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::run_checker_texture_backproject > 0)
    {
      rgbActiveFilters++;
    }


    std::vector<std::vector<float> > rgbProbabilitiesVector2D(rgbActiveFilters,
      std::vector<float>(depthHolesConveyor_.keyPoints.size(), 0.0));

    //!< check holes for debugging purposes
    RgbFilters::checkHoles(
      rgbImage_,
      wallsHistogram_,
      &depthHolesConveyor_,
      &rgbProbabilitiesVector2D);

    /*
     *for (int i = 0; i < rgbActiveFilters; i++)
     *{
     *  for (int j = 0; j < depthHolesConveyor_.keyPoints.size(); j++)
     *  {
     *    ROS_ERROR("P[%d %d] = %f", i, j, probabilitiesVector2D[i][j]);
     *  }
     *}
     */



    #ifdef DEBUG_SHOW
    std::vector<std::string> msgs;
    std::vector<cv::Mat> imgs;
    if(Parameters::debug_show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Final keypoints");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showKeypoints(
          msg,
          interpolatedDepthImage_,
          -1,
          depthHolesConveyor_.keyPoints)
        );
    }
    if(Parameters::debug_show_find_holes)
    {
      Visualization::multipleShow("depthCandidateHolesCallback function",
        imgs, msgs, Parameters::debug_show_find_holes_size, 1);
    }
    #endif
    //!< Processing complete.

    #ifdef DEBUG_TIME
    Timer::tick("processCandidateHoles");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Unpacks the the HoleFilters::HolesConveyor struct for the
    candidate holes, the interpolated depth image and the point cloud
    from the vision_communications::DepthCandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::DepthCandidateHolesVectorMsg&] The input
    candidate holes message obtained through the depth node
    @param[out] conveyor [HoleFilters::HolesConveyor*] The output conveyor
    struct
    @param[out] pointCloudXYZ [PointCloudXYZPtr*] The output point cloud
    @param[out] interpolatedDepthImage [cv::Mat*] The output interpolated
    depth image
    @return void
   **/
  void HoleFusion::unpackDepthMessage(
    const vision_communications::DepthCandidateHolesVectorMsg& holesMsg,
    HoleFilters::HolesConveyor* conveyor, PointCloudXYZPtr* pointCloudXYZ,
    cv::Mat* interpolatedDepthImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackDepthMessage", "depthCandidateHolesCallback");
    #endif

    //!< Recreate the conveyor
    fromCandidateHoleMsgToConveyor(holesMsg.candidateHoles, conveyor);

    //!< If the depth node has detected holes
    if (conveyor->keyPoints.size() > 0)
    {
      //!< Unpack the interpolated depth image
      MessageConversions::extractDepthImageFromMessageContainer(
        holesMsg,
        interpolatedDepthImage,
        sensor_msgs::image_encodings::TYPE_32FC1);

      //!< Unpack the point cloud
      MessageConversions::extractPointCloudXYZFromMessageContainer(holesMsg,
        pointCloudXYZ);
    }
    #ifdef DEBUG_TIME
    Timer::tick("unpackDepthMessage");
    #endif
  }



  /**
    @brief Unpacks the the HoleFilters::HolesConveyor struct for the
    candidate holes, the RGB image
    from the vision_communications::DepthCandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::RgbCandidateHolesVectorMsg&] The input
    candidate holes message obtained throught the RGB node
    @param[out] conveyor [HoleFilters::HolesConveyor*] The output conveyor
    struct
    @param[out] rgbImage [cv::Mat*] The output RGB image
    @return void
   **/
  void HoleFusion::unpackRgbMessage(
    const vision_communications::RgbCandidateHolesVectorMsg& holesMsg,
    HoleFilters::HolesConveyor* conveyor, cv::Mat* rgbImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackRgbMessage", "rgbCandidateHolesCallback");
    #endif

    //!< Recreate the conveyor
    fromCandidateHoleMsgToConveyor(holesMsg.candidateHoles, conveyor);

    //!< If the RGB node has detected holes
    if (conveyor->keyPoints.size() > 0)
    {
      //!< Unpack the RGB image
      MessageConversions::extractRgbImageFromMessageContainer(
        holesMsg,
        rgbImage,
        sensor_msgs::image_encodings::TYPE_32FC3);
    }
    #ifdef DEBUG_TIME
    Timer::tick("unpackRgbMessage");
    #endif
  }



  /**
    @brief Requests from the synchronizer to process a new point cloud
    @return void
   **/
  void HoleFusion::unlockSynchronizer()
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Sending unlock message");
    #endif

    std_msgs::Empty unlockMsg;
    unlockPublisher_.publish(unlockMsg);
  }

  /**
    @brief The function called when a parameter is changed
    @param[in] config
    [const pandora_vision_hole_detector::hole_fusion_cfgConfig&]
    @param[in] level [const uint32_t] The level (?)
    @return void
   **/
  void HoleFusion::parametersCallback(
    const pandora_vision_hole_detector::hole_fusion_cfgConfig &config,
    const uint32_t& level)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Parameters callback called");
    #endif

    Parameters::kanny_ratio =
      config.kanny_ratio;
    Parameters::kanny_kernel_size =
      config.kanny_kernel_size;
    Parameters::kanny_low_threshold =
      config.kanny_low_threshold;
    Parameters::kanny_blur_noise_kernel_size =
      config.kanny_blur_noise_kernel_size;
    Parameters::contrast_enhance_beta =
      config.contrast_enhance_beta;
    Parameters::contrast_enhance_alpha =
      config.contrast_enhance_alpha;
    Parameters::threshold_lower_value =
      config.threshold_lower_value;
    Parameters::adaptive_max_value =
      config.adaptive_max_value;
    Parameters::adaptive_method =
      config.adaptive_method;
    Parameters::adaptive_block_size =
      config.adaptive_block_size;
    Parameters::adaptive_c_value =
      config.adaptive_c_value;
    Parameters::blob_min_threshold =
      config.blob_min_threshold;
    Parameters::blob_max_threshold =
      config.blob_max_threshold;
    Parameters::blob_threshold_step =
      config.blob_threshold_step;
    Parameters::blob_min_area =
      config.blob_min_area;
    Parameters::blob_max_area =
      config.blob_max_area;
    Parameters::blob_min_convexity =
      config.blob_min_convexity;
    Parameters::blob_max_convexity =
      config.blob_max_convexity;
    Parameters::blob_min_inertia_ratio =
      config.blob_min_inertia_ratio;
    Parameters::blob_max_circularity =
      config.blob_max_circularity;
    Parameters::blob_min_circularity =
      config.blob_min_circularity;
    Parameters::blob_filter_by_color =
      config.blob_filter_by_color;
    Parameters::blob_filter_by_circularity =
      config.blob_filter_by_circularity;
    Parameters::bounding_box_min_area_threshold =
      config.bounding_box_min_area_threshold;
    Parameters::bounding_box_detection_method =
      config.bounding_box_detection_method;
    Parameters::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;
    Parameters::AB_to_MO_ratio =
      config.AB_to_MO_ratio;
    Parameters::interpolation_method =
      config.interpolation_method;

    Parameters::run_checker_depth_diff =
      config.run_checker_depth_diff;
    Parameters::run_checker_outline_of_rectangle =
      config.run_checker_outline_of_rectangle;
    Parameters::run_checker_depth_area =
      config.run_checker_depth_area;
    Parameters::run_checker_brushfire_outline_to_rectangle =
      config.run_checker_brushfire_outline_to_rectangle;
    Parameters::run_checker_depth_homogenity =
      config.run_checker_depth_homogenity;

    Parameters::rectangle_inflation_size =
      config.rectangle_inflation_size;
    Parameters::depth_difference =
      config.depth_difference;

    Parameters::run_checker_color_homogenity =
      config.run_checker_color_homogenity;
    Parameters::run_checker_luminosity_diff =
      config.run_checker_luminosity_diff;
    Parameters::run_checker_texture_diff =
      config.run_checker_texture_diff;
    Parameters::run_checker_texture_backproject =
      config.run_checker_texture_backproject;

    Parameters::segmentation_method =
      config.segmentation_method;
    Parameters::max_iterations =
      config.max_iterations;
    Parameters::num_points_to_exclude =
      config.num_points_to_exclude;
    Parameters::point_to_plane_distance_threshold =
      config.point_to_plane_distance_threshold;
    Parameters::scale_method =
      config.scale_method;
    Parameters::debug_show_find_holes =
      config.debug_show_find_holes;
    Parameters::debug_show_find_holes_size =
      config.debug_show_find_holes_size;
    Parameters::debug_time_find_holes =
      config.debug_time_find_holes;
    Parameters::debug_show_denoise_edges =
      config.debug_show_denoise_edges;
    Parameters::debug_show_denoise_edges_size =
      config.debug_show_denoise_edges_size;
    Parameters::debug_show_connect_pairs =
      config.debug_show_connect_pairs;
    Parameters::debug_show_connect_pairs_size =
      config.debug_show_connect_pairs_size;

    Parameters::debug_show_get_shapes_clear_border  =
      config.debug_show_get_shapes_clear_border;
    Parameters::debug_show_get_shapes_clear_border_size =
      config.debug_show_get_shapes_clear_border_size;

    Parameters::debug_show_check_holes =
      config.debug_show_check_holes;
    Parameters::debug_show_check_holes_size =
      config.debug_show_check_holes_size;

    Parameters::minimum_curve_points =
      config.minimum_curve_points;

    Parameters::match_texture_threshold =
      config.match_texture_threshold;

    Parameters::non_zero_points_in_box_blob_histogram =
      config.non_zero_points_in_box_blob_histogram;

    Parameters::num_bins_threshold =
      config.num_bins_threshold;

    Parameters::number_of_hue_bins =
      config.number_of_hue_bins;
    Parameters::number_of_saturation_bins =
      config.number_of_saturation_bins;
    Parameters::number_of_value_bins =
      config.number_of_value_bins;
  }

} // namespace pandora_vision

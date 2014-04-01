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
    @brief Recreates the HolesConveyor struct for the
    candidate holes from the
    vision_communications::CandidateHolerMsg message
    @param[in]candidateHolesVector
    [const std::vector<vision_communications::CandidateHoleMsg>&]
    The input candidate holes
    @param[out] conveyor [HolesConveyor*] The output conveyor
    struct
    @return void
   **/
  void HoleFusion::fromCandidateHoleMsgToConveyor(
    const std::vector<vision_communications::CandidateHoleMsg>&
    candidateHolesVector,
    HolesConveyor* conveyor)
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
    ROS_INFO("Processing candidate holes");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("processCandidateHoles", "depthCandidateHolesCallback");
    #endif

    fuseHoles();



    #ifdef DEBUG_TIME
    Timer::tick("processCandidateHoles");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Unpacks the the HolesConveyor struct for the
    candidate holes, the interpolated depth image and the point cloud
    from the vision_communications::DepthCandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::DepthCandidateHolesVectorMsg&] The input
    candidate holes message obtained through the depth node
    @param[out] conveyor [HolesConveyor*] The output conveyor
    struct
    @param[out] pointCloudXYZ [PointCloudXYZPtr*] The output point cloud
    @param[out] interpolatedDepthImage [cv::Mat*] The output interpolated
    depth image
    @return void
   **/
  void HoleFusion::unpackDepthMessage(
    const vision_communications::DepthCandidateHolesVectorMsg& holesMsg,
    HolesConveyor* conveyor, PointCloudXYZPtr* pointCloudXYZ,
    cv::Mat* interpolatedDepthImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackDepthMessage", "depthCandidateHolesCallback");
    #endif

    //!< Recreate the conveyor
    fromCandidateHoleMsgToConveyor(holesMsg.candidateHoles, conveyor);

    //!< Unpack the interpolated depth image
    MessageConversions::extractDepthImageFromMessageContainer(
      holesMsg,
      interpolatedDepthImage,
      sensor_msgs::image_encodings::TYPE_32FC1);

    //!< Unpack the point cloud
    MessageConversions::extractPointCloudXYZFromMessageContainer(holesMsg,
      pointCloudXYZ);

    #ifdef DEBUG_TIME
    Timer::tick("unpackDepthMessage");
    #endif
  }



  /**
    @brief Unpacks the the HolesConveyor struct for the
    candidate holes, the RGB image
    from the vision_communications::DepthCandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::RgbCandidateHolesVectorMsg&] The input
    candidate holes message obtained throught the RGB node
    @param[out] conveyor [HolesConveyor*] The output conveyor
    struct
    @param[out] rgbImage [cv::Mat*] The output RGB image
    @return void
   **/
  void HoleFusion::unpackRgbMessage(
    const vision_communications::RgbCandidateHolesVectorMsg& holesMsg,
    HolesConveyor* conveyor, cv::Mat* rgbImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackRgbMessage", "rgbCandidateHolesCallback");
    #endif

    //!< Recreate the conveyor
    fromCandidateHoleMsgToConveyor(holesMsg.candidateHoles, conveyor);

    //!< Unpack the RGB image
    MessageConversions::extractRgbImageFromMessageContainer(
      holesMsg,
      rgbImage,
      sensor_msgs::image_encodings::TYPE_8UC3);

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
    @brief Runs candidate holes obtained through Depth and RGB analysis
    through selected filters, from a respective viewpoint (keypoints
    obtained through Depth analysis are checked against Depth-based
    filters, etc). Probabilities for each candidate hole and filter
    are printed in the console, with an order specified by the
    hole_fusion_cfg of the dynamic reconfigure utility
    @param void
    @return void
   **/
  void HoleFusion::viewRespectiveProbabilities()
  {
    #ifdef DEBUG_SHOW
    ROS_ERROR("===========================================");
    ROS_ERROR("#Depth keypoints : %zu", depthHolesConveyor_.keyPoints.size());
    ROS_ERROR("#RGB keypoints : %zu", rgbHolesConveyor_.keyPoints.size());
    ROS_ERROR("-------------------------------------------");
    #endif

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

    #ifdef DEBUG_SHOW
    if (depthHolesConveyor_.keyPoints.size() > 0)
    {
      ROS_ERROR("Depth : Candidate Holes' probabilities");
      for (int j = 0; j < depthHolesConveyor_.keyPoints.size(); j++)
      {
        std::string probsString;
        for (int i = 0; i < depthActiveFilters; i++)
        {
          probsString += TOSTR(depthProbabilitiesVector2D[i][j]) + " | ";
        }

        ROS_ERROR("P_%d = %s", j, probsString.c_str());
        ROS_ERROR("------------------");
      }
    }
    #endif

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
      std::vector<float>(rgbHolesConveyor_.keyPoints.size(), 0.0));

    //!< check holes for debugging purposes
    RgbFilters::checkHoles(
      rgbImage_,
      wallsHistogram_,
      &rgbHolesConveyor_,
      &rgbProbabilitiesVector2D);

    #ifdef DEBUG_SHOW
    ROS_ERROR("-------------------------------------------");
    if (rgbHolesConveyor_.keyPoints.size() > 0)
    {
      ROS_ERROR("RGB: Candidate Holes' probabilities");
      for (int j = 0; j < rgbHolesConveyor_.keyPoints.size(); j++)
      {
        std::string probsString;
        for (int i = 0; i < rgbActiveFilters; i++)
        {
          probsString += TOSTR(rgbProbabilitiesVector2D[i][j]) + " | ";
        }

        ROS_ERROR("P_%d = %s", j, probsString.c_str());
        ROS_ERROR("------------------");
      }
    }
    #endif


    #ifdef DEBUG_SHOW
    if(Parameters::debug_show_find_holes) // Debug
    {
      //!< Push back the identities of each keypoint originated from
      //!< depth analysis
      std::vector<std::string> depthMsgs;
      for (int i = 0; i < depthHolesConveyor_.keyPoints.size(); i++)
      {
        depthMsgs.push_back(TOSTR(i));
      }

      Visualization::showHoles(
        "Depth : Final KeyPoints",
        interpolatedDepthImage_,
        1,
        depthHolesConveyor_.keyPoints,
        depthHolesConveyor_.rectangles,
        depthMsgs,
        depthHolesConveyor_.outlines);

      //!< Push back the identities of each keypoint originated from
      //!< RGB analysis
      std::vector<std::string> rgbMsgs;
      for (int i = 0; i < rgbHolesConveyor_.keyPoints.size(); i++)
      {
        rgbMsgs.push_back(TOSTR(i));
      }

      Visualization::showHoles(
        "RGB: Final KeyPoints",
        rgbImage_,
        1,
        rgbHolesConveyor_.keyPoints,
        rgbHolesConveyor_.rectangles,
        rgbMsgs,
        rgbHolesConveyor_.outlines);
    }
    #endif
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

    //!< Kanny parameters
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

    //!< Threshold parameters
    Parameters::threshold_lower_value =
      config.threshold_lower_value;

    //!< Blob detection parameters
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

    //!< Bounding boxes parameters
    Parameters::bounding_box_min_area_threshold =
      config.bounding_box_min_area_threshold;

    //!< The bounding box detection zzmethod
    //!< 0 for detecting by means of brushfire starting
    //!< from the keypoint of the blob
    //!< 1 for detecting by means of contours around the edges of the blob
    Parameters::bounding_box_detection_method =
      config.bounding_box_detection_method;

    //!< When using raycast instead of brushfire to find the (approximate here)
    //!< outline of blobs, raycast_keypoint_partitions dictates the number of
    //!< rays, or equivalently, the number of partitions in which the blob is
    //!< partitioned in search of the blob's borders
    Parameters::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;

    //<! Loose ends connection parameters
    Parameters::AB_to_MO_ratio =
      config.AB_to_MO_ratio;
    Parameters::minimum_curve_points =
      config.minimum_curve_points;

    ////!< Interpolation parameters

    //!< The interpolation method for noise removal
    //!< 0 for averaging the pixel's neighbor values
    //!< 1 for brushfire near
    //!< 2 for brushfire far
    Parameters::interpolation_method =
      config.interpolation_method;
    //!< Method to scale the CV_32FC1 image to CV_8UC1
    Parameters::scale_method =
      config.scale_method;

    //!< Hole checkers
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


    //!< Debug
    Parameters::debug_show_find_holes =
      config.debug_show_find_holes;
    Parameters::debug_show_find_holes_size =
      config.debug_show_find_holes_size;
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


    //!< Texture parameters
    //!< The threshold for texture matching
    Parameters::match_texture_threshold =
      config.match_texture_threshold;

    Parameters::non_zero_points_in_box_blob_histogram =
      config.non_zero_points_in_box_blob_histogram;

    //!<Color homogenity parameters
    Parameters::num_bins_threshold =
      config.num_bins_threshold;

    //!< Histogram parameters
    Parameters::number_of_hue_bins =
      config.number_of_hue_bins;
    Parameters::number_of_saturation_bins =
      config.number_of_saturation_bins;
    Parameters::number_of_value_bins =
      config.number_of_value_bins;
  }



  void HoleFusion::fuseHoles()
  {
    //!< Merge the conveyors from the RGB and Depth sources
    HolesConveyor rgbdHolesConveyor;
    HolesConveyorUtils::merge(depthHolesConveyor_, rgbHolesConveyor_,
      &rgbdHolesConveyor);

    bool isFuseComplete = false;
    while(!isFuseComplete)
    {
      //!< Initialize needed variables

      //!< The index of the candidate hole that will
      //!< {assimilate, amalgamate, connect} the j-th candidate hole
      int i = 0;

      //!< The index of the candidate hole that will be
      //!< {assimilated, amalgamated, connected} by / with
      //!< the i-th candidate hole
      int j = 1;

      //!< The identifier of the merge process
      //!< {(assimilate, 0), (amalgamate, 1), (connect, 2)}
      int mergeProcessId = 0;

      if (i >= rgbdHolesConveyor.keyPoints.size() ||
        j >= rgbdHolesConveyor.keyPoints.size())
      {
        return;
      }

      bool isAble = false;
      if (mergeProcessId == 0)
      {
        //!< Is the i-th candidate hole able to assimilate the
        //!< j-th candidate hole?
        isAble = GenericFilters::isCapableOfAssimilating(i,
          rgbdHolesConveyor, j, rgbdHolesConveyor);
      }
      if (mergeProcessId == 1)
      {
        //!< Is the i-th candidate hole able to amalgamate the
        //!< j-th candidate hole?
        isAble = GenericFilters::isCapableOfAmalgamating(i,
          rgbdHolesConveyor, j, rgbdHolesConveyor);
      }
      if (mergeProcessId == 2)
      {
        //!< Is the j-th candidate hole able to be connected with the
        //!< i-th candidate hole?
        isAble = GenericFilters::isCapableOfConnecting(i,
          rgbdHolesConveyor, j, rgbdHolesConveyor, pointCloudXYZ_);
      }

      if (isAble)
      {
        //!< Copy the original holes conveyor to a temp one.
        //!< The temp one will be tested through the hole filters
        //!< On success, temp will replace rgbdHolesConveyor,
        //!< on failure, rgbdHolesConveyor will remain unchanged
        HolesConveyor tempHolesConveyor;
        HolesConveyorUtils::copyTo(rgbdHolesConveyor, &tempHolesConveyor);

        if (mergeProcessId == 0)
        {
          //!< Delete the j-th candidate hole
          GenericFilters::assimilateOnce(j, &tempHolesConveyor);
        }
        else if (mergeProcessId == 1)
        {
          //!< Delete the j-th candidate hole,
          //!< alter the i-th candidate hole so that it has amalgamated
          //!< the j-th candidate hole
          GenericFilters::amalgamateOnce(i, &tempHolesConveyor,
            j, &tempHolesConveyor);
        }
        else if (mergeProcessId == 2)
        {
          //!< Delete the j-th candidate hole,
          //!< alter the i-th candidate hole so that it has been connected
          //!< with the j-th candidate hole
          GenericFilters::connectOnce(i, &tempHolesConveyor,
            j, &tempHolesConveyor);
        }

        //!< Obtain the i-th candidate hole in order for it
        //!< to be checked against the selected filters
        HolesConveyor ithHole =
          HolesConveyorUtils::getHole(tempHolesConveyor, i);

        //!< Determines the selected filters execution
        std::map<int, int> filtersOrder;

        //!< Depth diff runs first
        filtersOrder[1] = 1;

        //!< Depth / Area runs second
        filtersOrder[2] = 3;

        //!< Bounding rectangle's plane constitution runs third
        filtersOrder[3] = 2;


        std::vector<cv::Mat> imgs;
        std::vector<std::string> msgs;

        std::vector<std::vector<float> >probabilitiesVector(3,
          std::vector<float>(1, 0.0));

        int counter = 0;
        for (std::map<int, int>::iterator o_it = filtersOrder.begin();
          o_it != filtersOrder.end(); ++o_it)
        {
          DepthFilters::applyFilter(
            o_it->second,
            interpolatedDepthImage_,
            pointCloudXYZ_,
            &ithHole,
            Parameters::rectangle_inflation_size,
            &probabilitiesVector.at(counter),
            &imgs,
            &msgs);

          counter++;
        } //!< o_it iterator ends

        float dd = probabilitiesVector[0][0];
        float da = probabilitiesVector[1][0];
        float pc = probabilitiesVector[2][0];

        //!< Probabilities threshold for merge acceptance
        if (dd > 0.5 && da > 0.5 && pc > 0.7)
        {
          HolesConveyorUtils::replace(tempHolesConveyor, &rgbdHolesConveyor);
        }
      }
    }
  }

} // namespace pandora_vision

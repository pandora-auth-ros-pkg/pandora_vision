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

    //!< Subscribe to the topic where the synchronizer node publishes
    //!< the point cloud
    pointCloudSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/depth/points", 1,
      &HoleFusion::pointCloudCallback, this);

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
    @brief Applies a merging operation of @param operationId, until
    every candidate hole, even as it changes through the various merges that
    happen, has been merged with every candidate hole that can be merged
    with it.
    @param[in][out] rgbdHolesConveyor [HolesConveyor*] The unified rgb-d
    candidate holes conveyor
    @param[in] operationId [const int&] The identifier of the merging
    process. Values: 0 for assimilation, 1 for amalgamation and
    2 for connecting
    @return void
   **/
  void HoleFusion::applyMergeOperation(HolesConveyor* rgbdHolesConveyor,
    const int& operationId)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyMergeOperation", "processCandidateHoles");
    #endif

    //!< If there are no candidate holes,
    //!< or there is only one candidate hole,
    //!< there is no meaning to this operation
    if (rgbdHolesConveyor->keyPoints.size() < 2)
    {
      return;
    }


    //!< The holesMaskSetVector vector is used in the merging processes
    std::vector<std::set<unsigned int> > holesMasksSetVector;
    createHolesMasksSetVector(*rgbdHolesConveyor, interpolatedDepthImage_,
      &holesMasksSetVector);


    //!< A vector that indicates when a specific hole has finished
    //!< examining all the other holes in the conveyor for merging.
    //!< Initialized at 0 for all conveyor entries, the specific hole
    //!< that corresponds to a vector's entry is given a 1 when it has
    //!< finished examining all other holes.
    std::vector<int> finishVector(rgbdHolesConveyor->keyPoints.size(), 0);

    //!< The index of the candidate hole that will
    //!< {assimilate, amalgamate, connect} the passiveId-th candidate hole.
    //!< The activeId always has a value of 0 due to the implementation's
    //!< rationale: The candidate hole that examines each hole in the
    //!< rgbdHolesConveyor is always the first one. When it has finished,
    //!< it goes back into the rgbdHolesConveyor, at the last position
    const int activeId = 0;

    //!< The index of the candidate hole that will be
    //!< {assimilated, amalgamated, connected} by / with
    //!< the activeId-th candidate hole
    int passiveId = 1;

    bool isFuseComplete = false;
    while(!isFuseComplete)
    {
      //!< Is the activeId-th candidate hole able to
      //!< {assimilate, amalgamate, connect to} the passiveId-th candidate hole?
      bool isAble = false;

      if (operationId == 0)
      {
        //!< Is the activeId-th candidate hole able to assimilate the
        //!< passiveId-th candidate hole?
        isAble = HoleMerger::isCapableOfAssimilating(
          holesMasksSetVector[activeId],
          holesMasksSetVector[passiveId]);
      }
      if (operationId == 1)
      {
        //!< Is the activeId-th candidate hole able to amalgamate the
        //!< passiveId-th candidate hole?
        isAble = HoleMerger::isCapableOfAmalgamating(
          holesMasksSetVector[activeId],
          holesMasksSetVector[passiveId]);
      }
      if (operationId == 2)
      {
        //!< Is the passiveId-th candidate hole able to be connected with the
        //!< activeId-th candidate hole?
        isAble = HoleMerger::isCapableOfConnecting(*rgbdHolesConveyor,
          activeId,
          passiveId,
          holesMasksSetVector[activeId],
          holesMasksSetVector[passiveId],
          pointCloudXYZ_);
      }



      if (isAble)
      {
        //!< Copy the original holes conveyor to a temp one.
        //!< The temp one will be tested through the hole filters
        //!< On success, temp will replace rgbdHolesConveyor,
        //!< on failure, rgbdHolesConveyor will remain unchanged
        HolesConveyor tempHolesConveyor;
        HolesConveyorUtils::copyTo(*rgbdHolesConveyor, &tempHolesConveyor);

        //!< Copy the original holes masks set to a temp one.
        //!< If the temp conveyor is tested successfully through the hole
        //!< filters, temp will replace the original.
        //!< On failure, the original will remain unchanged
        std::vector<std::set<unsigned int> > tempHolesMasksSetVector;
        tempHolesMasksSetVector = holesMasksSetVector;

        if (operationId == 0)
        {
          //!< Delete the passiveId-th candidate hole
          HolesConveyorUtils::removeHole(&tempHolesConveyor, passiveId);
        }
        else if (operationId == 1)
        {
          //!< Delete the passiveId-th candidate hole,
          //!< alter the activeId-th candidate hole so that it has amalgamated
          //!< the passiveId-th candidate hole
          HoleMerger::amalgamateOnce(&tempHolesConveyor,
            activeId,
            &tempHolesMasksSetVector[activeId],
            tempHolesMasksSetVector[passiveId],
            interpolatedDepthImage_);

          HolesConveyorUtils::removeHole(&tempHolesConveyor, passiveId);
        }
        else if (operationId == 2)
        {
          //!< Delete the passiveId-th candidate hole,
          //!< alter the activeId-th candidate hole so that it has been
          //!< connected with the passiveId -th candidate hole
          HoleMerger::connectOnce(&tempHolesConveyor,
            activeId, passiveId,
            &tempHolesMasksSetVector[activeId],
            tempHolesMasksSetVector[passiveId]);

          HolesConveyorUtils::removeHole(&tempHolesConveyor, passiveId);
        }

        //!< Since the {assimilator, amalgamator, connector} is able,
        //!< delete the assimilable's entries in the vectors needed
        //!< for filtering and merging
        tempHolesMasksSetVector.erase(
          tempHolesMasksSetVector.begin() + passiveId);


        //!< Obtain the activeId-th candidate hole in order for it
        //!< to be checked against the selected filters
        HolesConveyor ithHole =
          HolesConveyorUtils::getHole(tempHolesConveyor, activeId);




        //!< TODO make more flexible
        //!< Determines the selected filters execution
        std::map<int, int> filtersOrder;

        //!< Depth diff runs first
        filtersOrder[1] = 1;

        //!< Depth / Area runs second
        filtersOrder[2] = 3;

        //!< Bounding rectangle's plane constitution runs third
        filtersOrder[3] = 2;

        //!< Create the necessary vectors for each hole checker and
        //!< merger used
        std::vector<cv::Mat> imgs;
        std::vector<std::string> msgs;
        std::vector<std::vector<cv::Point2f> > rectanglesVector;
        std::vector<int> rectanglesIndices;
        std::vector<std::set<unsigned int> > intermediatePointsSetVector;

        //!< The inflated rectangles vector is used in the
        //!< checkHolesDepthDiff and checkHolesRectangleEdgesPlaneConstitution
        //!< checkers
        createInflatedRectanglesVector(
          ithHole,
          interpolatedDepthImage_,
          Parameters::rectangle_inflation_size,
          &rectanglesVector,
          &rectanglesIndices);



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
            ithHole,
            tempHolesMasksSetVector,
            rectanglesVector,
            rectanglesIndices,
            intermediatePointsSetVector,
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
          //!< Since the tempHolesConveyor's ithHole has been positively tested,
          //!< the tempHolesConveyor is now the new rgbdHolesConveyor
          HolesConveyorUtils::replace(tempHolesConveyor, rgbdHolesConveyor);


          //!< ..and the new holesMasksSetVector is the positively tested
          //!< temp one
          holesMasksSetVector = tempHolesMasksSetVector;


          //!< Delete the passiveId-th entry of the finishVector since the
          //!< passiveId-th hole has been absorbed by the activeId-th hole
          finishVector.erase(finishVector.begin() + passiveId);


          //!< Because of the merge happening, the activeId-th
          //!< candidate hole must re-examine all the other holes
          passiveId = 1;
        }
        else //!< rgbdHolesConveyor remains unchanged
        {
          //!< passiveId-th hole not merged. let's see about the next one
          passiveId++;
        }
      }
      else //!< isAble == false
      {
        //!< passiveId-th hole not merged. let's see about the next one
        passiveId++;
      }

      //!< If the passiveId-th hole was the last one to be checked for merge,
      //!< the one doing the merge is rendered obsolete, so go to the next one
      //!< by moving the current activeId-th candidate hole to the back
      //!< of the rgbdHolesConveyor. This way the new activeId-th candidate
      //!< hole still has a value of 0, but now points to the candidate hole
      //!< next to the one that was moved back
      if (passiveId >= rgbdHolesConveyor->keyPoints.size())
      {
        //!< No meaning moving to the back of the rgbdHolesConveyor if
        //!< there is only one candidate hole
        if (rgbdHolesConveyor->keyPoints.size() > 1)
        {
          //!< activeId-th hole candidate finished examining the rest of the
          //!< hole candidates. move it to the back of the rgbdHolesConveyor
          HolesConveyorUtils::append(
            HolesConveyorUtils::getHole(*rgbdHolesConveyor, activeId),
            rgbdHolesConveyor);

          //!< Remove the activeId-th candidate hole from its former position
          HolesConveyorUtils::removeHole(rgbdHolesConveyor, activeId);


          //!< Remove the activeId-th set from its position and append it
          holesMasksSetVector.push_back(holesMasksSetVector[activeId]);
          holesMasksSetVector.erase(holesMasksSetVector.begin() + activeId);


          //!< Since the candidate hole was appended at the end of the
          //!< rgbdHolesConveyor, the finish vector needs to be shifted
          //!< once to the left because the value 1 is always set at the end
          //!< of the finishVector vector. See below.
          std::rotate(finishVector.begin(), finishVector.begin() + 1,
            finishVector.end());

          //!< Return the passive's candidate hole identifier to the
          //!< next of the active's candidate hole identifier, which is 0
          passiveId = 1;
        }

        //!< Since the ith candidate hole was appended at the end of the
        //!< rgbdHolesConveyor, the position to which it corresponds in the
        //!< finishVector is at the end of the vector.
        //!< Place the value of 1 in the last position, indicating that the
        //!< previously activeId-th candidate hole has finished examining all
        //!< other candidate holes for merging
        std::vector<int>::iterator finishVectorIterator =
          finishVector.end() - 1;

        *finishVectorIterator = 1;

        //!< Count how many aces there are in the finishVector
        //!< If they amount to the size of the vector, that means that
        //!< each hole has finished examining the others, and the current
        //!< operation is complete
        int numAces = 0;
        for (int i = 0; i < finishVector.size(); i++)
        {
          numAces += finishVector[i];
        }

        if (numAces == finishVector.size())
        {
          isFuseComplete = true;
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("applyMergeOperation");
    #endif
  }



  /**
    @brief Each Depth and RGB filter requires the construction of a set
    of vectors which uses to determine the validity of each hole.
    The total number of vectors is finite; every filter uses vectors from
    this pool of vectors. This method centrally constructs the necessary
    vectors in runtime, depending on which filters are commanded to run
    @param[in] conveyor [const HolesConeveyor&] The candidate holes
    from which each element of the vector will be constructed
    @param[in] image [const cv::Mat&] An image needed for its size
    @param[in] inflationSize [const int&] The bounding rectangles
    inflation size
    @param[out] holesMasksImageVector [std::vector<cv::Mat>*]
    A vector containing an image (the mask) for each hole
    @param[out] holesMasksSetVector [std::vector<std::set<unsigned int> >*]
    A vector that holds sets of points's indices; each set holds the
    indices of the points inside the outline of each hole
    @param[out] inflatedRectanglesVector
    [std::vector<std::vector<cv::Point2f> >*] The vector that holds the
    vertices of the in-image-bounds inflated rectangles
    @param[out] inflatedRectanglesIndices [std::vector<int>*]
    The vector that holds the indices of the original holes whose
    inflated bounding rectangles is within the image's bounds.
    @param[out] intermediatePointsImageVector [std::vector<cv::Mat>*]
    A vector that holds the image of the intermediate points between
    a hole's outline and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @param[out] intermediatePointsSetVector [std::vector<std::set<int> >*]
    A vector that holds the intermediate points' between a hole's outline
    and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @return void
   **/
  void HoleFusion::createCheckerRequiredVectors(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const int& inflationSize,
    std::vector<cv::Mat>* holesMasksImageVector,
    std::vector<std::set<unsigned int> >* holesMasksSetVector,
    std::vector<std::vector<cv::Point2f> >* inflatedRectanglesVector,
    std::vector<int>* inflatedRectanglesIndices,
    std::vector<cv::Mat>* intermediatePointsImageVector,
    std::vector<std::set<unsigned int> >* intermediatePointsSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCheckerRequiredVectors", "sift");
    #endif

    bool enable_holesMasksImageVector = false;
    bool enable_holesMasksSetVector = false;
    bool enable_inflatedRectanglesVectorAndIndices = false;
    bool enable_intermediatePointsImageVector = false;
    bool enable_intermediatePointsSetVector = false;

    //!< The color homogeneity filter requires a vector of holes' masks
    //!< that will be used to extract their histograms
    if (Parameters::run_checker_color_homogeneity > 0)
    {
      enable_holesMasksImageVector = true;
    }

    //!< The luminosity diff filter requires the set of points' indices
    //!< that are inside a hole's outline,
    //!< the set of points' indices that are outside a hole's outline
    //!< but inside its (inflated) bounding rectangle
    //!< and the inflated rectangles and indices of the respective
    //!< valid keypoints
    if (Parameters::run_checker_luminosity_diff > 0)
    {
      enable_holesMasksSetVector = true;
      enable_intermediatePointsSetVector = true;
      enable_inflatedRectanglesVectorAndIndices = true;
    }

    //!< The texture diff filter requires the construction of an image mask
    //!< vector for the points inside holes' outline and of image and set
    //!< masks for the points outside holes' outline but inside their (inflated)
    //!< bounding box
    //!< as it checks for texture metrics difference between the
    //!< histograms of the points inside a hole's outline and outside
    //!< the hole's outline but inside its (inflated) bounding rectangle
    if (Parameters::run_checker_texture_diff > 0)
    {
      enable_holesMasksImageVector = true;
      enable_intermediatePointsSetVector = true;
      enable_intermediatePointsImageVector = true;
      enable_inflatedRectanglesVectorAndIndices = true;
    }

    //!< The texture backproject filter uses two sets: they respectively contain
    //!< the indices of points inside holes' outlines and the indices of points
    //!< outside holes' outlines but inside their (inflated) bounding rectangle.
    //!< Hence, we also need the construction of inflated rectangles' vectors
    if (Parameters::run_checker_texture_backproject > 0)
    {
      enable_holesMasksSetVector = true;
      enable_intermediatePointsSetVector = true;
      enable_inflatedRectanglesVectorAndIndices = true;
    }

    //!< The depth diff filter requires only the contruction of the vectors that
    //!< have to do with the inflation of holes' rectangles
    if (Parameters::run_checker_depth_diff > 0)
    {
      enable_inflatedRectanglesVectorAndIndices = true;
    }

    //!< The depth/area filter requires only the construction of sets that
    //!< hold the indices of points inside holes' outlines
    if (Parameters::run_checker_depth_area > 0)
    {
      enable_holesMasksSetVector = true;
    }

    //!< The intermediate points plane constitution filter requires exactly
    //!< the construction of vectors pertaining to holes' inflation and
    //!< and a vector of sets of indices of points between holes' outline and
    //!< their respective (inflated) bounding rectangle
    if (Parameters::run_checker_brushfire_outline_to_rectangle > 0)
    {
      enable_intermediatePointsSetVector = true;
      enable_inflatedRectanglesVectorAndIndices = true;
    }

    //!< The outline of rectangle plane constitution filter requires
    //!< the construction of vectors pertaining to holes' inflation
    if (Parameters::run_checker_outline_of_rectangle > 0)
    {
      enable_inflatedRectanglesVectorAndIndices = true;
    }

    //!< The depth homogeneity filter requires the construction of sets of
    //!< points' indices; these points are the ones inside holes' outlines
    if (Parameters::run_checker_depth_homogeneity > 0)
    {
      enable_holesMasksSetVector = true;
    }


    //!< Create the necessary resources

    if (enable_holesMasksImageVector && !enable_holesMasksSetVector)
    {
      createHolesMasksImageVector(conveyor, image, holesMasksImageVector);
    }

    if (enable_holesMasksSetVector && !enable_holesMasksImageVector)
    {
      createHolesMasksSetVector(conveyor, image, holesMasksSetVector);
    }

    if (enable_holesMasksSetVector && enable_holesMasksImageVector)
    {
      createHolesMasksVectors(conveyor, image,
        holesMasksImageVector, holesMasksSetVector);
    }

    if (enable_inflatedRectanglesVectorAndIndices)
    {
      createInflatedRectanglesVector(conveyor,
        interpolatedDepthImage_,
        inflationSize,
        inflatedRectanglesVector,
        inflatedRectanglesIndices);
    }

    if (enable_intermediatePointsImageVector &&
      !enable_intermediatePointsSetVector)
    {
      //!< The intermediate points images vector depends on the
      //!< inflated rectangles vectors
      if (!enable_inflatedRectanglesVectorAndIndices)
      {
        createInflatedRectanglesVector(conveyor,
          interpolatedDepthImage_,
          inflationSize,
          inflatedRectanglesVector,
          inflatedRectanglesIndices);
      }

      createIntermediateHolesPointsImageVector(conveyor,
        interpolatedDepthImage_,
        *inflatedRectanglesVector,
        *inflatedRectanglesIndices,
        intermediatePointsImageVector);
    }

    if (enable_intermediatePointsSetVector &&
      !enable_intermediatePointsImageVector)
    {
      //!< The intermediate points set vector depends on the
      //!< inflated rectangles vectors
      if (!enable_inflatedRectanglesVectorAndIndices)
      {
        createInflatedRectanglesVector(conveyor,
          interpolatedDepthImage_,
          inflationSize,
          inflatedRectanglesVector,
          inflatedRectanglesIndices);
      }

      createIntermediateHolesPointsSetVector(conveyor,
        interpolatedDepthImage_,
        *inflatedRectanglesVector,
        *inflatedRectanglesIndices,
        intermediatePointsSetVector);
    }

    if (enable_intermediatePointsSetVector &&
      enable_intermediatePointsImageVector)
    {
      //!< The intermediate points set vector depends on the
      //!< inflated rectangles vectors
      if (!enable_inflatedRectanglesVectorAndIndices)
      {
        createInflatedRectanglesVector(conveyor,
          interpolatedDepthImage_,
          inflationSize,
          inflatedRectanglesVector,
          inflatedRectanglesIndices);
      }

      createIntermediateHolesPointsVectors(conveyor,
        interpolatedDepthImage_,
        *inflatedRectanglesVector,
        *inflatedRectanglesIndices,
        intermediatePointsImageVector,
        intermediatePointsSetVector);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createCheckerRequiredVectors");
    #endif
  }



  /**
    @brief Some hole checkers require the construction of a hole's mask,
    that is, the pixels inside the hole; either in cv::Mat form or in
    a set form which contains points' indices.
    Construct each form here; this method makes it possible to brushfire
    once for every hole, instead of twice, if the image and set vectors
    are needed
    @param[in] conveyor [const HolesConveyor&] The conveyor of holes
    @param[in] image [const cv::Mat&] An image required only for the
    masks' size
    @param[out] holesMasksImageVector [std::vector<cv::Mat>*]
    A vector containing an image (the mask) for each hole
    @param[out] holesMasksSetVector [std::vector<std::set<unsigned int> >*]
    A vector that holds sets of points;
    each set holds the inside points of each hole
    @return void
   **/
  void HoleFusion::createHolesMasksVectors(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    std::vector<cv::Mat>* holesMasksImageVector,
    std::vector<std::set<unsigned int> >* holesMasksSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createHolesMasksVectors", "createCheckerRequiredVectors");
    #endif

    for (unsigned int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      //!< The current hole's image mask
      cv::Mat holeMask = cv::Mat::zeros(image.size(), CV_8UC1);

      //!< The set of points' indices inside the hole's outline
      std::set<unsigned int> holeMaskSet;


      //!< A pointer to the hole mask image
      unsigned char* ptr = holeMask.ptr();

      //!< Draw the outline of the i-th hole onto holeMask
      for(unsigned int j = 0; j < conveyor.outlines[i].size(); j++)
      {
        holeMask.at<unsigned char>(
          conveyor.outlines[i][j].y, conveyor.outlines[i][j].x) = 255;
      }


      cv::Point2f keypoint(
        conveyor.keyPoints[i].pt.x, conveyor.keyPoints[i].pt.y);

      //!< Brushfire from the keypoint to the hole's outline
      //!< to obtain the points inside the hole's outline
      BlobDetection::brushfirePoint(keypoint, &holeMask, &holeMaskSet);

      holesMasksSetVector->push_back(holeMaskSet);

      //!< Draw the current hole's mask
      for (std::set<unsigned int>::iterator it = holeMaskSet.begin();
        it != holeMaskSet.end(); it++)
      {
        ptr[*it] = 255;
      }

      holesMasksImageVector->push_back(holeMask);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createHolesMasksVectors");
    #endif
  }



  /**
    @brief Some hole checkers require the construction of a hole's mask,
    that is, the pixels inside the hole with a value of
    value 255 while the background pixels are with 0 value.
    Construct each mask here, instead of in each checker.
    @param[in] conveyor [const HolesConveyor&] The conveyor of holes
    @param[in] image [const cv::Mat&] An image required only for the
    masks' size
    @param[out] holesMasksImageVector [std::vector<cv::Mat>*]
    A vector containing an image (the mask) for each hole
    @return void
   **/
  void HoleFusion::createHolesMasksImageVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    std::vector<cv::Mat>* holesMasksImageVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createHolesMasksImageVector", "createCheckerRequiredVectors");
    #endif

    for (unsigned int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      //!< The current hole's image mask
      cv::Mat holeMask = cv::Mat::zeros(image.size(), CV_8UC1);

      //!< A pointer to the hole mask image
      unsigned char* ptr = holeMask.ptr();

      //!< Draw the outline of the i-th hole onto holeMask
      for(unsigned int j = 0; j < conveyor.outlines[i].size(); j++)
      {
        holeMask.at<unsigned char>(
          conveyor.outlines[i][j].y, conveyor.outlines[i][j].x) = 255;
      }

      //!< The set of points' indices inside the hole's outline
      std::set<unsigned int> visitedPoints;

      cv::Point2f keypoint(
        conveyor.keyPoints[i].pt.x, conveyor.keyPoints[i].pt.y);

      //!< Brushfire from the keypoint to the hole's outline
      //!< to obtain the points inside the hole's outline
      BlobDetection::brushfirePoint(keypoint, &holeMask, &visitedPoints);


      //!< Draw the current hole's mask
      for (std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        ptr[*it] = 255;
      }

      holesMasksImageVector->push_back(holeMask);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createHolesMasksImageVector");
    #endif
  }



  /**
    @brief Some hole checkers require access to a hole's inside points,
    Construct a set of points for each hole, and store all of them in
    a vector
    @param[in] conveyor [const HolesConveyor&] The conveyor of holes
    @param[in] image [const cv::Mat&] An image required to access
    each hole
    @param[out] holesMasksSetVector [std::vector<std::set<unsigned int> >*]
    A vector that holds sets of points;
    each set holds the inside points of each hole
    @return void
   **/
  void HoleFusion::createHolesMasksSetVector(const HolesConveyor& conveyor,
    const cv::Mat& image,
    std::vector<std::set<unsigned int> >* holesMasksSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createHolesMasksSetVector", "createCheckerRequiredVectors");
    #endif

    for (int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      cv::Mat holeMask = cv::Mat::zeros(image.size(), CV_8UC1);

      //!< Draw the outline of the i-th hole onto holeMask
      for(unsigned int j = 0; j < conveyor.outlines[i].size(); j++)
      {
        holeMask.at<unsigned char>(
          conveyor.outlines[i][j].y, conveyor.outlines[i][j].x) = 255;
      }

      cv::Point2f keypoint(
        conveyor.keyPoints[i].pt.x, conveyor.keyPoints[i].pt.y);

      std::set<unsigned int> holeSet;

      //!< Brushfire from the keypoint to the hole's outline
      //!< to obtain the points inside the hole's outline
      BlobDetection::brushfirePoint(keypoint, &holeMask, &holeSet);

      holesMasksSetVector->push_back(holeSet);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createHolesMasksSetVector");
    #endif
  }



  /**
    @brief Some checkers require the construction of a hole's inflated
    rectangle in order to validate a hole. Construct each mask here.
    Each vector element contains the four vertices of the inflated
    rectangle. A hole's bounding rectangle is inflated by a standard size;
    inflated rectangles that go beyond the image's bounds are discarded,
    that is, the output vector contains the indices of the original
    keypoints whose inflated bounding rectangles is within the image's
    bounds.
    @param[in] conveyor [const HolesConveyor&] The conveyor of holes
    @param[in] image [const cv::Mat&] An image needed only for
    its size
    @param[in] inflationSize [const int&] The bounding rectangles
    inflation size in pixels
    @param[out] inflatedRectanglesVector
    [std::vector<std::vector<cv::Point2f> >*] The vector that holds the
    vertices of the in-image-bounds inflated rectangles
    @param[out] inflatedRectanglesIndices [std::vector<int>*]
    The vector that holes the indices of the original holes whose
    inflated bounding rectangles is within the image's bounds.
    @param[out] brushfireBeginPoints [std::vector<cv::Point2f>*] A vector
    used from checker methods that use the brushfire algorithm
    between a hole's outline and its inflated rectangle
    @return void
   **/
  void HoleFusion::createInflatedRectanglesVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const int& inflationSize,
    std::vector<std::vector<cv::Point2f> >* inflatedRectanglesVector,
    std::vector<int>* inflatedRectanglesIndices)
  {
    #ifdef DEBUG_TIME
    Timer::start("createInflatedRectanglesVector",
      "createCheckerRequiredVectors");
    #endif

    //!< Store the vertices of the inside-of-image-bounds inflated bounding
    //!< rectangles in the inflatedRectangles vector
    std::vector<std::vector<cv::Point2f> > inflatedRectangles;

    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (int i = 0; i < conveyor.rectangles.size(); i++)
    {
      std::vector<cv::Point2f> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = conveyor.keyPoints[i].pt.y;
        key_x = conveyor.keyPoints[i].pt.x;

        vert_y = conveyor.rectangles[i][j].y;
        vert_x = conveyor.rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        //!< check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < image.cols &&
          vert_x - inflationSize * cos(theta) >= 0 &&
          vert_y - inflationSize * sin(theta) < image.rows &&
          vert_y - inflationSize * sin(theta) >= 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point2f(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } //!< end for rectangle's points

      //!< If one or more vertices are out of bounds discard the whole
      //!< inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        inflatedVertices.clear();
        continue;
      }
      else
      {
        inflatedRectanglesIndices->push_back(i);
        inflatedRectanglesVector->push_back(inflatedVertices);
      }
    } //!< end for each hole

    #ifdef DEBUG_TIME
    Timer::tick("createInflatedRectanglesVector");
    #endif
  }



  /**
    @brief Some hole checkers require the construction of a hole's mask
    for the points between a hole's outline and its inflated bounding
    rectangle, either in cv::Mat form or in a set form which contains
    points' indices.
    Construct each form here; this method makes it possible to brushfire
    once for every hole, instead of twice, if the image and set vectors
    are needed
    @param[in] conveyor [const HolesConveyor&] The conveyor of holes
    @param[in] rectanglesVector
    [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
    the vertices of each rectangle that corresponds to a specific hole
    inside the coveyor
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[in] image [const cv::Mat&] An image needed only for
    its size
    @param[in] inflationSize [const int&] The bounding rectangles
    inflation size in pixels
    A vector that holds the image of the intermediate points between
    a hole's outline and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @param[out] intermediatePointsSetVector [std::vector<std::set<int> >*]
    A vector that holds the intermediate points' indices between a hole's
    outline and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @return void
   **/
  void HoleFusion::createIntermediateHolesPointsVectors(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point2f> >& rectanglesVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<cv::Mat>* intermediatePointsImageVector,
    std::vector<std::set<unsigned int> >* intermediatePointsSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createIntermediateHolesPointsVectors",
      "createCheckerRequiredVectors");
    #endif

    for (int i = 0; i < rectanglesVector.size(); i++)
    {
      //!< The current hole's mask
      cv::Mat intermediatePointsMask = cv::Mat::zeros(image.size(), CV_8UC1);

      //!< An image whose non-zero value pixels are the ones inside the
      //!< hole's outline
      cv::Mat holeOutlineFilled = cv::Mat::zeros(image.size(), CV_8UC1);
      unsigned char* ptr_h = holeOutlineFilled.ptr();

      //!< An image whose non-zero value pixels are the ones inside the
      //!< hole's bounding rectangle
      cv::Mat rectangleOutlineFilled = cv::Mat::zeros(image.size(), CV_8UC1);
      unsigned char* ptr_r = rectangleOutlineFilled.ptr();

      //!< The brushfire start point is the hole's keypoint
      cv::Point2f keypoint(
        conveyor.keyPoints[rectanglesIndices[i]].pt.x,
        conveyor.keyPoints[rectanglesIndices[i]].pt.y);


      //!< Draw the outline of the i-th hole onto holeOutlineFilled
      for(unsigned int j = 0;
        j < conveyor.outlines[rectanglesIndices[i]].size(); j++)
      {
        holeOutlineFilled.at<uchar>(
          conveyor.outlines[rectanglesIndices[i]][j].y,
          conveyor.outlines[rectanglesIndices[i]][j].x) = 255;
      }

      //!< Fill the inside of the hole
      std::set<unsigned int> holeOutlineFilledSet;
      BlobDetection::brushfirePoint(keypoint,
        &holeOutlineFilled, &holeOutlineFilledSet);

      for(std::set<unsigned int>::iterator it = holeOutlineFilledSet.begin();
        it != holeOutlineFilledSet.end(); it++)
      {
        ptr_h[*it] = 255;
      }


      //!< Draw the bounding rectangle of the i-th hole onto
      //!< rectangleOutlineFilled
      for(unsigned int j = 0; j < rectanglesVector[i].size(); j++)
      {
        cv::line(rectangleOutlineFilled,
          rectanglesVector[i][j],
          rectanglesVector[i][(j + 1) % rectanglesVector[i].size()],
          cv::Scalar(255, 0, 0), 1, 8 );
      }

      //!< Fill the inside of the rectangle
      std::set<unsigned int> rectangleOutlineFilledSet;
      BlobDetection::brushfirePoint(keypoint,
        &rectangleOutlineFilled, &rectangleOutlineFilledSet);

      for(std::set<unsigned int>::iterator it
        = rectangleOutlineFilledSet.begin();
        it != rectangleOutlineFilledSet.end(); it++)
      {
        ptr_r[*it] = 255;
      }


      //!< The current hole's intermediate points mask will be the
      //!< difference between the filled rectangle image and the
      //!< filled outline image
      intermediatePointsMask = rectangleOutlineFilled - holeOutlineFilled;

      intermediatePointsImageVector->push_back(intermediatePointsMask);


      //!< The final set of points' indices
      std::set<unsigned int> intermediatePointsSet;

      std::set_difference(rectangleOutlineFilledSet.begin(),
        rectangleOutlineFilledSet.end(),
        holeOutlineFilledSet.begin(),
        holeOutlineFilledSet.end(),
        std::inserter(intermediatePointsSet, intermediatePointsSet.end()));

      intermediatePointsSetVector->push_back(intermediatePointsSet);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createIntermediateHolesPointsVectors");
    #endif
  }


  /**
    @brief For each hole, this function finds the points between the hole's
    outline and the rectangle (inflated or not) that corrensponds to it.
    These points are then stored in an image.
    @param[in] conveyor [const HolesConveyor&] The conveyor of holes
    @param[in] rectanglesVector
    [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
    the vertices of each rectangle that corresponds to a specific hole
    inside the coveyor
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[in] image [const cv::Mat&] An image needed only for
    its size
    A vector that holds the image of the intermediate points between
    a hole's outline and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @return void
   **/
  void HoleFusion::createIntermediateHolesPointsImageVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point2f> >& rectanglesVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<cv::Mat>* intermediatePointsImageVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createIntermediateHolesPointsImageVector",
      "createCheckerRequiredVectors");
    #endif

    for (int i = 0; i < rectanglesVector.size(); i++)
    {
      //!< The current hole's mask
      cv::Mat intermediatePointsMask = cv::Mat::zeros(image.size(), CV_8UC1);

      //!< An image whose non-zero value pixels are the ones inside the
      //!< hole's outline
      cv::Mat holeOutlineFilled = cv::Mat::zeros(image.size(), CV_8UC1);
      unsigned char* ptr_h = holeOutlineFilled.ptr();

      //!< An image whose non-zero value pixels are the ones inside the
      //!< hole's bounding rectangle
      cv::Mat rectangleOutlineFilled = cv::Mat::zeros(image.size(), CV_8UC1);
      unsigned char* ptr_r = rectangleOutlineFilled.ptr();

      //!< The brushfire start point is the hole's keypoint
      cv::Point2f keypoint(
        conveyor.keyPoints[rectanglesIndices[i]].pt.x,
        conveyor.keyPoints[rectanglesIndices[i]].pt.y);


      //!< Draw the outline of the i-th hole onto holeOutlineFilled
      for(unsigned int j = 0;
        j < conveyor.outlines[rectanglesIndices[i]].size(); j++)
      {
        holeOutlineFilled.at<uchar>(
          conveyor.outlines[rectanglesIndices[i]][j].y,
          conveyor.outlines[rectanglesIndices[i]][j].x) = 255;
      }

      //!< Fill the inside of the hole
      std::set<unsigned int> holeOutlineFilledSet;
      BlobDetection::brushfirePoint(keypoint,
        &holeOutlineFilled, &holeOutlineFilledSet);

      for(std::set<unsigned int>::iterator it = holeOutlineFilledSet.begin();
        it != holeOutlineFilledSet.end(); it++)
      {
        ptr_h[*it] = 255;
      }


      //!< Draw the bounding rectangle of the i-th hole onto
      //!< rectangleOutlineFilled
      for(unsigned int j = 0; j < rectanglesVector[i].size(); j++)
      {
        cv::line(rectangleOutlineFilled,
          rectanglesVector[i][j],
          rectanglesVector[i][(j + 1) % rectanglesVector[i].size()],
          cv::Scalar(255, 0, 0), 1, 8 );
      }

      //!< Fill the inside of the rectangle
      std::set<unsigned int> rectangleOutlineFilledSet;
      BlobDetection::brushfirePoint(keypoint,
        &rectangleOutlineFilled, &rectangleOutlineFilledSet);

      for(std::set<unsigned int>::iterator it
        = rectangleOutlineFilledSet.begin();
        it != rectangleOutlineFilledSet.end(); it++)
      {
        ptr_r[*it] = 255;
      }


      //!< The current hole's intermediate points mask will be the
      //!< difference between the filled rectangle image and the
      //!< filled outline image
      intermediatePointsMask = rectangleOutlineFilled - holeOutlineFilled;

      intermediatePointsImageVector->push_back(intermediatePointsMask);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createIntermediateHolesPointsImageVector");
    #endif
  }



  /**
    @brief For each hole, this function finds the points between the hole's
    outline and the rectangle (inflated or not) that corrensponds to it.
    These points are then stored in a std::set of ints.
    @param[in] conveyor [const HolesConveyor&] The conveyor of holes
    @param[in] rectanglesVector
    [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
    the vertices of each rectangle that corresponds to a specific hole
    inside the coveyor
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[out] intermediatePointsSetVector [std::vector<std::set<int> >*]
    A vector that holds the intermediate points' indices between a hole's
    outline and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @return void
   **/
  void HoleFusion::createIntermediateHolesPointsSetVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point2f> >& rectanglesVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<std::set<unsigned int> >* intermediatePointsSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createIntermediateHolesPointsSetVector",
      "createCheckerRequiredVectors");
    #endif

    for (int i = 0; i < rectanglesVector.size(); i++)
    {
      //!< The current hole's mask
      cv::Mat intermediatePointsMask = cv::Mat::zeros(image.size(), CV_8UC1);

      //!< An image whose non-zero value pixels are the ones inside the
      //!< hole's outline
      cv::Mat holeOutlineFilled = cv::Mat::zeros(image.size(), CV_8UC1);
      unsigned char* ptr_h = holeOutlineFilled.ptr();

      //!< An image whose non-zero value pixels are the ones inside the
      //!< hole's bounding rectangle
      cv::Mat rectangleOutlineFilled = cv::Mat::zeros(image.size(), CV_8UC1);
      unsigned char* ptr_r = rectangleOutlineFilled.ptr();

      //!< The brushfire start point is the hole's keypoint
      cv::Point2f keypoint(
        conveyor.keyPoints[rectanglesIndices[i]].pt.x,
        conveyor.keyPoints[rectanglesIndices[i]].pt.y);


      //!< Draw the outline of the i-th hole onto holeOutlineFilled
      for(unsigned int j = 0;
        j < conveyor.outlines[rectanglesIndices[i]].size(); j++)
      {
        holeOutlineFilled.at<uchar>(
          conveyor.outlines[rectanglesIndices[i]][j].y,
          conveyor.outlines[rectanglesIndices[i]][j].x) = 255;
      }

      //!< Fill the inside of the hole
      std::set<unsigned int> holeOutlineFilledSet;
      BlobDetection::brushfirePoint(keypoint,
        &holeOutlineFilled, &holeOutlineFilledSet);



      //!< Draw the bounding rectangle of the i-th hole onto
      //!< rectangleOutlineFilled
      for(unsigned int j = 0; j < rectanglesVector[i].size(); j++)
      {
        cv::line(rectangleOutlineFilled,
          rectanglesVector[i][j],
          rectanglesVector[i][(j + 1) % rectanglesVector[i].size()],
          cv::Scalar(255, 0, 0), 1, 8 );
      }

      //!< Fill the inside of the rectangle
      std::set<unsigned int> rectangleOutlineFilledSet;
      BlobDetection::brushfirePoint(keypoint,
        &rectangleOutlineFilled, &rectangleOutlineFilledSet);


      //!< The final set of points' indices
      std::set<unsigned int> intermediatePointsSet;

      std::set_difference(rectangleOutlineFilledSet.begin(),
        rectangleOutlineFilledSet.end(),
        holeOutlineFilledSet.begin(),
        holeOutlineFilledSet.end(),
        std::inserter(intermediatePointsSet, intermediatePointsSet.end()));

      intermediatePointsSetVector->push_back(intermediatePointsSet);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createIntermediateHolesPointsSetVector");
    #endif
  }



  /**
    @brief Callback for the candidate holes via the depth node
    @param[in] depthCandidateHolesVector
    [const vision_communications::CandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the depth node
    @return void
   **/
  void HoleFusion::depthCandidateHolesCallback(
    const vision_communications::CandidateHolesVectorMsg&
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
    HolesConveyorUtils::clear(&depthHolesConveyor_);

    //!< Unpack the message
    unpackMessage(depthCandidateHolesVector,
      &depthHolesConveyor_,
      &interpolatedDepthImage_,
      sensor_msgs::image_encodings::TYPE_32FC1);

    numNodesReady_++;

    //!< If the RGB and the depth nodes are ready
    //!< and the point cloud has been delivered and interpolated,
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 3)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("depthCandidateHolesCallback");
    Timer::printAllMeansTree();
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
      std::vector<cv::Point2f> outlinePoints;
      for (unsigned int o = 0;
        o < candidateHolesVector[i].outlineX.size(); o++)
      {
        cv::Point2f outlinePoint;
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
    #ifdef DEBUG_TIME
    Timer::start("getWallsHistogram", "HoleFusion");
    #endif

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

    #ifdef DEBUG_TIME
    Timer::tick("getWallsHistogram");
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
    Parameters::run_checker_depth_homogeneity =
      config.run_checker_depth_homogeneity;

    Parameters::rectangle_inflation_size =
      config.rectangle_inflation_size;
    Parameters::depth_difference =
      config.depth_difference;

    Parameters::run_checker_color_homogeneity =
      config.run_checker_color_homogeneity;
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

    Parameters::debug_show_merge_holes =
      config.debug_show_merge_holes;
    Parameters::debug_show_merge_holes_size =
      config.debug_show_merge_holes_size;


    //!< Texture parameters
    //!< The threshold for texture matching
    Parameters::match_texture_threshold =
      config.match_texture_threshold;

    Parameters::non_zero_points_in_box_blob_histogram =
      config.non_zero_points_in_box_blob_histogram;

    //!<Color homogeneity parameters
    Parameters::num_bins_threshold =
      config.num_bins_threshold;

    //!< Histogram parameters
    Parameters::number_of_hue_bins =
      config.number_of_hue_bins;
    Parameters::number_of_saturation_bins =
      config.number_of_saturation_bins;
    Parameters::number_of_value_bins =
      config.number_of_value_bins;

    //!< Holes connection - merger
    Parameters::connect_holes_min_distance =
      config.connect_holes_min_distance;
    Parameters::connect_holes_max_distance =
      config.connect_holes_max_distance;
  }



  /**
    @brief Callback for the point cloud that the synchronizer node
    publishes
    @param[in] msg [const sensor_msgs::PointCloud2ConstPtr&] The message
    containing the point cloud
    @return void
   **/
  void HoleFusion::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("pointCloudCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Hole Fusion Point Cloud callback");
    #endif

    //!< Unpack the point cloud and store it in the member variable
    //!< pointCloudXYZ_
    MessageConversions::extractPointCloudXYZFromMessage(msg,
      &pointCloudXYZ_);

    //!< Extract the depth image from the point cloud message
    cv::Mat depthImage = MessageConversions::convertPointCloudMessageToImage(
      msg, CV_32FC1);

    //!< Interpolate the depthImage
    cv::Mat interpolatedDepthImage;
    NoiseElimination::performNoiseElimination(depthImage,
      &interpolatedDepthImage);

    //!< Set the interpolatedDepthImage's values as the depth values
    //!< of the point cloud
    setDepthValuesInPointCloud(interpolatedDepthImage, &pointCloudXYZ_);

    numNodesReady_++;

    //!< If the RGB and the depth nodes are ready
    //!< and the point cloud has been delivered and interpolated,
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 3)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("pointCloudCallback");
    Timer::printAllMeansTree();
    #endif
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
    Timer::start("processCandidateHoles", "", true);
    #endif

    //!< Merge the conveyors from the RGB and Depth sources
    HolesConveyor rgbdHolesConveyor;
    HolesConveyorUtils::merge(depthHolesConveyor_, rgbHolesConveyor_,
      &rgbdHolesConveyor);

/*
 *
 *    //!< Uncomment for testing artificial holes' merging process
 *    HolesConveyor dummy;
 *    testDummyHolesMerging(&dummy);
 *    return;
 *
 */

    //!< Keep a copy of the initial (not merged) candidate holes for
    //!< debugging and exibition purposes
    HolesConveyor rgbdHolesConveyorBeforeMerge;

    HolesConveyorUtils::copyTo(rgbdHolesConveyor,
      &rgbdHolesConveyorBeforeMerge);


    #ifdef DEBUG_SHOW
    std::vector<std::string> msgs;
    std::vector<cv::Mat> canvases;
    std::vector<std::string> titles;

    if(Parameters::debug_show_merge_holes)
    {
      //!< Push back the identifier of each keypoint
      for (int i = 0; i < rgbdHolesConveyorBeforeMerge.keyPoints.size(); i++)
      {
        msgs.push_back(TOSTR(i));
      }

      canvases.push_back(
        Visualization::showHoles(
          "",
          interpolatedDepthImage_,
          rgbdHolesConveyorBeforeMerge,
          -1,
          msgs));

      titles.push_back(
        TOSTR(rgbdHolesConveyor.keyPoints.size()) + " holes before merging");
    }
    #endif


    //!< Try to merge holes that can be assimilated, amalgamated or connected
    for (int i = 0; i < 3; i++)
    {
      applyMergeOperation(&rgbdHolesConveyor, i);
    }


    #ifdef DEBUG_SHOW
    if(Parameters::debug_show_merge_holes)
    {
      msgs.clear();
      //!< Push back the identifier of each keypoint
      for (int i = 0; i < rgbdHolesConveyor.keyPoints.size(); i++)
      {
        msgs.push_back(TOSTR(i));
      }

      canvases.push_back(
        Visualization::showHoles(
          "",
          interpolatedDepthImage_,
          rgbdHolesConveyor,
          -1,
          msgs));

      titles.push_back(
        TOSTR(rgbdHolesConveyor.keyPoints.size()) + " holes after merging");

      Visualization::multipleShow("Merged Keypoints",
        canvases, titles, Parameters::debug_show_merge_holes_size, 1);
    }
    #endif

    //!< Apply all active filters
    sift(rgbdHolesConveyor);

    #ifdef DEBUG_TIME
    Timer::tick("processCandidateHoles");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Callback for the candidate holes via the rgb node
    @param[in] depthCandidateHolesVector
    [const vision_communications::CandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the rgb node
    @return void
   **/
  void HoleFusion::rgbCandidateHolesCallback(
    const vision_communications::CandidateHolesVectorMsg&
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
    HolesConveyorUtils::clear(&rgbHolesConveyor_);

    //!< Unpack the message
    unpackMessage(rgbCandidateHolesVector,
      &rgbHolesConveyor_,
      &rgbImage_,
      sensor_msgs::image_encodings::TYPE_8UC3);

    numNodesReady_++;

    //!< If the RGB and the depth nodes are ready
    //!< and the point cloud has been delivered and interpolated,
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 3)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("rgbCandidateHolesCallback");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Sets the depth values of a point cloud according to the
    values of a depth image
    @param[in] inImage [const cv::Mat&] The depth image in CV_32FC1 format
    @param[out] pointCloudXYZPtr [PointCloudXYZPtr*] The point cloud
    @return void
   **/
  void HoleFusion::setDepthValuesInPointCloud(const cv::Mat& inImage,
    PointCloudXYZPtr* pointCloudXYZPtr)
  {
    #ifdef DEBUG_TIME
    Timer::start("setDepthValuesInPointCloud", "pointCloudCallback");
    #endif

    //!< If the inImage is not of type CV_32FC1, return
    if(inImage.type() != CV_32FC1)
    {
      return;
    }

    for (unsigned int row = 0; row < (*pointCloudXYZPtr)->height; ++row)
    {
      for (unsigned int col = 0; col < (*pointCloudXYZPtr)->width; ++col)
      {
        (*pointCloudXYZPtr)->points[col + (*pointCloudXYZPtr)->width * row].z =
          inImage.at<float>(row, col);
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("setDepthValuesInPointCloud");
    #endif
  }


  /**
    @brief Runs candidate holes through selected filters.
    Probabilities for each candidate hole and filter
    are printed in the console, with an order specified by the
    hole_fusion_cfg of the dynamic reconfigure utility
    @param[in] conveyor [const HolesConveyor&] The conveyor
    containing candidate holes
    @return void
   **/
  void HoleFusion::sift(const HolesConveyor& conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("sift", "processCandidateHoles");
    #endif


    //!< A vector of images that each one of them represents the corresponding
    //!< hole's mask: non-zero value pixels are within a hole's outline points
    std::vector<cv::Mat> holesMasksImageVector;

    //!< A vector of sets that each one of them contains indices of points
    //!< inside the hole's outline
    std::vector<std::set<unsigned int> > holesMasksSetVector;

    //!< A vector of vertices of each inflated bounding rectangle.
    std::vector<std::vector<cv::Point2f> > inflatedRectanglesVector;

    //!< Since inflated rectangle's vertices may go outside the image's bounds,
    //!< this vector stores the indices of the keypoints whose corresponding
    //!< inflated rectangle is in totality within the image's bounds
    std::vector<int> inflatedRectanglesIndices;

    //!< A vector of sets that each one of them contains indices of points
    //!< between the hole's outline and its respective bounding box
    std::vector<std::set<unsigned int> > intermediatePointsSetVector;

    //!< A vector of images that each one of them contains points
    //!< between the hole's outline and its respective bounding box
    std::vector<cv::Mat> intermediatePointsImageVector;

    //!< Construct the necessary vectors, depending on which filters
    //!< are to run in runtime
    createCheckerRequiredVectors(
      conveyor,
      interpolatedDepthImage_,
      Parameters::rectangle_inflation_size,
      &holesMasksImageVector,
      &holesMasksSetVector,
      &inflatedRectanglesVector,
      &inflatedRectanglesIndices,
      &intermediatePointsImageVector,
      &intermediatePointsSetVector);


    //!< Initialize the depth probabilities 2D vector. But first we need to know
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
    if (Parameters::run_checker_depth_homogeneity > 0)
    {
      depthActiveFilters++;
    }

    if (depthActiveFilters > 0)
    {
      std::vector<std::vector<float> > depthProbabilitiesVector2D(
        depthActiveFilters,
        std::vector<float>(conveyor.keyPoints.size(), 0.0));

      //!< check holes for debugging purposes
      DepthFilters::checkHoles(
        conveyor,
        interpolatedDepthImage_,
        pointCloudXYZ_,
        holesMasksSetVector,
        inflatedRectanglesVector,
        inflatedRectanglesIndices,
        intermediatePointsSetVector,
        &depthProbabilitiesVector2D);

      #ifdef DEBUG_SHOW
      ROS_ERROR("-------------------------------------------");
      if (conveyor.keyPoints.size() > 0)
      {
        ROS_ERROR("Depth : Candidate Holes' probabilities");
        for (int j = 0; j < conveyor.keyPoints.size(); j++)
        {
          std::string probsString;
          for (int i = 0; i < depthActiveFilters; i++)
          {
            probsString += TOSTR(depthProbabilitiesVector2D[i][j]) + " | ";
          }

          ROS_ERROR("P_%d [%f %f]= %s", j, conveyor.keyPoints[j].pt.x,
            conveyor.keyPoints[j].pt.y, probsString.c_str());
        }
      }
      #endif
    }

    //!< Initialize the rgb probabilities 2D vector. But first we need to know
    //!< how many rows the vector will accomodate
    int rgbActiveFilters = 0;

    if (Parameters::run_checker_color_homogeneity > 0)
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

    if (rgbActiveFilters > 0)
    {
      std::vector<std::vector<float> > rgbProbabilitiesVector2D(
        rgbActiveFilters,
        std::vector<float>(conveyor.keyPoints.size(), 0.0));

      //!< check holes for debugging purposes
      RgbFilters::checkHoles(
        conveyor,
        rgbImage_,
        wallsHistogram_,
        inflatedRectanglesIndices,
        holesMasksImageVector,
        holesMasksSetVector,
        intermediatePointsImageVector,
        intermediatePointsSetVector,
        &rgbProbabilitiesVector2D);

      #ifdef DEBUG_SHOW
      if (conveyor.keyPoints.size() > 0)
      {
        ROS_ERROR("RGB: Candidate Holes' probabilities");
        for (int j = 0; j < conveyor.keyPoints.size(); j++)
        {
          std::string probsString;
          for (int i = 0; i < rgbActiveFilters; i++)
          {
            probsString += TOSTR(rgbProbabilitiesVector2D[i][j]) + " | ";
          }

          ROS_ERROR("P_%d [%f %f] = %s", j, conveyor.keyPoints[j].pt.x,
            conveyor.keyPoints[j].pt.y, probsString.c_str());
        }
      }
      #endif
    }

    #ifdef DEBUG_TIME
    Timer::tick("sift");
    #endif
  }



  /**
    @brief Unpacks the the HolesConveyor struct for the
    candidate holes, the interpolated depth image and the point cloud
    from the vision_communications::CandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::CandidateHolesVectorMsg&] The input
    candidate holes message obtained through the depth node
    @param[out] conveyor [HolesConveyor*] The output conveyor
    struct
    @param[out] pointCloudXYZ [PointCloudXYZPtr*] The output point cloud
    @param[out] interpolatedDepthImage [cv::Mat*] The output interpolated
    depth image
    @return void
   **/
  void HoleFusion::unpackMessage(
    const vision_communications::CandidateHolesVectorMsg& holesMsg,
    HolesConveyor* conveyor, cv::Mat* image, const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackMessage");
    #endif

    //!< Recreate the conveyor
    fromCandidateHoleMsgToConveyor(holesMsg.candidateHoles, conveyor);

    //!< Unpack the image
    MessageConversions::extractImageFromMessageContainer(
      holesMsg,
      image,
      encoding);

    #ifdef DEBUG_TIME
    Timer::tick("unpackMessage");
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
    @brief Tests the merging operations on artificial holes
    @param[out] dummy [HolesConveyor*] The hole candidates
    @return void
   **/
  void HoleFusion::testDummyHolesMerging(HolesConveyor* dummy)
  {
    #ifdef DEBUG_TIME
    Timer::start("testDummyHolesMerging", "processCandidateHoles");
    #endif

    //!< Invalid
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(20, 20), cv::Point2f(30, 30), 50, 50, 30, 30, dummy);

    //!< Invalid
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(80, 80), cv::Point2f(90, 90), 50, 50, 30, 30, dummy);


    //!< 0-th assimilator - amalgamator - connector
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(370.0, 130.0), cv::Point2f(372.0, 132.0), 80, 80, 76, 76,
      dummy);

    //!< 0-th overlapper
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(372.0, 132.0), cv::Point2f(374.0, 134.0), 80, 80, 76, 76,
      dummy);

    //!< 0-th assimilable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(380.0, 140.0), cv::Point2f(382.0, 142.0), 20, 20, 16, 16,
      dummy);

    //!< 0-th amalgamatable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(420.0, 140.0), cv::Point2f(422.0, 142.0), 120, 40, 116, 36,
      dummy);

    //!< 0-th connectable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(510.0, 80.0), cv::Point2f(512.0, 82.0), 40, 40, 36, 36,
      dummy);


    //!< 1-st assimilator - amalgamator - connector
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(300.0, 300.0), cv::Point2f(302.0, 302.0), 100, 100, 96, 96,
      dummy);

    //!< 1-st connectable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(410.0, 350.0), cv::Point2f(412.0, 352.0), 50, 50, 46, 46,
      dummy);


    HolesConveyorUtils::shuffle(dummy);

    ROS_ERROR("keypoints before: %d ", HolesConveyorUtils::size(*dummy));

    std::vector<std::string> msgs;
    Visualization::showHoles("before", interpolatedDepthImage_, *dummy,
      1, msgs);

    for (int i = 0; i < 3; i++)
    {
      applyMergeOperation(dummy, i);
    }

    ROS_ERROR("keypoints after: %d ", HolesConveyorUtils::size(*dummy));
    Visualization::showHoles("after", interpolatedDepthImage_, *dummy,
      1, msgs);

    #ifdef DEBUG_TIME
    Timer::tick("testDummyHolesMerging");
    #endif
  }

} // namespace pandora_vision

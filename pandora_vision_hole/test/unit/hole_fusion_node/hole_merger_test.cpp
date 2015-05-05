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
 * Author: Alexandros Philotheou
 *********************************************************************/

#include "hole_fusion_node/hole_merger.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  /**
    @class HoleMergerTest
    @brief Tests the integrity of methods of class HoleMerger
   **/
  class HoleMergerTest : public ::testing::Test
  {
    protected:

      HoleMergerTest() : cloud ( new PointCloud ) {}

      /**
        @brief Constructs a rectangle of width @param x and height of @param y.
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The rectangle's width
        @param[in] y [const int&] The rectangle's height
        @param[in] depthIn [const float&] The depth value for all points inside
        the rectangle
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted
        return void
       **/
      void generateDepthRectangle (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        const float& depthIn,
        cv::Mat* image );

      /**
        @brief Constructs the internals of a rectangular hole
        of width @param x and height of @param y
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The recgangle's width
        @param[in] y [const int&] The rectangle's height
        return [BlobVector] A struct containing the elements of one hole
       **/
      BlobVector getConveyor (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y );

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // The image upon which the squares will be inprinted
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // Construct the squares_ image

        // Set the depth for each point of the squares_ image to 1.0
        for ( int rows = 0; rows < squares_.rows; rows++ )
        {
          for ( int cols = 0; cols < squares_.cols; cols++ )
          {
            squares_.at< float >( rows, cols ) = 1.0;
          }
        }

        // Construct the main square. This will be the assimilator,
        // amalgamator and connector
        cv::Mat mainSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

        HoleMergerTest::generateDepthRectangle
          ( cv::Point2f ( 250, 250 ),
            100,
            100,
            0.2,
            &mainSquare );

        conveyor.extend(
          getConveyor( cv::Point2f ( 250, 250 ),
            100,
            100 ));

        // Construct the assimilable
        cv::Mat assimilable = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

        HoleMergerTest::generateDepthRectangle
          ( cv::Point2f ( 300, 300 ),
            10,
            10,
            0.2,
            &assimilable );

        conveyor.extend(
          getConveyor( cv::Point2f ( 300, 300 ),
            10,
            10 ));

        // Construct the amalgamatable
        cv::Mat amalgamatable = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        HoleMergerTest::generateDepthRectangle
          ( cv::Point2f ( 340, 300 ),
            20,
            20,
            0.3,
            &amalgamatable );

        conveyor.extend(
          getConveyor( cv::Point2f ( 340, 300 ),
            20,
            20 ));

        // Construct the connectable
        cv::Mat connectable = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        HoleMergerTest::generateDepthRectangle
          ( cv::Point2f ( 200, 300 ),
            40,
            40,
            0.3,
            &connectable );

        conveyor.extend(
          getConveyor( cv::Point2f ( 200, 300 ),
            40,
            40 ));

        // Construct the outlier
        cv::Mat outlier = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        HoleMergerTest::generateDepthRectangle
          ( cv::Point2f ( 400, 40 ),
            40,
            40,
            0.3,
            &outlier);

        conveyor.extend(
          getConveyor( cv::Point2f ( 400, 40 ),
            40,
            40 ));

        // Compose the final squares_ image
        squares_ +=
          mainSquare + assimilable + amalgamatable + connectable + outlier;

        // Construct the point cloud corresponding to the squares_ image

        // Fill in the cloud data
        cloud->width = WIDTH;
        cloud->height = HEIGHT;
        cloud->points.resize ( cloud->width * cloud->height );

        // Generate the data
        for ( int i = 0; i < cloud->points.size (); i++ )
        {
          // Row
          int x = i / WIDTH;

          // Column
          int y = i % WIDTH;

          cloud->points[i].x = 1 + x / 10;
          cloud->points[i].y = 1 + y / 10;
          cloud->points[i].z = squares_.at< float >( x, y );
        }
      }
      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image that will be used to locate blobs in
      cv::Mat squares_;

      // The conveyor of holes that will be used to test methods of class
      // DepthFilters
      BlobVector conveyor;

      // The point cloud corresponding to the squares_ image
      PointCloudPtr cloud;
  };

  /**
    @brief Constructs a rectangle of width @param x and height of @param y.
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The rectangle's width
    @param[in] y [const int&] The rectangle's height
    @param[in] depthIn [const float&] The depth value for all points inside
    the rectangle
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted on
    return void
   **/
  void HoleMergerTest::generateDepthRectangle (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y,
    const float& depthIn,
    cv::Mat* image )
  {
    // Fill the inside of the desired rectangle with the @param depthIn provided
    for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
    {
      for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
      {
        image->at< float >( rows, cols ) = depthIn;
      }
    }
  }

  /**
    @brief Constructs the internals of a rectangular hole
    of width @param x and height of @param y
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The recgangle's width
    @param[in] y [const int&] The rectangle's height
    return [BlobVector] A struct containing the elements of one hole
   **/
  BlobVector HoleMergerTest::getConveyor (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y )
  {
    // A single hole
    pandora_vision_msgs::Blob hole;

    // The hole's keypoint
    hole.areaOfInterest.center.x = upperLeft.x + x / 2;
    hole.areaOfInterest.center.y = upperLeft.y + y / 2;

    // The width and height of the rectangle
    hole.areaOfInterest.width = x;
    hole.areaOfInterest.height = y;

    // The outline points of the hole will be obtained through the depiction
    // of the points consisting the rectangle
    cv::Mat image = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
    
    cv::Point2f vertex_1( upperLeft.x, upperLeft.y );
    cv::Point2f vertex_2( upperLeft.x, upperLeft.y + y - 1 );
    cv::Point2f vertex_3( upperLeft.x + x - 1, upperLeft.y + y - 1 );
    cv::Point2f vertex_4( upperLeft.x + x - 1, upperLeft.y );
    
    cv::Point2f a[] = {vertex_1, vertex_2, vertex_3, vertex_4};

    for (unsigned int j = 0; j < 4; j++)
    {
      cv::line(image, a[j], a[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8);
    }

    std::vector<cv::Point2f> outline;
    for ( int rows = 0; rows < image.rows; rows++ )
    {
      for ( int cols = 0; cols < image.cols; cols++ )
      {
        if ( image.at<unsigned char>( rows, cols ) != 0 )
        {
          outline.push_back( cv::Point2f ( cols, rows ) );
        }
      }
    }
    hole.outline = MessageConversions::vecToMsg(outline);

    // Push hole back into a BlobVector
    BlobVector conveyor;
    conveyor.append(hole);

    return conveyor;
  }

  //! Tests HoleMerger::applyMergeOperation
  TEST_F ( HoleMergerTest, applyMergeOperationTest )
  {
    // Keep a backup of the original conveyor
    BlobVector originConveyor;
    originConveyor.copy(conveyor);

    // Run HoleMerger::applyMergeOperation for operationId = 0 : assimilation
    HoleMerger::applyMergeOperation(
      &conveyor,
      squares_,
      cloud,
      ASSIMILATION );

    // The number of holes should have shrunk by one
    EXPECT_EQ ( conveyor.size(), originConveyor.size() - 1 );

    // Entry #0 should be intact
    EXPECT_EQ ( conveyor.getBlob(0).areaOfInterest.center.x,
      originConveyor.getBlob(0).areaOfInterest.center.x );

    EXPECT_EQ ( conveyor.getBlob(0).areaOfInterest.center.y,
      originConveyor.getBlob(0).areaOfInterest.center.y );

    EXPECT_EQ ( conveyor.getBlob(0).areaOfInterest.width,
      originConveyor.getBlob(0).areaOfInterest.width );

    EXPECT_EQ ( conveyor.getBlob(0).areaOfInterest.height,
      originConveyor.getBlob(0).areaOfInterest.height );

    for ( int o = 0; o < conveyor.getBlob(0).outline.size(); o++ )
    {
      EXPECT_EQ ( conveyor.getBlob(0).outline[o].x,
        originConveyor.getBlob(0).outline[o].x );

      EXPECT_EQ ( conveyor.getBlob(0).outline[o].y,
        originConveyor.getBlob(0).outline[o].y );
    }

    // Original entry #2 should now be entry #1
    // Original entry #3 should now be entry #2
    // Original entry #4 should now be entry #3
    for ( int i = 2; i < originConveyor.size(); i++ )
    {
      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.center.x,
        originConveyor.getBlob(i).areaOfInterest.center.x );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.center.y,
        originConveyor.getBlob(i).areaOfInterest.center.y );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.width,
        originConveyor.getBlob(i).areaOfInterest.width );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.height,
        originConveyor.getBlob(i).areaOfInterest.height );

      for ( int o = 0; o < conveyor.getBlob(i - 1).outline.size(); o++ )
      {
        EXPECT_EQ ( conveyor.getBlob(i - 1).outline[o].x,
          originConveyor.getBlob(i).outline[o].x );

        EXPECT_EQ ( conveyor.getBlob(i - 1).outline[o].y,
          originConveyor.getBlob(i).outline[o].y );
      }
    }
    // Restore conveyor to its original state
    conveyor.replace(originConveyor);

    // Run HoleMerger::applyMergeOperation for operationId = 1 : amalgamation
    HoleMerger::applyMergeOperation(
      &conveyor,
      squares_,
      cloud,
      AMALGAMATION );


    // The number of holes should have shrunk by one
    EXPECT_EQ ( conveyor.size(), originConveyor.size() - 1 );

    // The amalgamator should have grown in terms of outline points
    EXPECT_LT ( originConveyor.getBlob(0).outline.size(),
      conveyor.getBlob(0).outline.size() );

    // The amalgamator's keypoint should have moved a tiny bit to the right,
    // but not significantly vertically
    EXPECT_LT ( originConveyor.getBlob(0).areaOfInterest.center.x,
      conveyor.getBlob(0).areaOfInterest.center.x );
    EXPECT_NEAR ( originConveyor.getBlob(0).areaOfInterest.center.y,
      conveyor.getBlob(0).areaOfInterest.center.y, 1 );

    // Entry #1 should be intact
    EXPECT_EQ ( conveyor.getBlob(1).areaOfInterest.center.x,
      originConveyor.getBlob(1).areaOfInterest.center.x );

    EXPECT_EQ ( conveyor.getBlob(1).areaOfInterest.center.y,
      originConveyor.getBlob(1).areaOfInterest.center.y );

    EXPECT_EQ ( conveyor.getBlob(1).areaOfInterest.width,
      originConveyor.getBlob(1).areaOfInterest.width );

    EXPECT_EQ ( conveyor.getBlob(1).areaOfInterest.height,
      originConveyor.getBlob(1).areaOfInterest.height );

    for ( int o = 0; o < conveyor.getBlob(1).outline.size(); o++ )
    {
      EXPECT_EQ ( conveyor.getBlob(1).outline[o].x,
        originConveyor.getBlob(1).outline[o].x );

      EXPECT_EQ ( conveyor.getBlob(1).outline[o].y,
        originConveyor.getBlob(1).outline[o].y );
    }

    for ( int i = 3; i < 5; i++ )
    {
      // Original entry #3 should now be #2
      // Original entry #4 should now be #3
      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.center.x,
        originConveyor.getBlob(i).areaOfInterest.center.x );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.center.y,
        originConveyor.getBlob(i).areaOfInterest.center.y );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.width,
        originConveyor.getBlob(i).areaOfInterest.width );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.height,
        originConveyor.getBlob(i).areaOfInterest.height );

      for ( int o = 0; o < conveyor.getBlob(i - 1).outline.size(); o++ )
      {
        EXPECT_EQ ( conveyor.getBlob(i - 1).outline[o].x,
          originConveyor.getBlob(i).outline[o].x );

        EXPECT_EQ ( conveyor.getBlob(i - 1).outline[o].y,
          originConveyor.getBlob(i).outline[o].y );
      }
    }
    // Run HoleMerger::applyMergeOperation for operationId = 2 : connection

    // But first, restore conveyor to its original state
    conveyor.replace(originConveyor);

    // Modify the connection parameters
    Parameters::HoleFusion::Merger::connect_holes_min_distance = 3;
    Parameters::HoleFusion::Merger::connect_holes_max_distance = 20;

    HoleMerger::applyMergeOperation(
      &conveyor,
      squares_,
      cloud,
      CONNECTION );

    // The number of holes should have shrunk by one
    EXPECT_EQ ( conveyor.size(), originConveyor.size() - 1 );

    // The connector should have grown in terms of outline points
    EXPECT_LT ( originConveyor.getBlob(0).outline.size(),
      conveyor.getBlob(0).outline.size() );

    // The connector's keypoint should have moved a bit to the left,
    // and a bit lower than before
    EXPECT_GT ( originConveyor.getBlob(0).areaOfInterest.center.x,
      conveyor.getBlob(0).areaOfInterest.center.x );
    EXPECT_LT ( originConveyor.getBlob(0).areaOfInterest.center.y,
      conveyor.getBlob(0).areaOfInterest.center.y );

    // Original entry #1 should now be entry #1
    // Original entry #2 should now be entry #2
    for ( int i = 1; i < 3; i++ )
    {
      EXPECT_EQ ( conveyor.getBlob(i).areaOfInterest.center.x,
        originConveyor.getBlob(i).areaOfInterest.center.x );
      EXPECT_EQ ( conveyor.getBlob(i).areaOfInterest.center.y,
        originConveyor.getBlob(i).areaOfInterest.center.y );

      EXPECT_EQ ( conveyor.getBlob(i).areaOfInterest.width,
        originConveyor.getBlob(i).areaOfInterest.width );
      EXPECT_EQ ( conveyor.getBlob(i).areaOfInterest.height,
        originConveyor.getBlob(i).areaOfInterest.height );

      for ( int o = 0; o < conveyor.getBlob(i).outline.size(); o++ )
      {
        EXPECT_EQ ( conveyor.getBlob(i).outline[o].x,
          originConveyor.getBlob(i).outline[o].x );
        EXPECT_EQ ( conveyor.getBlob(i).outline[o].y,
          originConveyor.getBlob(i).outline[o].y );
      }
    }
    // Original entry #4 should now be entry #3
    EXPECT_EQ ( conveyor.getBlob(3).areaOfInterest.center.x,
      originConveyor.getBlob(4).areaOfInterest.center.x );
    EXPECT_EQ ( conveyor.getBlob(3).areaOfInterest.center.y,
      originConveyor.getBlob(4).areaOfInterest.center.y );

    EXPECT_EQ ( conveyor.getBlob(3).areaOfInterest.width,
      originConveyor.getBlob(4).areaOfInterest.width );
    EXPECT_EQ ( conveyor.getBlob(3).areaOfInterest.height,
      originConveyor.getBlob(4).areaOfInterest.height );

    for ( int o = 0; o < conveyor.getBlob(3).outline.size(); o++ )
    {
      EXPECT_EQ ( conveyor.getBlob(3).outline[o].x,
        originConveyor.getBlob(4).outline[o].x );
      EXPECT_EQ ( conveyor.getBlob(3).outline[o].y,
        originConveyor.getBlob(4).outline[o].y );
    }

    // Restore conveyor to its original state
    conveyor.replace(originConveyor);

    // Run HoleMerger::applyMergeOperation for all operations
    for ( int i = 0; i < 3; i++ )
    {
      HoleMerger::applyMergeOperation(
        &conveyor,
        squares_,
        cloud,
        i);
    }
    // The number of holes should have shrunk to two
    EXPECT_EQ ( 2, conveyor.size() );

    // Apply preposterous thresholds
    Parameters::HoleFusion::Merger::depth_diff_threshold = 1.0;
    Parameters::HoleFusion::Merger::depth_area_threshold = 1.0;

    // Restore conveyor to its original state
    conveyor.replace(originConveyor);

    // Run HoleMerger::applyMergeOperation for all operations
    for ( int i = 0; i < 3; i++ )
    {
      HoleMerger::applyMergeOperation(
        &conveyor,
        squares_,
        cloud,
        i);
    }

    // No assimilation or amalgamation or connection should have happened
    EXPECT_EQ ( 5, conveyor.size() );
  }

  //! Tests HoleMerger::applyMergeOperationWithoutValidation
  TEST_F ( HoleMergerTest, applyMergeOperationWithoutValidation )
  {
    // Keep a backup of the original conveyor
    BlobVector originConveyor;
    originConveyor.copy(conveyor);

    // Run HoleMerger::applyMergeOperationWithoutValidation
    // for operationId = 0 : assimilation
    HoleMerger::applyMergeOperationWithoutValidation(
      &conveyor,
      squares_,
      ASSIMILATION );

    // The number of holes should have shrunk by one
    EXPECT_EQ ( conveyor.size(), originConveyor.size() - 1 );

    // Entry #0 should be intact
    EXPECT_EQ ( conveyor.getBlob(0).areaOfInterest.center.x, 
      originConveyor.getBlob(0).areaOfInterest.center.x);
    EXPECT_EQ ( conveyor.getBlob(0).areaOfInterest.center.y, 
      originConveyor.getBlob(0).areaOfInterest.center.y);

    EXPECT_EQ ( conveyor.getBlob(0).areaOfInterest.width, 
      originConveyor.getBlob(0).areaOfInterest.width);
    EXPECT_EQ ( conveyor.getBlob(0).areaOfInterest.height, 
      originConveyor.getBlob(0).areaOfInterest.height);

    for ( int o = 0; o < conveyor.getBlob(0).outline.size(); o++ )
    {
      EXPECT_EQ ( conveyor.getBlob(0).outline[o].x,
        originConveyor.getBlob(0).outline[o].x );

      EXPECT_EQ ( conveyor.getBlob(0).outline[o].y,
        originConveyor.getBlob(0).outline[o].y );
    }

    // Original entry #2 should now be entry #1
    // Original entry #3 should now be entry #2
    // Original entry #4 should now be entry #3
    for ( int i = 2; i < originConveyor.size(); i++ )
    {
      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.center.x,
        originConveyor.getBlob(i).areaOfInterest.center.x );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.center.y,
        originConveyor.getBlob(i).areaOfInterest.center.y );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.width,
        originConveyor.getBlob(i).areaOfInterest.width );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.height,
        originConveyor.getBlob(i).areaOfInterest.height );

      for ( int o = 0; o < conveyor.getBlob(i - 1).outline.size(); o++ )
      {
        EXPECT_EQ ( conveyor.getBlob(i - 1).outline[o].x,
          originConveyor.getBlob(i).outline[o].x );

        EXPECT_EQ ( conveyor.getBlob(i - 1).outline[o].y,
          originConveyor.getBlob(i).outline[o].y );
      }
    }
    // Restore conveyor to its original state
    conveyor.replace(originConveyor);

    // Run HoleMerger::applyMergeOperationWithoutValidation
    // for operationId = 1 : amalgamation
    HoleMerger::applyMergeOperationWithoutValidation(
      &conveyor,
      squares_,
      AMALGAMATION );

    // The number of holes should have shrunk by one
    EXPECT_EQ ( conveyor.size(), originConveyor.size() - 1 );

    // The amalgamator should have grown in terms of outline points
    EXPECT_LT ( originConveyor.getBlob(0).outline.size(),
      conveyor.getBlob(0).outline.size() );

    // The amalgamator's keypoint should have moved a tiny bit to the right,
    // but not significantly vertically
    EXPECT_LT ( originConveyor.getBlob(0).areaOfInterest.center.x,
      conveyor.getBlob(0).areaOfInterest.center.x );
    EXPECT_NEAR ( originConveyor.getBlob(0).areaOfInterest.center.y,
      conveyor.getBlob(0).areaOfInterest.center.y, 1 );

    // Entry #1 should be intact
    EXPECT_EQ ( conveyor.getBlob(1).areaOfInterest.center.x,
      originConveyor.getBlob(1).areaOfInterest.center.x );

    EXPECT_EQ ( conveyor.getBlob(1).areaOfInterest.center.y,
      originConveyor.getBlob(1).areaOfInterest.center.y );

    EXPECT_EQ ( conveyor.getBlob(1).areaOfInterest.width,
      originConveyor.getBlob(1).areaOfInterest.width );

    EXPECT_EQ ( conveyor.getBlob(1).areaOfInterest.height,
      originConveyor.getBlob(1).areaOfInterest.height );

    for ( int o = 0; o < conveyor.getBlob(1).outline.size(); o++ )
    {
      EXPECT_EQ ( conveyor.getBlob(1).outline[o].x,
        originConveyor.getBlob(1).outline[o].x );

      EXPECT_EQ ( conveyor.getBlob(1).outline[o].y,
        originConveyor.getBlob(1).outline[o].y );
    }

    for ( int i = 3; i < 5; i++ )
    {
      // Original entry #3 should now be #2
      // Original entry #4 should now be #3
      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.center.x,
        originConveyor.getBlob(i).areaOfInterest.center.x );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.center.y,
        originConveyor.getBlob(i).areaOfInterest.center.y );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.width,
        originConveyor.getBlob(i).areaOfInterest.width );

      EXPECT_EQ ( conveyor.getBlob(i - 1).areaOfInterest.height,
        originConveyor.getBlob(i).areaOfInterest.height );

      for ( int o = 0; o < conveyor.getBlob(i - 1).outline.size(); o++ )
      {
        EXPECT_EQ ( conveyor.getBlob(i - 1).outline[o].x,
          originConveyor.getBlob(i).outline[o].x );

        EXPECT_EQ ( conveyor.getBlob(i - 1).outline[o].y,
          originConveyor.getBlob(i).outline[o].y );
      }
    }
    // Restore conveyor to its original state
    conveyor.replace(originConveyor);

    // Run HoleMerger::applyMergeOperationWithoutValidation
    // for all operations
    for ( int i = 0; i < 3; i++ )
    {
      HoleMerger::applyMergeOperationWithoutValidation(
        &conveyor,
        squares_,
        i);
    }

    // The number of holes should have shrunk to three
    EXPECT_EQ ( 3, conveyor.size() );
  }

  //! Tests HoleMerger::isCapableOfAssimilating
  TEST_F ( HoleMergerTest, isCapableOfAssimilatingTest )
  {
    // Construct the hole mask sets for all the holes
    // Here, the main square will be the assimilator, amalgamator and connector

    std::vector< std::set< unsigned int > > holesMasksSetVector;
    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );

    for ( int i = 1; i < conveyor.size(); i++ )
    {
      // Run HoleMerger::isCapableOfAssimilating
      bool result = HoleMerger::isCapableOfAssimilating(
        holesMasksSetVector[0],
        holesMasksSetVector[i] );

      // The main square should be able to assimilate only the assimilable
      if ( i == 1 )
      {
        EXPECT_EQ ( true, result );
      }
      else
      {
        EXPECT_EQ ( false, result );
      }
    }
  }

  //! Tests HoleMerger::isCapableOfAmalgamating
  TEST_F ( HoleMergerTest, isCapableOfAmalgamatingTest )
  {
    // Construct the hole mask sets for all the holes
    // Here, the main square will be the assimilator, amalgamator and connector

    std::vector< std::set< unsigned int > > holesMasksSetVector;
    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );

    for ( int i = 1; i < conveyor.size(); i++ )
    {
      // Run HoleMerger::isCapableOfAmalgamating
      bool result = HoleMerger::isCapableOfAmalgamating (
        holesMasksSetVector[0],
        holesMasksSetVector[i] );

      // The main square should be able to amalgamate only the amalgamatable
      if ( i == 2 )
      {
        EXPECT_EQ ( true, result );
      }
      else
      {
        EXPECT_EQ ( false, result );
      }
    }
  }

  //! Tests HoleMerger::amalgamateOnce
  TEST_F ( HoleMergerTest, amalgamateOnceTest )
  {
    // Construct the hole mask sets for all the holes
    // Here, the main square will be the assimilator, amalgamator and connector

    std::vector< std::set< unsigned int > > holesMasksSetVector;
    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );

    // Keep a backup of the original amalgamator
    BlobVector amalgamator;
    amalgamator.append(conveyor.getBlob(0));

    HoleMerger::amalgamateOnce(&conveyor,
      0,
      &holesMasksSetVector[0],
      holesMasksSetVector[2],
      squares_ );

    // The amalgamator should have grown in terms of outline points
    EXPECT_LT ( amalgamator.getBlob(0).outline.size(),
     conveyor.getBlob(0).outline.size() );

    // The amalgamator's keypoint should have moved a tiny bit to the right,
    // but not significantly vertically
    EXPECT_LT ( amalgamator.getBlob(0).areaOfInterest.center.x ,
      conveyor.getBlob(0).areaOfInterest.center.x );
    EXPECT_NEAR ( amalgamator.getBlob(0).areaOfInterest.center.y ,
      conveyor.getBlob(0).areaOfInterest.center.y, 1 );
  }

  //! Tests HoleMerger::isCapableOfConnecting
  TEST_F ( HoleMergerTest, isCapableOfConnectingTest )
  {
    // Construct the hole mask sets for all the holes
    // Here, the main square will be the assimilator, amalgamator and connector

    std::vector< std::set< unsigned int > > holesMasksSetVector;
    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );

    // Modify the connection parameters
    Parameters::HoleFusion::Merger::connect_holes_min_distance = 3;
    Parameters::HoleFusion::Merger::connect_holes_max_distance = 20;

    for ( int i = 1; i < conveyor.size(); i++ )
    {
      // Run HoleMerger::isCapableOfConnecting
      bool result = HoleMerger::isCapableOfConnecting(
        conveyor,
        0,
        i,
        holesMasksSetVector[0],
        holesMasksSetVector[i],
        cloud );

      // The main square should be able to amalgamate only the amalgamatable
      if ( i == 3 )
      {
        EXPECT_EQ ( true, result );
      }
      else
      {
        EXPECT_EQ ( false, result );
      }
    }

    // Modify the connection parameters so as to test that not only distance
    // plays a role. There has to be mutual exclusion regarding the points
    // inside each hole too
    Parameters::HoleFusion::Merger::connect_holes_min_distance = 10;
    Parameters::HoleFusion::Merger::connect_holes_max_distance = 100;

    for ( int i = 1; i < conveyor.size(); i++ )
    {
      // Run HoleMerger::isCapableOfConnecting
      bool result = HoleMerger::isCapableOfConnecting(
        conveyor,
        0,
        i,
        holesMasksSetVector[0],
        holesMasksSetVector[i],
        cloud );

      // The main square should be able to connect only with the connectable
      if ( i == 3 )
      {
        EXPECT_EQ ( true, result );
      }
      else
      {
        EXPECT_EQ ( false, result );
      }
    }

    // Modify the connection parameters so as to test that not only distance
    // plays a role. There has to be mutual exclusion regarding the points
    // inside each hole too
    Parameters::HoleFusion::Merger::connect_holes_min_distance = 100;
    Parameters::HoleFusion::Merger::connect_holes_max_distance = 10;

    for ( int i = 1; i < conveyor.size(); i++ )
    {
      // Run HoleMerger::isCapableOfConnecting
      bool result = HoleMerger::isCapableOfConnecting(
        conveyor,
        0,
        i,
        holesMasksSetVector[0],
        holesMasksSetVector[i],
        cloud );

      // The connectable should not be able to be connected with the
      // connector for max_distance = 10
      EXPECT_EQ ( false, result );
    }

    // Default the connection parameters
    Parameters::HoleFusion::Merger::connect_holes_min_distance = 3;
    Parameters::HoleFusion::Merger::connect_holes_max_distance = 20;
  }

  //! Tests HoleMerger::connectOnce
  TEST_F ( HoleMergerTest, connectOnceTest )
  {
    // Construct the hole mask sets for all the holes
    // Here, the main square will be the assimilator, amalgamator and connector

    std::vector< std::set< unsigned int > > holesMasksSetVector;
    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );

    // Keep a backup of the original amalgamator
    BlobVector connector;
    connector.append(conveyor.getBlob(0));

    // Run HoleMerger::connectOnce
    HoleMerger::connectOnce(
      &conveyor,
      0,
      3,
      &holesMasksSetVector[0],
      squares_ );

    // The connector should have grown in terms of outline points
    EXPECT_LT ( connector.getBlob(0).outline.size(),
     conveyor.getBlob(0).outline.size() );

    // The connector's keypoint should have moved a bit to the left,
    // and a bit lower than before
    EXPECT_GT ( connector.getBlob(0).areaOfInterest.center.x,
      conveyor.getBlob(0).areaOfInterest.center.x );
    EXPECT_LT ( connector.getBlob(0).areaOfInterest.center.y,
      conveyor.getBlob(0).areaOfInterest.center.y );
  }

  //! Tests HoleMerger::mergeHoles
  TEST_F ( HoleMergerTest, mergeHolesTest )
  {
    // Keep a backup of the original conveyor
    BlobVector originConveyor;
    originConveyor.copy(conveyor);

    // The interpolation method. 0 for using depth filters validation
    int interpolationMethod = 0;

    // Apply reasonable thresholds
    Parameters::HoleFusion::Merger::depth_diff_threshold = 0.4;
    Parameters::HoleFusion::Merger::depth_area_threshold = 0.8;

    // Restore conveyor to its original state
    //HolesConveyorUtils::replace( originConveyor, &conveyor );

    // Run HoleMerger::mergeHoles
    HoleMerger::mergeHoles(
      &conveyor,
      interpolationMethod,
      squares_,
      cloud);

    // The number of holes should have shrunk to two
    EXPECT_EQ ( 2, conveyor.size() );

    // The interpolation method. 1 for not using depth filters validation
    interpolationMethod = 1;

    // Restore conveyor to its original state
    conveyor.replace(originConveyor);

    // Run HoleMerger::mergeHoles
    HoleMerger::mergeHoles(
      &conveyor,
      interpolationMethod,
      squares_,
      cloud);

    // The number of holes should have shrunk to three
    EXPECT_EQ ( 3, conveyor.size() );
  }

}  // namespace pandora_vision

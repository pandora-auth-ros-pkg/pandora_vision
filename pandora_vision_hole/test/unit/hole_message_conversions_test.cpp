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

#include <math.h>
#include "hole_message_conversions.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class HoleMessageConversionsTest
    @brief Tests the integrity of methods of class HoleMessageConversions
   **/
  class HoleMessageConversionsTest : public ::testing::Test
  {
    protected:

      HoleMessageConversionsTest() {}

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // An image of dimensions HEIGHT x WIDTH, representing the conveyor
        image = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        for ( int rows = 100; rows < 200; rows++ )
        {
          image.at< unsigned char >( rows, 100 ) = 255;
          image.at< unsigned char >( rows, 199 ) = 255;
        }

        for ( int cols = 100; cols < 200; cols++ )
        {
          image.at< unsigned char >( 100, cols ) = 255;
          image.at< unsigned char >( 199, cols ) = 255;
        }

        // A single hole
        HoleConveyor hole;

        // Construct a dummy conveyor
        cv::KeyPoint k_1 ( 150, 150, 1 );

        hole.keypoint = k_1;

        // The outline of the hole
        std::vector< cv::Point2f > outline;

        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++ )
          {
            if ( image.at< unsigned char >( rows, cols ) != 0)
            {
              outline.push_back( cv::Point2f( cols, rows ) );
            }
          }
        }

        hole.outline = outline;

        // The vertices of the hole's bounding box
        std::vector< cv::Point2f > rectangle;

        rectangle.push_back( cv::Point2f( 100, 100 ) );
        rectangle.push_back( cv::Point2f( 100, 199 ) );
        rectangle.push_back( cv::Point2f( 199, 199 ) );
        rectangle.push_back( cv::Point2f( 199, 100 ) );

        hole.rectangle = rectangle;

        // Push hole back into the conveyor
        conveyor.holes.push_back( hole );
      }

      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // A conveyor of dummy holes
      HolesConveyor conveyor;

      // An image of dimensions HEIGHT x WIDTH, representing the conveyor
      cv::Mat image;
  };


  //! Tests HoleMessageConversions::createCandidateHolesVector
  TEST_F ( HoleMessageConversionsTest, createCandidateHolesVectorTest )
  {
    // The vector of messages of candidate holes
    std::vector<pandora_vision_hole::CandidateHoleMsg> candidateHolesVector;

    // Run HoleMessageConversions::createCandidateHolesVector
    HoleMessageConversions::createCandidateHolesVector( conveyor,
      &candidateHolesVector );

    for (unsigned int i = 0; i < conveyor.size(); i++)
    {
      // The keypoints should be the same
      EXPECT_EQ ( conveyor.holes[i].keypoint.pt.x,
        candidateHolesVector[i].keypointX );
      EXPECT_EQ ( conveyor.holes[i].keypoint.pt.y,
        candidateHolesVector[i].keypointY );

      for (int v = 0; v < conveyor.holes[i].rectangle.size(); v++)
      {
        // The rectangle's vertices should be the same
        EXPECT_EQ ( conveyor.holes[i].rectangle[v].x,
          candidateHolesVector[i].verticesX[v]);
        EXPECT_EQ ( conveyor.holes[i].rectangle[v].y,
          candidateHolesVector[i].verticesY[v]);
      }

      for (int o = 0; o < conveyor.holes[i].outline.size(); o++)
      {
        // The outline points should be the same
        EXPECT_EQ ( conveyor.holes[i].outline[o].x,
          candidateHolesVector[i].outlineX[o]);
        EXPECT_EQ ( conveyor.holes[i].outline[o].y,
          candidateHolesVector[i].outlineY[o]);
      }
    }
  }



  //! Tests HoleMessageConversions::createCandidateHolesVectorMessage
  TEST_F ( HoleMessageConversionsTest, createCandidateHolesVectorMessageTest )
  {
    // Create a grayscale image
    cv::Mat image= cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    // Fill it with randomness
    unsigned int seed = 0;
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        image.at< float >( rows, cols ) =
          static_cast< float >( rand_r(&seed) % 2 );
      }
    }

    // The message of candidate holes
    pandora_vision_hole::CandidateHolesVectorMsg candidateHolesVectorMsg;

    // A dummy image. Needed only for its header.
    sensor_msgs::Image msg;
    msg.header.frame_id = "a";

    // Run HoleMessageConversions::createCandidateHolesVector
    HoleMessageConversions::createCandidateHolesVectorMessage(
      conveyor,
      image,
      &candidateHolesVectorMsg,
      sensor_msgs::image_encodings::TYPE_8UC1,
      msg);

    // Check the integrity of image
    cv::Mat extracted;
    HoleMessageConversions::extractImageFromMessageContainer(
      candidateHolesVectorMsg,
      &extracted,
      sensor_msgs::image_encodings::TYPE_8UC1);

    // The number of pixels differing between image and extracted
    int diff = 0;

    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        if ( image.at< unsigned char >( rows, cols ) !=
          extracted.at< unsigned char >( rows, cols ))
        {
          diff++;
        }
      }
    }

    // There should be no discrepancies
    ASSERT_EQ ( 0, diff );


    // Check the integrity of the conveyor
    for (unsigned int i = 0; i < conveyor.size(); i++)
    {
      // The keypoints should be the same
      EXPECT_EQ ( conveyor.holes[i].keypoint.pt.x,
        candidateHolesVectorMsg.candidateHoles[i].keypointX );
      EXPECT_EQ ( conveyor.holes[i].keypoint.pt.y,
        candidateHolesVectorMsg.candidateHoles[i].keypointY );

      for (int v = 0; v < conveyor.holes[i].rectangle.size(); v++)
      {
        // The rectangle's vertices should be the same
        EXPECT_EQ ( conveyor.holes[i].rectangle[v].x,
          candidateHolesVectorMsg.candidateHoles[i].verticesX[v]);
        EXPECT_EQ ( conveyor.holes[i].rectangle[v].y,
          candidateHolesVectorMsg.candidateHoles[i].verticesY[v]);
      }

      for (int o = 0; o < conveyor.holes[i].outline.size(); o++)
      {
        // The outline points should be the same
        EXPECT_EQ ( conveyor.holes[i].outline[o].x,
          candidateHolesVectorMsg.candidateHoles[i].outlineX[o]);
        EXPECT_EQ ( conveyor.holes[i].outline[o].y,
          candidateHolesVectorMsg.candidateHoles[i].outlineY[o]);
      }
    }

  }


  //! Tests HoleMessageConversions::extractImageFromMessageContainer
  TEST_F ( HoleMessageConversionsTest, extractImageFromMessageContainerTest )
  {
    // Create a grayscale image
    cv::Mat image = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    // Fill it with randomness
    unsigned int seed = 0;
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        image.at< unsigned char >( rows, cols ) =
          static_cast< unsigned char >( rand_r(&seed) % 2 );
      }
    }

    cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());
    msgPtr->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    msgPtr->image = image;

    // The message of candidate holes
    pandora_vision_hole::CandidateHolesVectorMsg msg;
    msg.image = *msgPtr->toImageMsg();

    // Run HoleMessageConversions::extractImageFromMessageContainer
    cv::Mat extracted;
    HoleMessageConversions::extractImageFromMessageContainer( msg,
      &extracted, sensor_msgs::image_encodings::TYPE_8UC1 );

    // The number of pixels differing between image and extracted
    int diff = 0;
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        if ( image.at< unsigned char >( rows, cols ) !=
          extracted.at< unsigned char >( rows, cols ))
        {
          diff++;
        }
      }
    }

    // There should be no discrepancies
    ASSERT_EQ ( 0 , diff );
  }



  //! Tests HoleMessageConversions::fromCandidateHoleMsgToConveyor
  TEST_F ( HoleMessageConversionsTest, fromCandidateHoleMsgToConveyorTest )
  {
    // The vector of candidate holes
    std::vector<pandora_vision_hole::CandidateHoleMsg> candidateHolesVector;

    for ( int i = 0; i < conveyor.size(); i++ )
    {
      pandora_vision_hole::CandidateHoleMsg candidateHole;

      // The keypoints
      candidateHole.keypointX = conveyor.holes[i].keypoint.pt.x;
      candidateHole.keypointY = conveyor.holes[i].keypoint.pt.y;

      // The rectangles
      for ( int v = 0; v < conveyor.holes[i].rectangle.size(); v++ )
      {
        candidateHole.verticesX.push_back(conveyor.holes[i].rectangle[v].x);
        candidateHole.verticesY.push_back(conveyor.holes[i].rectangle[v].y);
      }

      // The outlines
      for ( int o = 0; o < conveyor.holes[i].outline.size(); o++ )
      {
        candidateHole.outlineX.push_back(conveyor.holes[i].outline[o].x);
        candidateHole.outlineY.push_back(conveyor.holes[i].outline[o].y);
      }

      candidateHolesVector.push_back(candidateHole);
    }

    // The conveyor extracted from the message
    HolesConveyor extractedConveyor;

    // Run HoleMessageConversions::fromCandidateHoleMsgToConveyor
    // for representationMethod 0
    HoleMessageConversions::fromCandidateHoleMsgToConveyor(
      candidateHolesVector,
      &extractedConveyor,
      image,
      0,
      32);

    // Check the integrity of the extracted conveyor
    for ( int i = 0; i < conveyor.size(); i++ )
    {
      EXPECT_EQ ( extractedConveyor.holes[i].keypoint.pt.x,
        conveyor.holes[i].keypoint.pt.x );

      EXPECT_EQ ( extractedConveyor.holes[i].keypoint.pt.y,
        conveyor.holes[i].keypoint.pt.y );

      for ( int v = 0; v < conveyor.holes[i].rectangle.size(); v++ )
      {
        EXPECT_EQ ( extractedConveyor.holes[i].rectangle[v].x,
          conveyor.holes[i].rectangle[v].x );
        EXPECT_EQ ( extractedConveyor.holes[i].rectangle[v].y,
          conveyor.holes[i].rectangle[v].y );
      }

      for ( int o = 0; o < conveyor.holes[i].outline.size(); o++ )
      {
        EXPECT_EQ ( extractedConveyor.holes[i].outline[o].x,
          conveyor.holes[i].outline[o].x );
        EXPECT_EQ ( extractedConveyor.holes[i].outline[o].y,
          conveyor.holes[i].outline[o].y );
      }
    }

    // Clear the conveyor
    HolesConveyorUtils::clear( &extractedConveyor );

    // Run HoleMessageConversions::fromCandidateHoleMsgToConveyor
    // for representationMethod 1
    HoleMessageConversions::fromCandidateHoleMsgToConveyor(
      candidateHolesVector,
      &extractedConveyor,
      image,
      1,
      32);

    // Check the inflated of the extracted conveyor
    for ( int i = 0; i < conveyor.size(); i++ )
    {
      EXPECT_EQ ( extractedConveyor.holes[i].keypoint.pt.x,
        2 * conveyor.holes[i].keypoint.pt.x );

      EXPECT_EQ ( extractedConveyor.holes[i].keypoint.pt.y,
        2 * conveyor.holes[i].keypoint.pt.y );

      EXPECT_EQ ( extractedConveyor.holes[i].rectangle.size(),
        conveyor.holes[i].rectangle.size() );

      EXPECT_GT ( extractedConveyor.holes[i].outline.size(),
        conveyor.holes[i].outline.size() );
    }
  }



  //! Tests HoleMessageConversions::unpackMessage
  TEST_F ( HoleMessageConversionsTest, unpackMessageTest )
  {
    // The overall message
    pandora_vision_hole::CandidateHolesVectorMsg candidateHolesVectorMsg;

    // The vector of candidate holes
    std::vector<pandora_vision_hole::CandidateHoleMsg> candidateHolesVector;

    for ( int i = 0; i < conveyor.size(); i++ )
    {
      pandora_vision_hole::CandidateHoleMsg candidateHole;

      // The keypoints
      candidateHole.keypointX = conveyor.holes[i].keypoint.pt.x;
      candidateHole.keypointY = conveyor.holes[i].keypoint.pt.y;

      // The rectangle points
      for ( int v = 0; v < conveyor.holes[i].rectangle.size(); v++ )
      {
        candidateHole.verticesX.push_back(conveyor.holes[i].rectangle[v].x);
        candidateHole.verticesY.push_back(conveyor.holes[i].rectangle[v].y);
      }

      // The outline points
      for ( int o = 0; o < conveyor.holes[i].outline.size(); o++ )
      {
        candidateHole.outlineX.push_back(conveyor.holes[i].outline[o].x);
        candidateHole.outlineY.push_back(conveyor.holes[i].outline[o].y);
      }

      candidateHolesVector.push_back(candidateHole);
    }

    candidateHolesVectorMsg.candidateHoles = candidateHolesVector;

    // A grayscale image
    cv::Mat image_8UC1 = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    // Insert randomness into image
    unsigned int seed = 0;

    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        image_8UC1.at< unsigned char >( rows, cols ) =
          static_cast< unsigned char >(rand_r( &seed ) % 255);
      }
    }

    // Pack image_8UC1 into the message
    cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

    msgPtr->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    msgPtr->image = image_8UC1;

    candidateHolesVectorMsg.image = *msgPtr->toImageMsg();

    // The extracted conveyor
    HolesConveyor extractedConveyor;

    // The extracted image
    cv::Mat extractedImage;

    // Run HoleMessageConversions::unpackMessage for representationMethod 0
    HoleMessageConversions::unpackMessage(
      candidateHolesVectorMsg,
      &extractedConveyor,
      &extractedImage,
      0,
      sensor_msgs::image_encodings::TYPE_8UC1,
      32);


    // Check the integrity of the extracted conveyor
    for ( int i = 0; i < conveyor.size(); i++ )
    {
      EXPECT_EQ ( extractedConveyor.holes[i].keypoint.pt.x,
        conveyor.holes[i].keypoint.pt.x );

      EXPECT_EQ ( extractedConveyor.holes[i].keypoint.pt.y,
        conveyor.holes[i].keypoint.pt.y );

      for ( int v = 0; v < conveyor.holes[i].rectangle.size(); v++ )
      {
        EXPECT_EQ ( extractedConveyor.holes[i].rectangle[v].x,
          conveyor.holes[i].rectangle[v].x );
        EXPECT_EQ ( extractedConveyor.holes[i].rectangle[v].y,
          conveyor.holes[i].rectangle[v].y );
      }

      for ( int o = 0; o < conveyor.holes[i].outline.size(); o++ )
      {
        EXPECT_EQ ( extractedConveyor.holes[i].outline[o].x,
          conveyor.holes[i].outline[o].x );
        EXPECT_EQ ( extractedConveyor.holes[i].outline[o].y,
          conveyor.holes[i].outline[o].y );
      }
    }


    // The number of pixels differing between image_8UC1 and extractedImage
    int diff = 0;
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        if ( image_8UC1.at< unsigned char >( rows, cols ) !=
          extractedImage.at< unsigned char >( rows, cols ))
        {
          diff++;
        }
      }
    }

    // There should be no discrepancies
    ASSERT_EQ ( 0 , diff );


    // Clear extractedConveyor
    HolesConveyorUtils::clear( &extractedConveyor );

    // Run HoleMessageConversions::unpackMessage for representationMethod 1
    HoleMessageConversions::unpackMessage(
      candidateHolesVectorMsg,
      &extractedConveyor,
      &extractedImage,
      1,
      sensor_msgs::image_encodings::TYPE_8UC1,
      32);

    // Check the inflated of the extracted conveyor
    for ( int i = 0; i < conveyor.size(); i++ )
    {
      EXPECT_EQ ( extractedConveyor.holes[i].keypoint.pt.x,
        2 * conveyor.holes[i].keypoint.pt.x );

      EXPECT_EQ ( extractedConveyor.holes[i].keypoint.pt.y,
        2 * conveyor.holes[i].keypoint.pt.y );

      EXPECT_EQ ( extractedConveyor.holes[i].rectangle.size(),
        conveyor.holes[i].rectangle.size() );

      EXPECT_GT ( extractedConveyor.holes[i].outline.size(),
        conveyor.holes[i].outline.size() );
    }

    // The number of pixels differing between image_8UC1 and extractedImage
    diff = 0;
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        if ( image_8UC1.at< unsigned char >( rows, cols ) !=
          extractedImage.at< unsigned char >( rows, cols ))
        {
          diff++;
        }
      }
    }

    // There should be no discrepancies
    ASSERT_EQ ( 0 , diff );
  }

} // namespace pandora_vision

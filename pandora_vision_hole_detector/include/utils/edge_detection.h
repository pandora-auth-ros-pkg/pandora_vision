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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef UTILS_EDGE_DETECTION_H
#define UTILS_EDGE_DETECTION_H

#include "utils/histogram.h"
#include "utils/morphological_operators.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  struct GraphNode
  {
    GraphNode(int x, int y)
    {
      this->x = x;
      this->y = y;
    }

    GraphNode(void)
    {
      this->x = 0;
      this->y = 0;
    }

    int x;
    int y;

    std::vector<GraphNode*> connections;

    std::vector<int> dists;

    unsigned int dist;
  };

  /**
    @class EdgeDetection
    @brief Provides methods for edge detection
   **/
  class EdgeDetection
  {
    public:

      /**
        @brief Applies the Canny edge transform
        @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
        @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
        @return void
       **/
      static void applyCanny (const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Applies the Scharr edge transform
        @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
        @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
        @return void
       **/
      static void applyScharr (const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Applies the Sobel edge transform
        @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
        @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
        @return void
       **/
      static void applySobel (const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Applies the Laplacian edge transform
        @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
        @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
        @return void
       **/
      static void applyLaplacian (const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Applies contamination to the edges image. It keeps only the edges
        that are not iteratively neighbors to the image's limits
        @param[in,out] inImage [cv::Mat*] Input image in CV_8UC1 format
        @return void
       **/
      static void applyEdgeContamination (cv::Mat* inImage);

      /**
        @brief Takes as input a depth image containing floats,
        locates the edges in it and tries to clear as much noise as possible
        in the edges image. As noise we identify everything that is not,
        or does not look like, hole-like shapes,
        with the knowledge that these shapes might be open curves, or that
        holes-like shapes in a edge image are not connected to anything else,
        ergo they are standalone shapes in it.
        It outputs a binary image that contains areas that we wish to validate
        as holes.
        @param[in] inImage [const cv::Mat&] The depth image extracted from the
        depth sensor, of type CV_32FC1
        @param[out] edges [cv::Mat*] The final denoised edges image that
        corresponds to the input image
        @return void
       **/
      static void computeDepthEdges(const cv::Mat& inImage, cv::Mat* edges);

      /**
        @brief Takes as input a RGB image of type CV_8UC3,
        locates the edges in it and tries to clear as much noise as possible
        in the edges image. As noise we identify everything that is not,
        or does not look like, hole-like shapes,
        with the knowledge that these shapes might be open curves, or that
        holes-like shapes in a edge image are not connected to anything else,
        ergo they are standalone shapes in it.
        It outputs a binary image that contains areas that we wish to validate
        as holes.
        @param[in] inImage [const cv::Mat&] The RGB image of type CV_32FC1
        @param[in] extractionMethod [const int&] Chooses by which process the
        edges will be extracted. 0 for extraction via segmentation,
        1 for extraction via backprojection and watersheding
        @param[in] inHistogram [const cv::MatND&] The model histogram needed
        in order to obtain the backprojection of @param inImage
        @param[out] edges [cv::Mat*] The final denoised edges image that
        corresponds to the input image
        @return void
       **/
      static void computeRgbEdges(const cv::Mat& inImage,
        const int& extractionMethod,
        const cv::MatND& inHistogram,
        cv::Mat* edges);

      /**
        @brief Connects each point of a number of pair of points  with a line or
        an elliptic arc
        @param[in,out] inImage [cv::Mat*] The image whose selected points will
        be connected and line or the elliptic arc drawn on
        @param[in] pairs [const std::vector<std::pair<GraphNode,GraphNode> >&]
        The vector of pair points
        @param[in] method [const int&] Denotes the connection type. 0 for line,
        1 for elliptic arc
        @return void
       **/
      static void connectPairs(cv::Mat* inImage,
        const std::vector<std::pair<GraphNode, GraphNode> >& pairs,
        const int& method);

      /**
        @brief Enhances an image's contrast, regardless of its format.
        @param[in] inImage [const cv::Mat &] The input image
        @param[out] outImage [cv::Mat*] The contrast-enhanced image
        @return void
       **/
      static void enhanceContrast(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Takes an input image in unsigned char format and tries to isolate
        hole-like shapes so as to facilitate the blob detection process
        @param[in,out] img [cv::Mat*] The input image in unsigned char format
        @return void
       **/
      static void denoiseEdges(cv::Mat* img);

      /**
        @brief Identifies in which curve a point lies on and returns the
        curve's two end points. If a point does not lie on a curve,
        the pair returned has both coordinates (x,y) set to zero, and the
        size of @param ret is one.
        @param[in] img [cv::Mat*] The input binary image
        @param[in] x_ [const int&] The x coordinate of the point
        @param[in] y_ [const int&] The y coordinate of the point
        @param[out] ret [std::set<unsigned int>&] The points that represent the
        curve on which the point lies on
        @return edgePoints [std::pair<GraphNode, GraphNode>*] The curve's pair
        of end points
       **/
      static std::pair<GraphNode, GraphNode> findNeighs(cv::Mat* img,
        const int& x_, const int& y_, std::set<unsigned int>* ret);

      /**
        @brief Fills an image with random colours per image segment
        @param[in,out] image [cv::Mat*] The image to be processed
        (see http://docs.opencv.org/modules/imgproc/doc/
        miscellaneous_transformations.html#floodfill)
        @return void
       **/
      static void floodFillPostprocess(cv::Mat* image);

      /**
        @brief With an binary input image (quantized in 0 and 255 levels),
        this function fills closed regions, at first, and then extracts the
        outline of each region. Used when there is a closed region with garbage
        pixels with a value of 255 within it.
        @param[in,out] inImage [cv::Mat*] The input image
        @return void
       **/
      static void getShapesClearBorder (cv::Mat* inImage);

      /**
        @brief With an binary input image (quantized in 0 and 255 levels),
        this function fills closed regions, at first, and then extracts the
        outermost outline of each region.
        Used when there is a closed region with garbage pixels with
        a value of 255 within it.
        Caution: Only the outermost outline of shapes is computed. If there are
        closed shapes inside of other closed shapes, only the latter's outline
        will be computed, as opposed to the getShapesClearBorder function
        @param[in,out] inImage [cv::Mat*] The input image
        @return void
       **/
      static void getShapesClearBorderSimple (cv::Mat* inImage);

      /**
        @brief This method takes as input a RGB image and uses
        its backprojection (which is based on the precalculated
        histogram histogram_) in order to identify whole regions whose
        histogram matches histogram_, although the backprojection image
        might be sparcely populated. After the identification of the regions
        of interest, this method extracts their edges and returns the image
        depicting them.
        @param[in] inImage [const cv::Mat&] The input RGB image
        @param[in] inHistogram [const cv::MatND&] The model histogram needed
        in order to obtain the backprojection of @param inImage
        @param[out] outImage [cv::Mat*] The output edges image, in CV_8UC1
        format
        @return void
       **/
      static void produceEdgesViaBackprojection (const cv::Mat& inImage,
        const cv::MatND& inHistogram, cv::Mat* outImage);

      /**
        @brief This method takes as input a RGB image, segments it,
        and extracts its edges.
        @param[in] inImage [const cv::Mat&] The input RGB image,
        of type CV_8UC3
        @param[out] outImage [cv::Mat*] The output edges image,
        of type CV_8UC1
        @return void
       **/
      static void produceEdgesViaSegmentation (const cv::Mat& inImage,
        cv::Mat* edges);

      /**
        @brief Segments a RGB image
        @param[in] inImage [const cv::Mat&] The RGB image to be segmented
        @param[out] outImage [cv::Mat*] The posterized image
        @return void
       **/
      static void segmentation(const cv::Mat& inImage, cv::Mat* outImage);

  };

} // namespace pandora_vision

#endif  // UTILS_EDGE_DETECTION_H

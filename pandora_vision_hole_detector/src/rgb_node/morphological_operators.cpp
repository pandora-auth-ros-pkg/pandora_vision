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

#include "rgb_node/morphological_operators.h"

namespace pandora_vision
{
  /**
    @brief Checks if a kernel in a specific point in an image is satisfied
    @param kernal [const char [3][3]] The kernel
    @param img [const cv::Mat&] The input image (uchar)
    @param center [const cv::Point&] The center of the kernel
    @return bool : True on match
   **/
  bool Morphology::kernelCheck(const char kernel[3][3], const cv::Mat& img,
    const cv::Point& center)
  {
    static unsigned char *ptr;
    ptr = (unsigned char *) img.data;

    for(int i = -1; i <= 1; i++)
    {
      for(int j = -1; j <= 1; j++)
      {
        if(kernel[i+1][j+1] == 0 &&
            ptr[ (center.y + j) * img.cols + center.x + i ] != 0)
        {
          return false;
        }
        else if(kernel[i+1][j+1] == 1 &&
            ptr[(center.y + j) * img.cols + center.x + i] == 0)
        {
          return false;
        }
      }
    }
    return true;
  }



  /**
    @brief Performs steps of erosion
    @param img [cv::Mat*] The input image in CV_8UC1 format
    @param steps [const int&] Number of operator steps
    @param visualize [const bool&] True for step-by-step visualization
    @return void
   **/
  void Morphology::erosion(cv::Mat* img, const int& steps,
    const bool& visualize)
  {
    #ifdef DEBUG_TIME
    Timer::start("erosion");
    #endif

    cv::Mat helper;
    img->copyTo(helper);

    for(unsigned int s = 0 ; s < steps ; s++)
    {
      if(visualize)
      {
        Visualization::show("Erosion iteration", *img, 500);
      }
      for(unsigned int i = 1 ; i < img->rows - 1 ; i++)
      {
        for(unsigned int j = 1 ; j < img->cols - 1 ; j++)
        {
          if(img->at<unsigned char>(i,j) != 0) //!< That's foreground
          {
            //!< Check for all adjacent
            if(img->at<unsigned char>(i + 1,j + 1) == 0)
            {
              helper.at<unsigned char>(i,j) = 0;
              continue;
            }
            if(img->at<unsigned char>(i + 1,j) == 0)
            {
              helper.at<unsigned char>(i,j) = 0;
              continue;
            }
            if(img->at<unsigned char>(i + 1,j - 1) == 0)
            {
              helper.at<unsigned char>(i,j) = 0;
              continue;
            }
            if(img->at<unsigned char>(i,j + 1) == 0)
            {
              helper.at<unsigned char>(i,j) = 0;
              continue;
            }
            if(img->at<unsigned char>(i,j - 1) == 0)
            {
              helper.at<unsigned char>(i,j) = 0;
              continue;
            }
            if(img->at<unsigned char>(i - 1,j - 1) == 0)
            {
              helper.at<unsigned char>(i,j) = 0;
              continue;
            }
            if(img->at<unsigned char>(i - 1,j) == 0)
            {
              helper.at<unsigned char>(i,j) = 0;
              continue;
            }
            if(img->at<unsigned char>(i - 1,j + 1) == 0)
            {
              helper.at<unsigned char>(i,j) = 0;
              continue;
            }
          }
        }
      }
      helper.copyTo(*img);
    }
    #ifdef DEBUG_TIME
    Timer::tick("erosion");
    #endif
  }



  /**
    @brief Performs steps of dilation
    @param img [cv::Mat&*] The input image in CV_8UC1 format
    @param steps [const int&] Number of operator steps
    @param visualize [const bool&] True for step-by-step visualization
    @return void
   **/
  void Morphology::dilation(cv::Mat* img, const int& steps,
    const bool& visualize)
  {
    #ifdef DEBUG_TIME
    Timer::start("dilation", "denoiseEdges");
    #endif

    cv::Mat helper;
    img->copyTo(helper);

    static unsigned int p = 0;

    for(unsigned int s = 0 ; s < steps ; s++)
    {
      for(unsigned int i = 1 ; i < img->rows - 1 ; i++)
      {
        for(unsigned int j = 1 ; j < img->cols - 1 ; j++)
        {
          p = i * img->cols + j;
          if(img->data[i * img->cols + j] == 0) //!< That's foreground
          {
            //!< Check for all adjacent
            if(img->data[p + img->cols + 1] != 0)
            {
              helper.data[p] = 255;
              continue;
            }
            if(img->data[p + img->cols] != 0)
            {
              helper.data[p] = 255;
              continue;
            }
            if(img->data[p + img->cols - 1] != 0)
            {
              helper.data[p] = 255;
              continue;
            }
            if(img->data[p + 1] != 0)
            {
              helper.data[p] = 255;
              continue;
            }
            if(img->data[p - 1] != 0)
            {
              helper.data[p] = 255;
              continue;
            }
            if(img->data[p - img->cols + 1] != 0)
            {
              helper.data[p] = 255;
              continue;
            }
            if(img->data[p - img->cols] != 0)
            {
              helper.data[p] = 255;
              continue;
            }
            if(img->data[p - img->cols - 1] != 0)
            {
              helper.data[p] = 255;
              continue;
            }
          }
        }
      }
      helper.copyTo(*img);
    }
    #ifdef DEBUG_TIME
    Timer::tick("dilation");
    #endif
  }



  /**
    @brief Performs steps of opening
    @param img [cv::Mat&] The input image in CV_8UC1 format
    @param steps [const int&] Number of operator steps
    @param visualize [const bool&] True for step-by-step visualization
    @return void
   **/
  void Morphology::opening(cv::Mat* img, const int& steps,
    const bool& visualize)
  {
    #ifdef DEBUG_TIME
    Timer::start("opening");
    #endif

    for (unsigned int i = 0; i < steps; i++)
    {
      if(visualize)
      {
        Visualization::show("Opening iteration", *img, 500);
      }

      erosion(img, 1, false);
      dilation(img, 1, false);
    }
    #ifdef DEBUG_TIME
    Timer::tick("opening");
    #endif
  }



  /**
    @brief Performs steps of closing
    @param img [cv::Mat*] The input image in CV_8UC1 format
    @param steps [const int&] Number of operator steps
    @param visualize [const bool&] True for step-by-step visualization
    @return void
   **/
  void Morphology::closing(cv::Mat* img, const int& steps,
    const bool& visualize)
  {
    #ifdef DEBUG_TIME
    Timer::start("closing");
    #endif

    for (unsigned int i = 0; i < steps; i++)
    {
      if(visualize)
      {
        Visualization::show("Closing iteration", *img, 500);
      }

      dilation(img, 1, false);
      erosion(img, 1, false);
    }
    #ifdef DEBUG_TIME
    Timer::tick("closing");
    #endif
  }



  /**
    @brief Performs steps of thickenning
    @param inImage [const cv::Mat&] The input image in CV_8UC1 format
    @param outImage [cv::Mat*] The output image in CV_8UC1 format
    @param steps [const int&] Number of operator steps
    @param visualize [const bool&] True for step-by-step visualization
    @return void
   **/
  void Morphology::thickenning(const cv::Mat& inImage, cv::Mat* outImage,
      const int& steps, const bool& visualize)
  {
    #ifdef DEBUG_TIME
    Timer::start("thickenning");
    #endif

    static const char kernels[8][3][3] = {
      { {1, 1, 2},
        {1, 0, 2},
        {1, 2, 0} },

      { {1, 1, 1},
        {2, 0, 1},
        {0, 2, 2} },

      { {0, 2, 1},
        {2, 0, 1},
        {2, 1, 1} },

      { {2, 2, 0},
        {1, 0, 2},
        {1, 1, 1} },

      { {2, 1, 1},
        {2, 0, 1},
        {0, 2, 1} },

      { {0, 2, 2},
        {2, 0, 1},
        {1, 1, 1} },

      { {1, 2, 0},
        {1, 0, 2},
        {1, 1, 2} },

      { {1, 1, 1},
        {1, 0, 2},
        {2, 2, 0} }
    };

    inImage.copyTo(*outImage);

    //!< if the image is saturated by the thickenning operator,
    //!< cease its operation
    bool isRunning;
    for (unsigned int s = 0; s < steps; s++)
    {
      if(visualize)
      {
        Visualization::show("Thickening iteration", *outImage, 500);
      }

      isRunning = false;
      for (int kernelId = 0; kernelId < 8; kernelId++)
      {
        for (unsigned int rows = 1; rows < outImage->rows - 1; rows++)
        {
          for (unsigned int cols = 1; cols < outImage->cols - 1; cols++)
          {
            if(outImage->at<unsigned char>(rows,cols) == 0)
            {
              if (kernelCheck(kernels[kernelId], *outImage,
                  cv::Point(cols, rows)))
              {
                outImage->at<unsigned char>(rows, cols) = 255;
                isRunning = true;
              }
            }
          }
        }
      }
      if (!isRunning)
      {
        break;
      }
    }
    #ifdef DEBUG_TIME
    Timer::tick("thickenning");
    #endif
  }



  /**
    @brief Performs steps of thinning
    (http://homepages.inf.ed.ac.uk/rbf/HIPR2/thin.htm)
    @param inImage [const cv::Mat&] The input image in CV_8UC1 format
    @param outImage [cv::Mat&] The input image in CV_8UC1 format
    @param steps [const int&] Number of operator steps
    @param visualize [const bool&] True for step-by-step visualization
    @return void
   **/
  void Morphology::thinning(const cv::Mat& inImage, cv::Mat* outImage,
      const int& steps, const bool& visualize)
  {
    #ifdef DEBUG_TIME
    Timer::start("thinning", "denoiseEdges");
    #endif

    static const char kernels[8][3][3] = {
      { {0, 0, 0},
        {2, 1, 2},
        {1, 1, 1} },

      { {2, 0, 0},
        {1, 1, 0},
        {2, 1, 2} },

      { {1, 2, 0},
        {1, 1, 0},
        {1, 2, 0} },

      { {2, 1, 2},
        {1, 1, 0},
        {2, 0, 0} },

      { {1, 1, 1},
        {2, 1, 2},
        {0, 0, 0} },

      { {2, 1, 2},
        {0, 1, 1},
        {0, 0, 2} },

      { {0, 2, 1},
        {0, 1, 1},
        {0, 2, 1} },

      { {0, 0, 2},
        {0, 1, 1},
        {2, 1, 2} }
    };

    inImage.copyTo(*outImage);

    //!< if the image is saturated by the thinning operator,
    //!< cease its operation
    bool isRunning;
    static unsigned int limit = 0;
    static unsigned int *pts =
      new unsigned int[outImage->cols * outImage->rows];
    limit = 0;

    for (unsigned int rows = 1; rows < outImage->rows - 1; rows++)
    {
      for (unsigned int cols = 1; cols < outImage->cols - 1; cols++)
      {
        if(inImage.data[rows * outImage->cols + cols] != 0)
        {
          pts[limit]= rows * outImage->cols + cols;
          limit++;
        }
      }
    }

    for (unsigned int s = 0; s < steps; s++)
    {
      isRunning = false;

      for(unsigned int i = 0 ; i < limit ; i++)
      {
        for (int kernelId = 0; kernelId < 8; kernelId++)
        {
          if (kernelCheck(kernels[kernelId], *outImage,
            cv::Point(pts[i] % outImage->cols, pts[i] / outImage->cols)))
          {
            outImage->data[pts[i]] = 0;
            isRunning = true;
            break;
          }
        }
      }
      if (!isRunning)
      {
        break;
      }
    }
    #ifdef DEBUG_TIME
    Timer::tick("thinning");
    #endif
  }



  /**
    @brief Performs steps of pruning
    @param img [cv::Mat*] The input image in CV_8UC1 format
    @param steps [const int&] Number of operator steps
    @param visualize [const bool&] True for step-by-step visualization
    @return void
   **/
  void Morphology::pruning(cv::Mat* img, const int& steps,
    const bool& visualize)
  {
    #ifdef DEBUG_TIME
    Timer::start("pruning");
    #endif

    static const char kernels[9][3][3] = {
      { {0, 0, 0},
        {0, 1, 0},
        {0, 0, 0} },

      { {1, 0, 0},
        {0, 1, 0},
        {0, 0, 0} },

      { {0, 1, 0},
        {0, 1, 0},
        {0, 0, 0} },

      { {0, 0, 1},
        {0, 1, 0},
        {0, 0, 0} },

      { {0, 0, 0},
        {1, 1, 0},
        {0, 0, 0} },

      { {0, 0, 0},
        {0, 1, 1},
        {0, 0, 0} },

      { {0, 0, 0},
        {0, 1, 0},
        {1, 0, 0} },

      { {0, 0, 0},
        {0, 1, 0},
        {0, 1, 0} },

      { {0, 0, 0},
        {0, 1, 0},
        {0, 0, 1} }
    };

    bool isRunning;
    for (unsigned int s = 0; s < steps; s++)
    {
      if(visualize)
      {
        Visualization::show("Pruning iteration", *img, 100);
      }

      isRunning = false;
      for (int kernelId = 0; kernelId < 9; kernelId++)
      {
        for (unsigned int rows = 1; rows < img->rows - 1; rows++)
        {
          for (unsigned int cols = 1; cols < img->cols - 1; cols++)
          {
            if(img->at<unsigned char>(rows,cols) != 0)
            {
              if (kernelCheck(kernels[kernelId], *img, cv::Point(cols, rows)))
              {
                img->at<unsigned char>(rows, cols) = 0;
                isRunning = true;
              }
            }
          }
        }
      }
      if (!isRunning)
      {
        break;
      }
    }
    #ifdef DEBUG_TIME
    Timer::tick("pruning");
    #endif
  }



  /**
    @brief Performs steps of strict pruning (removes more stuff)
    @param img [cv::Mat*] The input image in CV_8UC1 format
    @param steps [const int*] Number of operator steps
    @param visualize [const bool&] True for step-by-step visualization
    @return void
   **/
  void Morphology::pruningStrict(cv::Mat* img, const int& steps,
      const bool& visualize)
  {
    #ifdef DEBUG_TIME
    Timer::start("pruningStrict");
    #endif

    static const char kernels[9][3][3] = {
      { {0, 0, 0},
        {0, 1, 0},
        {0, 0, 0} },

      { {1, 2, 0},
        {2, 1, 0},
        {0, 0, 0} },

      { {2, 1, 2},
        {0, 1, 0},
        {0, 0, 0} },

      { {0, 2, 1},
        {0, 1, 2},
        {0, 0, 0} },

      { {2, 0, 0},
        {1, 1, 0},
        {2, 0, 0} },

      { {0, 0, 2},
        {0, 1, 1},
        {0, 0, 2} },

      { {0, 0, 0},
        {2, 1, 0},
        {1, 2, 0} },

      { {0, 0, 0},
        {0, 1, 0},
        {2, 1, 2} },

      { {0, 0, 0},
        {0, 1, 2},
        {0, 2, 1} }
    };

    bool isRunning;
    for (unsigned int s = 0; s < steps; s++)
    {
      if(visualize)
      {
        Visualization::show("Pruning iteration", *img, 100);
      }

      isRunning = false;
      for (int kernelId = 0; kernelId < 9; kernelId++)
      {
        for (unsigned int rows = 1; rows < img->rows - 1; rows++)
        {
          for (unsigned int cols = 1; cols < img->cols - 1; cols++)
          {
            if(img->at<unsigned char>(rows,cols) != 0)
            {
              if (kernelCheck(kernels[kernelId], *img, cv::Point(cols, rows)))
              {
                img->at<unsigned char>(rows, cols) = 0;
                isRunning = true;
              }
            }
          }
        }
      }
      if (!isRunning)
      {
        break;
      }
    }
    #ifdef DEBUG_TIME
    Timer::tick("pruningStrict");
    #endif
  }



  /**
    @brief Performs steps of strict pruning (removes more stuff)
    @param img [cv::Mat*] The input image in CV_8UC1 format
    @param steps [const int&] Number of operator steps
    @return void
   **/
  void Morphology::pruningStrictIterative(cv::Mat* img, const int& steps)
  {
    #ifdef DEBUG_TIME
    Timer::start("pruningStrictIterative","denoiseEdges");
    #endif

    static const unsigned int nOfKernels = 9;

    static const char kernels[9][3][3] = {
      { {0, 0, 0},
        {0, 1, 0},
        {0, 0, 0} },

      { {1, 2, 0},
        {2, 1, 0},
        {0, 0, 0} },

      { {2, 1, 2},
        {0, 1, 0},
        {0, 0, 0} },

      { {0, 2, 1},
        {0, 1, 2},
        {0, 0, 0} },

      { {2, 0, 0},
        {1, 1, 0},
        {2, 0, 0} },

      { {0, 0, 2},
        {0, 1, 1},
        {0, 0, 2} },

      { {0, 0, 0},
        {2, 1, 0},
        {1, 2, 0} },

      { {0, 0, 0},
        {0, 1, 0},
        {2, 1, 2} },

      { {0, 0, 0},
        {0, 1, 2},
        {0, 2, 1} }
    };

    std::set<unsigned int> current, next;

    for (unsigned int rows = 1; rows < img->rows - 1; rows++)
    {
      for (unsigned int cols = 1; cols < img->cols - 1; cols++)
      {
        if(img->at<unsigned char>(rows,cols) != 0)
        {
          //!< Check for initial stuff
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(cols,rows)))
            {
              current.insert(rows * img->cols + cols);
              break;
            }
          }
        }
      }
    }

    bool isRunning;
    for (unsigned int s = 0; s < steps; s++)
    {
      next.clear();
      isRunning = false;

      for(std::set<unsigned int>::iterator it = current.begin() ;
        it != current.end() ; it++)
      {
        static unsigned int x = 0;
        static unsigned int y = 0;
        x = *it / img->cols;
        y = *it % img->cols;

        img->at<unsigned char>(x,y) = 0;

        if(img->at<unsigned char>(x - 1, y - 1) != 0)
        {
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(y - 1,x - 1)))
            {
              isRunning = true;
              next.insert( (x - 1) * img->cols + (y - 1) );
              break;
            }
          }
        }
        if(img->at<unsigned char>(x - 1, y) != 0)
        {
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(y,x - 1)))
            {
              isRunning = true;
              next.insert( (x - 1) * img->cols + (y) );
              break;
            }
          }
        }
        if(img->at<unsigned char>(x - 1, y + 1) != 0)
        {
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(y + 1,x - 1)))
            {
              isRunning = true;
              next.insert( (x - 1) * img->cols + (y + 1) );
              break;
            }
          }
        }
        if(img->at<unsigned char>(x, y - 1) != 0)
        {
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(y - 1,x)))
            {
              isRunning = true;
              next.insert( (x) * img->cols + (y - 1) );
              break;
            }
          }
        }
        if(img->at<unsigned char>(x, y + 1) != 0)
        {
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(y + 1,x)))
            {
              isRunning = true;
              next.insert( (x) * img->cols + (y + 1) );
              break;
            }
          }
        }
        if(img->at<unsigned char>(x + 1, y - 1) != 0)
        {
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(y - 1,x + 1)))
            {
              isRunning = true;
              next.insert( (x + 1) * img->cols + (y - 1) );
              break;
            }
          }
        }
        if(img->at<unsigned char>(x + 1, y) != 0)
        {
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(y,x + 1)))
            {
              isRunning = true;
              next.insert( (x + 1) * img->cols + (y) );
              break;
            }
          }
        }
        if(img->at<unsigned char>(x + 1, y + 1) != 0)
        {
          for(unsigned int i = 0 ; i < nOfKernels ; i++)
          {
            if(kernelCheck(kernels[i], *img, cv::Point(y + 1,x + 1)))
            {
              isRunning = true;
              next.insert( (x + 1) * img->cols + (y + 1) );
              break;
            }
          }
        }
      }
      if (!isRunning)
      {
        break;
      }
      next.swap(current);
    }
    #ifdef DEBUG_TIME
    Timer::tick("pruningStrictIterative");
    #endif
  }
}

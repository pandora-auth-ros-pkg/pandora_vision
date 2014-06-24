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
* Author:  Despoina Pascahlidou
*********************************************************************/

#ifndef PANDORA_VISION_MOTION_DBSCAN_H
#define PANDORA_VISION_MOTION_DBSCAN_H

#include "opencv2/opencv.hpp"
#include <map>
#include <sstream>
#include <iostream>

namespace pandora_vision{
  
  typedef std::vector<int32_t> Neighbors;
  typedef std::vector<bool> NoisePoints;
  typedef std::vector<bool> VisitedPoints;
  typedef std::vector<bool> ClusteredPoints;
  
  class DBSCAN
  {
      private:
      //!< Eps radius, two points are neighbors if their distance
      //!< is smaller than this threshold value
      double _eps;
      //!< Minimum number of points required to form a dense region
      size_t _minPts;
      
      int _cluster_id;
      
      //! Vector of visited points 
      VisitedPoints _visitedPoints;
      //! Vector of clustered data 
      ClusteredPoints _clusteredPoints;
      std::map<int, int> _labels;
      
      vector<Rect>& data;
      
      /**
       @brief Function that initializes all vectors to begin with the
       clustering process. At the beginning both visited and clustered data
       are set to false, for each point of interest.
       @param num_of_points: Number of points to be created
       @return void
      */  
      void init(unsigned int num_of_points);
      
      /**
       @brief Function that check if a point has already been visited, if yes
       true is returned, if no false is returned.
       @param iterator Index of vector _visitedPoints that corresponds
       to the current point we are checking
       @return void
      */ 
      bool isVisited(int iterator);
      
      /**
       @brief Function tha caclulates distance between two points 
       @param pt1: First point
       @param pt2: Second point
       @return their distance
      */ 
      double dist2d(cv::Point2d pt1, cv::Point2d pt2)
      
      /**
       @brief Function that returns all points with P's eps-neighborhood
       @param P:current point,we are processeing
       @return vector of all point in the neighborhoud 
      */ 
      std::vector<int> regionQuery(int p)
      
      void expandCluster(int p, vector<int> neighbours);
      
      double calculateDistance(int ai, int bi); 
      public:
      
      //!< Class constructor
      explicit DBSCAN(std::vector<cv::Rect>& _data, double eps, int minPts);
      
      ~DBSCAN();
      
      void dbscan_cluster();
      
      
      double* dp;
      //memoization table in case of complex dist functions
  #define DP(i,j) dp[(data.size()*i)+j]

      
     
      
      double distanceFunc(int ai,int bi)
      {
          if(DP(ai,bi)!=-1)
              return DP(ai,bi);
          Rect a = data[ai];
          Rect b = data[bi];
          /*
          Point2d cena= Point2d(a.x+a.width/2,
                                a.y+a.height/2);
          Point2d cenb = Point2d(b.x+b.width/2,
                                b.y+b.height/2);
          double dist = sqrt(pow(cena.x-cenb.x,2) + pow(cena.y-cenb.y,2));
          DP(ai,bi)=dist;
          DP(bi,ai)=dist;*/
          Point2d tla =Point2d(a.x,a.y);
          Point2d tra =Point2d(a.x+a.width,a.y);
          Point2d bla =Point2d(a.x,a.y+a.height);
          Point2d bra =Point2d(a.x+a.width,a.y+a.height);

          Point2d tlb =Point2d(b.x,b.y);
          Point2d trb =Point2d(b.x+b.width,b.y);
          Point2d blb =Point2d(b.x,b.y+b.height);
          Point2d brb =Point2d(b.x+b.width,b.y+b.height);

          double minDist = 9999999;

          minDist = min(minDist,dist2d(tla,tlb));
          minDist = min(minDist,dist2d(tla,trb));
          minDist = min(minDist,dist2d(tla,blb));
          minDist = min(minDist,dist2d(tla,brb));

          minDist = min(minDist,dist2d(tra,tlb));
          minDist = min(minDist,dist2d(tra,trb));
          minDist = min(minDist,dist2d(tra,blb));
          minDist = min(minDist,dist2d(tra,brb));

          minDist = min(minDist,dist2d(bla,tlb));
          minDist = min(minDist,dist2d(bla,trb));
          minDist = min(minDist,dist2d(bla,blb));
          minDist = min(minDist,dist2d(bla,brb));

          minDist = min(minDist,dist2d(bra,tlb));
          minDist = min(minDist,dist2d(bra,trb));
          minDist = min(minDist,dist2d(bra,blb));
          minDist = min(minDist,dist2d(bra,brb));
          DP(ai,bi)=minDist;
          DP(bi,ai)=minDist;
          return DP(ai,bi);
      }

      vector<vector<Rect> > getGroups()
      {
          vector<vector<Rect> > ret;
          for(int i=0;i<=C;i++)
          {
              ret.push_back(vector<Rect>());
              for(int j=0;j<data.size();j++)
              {
                  if(labels[j]==i)
                  {
                      ret[ret.size()-1].push_back(data[j]);
                  }
              }
          }
          return ret;
      }
  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_MOTION_DBSCAN

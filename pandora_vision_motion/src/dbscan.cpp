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
* Author:  Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_motion/dbscan.h"

namespace pandora_vision
{
  /**
   @brief Class constructor 
  */
  DBSCAN::DBSCAN(std::vector<cv::Rect>& data, double eps, int minPts)::_data(data)
  {
    _cluster_id = -1;
    _eps = eps;
    _minPts = minPts;
  } 
  
  DBSCAN::~DBSCAN()
  {
    
  }
  
   /**
   @brief Function that initializes all vectors to begin with the
   clustering process. At the beginning both visited and clustered data
   are set to false, for each point of interest.
   @param num_of_points: Number of points to be created
   @return void
  */ 
  void DBSCAN::init(unsigned int num_of_points)
  {
    _labels.resize(num_of_points);
    
    for(int i = 0; i < num_of_points; i++)
    {
      _labels[i] = -99;
      _visitedPoints.push_back(false);
      _clusteredPoints.push_back(false);
    }
  }
  
  /**
   @brief Function that check if a point has already been visited, if yes
   true is returned, if no false is returned.
   @param iterator Index of vector _visitedPoints that corresponds
   to the current point we are checking
   @return void
  */ 
  bool DBSCAN::isVisited(int iterator)
  {
    if(_visitedPoints.at(iterator) == false)
      return false;
    else
      return true;
  }
  
  /**
   @brief Function tha caclulates distance between two points 
   @param pt1: First point
   @param pt2: Second point
   @return their distance
  */ 
  double DBSCAN::dist2d(cv::Point2d pt1, cv::Point2d pt2)
  {
      return sqrt(pow(pt1.x-pt2.x,2) + pow(pt1.y-pt2.y,2));
  }
  
  /**
   @brief Function that returns all points with P's eps-neighborhood
   @param P:current point,we are processeing
   @return vector of all point in the neighborhoud 
  */ 
  std::vector<int> DBSCAN::regionQuery(int p)
  {
    std::vector<int> res;
    for(int i=0;i<data.size();i++)
    {
        if(distanceFunc(p,i) <= _eps)
            res.push_back(i);
    }
    return res;
  }
  
  void DBSCAN::expandCluster(int p, vector<int> neighbours)
    {
      _labels[p] = _cluster_id;
      for(int i=0;i<neighbours.size();i++)
      {
        if(!isVisited(neighbours[i]))
        {
            labels[neighbours[i]]=C;
            std::vector<int> neighbours_p = regionQuery(neighbours[i]);
            if (neighbours_p.size() >= _minPts)
                expandCluster(neighbours[i],neighbours_p);
        }
      }
  }
  
  void DBSCAN::dbscan_cluster()
      {
          dp = new double[data.size()*data.size()];
          for(int i=0;i<data.size();i++)
          {
              for(int j=0;j<data.size();j++)
              {
                  if(i==j)
                      DP(i,j)=0;
                  else
                      DP(i,j)=-1;
              }
          }
          for(int i=0;i<data.size();i++)
          {
              if(!isVisited(i))
              {
                  vector<int> neighbours = regionQuery(i);
                  if(neighbours.size()<mnpts)
                  {
                      labels[i]=-1;//noise
                  }else
                  {
                      C++;
                      expandCluster(i,neighbours);
                  }
              }
          }
          delete [] dp;
      }
}// namespace pandora_vision

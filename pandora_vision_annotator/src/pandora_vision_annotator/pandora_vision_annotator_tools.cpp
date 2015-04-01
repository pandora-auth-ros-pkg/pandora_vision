/******************************************************************************
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
* Authors: Manos Tsardoulias
*********************************************************************/

#include "pandora_vision_annotator/pandora_vision_annotator_tools.h"

namespace pandora_vision
{ int ImgAnnotations::annPerImage = 0;
  bool ImgAnnotations::secondpoint = false;
  std::vector<annotation> ImgAnnotations::annotations;
  std::ofstream ImgAnnotations::outFile;
  annotation ImgAnnotations::temp;
    void ImgAnnotations::writeToFile(const std::string& filename)
    {
        outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        if (!outFile)
            {
                qFatal("cannot load file");
                return;
            }
        else
        {  if(outFile.is_open())
           {
            qDebug("Writing to file" );
            for (unsigned int i = 0; i < annotations.size(); i++)
            {
                outFile << ImgAnnotations::annotations[i].imgName << ","
                       << ImgAnnotations::annotations[i].category << ","
                       << ImgAnnotations::annotations[i].x1 << ","
                       << ImgAnnotations::annotations[i].y1 << ","
                       << ImgAnnotations::annotations[i].x2 << ","
                       << ImgAnnotations::annotations[i].y2;
                if (ImgAnnotations::annotations[i].category == "Hazmat")
                  outFile << ","<< ImgAnnotations::annotations[i].type.toStdString();
                outFile << std::endl;
               qDebug("%s %s %d %d %d %d\n",ImgAnnotations::annotations[i].imgName.c_str(),
                      ImgAnnotations::annotations[i].category.c_str(),
                      ImgAnnotations::annotations[i].x1,
                      ImgAnnotations::annotations[i].y1,
                      ImgAnnotations::annotations[i].x2,
                      ImgAnnotations::annotations[i].y2);
            }
            }
        }
        outFile.close();
    }

    bool ImgAnnotations::is_file_exist(const char *fileName)
   {
     std::ifstream infile(fileName);
     return infile.good();
   }


    void ImgAnnotations::setAnnotations(const std::string &imgName, const std::string &category, int x, int y)
    {
      if(secondpoint)
      {
        ImgAnnotations::temp.x2 = x;
        ImgAnnotations::temp.y2 = y;
        ImgAnnotations::annotations.push_back(temp);
        qDebug(" Annotations in current frame %ld",ImgAnnotations::annotations.size());
        //ImgAnnotations::annPerImage++;
        ImgAnnotations::secondpoint = false;
      }
      else
      {
      ImgAnnotations::temp.imgName =imgName;
      ImgAnnotations::temp.category = category;
      ImgAnnotations::temp.x1 = x;
      ImgAnnotations::temp.y1 = y;
      ImgAnnotations::secondpoint = true;
      }
    }

    void ImgAnnotations::setAnnotations(const std::string &imgName, const std::string &category, int x, int y, QString type)
    {
      if(secondpoint)
      {
        ImgAnnotations::temp.x2 = x;
        ImgAnnotations::temp.y2 = y;
        ImgAnnotations::temp.type = type;
        ImgAnnotations::annotations.push_back(temp);
        qDebug(" Annotations in current frame %ld",ImgAnnotations::annotations.size());
        //ImgAnnotations::annPerImage++;
        ImgAnnotations::secondpoint = false;
      }
      else
      {
      ImgAnnotations::temp.imgName =imgName;
      ImgAnnotations::temp.category = category;
      ImgAnnotations::temp.x1 = x;
      ImgAnnotations::temp.y1 = y;
      ImgAnnotations::secondpoint = true;
      }
    }
}

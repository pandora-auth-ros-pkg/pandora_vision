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
{ 
  int ImgAnnotations::annPerImage = 0;
  bool ImgAnnotations::secondpoint = false;
  std::vector<annotation> ImgAnnotations::annotations;
  std::ofstream ImgAnnotations::outFile;
  std::ifstream ImgAnnotations::inFile;
  annotation ImgAnnotations::temp;

  void ImgAnnotations::deleteFromFile(const::std::string&filename, const std::string& frame)
  {
    std::string line;
    std::string package_path = ros::package::getPath("pandora_vision_annotator");
    std::stringstream tempFile;
    tempFile << package_path << "/data/temp.txt";
    bool deleted = false;
    int length = frame.length();
    inFile.open(filename.c_str());
    outFile.open(tempFile.str().c_str());
    if(!inFile)
    {
      ROS_ERROR("cannot load file");
    }
    while(std::getline(inFile,line))
    {

      if(line.substr(0,length) != frame)
      {
        outFile << line << std::endl;
        ROS_INFO_STREAM(line);
      }

      else
        deleted = true;

      if(deleted)
      {
        ROS_INFO_STREAM("Annotation for " << frame <<" deleted from file");
      }
    }

      inFile.close();
      outFile.close();
      remove(filename.c_str());
      rename(tempFile.str().c_str(), filename.c_str());
  }

  void ImgAnnotations::getLastFrameIndex(const::std::string& filename, int& index)
  {
    inFile.open(filename.c_str(), std::ios_base::ate);
    std::string line;
    int length = 0;
    char c ='\0';

    if (!inFile)
    {
      ROS_ERROR("cannot load file");
      return;
    } 

    length = inFile.tellg();
    if(length == 0)
    {
      index = -1;
    }

    else
    {

      for(int ii = length-2; ii > 0; ii--)
      {
        inFile.seekg(ii);
        c = inFile.get();
        if( c =='\r' || c == '\n' )
          break;
      }
      std::getline(inFile,line);
      std::string package_path = ros::package::getPath("pandora_vision_annotator");
      std::stringstream temp;
      temp << line;
      std::string imName;
      getline (temp, imName,',');
      unsigned s = imName.find_last_of(".");
      imName = imName.substr(0,s);
      s=imName.find_last_of("e");
      imName=imName.substr(s+1);
      index= atoi(imName.c_str());
    }
  }

  void ImgAnnotations::readFromFile(const std::string& filename, const std::string& frame)
  {
    std::string line,x1,y1,x2,y2;
    inFile.open(filename.c_str());
    int  i = 0;
    int length = frame.length();
    if (!inFile)
    {
      ROS_ERROR("cannot load file");
      return;
    }

    else
    {
      if(inFile.is_open())
      {
        ROS_INFO("loading from file");
        while(std::getline(inFile,line))
        {  

          if(line.substr(0,length) == frame)
          {ROS_INFO_STREAM("LENGTH"<<length<<" "<<line.substr(0,length));

            std::stringstream ss(line);
            ROS_INFO("stringstream %s",ss.str().c_str());
            getline (ss, ImgAnnotations::temp.imgName,',');
            getline (ss, ImgAnnotations::temp.category,',');
            getline (ss, x1,',');
            getline (ss, y1,',');
            getline (ss, x2,',');
            if(ImgAnnotations::temp.category == "Hazmat")
            {
              getline (ss, y2,',');
              getline (ss, ImgAnnotations::temp.type);
            }           
            else
              getline (ss, y2);
            ImgAnnotations::temp.x1 = atoi(x1.c_str());
            ImgAnnotations::temp.y1 = atoi(y1.c_str());
            ImgAnnotations::temp.x2 = atoi(x2.c_str());
            ImgAnnotations::temp.y2 = atoi(y2.c_str());

            ROS_INFO_STREAM("Loading Annotation no: " << i+1 <<" for "<< frame << "\n");
            ROS_INFO_STREAM(ImgAnnotations::temp.imgName << ","
                            << ImgAnnotations::temp.category << ","
                            << ImgAnnotations::temp.x1 << ","
                            << ImgAnnotations::temp.y1<< ","
                            << ImgAnnotations::temp.x2 << ","
                            << ImgAnnotations::temp.y2 << "\n" );
            ImgAnnotations::annotations.push_back(temp);
            i++;
        
          }          
        }
      }
    }
    inFile.close(); 
  }

  void ImgAnnotations::writeToFile(const std::string& filename)
  {
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
    if (!outFile)
    {
      ROS_ERROR("cannot load file");
      return;
    }
    else
    {  
      if(outFile.is_open())
      {
        ROS_INFO("Writing to file" );
        for (unsigned int i = 0; i < annotations.size(); i++)
        {
          outFile << ImgAnnotations::annotations[i].imgName << ","
                  << ImgAnnotations::annotations[i].category << ","
                  << ImgAnnotations::annotations[i].x1 << ","
                  << ImgAnnotations::annotations[i].y1 << ","
                  << ImgAnnotations::annotations[i].x2 << ","
                  << ImgAnnotations::annotations[i].y2;
          if (ImgAnnotations::annotations[i].category == "Hazmat")
            outFile << ","<< ImgAnnotations::annotations[i].type;
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
  
  void ImgAnnotations::setAnnotations(const std::string &imgName, const std::string &category, int x, int y, const std::string& type)
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

  void ImgAnnotations::writeToFile(const std::string& filename,const std_msgs::Header& msg )
  {
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
    if (!outFile)
    {
      ROS_ERROR("cannot load file");
      return;
    }
    else
    {
      if(outFile.is_open())
      {
        ROS_INFO("Writing to file" );
        outFile << msg.seq << "," << msg.stamp << "," << msg.frame_id << std::endl;
      }
    }
    outFile.close();
  }
}

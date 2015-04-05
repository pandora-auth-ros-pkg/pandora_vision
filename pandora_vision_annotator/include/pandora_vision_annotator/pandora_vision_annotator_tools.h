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

#ifndef PANDORA_VISION_ANNOTATOR_TOOLS
#define PANDORA_VISION_ANNOTATOR_TOOLS

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>


#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>



#include <QtUiTools/QUiLoader>

#include <QDebug>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QTime>

#include <QtGui/QMenu>
#include <QtGui/QApplication>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QImage>
#include <QtGui/QFileDialog>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QPainter>
#include <QtGui/QPixmap>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QScrollBar>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QTreeWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include <QtGui/QWindowsStyle>
#include <QtGui/QInputDialog>
#include <QtGui/QMouseEvent>
#include <QtGui/QMessageBox>
#include <QtGui/QTimeEdit>
#include <QtGui/QInputDialog>
#include <QtGui/QFont>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

/**
@namespace pandora_vision
@brief The namespace for pandora vision
**/
namespace pandora_vision
{
  struct annotation
  {
      std::string imgName;
      std::string category;
      int x1;
      int y1;
      int x2;
      int y2;
      std::string type;
  };

  class ImgAnnotations
  {
    public:
    static std::vector<annotation> annotations;
    static annotation temp;
    static std::ifstream inFile;
    static std::ofstream outFile;
    static int annPerImage;
    static bool secondpoint;
    static void writeToFile(const std::string& filename);
    static void setAnnotations(const::std::string& imgName, const std::string &category, int x, int y);
    static void setAnnotations(const std::string& imgName, const std::string &category, int x, int y, const std::string&  type);
    static bool is_file_exist(const char *fileName);
    static void writeToFile(const std::string& filename,const std_msgs::Header& msg );
    static void readFromFile(const std::string& filename, const std::string& frame);

  };

  }

#endif

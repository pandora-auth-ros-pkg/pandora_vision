/******************************************************************************
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
******************************************************************************/

#ifndef PANDORA_VISION_ANNOTATOR_CONTROLLER
#define PANDORA_VISION_ANNOTATOR_CONTROLLER

#include "pandora_vision_annotator/pandora_vision_annotator_connector.h"
/**
@namespace pandora_vision
@brief The main namespace for pandora vision
**/ 
namespace pandora_vision
{

  /**
  @class CController
  @brief The main controller for the pandora annotator. Inherits QThread
  **/ 
  class CController : 
    public QThread
  {
    Q_OBJECT
    
    //------------------------------------------------------------------------//
    private: 
     
      //!< Number of input arguments
      int  argc_;
      //!< Input arguments
      char** argv_;
     
      //!< ROS subscriber for the image topic
      ros::Subscriber img_subscriber_;
      
      //!< The ROS node handle
      ros::NodeHandle n_;
      //!< QImage created one time, containing the frame from the input topic
      QImage topic_img_;
      
      //!< Object of CConnector
      CConnector connector_;
   
    //------------------------------------------------------------------------//  
    public:
    
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CController(int argc,char **argv);
      
      /**
      @brief Default destructor
      @return void
      **/
      ~CController(void);
     
      /**
      @brief Initializes the Qt event connections and ROS subscribers and publishers
      @return void
      **/
      void initializeCommunications(void);
     
      /**
      @brief Initializes the ROS spin and Qt threads
      @return bool
      **/
      bool init();
      
      void receiveImage(const sensor_msgs::Image& msg);
      
    //------------------------------------------------------------------------//
    public Q_SLOTS:
      
      void rosTopicGiven(void);
      
    //------------------------------------------------------------------------//
    Q_SIGNALS:
      void updateImage(void);
    };
}

#endif


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

#include "pandora_vision_annotator/pandora_vision_annotator_controller.h"


namespace pandora_vision
{
  /**
  @brief Thread that performs the ros::spin functionality
  @return void
  **/
  void spinThreadFunction(void)
  {
    ros::spin();
  }
  
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CController::CController(int argc,char **argv):
    gui_connector_(argc,argv),
    argc_(argc),
    argv_(argv)
  {
  }

  /**
  @brief Default destructor
  @return void
  **/
  CController::~CController(void)
  {
    
  }
  
  /**
  @brief Initializes the Qt event connections and ROS subscribers and publishers
  @return void
  **/
  void CController::initializeCommunications(void)
  {
    //map_subscriber_ = n_.subscribe(
      //"map", 
      //1, 
      //&CGuiController::receiveMap,
      //this);
   
    //QObject::connect(
      //this, SIGNAL(setSoundSensorVisibility(QString,QString,char)),
      //&info_connector_, SLOT(setSoundSensorVisibility(QString,QString,char)));
  }
 
  /**
  @brief Initializes the ROS spin and Qt threads
  @return bool
  **/
  bool CController::init(void)
  {
    if ( ! ros::master::check() ) 
    {
      return false;
    }
    gui_connector_.show();

    initializeCommunications();
    boost::thread spinThread(&spinThreadFunction);
    return true;
  }
}



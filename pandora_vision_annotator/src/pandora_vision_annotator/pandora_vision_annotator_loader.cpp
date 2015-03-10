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
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/

#include "pandora_vision_annotator/pandora_vision_annotator_loader.h"

namespace pandora_vision
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CLoader::CLoader(int argc,char **argv):
    argc_(argc),
    argv_(argv)
  {
    setupUi(this);
    
    close_signal_ = false;
  }
  
 
  /**
  @brief Overloading of closeEvent function from QMainWindow
  @param event [QCloseEvent*] The exit event
  @return void
  **/
  void CLoader::closeEvent(QCloseEvent *event)
  {
    //~ ROS_ERROR("Shutdown signal!");
    if(close_signal_)
    {
      event->accept();
      //~ ROS_ERROR("Shutting down ros...");
      ros::shutdown();
      exit(0);
      return;
    }
    close_signal_ = true;
    event->ignore();
    event_ = event;
  }
  
  /**
  @brief Returns the exit event
  @return QEvent* 
  **/
  QEvent* CLoader::getCloseEvent(void)
  {
    return event_;
  }
  
  /**
  @brief Returns true if a close event was triggered
  @return bool
  **/
  bool CLoader::closeTriggered(void)
  {
    return close_signal_;
  }
  
  /**
  @brief Shuts down the main window
  @return void
  **/
  void CLoader::shutdown(void)
  {
    this->close();
  }
}

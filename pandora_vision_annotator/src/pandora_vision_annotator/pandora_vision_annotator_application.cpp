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

#include "pandora_vision_annotator/pandora_vision_annotator_application.h"

namespace pandora_vision
{
  /**
  @brief Default contructor
  @param argc [int&] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CApplication::CApplication(int &argc,char **argv):
    QApplication(argc,argv)  
  {
  }
  
  /**
  @brief Called at every Qt event 
  @param receiver [QObject*] The event receiver
  @param event [QEvent*] The event triggered
  @return bool : True if receiver was notified about event
  **/
  bool CApplication::notify(QObject * receiver, QEvent * event)
  {
    try 
    {
      return QApplication::notify(receiver, event);
    } 
    catch(std::exception& e) 
    {
      qDebug() << "Exception thrown:" << e.what();
    }
    return false;
  }
}


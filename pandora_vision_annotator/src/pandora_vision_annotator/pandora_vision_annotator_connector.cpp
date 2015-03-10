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

#include "pandora_vision_annotator/pandora_vision_annotator_connector.h"

namespace pandora_vision
{
  
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CConnector::CConnector(int argc, char **argv):
    QObject(),
    loader_(argc,argv),
    argc_(argc),
    argv_(argv)
  {
    
    //QObject::connect(
      //loader_.actionProperties,SIGNAL(triggered(bool)),
      //this,SLOT(actionPropertiesTriggered()));
     }
  
  /**
  @brief Qt slot that is called when the Properties tool button is pressed
  @return void
  **/
  //void CGuiConnector::actionPropertiesTriggered(void)
  //{
    //QMessageBox msg(static_cast<QMainWindow *>(&this->loader_));
    //msg.setWindowTitle(QString("Not finished yet :/"));
    //msg.exec();
  //}
  
  void CConnector::show(void)
  {
    loader_.show();
  }
}

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

#ifndef PANDORA_VISION_ANNOTATOR_CONNECTOR
#define PANDORA_VISION_ANNOTATOR_CONNECTOR

#include "pandora_vision_annotator/pandora_vision_annotator_loader.h"

/**
@namespace pandora_vision
@brief The main namespace for pandora_vision
**/ 
namespace pandora_vision
{
  
  /**
  @class CConnector
  @brief Serves the Qt events of the main GUI window. Inherits from QObject
  **/ 
  class CConnector:
    public QObject
  {
    Q_OBJECT
    
    //------------------------------------------------------------------------//
    private:
    
      //!< Number of input arguments
      int   argc_;  
      //!< Input arguments     
      char**  argv_;   
      
      //!< The loader of main GUI QWidget 
      CLoader loader_;    
    
    //------------------------------------------------------------------------//
    public:
      
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CConnector(int argc, char **argv);

      void show(void);
         

    //------------------------------------------------------------------------//
    public Q_SLOTS:
    
      /**
      @brief Qt slot that is called when the Properties tool button is pressed
      @return void
      **/
      //void actionPropertiesTriggered(void);
          
    //------------------------------------------------------------------------//
    Q_SIGNALS:
    
      /**
      @brief Qt signal that is emmited in GuiConnector::actionZoomInTriggered and connects to MapLoader::setCursorZoomIn
      @param state [bool] Toggle flag
      @return void
      **/
      //void setZoomInCursor(bool state);
  };
}

#endif

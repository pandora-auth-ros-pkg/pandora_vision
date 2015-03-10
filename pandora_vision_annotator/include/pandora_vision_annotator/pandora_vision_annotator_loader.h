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

#ifndef PANDORA_VISION_ANNOTATOR_LOADER
#define PANDORA_VISION_ANNOTATOR_LOADER

#include "pandora_vision_annotator/pandora_vision_annotator_tools.h"
#include "ui_pandora_annotator.h"
/**
@namespace pandora_vision
@brief The main namespace for pandora vision
**/ 
namespace pandora_vision
{
  /**
  @class CLoader
  @brief Implements the low level Qt functionalities of the main window. Inherits from Ui_MainWindow (generated from an ui file) and QMainWindow. 
  **/ 
  class CLoader : public Ui_MainWindow, public QMainWindow
  {
    //------------------------------------------------------------------------//
    private:
      //!< The number of input arguments
      int   argc_;
      //!< The input arguments
      char**  argv_;
      
      //!< True if the exit signal was emmited
      bool   close_signal_;
      //!< The exit event (when occurs)
      QCloseEvent   *event_;
      
    //------------------------------------------------------------------------//
    public:
      
      //!< The action of properties button
      //QAction *actionProperties;
         
      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CLoader(int argc,char **argv);
      
      /**
      @brief Overloading of closeEvent function from QMainWindow
      @param event [QCloseEvent*] The exit event
      @return void
      **/
      void closeEvent(QCloseEvent *event);
      
      /**
      @brief Returns the exit event
      @return QEvent* 
      **/
      QEvent* getCloseEvent(void);
      
      /**
      @brief Returns true if a close event was triggered
      @return bool
      **/
      bool closeTriggered(void);
      
      /**
      @brief Shuts down the main window
      @return void
      **/
      void shutdown(void);
      
  };
  
}

#endif

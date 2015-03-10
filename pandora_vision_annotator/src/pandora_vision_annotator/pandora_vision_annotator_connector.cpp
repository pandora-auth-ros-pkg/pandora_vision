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
    // Event filter for the image area
    loader_.imageLabel->installEventFilter(this);
    
    img_state_ = IDLE;
    
    QObject::connect(
      loader_.rosTopicPushButton,SIGNAL(clicked(bool)),
      this,SLOT(rosTopicPushButtonTriggered()));
      
    QObject::connect(
      loader_.victimPushButton,SIGNAL(clicked(bool)),
      this,SLOT(victimPushButtonTriggered()));
    QObject::connect(
      loader_.hazmatPushButton,SIGNAL(clicked(bool)),
      this,SLOT(hazmatPushButtonTriggered()));
    QObject::connect(
      loader_.landoltcPushButton,SIGNAL(clicked(bool)),
      this,SLOT(landoltcPushButtonTriggered()));
    QObject::connect(
      loader_.qrPushButton,SIGNAL(clicked(bool)),
      this,SLOT(qrPushButtonTriggered()));
  }
  
  /**
  @brief Qt slot that is called when the rosTopicPushButton is pressed
  @return void
  **/
  void CConnector::rosTopicPushButtonTriggered(void)
  {
    Q_EMIT rosTopicGiven();
  }
  
  void CConnector::show(void)
  {
    loader_.show();
  }
  
  QString CConnector::getRosTopic(void)
  {
    return loader_.rosTopicLineEdit->text();
  }
  
  void CConnector::setImage(QImage &img)
  {
    localImage_ = img.copy();
  }
  
  void CConnector::updateImage(void)
  {
    loader_.imageLabel->setPixmap(QPixmap().fromImage((localImage_)));
  }
  
  /**
  @brief General event filter. Captures all events
  @param watched [QObject*] The object in which the event was triggered
  @param event [QEvent*] The type of event
  @return bool : True is event served
  **/
  bool CConnector::eventFilter( QObject* watched, QEvent* event ) 
  {
    if(watched == loader_.imageLabel)
    {
      if(event->type() == QEvent::MouseButtonPress)
      {
        
        loader_.imageLabel->setFocus(Qt::MouseFocusReason);
        
        const QMouseEvent* const me = 
          static_cast<const QMouseEvent*>( event );
        QPoint p = me->pos();
        
        int container_width = loader_.imageLabel->width();
        int container_height = loader_.imageLabel->height();
        int img_height = localImage_.height();
        int diff = (container_height - img_height) / 2;

        if(me->button() == Qt::LeftButton)
        {
          if(img_state_ == IDLE)
          {
            return true;
          }
          else if(img_state_ == VICTIM_CLICK)
          {
            loader_.victimCoordsLabel->setText(
              loader_.victimCoordsLabel->text() + QString("[") +
              QString().setNum(p.x()) + QString(",") + 
              QString().setNum(p.y() - diff) + QString("]"));
          }
          else if(img_state_ == QR_CLICK)
          {
            loader_.qrCoordsLabel->setText(
              loader_.qrCoordsLabel->text() + QString("[") +
              QString().setNum(p.x()) + QString(",") + 
              QString().setNum(p.y() - diff) + QString("]"));
          }
          else if(img_state_ == HAZMAT_CLICK)
          {
            loader_.hazmatCoordsLabel->setText(
              loader_.hazmatCoordsLabel->text() + QString("[") +
              QString().setNum(p.x()) + QString(",") + 
              QString().setNum(p.y() - diff) + QString("]"));
          }
          else if(img_state_ == LANDOLTC_CLICK)
          {
            loader_.landoltcCoordsLabel->setText(
              loader_.landoltcCoordsLabel->text() + QString("[") +
              QString().setNum(p.x()) + QString(",") + 
              QString().setNum(p.y() - diff) + QString("]"));
          }
        }
      }
    }
    return false;
  }
  
  void CConnector::victimPushButtonTriggered(void)
  {
    img_state_ = VICTIM_CLICK;
  }
  void CConnector::qrPushButtonTriggered(void)
  {
    img_state_ = QR_CLICK;
  }
  void CConnector::hazmatPushButtonTriggered(void)
  {
    img_state_ = HAZMAT_CLICK;
  }
  void CConnector::landoltcPushButtonTriggered(void)
  {
    img_state_ = LANDOLTC_CLICK;
  }
}

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
    for (int i = 0; i<4; i++)
    {
        bbox_ready.push_back(0);
        qDebug("%d" , bbox_ready[i]);
    }

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
    QObject::connect(
      loader_.submitPushButton,SIGNAL(clicked(bool)),
      this,SLOT(submitPushButtonTriggered()));
    QObject::connect(
      loader_.clearPushButton,SIGNAL(clicked(bool)),
      this,SLOT(clearPushButtonTriggered()));
    QObject::connect(
      loader_.nextFramePushButton,SIGNAL(clicked(bool)),
      this,SLOT(nextFramePushButtonTriggered()));
    QObject::connect(
      loader_.previousFramePushButton,SIGNAL(clicked(bool)),
      this,SLOT(previousFramePushButtonTriggered()));


  }

  /**
  @brief Qt slot that is called when the rosTopicPushButton is pressed
  @return void
  **/
  void CConnector::rosTopicPushButtonTriggered(void)
  {    Q_EMIT rosTopicGiven();
  }

  void CConnector::show(void)
  {
    loader_.show();
  }

  QString CConnector::getRosTopic(void)
  {
    return loader_.rosTopicLineEdit->text();
  }
  void CConnector::setFrames(const std::vector<cv::Mat>& x)
  {
      frames = x;
      currFrame = 0;
  }

  void CConnector::setcurrentFrame()
  {
      QImage dest((const uchar *) frames[currFrame].data, frames[currFrame].cols, frames[currFrame].rows, frames[currFrame].step, QImage::Format_RGB888);
      dest.bits(); // enforce deep copy, see documentation
      setImage(dest);
      Q_EMIT updateImage();
  }

  void CConnector::setImage(QImage &img)
  {
       localImage_ = img.copy();
  }

  void CConnector::updateImage()
  {
    loader_.imageLabel->setPixmap(QPixmap().fromImage(localImage_));
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
      if(event -> type()== QEvent::KeyPress);
      {
          QKeyEvent*  me = static_cast<QKeyEvent*> (event);
          if(me->key() == Qt::Key_S)
          {
            qDebug("Save current Frame as: frame%04d.png",currFrame);
            std::string img_name = "/home/marios/frame000" + boost::to_string(currFrame) + ".png";
            cv::imwrite(img_name, frames[currFrame]);
          }
      }

      if(event->type() == QEvent::MouseButtonPress)
      {
        loader_.imageLabel->setFocus(Qt::MouseFocusReason);
        std::string img_name = "frame000" + boost::to_string(currFrame) + ".png";
        //qDebug() << "load Image" << img_name;
        const QMouseEvent* const me =
          static_cast<const QMouseEvent*>( event );
        QPoint p = me->pos();

        int container_width = loader_.imageLabel->width();
        int container_height = loader_.imageLabel->height();
        int img_height = localImage_.height();
        int diff = (container_height - img_height) / 2;
        /*qDebug("container_width x height %d %d",container_width, container_height);*/
        /*qDebug("imgheight %d %d %d" ,localImage_.width(), img_height, diff);*/


        if(me->button() == Qt::LeftButton)
        {

          switch (img_state_)
          {
              case IDLE:
                  {
                      return true;
                  }
              case VICTIM_CLICK:
                  {
                    loader_.victimCoordsLabel->setText(
                     loader_.victimCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() /*- diff*/) + QString("]"));
                     ImgAnnotations::setAnnotations(img_name,"Victim", p.x(), p.y()-diff);
                    bbox_ready[0]++;
                    if(bbox_ready[0] == 2)
                    {
                        drawBox();
                        bbox_ready[0] = 0;
                        ImgAnnotations::annPerImage++;
                    }
                    break;
                  }
              case QR_CLICK:
                  {
                    loader_.qrCoordsLabel->setText(
                     loader_.qrCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() - diff) + QString("]"));
                    ImgAnnotations::setAnnotations(img_name,"QR", p.x(), p.y()-diff);
                    bbox_ready[1]++;
                    if(bbox_ready[1] == 2)
                    {
                        drawBox();
                        bbox_ready[1] = 0;
                        ImgAnnotations::annPerImage++;
                    }
                    break;
                  }
               case HAZMAT_CLICK:
                  {
                    loader_.hazmatCoordsLabel->setText(
                     loader_.hazmatCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() - diff) + QString("]"));
                    ImgAnnotations::setAnnotations(img_name,"Hazmat", p.x(), p.y()-diff);
                    bbox_ready[2]++;
                    if(bbox_ready[2] == 2)
                    {
                        drawBox();
                        bbox_ready[2] = 0;
                        ImgAnnotations::annPerImage++;
                    }
                    break;
                  }
               case LANDOLTC_CLICK:
                  {
                    loader_.landoltcCoordsLabel->setText(
                     loader_.landoltcCoordsLabel->text() + QString("[") +
                     QString().setNum(p.x()) + QString(",") +
                     QString().setNum(p.y() - diff) + QString("]"));
                    ImgAnnotations::setAnnotations(img_name,"LandoltC", p.x(), p.y()-diff);
                    bbox_ready[3]++;
                    if(bbox_ready[3] == 2)
                    {
                        drawBox();
                        bbox_ready[3] = 0;
                        ImgAnnotations::annPerImage++;
                    }
                    break;
                  }
          }
        }
      }
    }
    return false;
  }

  void CConnector::victimPushButtonTriggered(void)
  { if(bbox_ready[0]== 0)
    img_state_ = VICTIM_CLICK;
  }
  void CConnector::qrPushButtonTriggered(void)
  {
    if(bbox_ready[1]== 0)
    img_state_ = QR_CLICK;
  }
  void CConnector::hazmatPushButtonTriggered(void)
  {
    if(bbox_ready[2]== 0)
    img_state_ = HAZMAT_CLICK;
  }
  void CConnector::landoltcPushButtonTriggered(void)
  {
    if(bbox_ready[3] == 0)
    img_state_ = LANDOLTC_CLICK;
  }

  void CConnector::submitPushButtonTriggered(void)
  { if(ImgAnnotations::is_file_exist("/home/marios/annotations.txt"))
    {
      remove("/home/marios/annotations.txt");

    }
    ImgAnnotations::writeToFile("/home/marios/annotations.txt");

  }

  void CConnector::clearPushButtonTriggered(void)
  {
      img_state_=IDLE;
      bbox_ready.clear();
      loader_.landoltcCoordsLabel->clear();
      loader_.qrCoordsLabel->clear();
      loader_.hazmatCoordsLabel->clear();
      loader_.victimCoordsLabel->clear();
      ImgAnnotations::annPerImage = 0;
      ImgAnnotations::annotations.clear();
      setcurrentFrame();
      //setImage(backupImage_);
      //updateImage();
  }
  void CConnector::nextFramePushButtonTriggered(void)
  {
    if(currFrame != frames.size()-1)
    {
    currFrame++;
    setcurrentFrame();
    }
  }
  void CConnector::previousFramePushButtonTriggered(void)
  {
    if(currFrame != 0)
    {
     currFrame--;
     setcurrentFrame();
    }
  }
  void CConnector::drawBox()
  {
    //annImage_ = localImage_.copy();
    if(ImgAnnotations::annPerImage == 0)
    backupImage_=localImage_.copy();
    QPainter painter(&localImage_);
    painter.setRenderHint(QPainter::Antialiasing);
    QPen pen;
    pen.setWidth( 3 );
    pen.setBrush(Qt::green);
    painter.setPen(pen);
    for (int i = 0; i < bbox_ready.size(); i++)
        qDebug("%d", bbox_ready[i]);
    qDebug("drawing %d annotation", ImgAnnotations::annPerImage);
               qDebug("%d %d %d %d\n",
                      ImgAnnotations::annotations[ImgAnnotations::annPerImage].x1,
                      ImgAnnotations::annotations[ImgAnnotations::annPerImage].y1,
                      ImgAnnotations::annotations[ImgAnnotations::annPerImage].x2,
                      ImgAnnotations::annotations[ImgAnnotations::annPerImage].y2);
    QPoint p1(ImgAnnotations::annotations[ImgAnnotations::annPerImage].x1,
              ImgAnnotations::annotations[ImgAnnotations::annPerImage].y1);
    QPoint p2(ImgAnnotations::annotations[ImgAnnotations::annPerImage].x2,
              ImgAnnotations::annotations[ImgAnnotations::annPerImage].y2);
    QRect r(p1,p2);
    painter.drawRect(r);
    updateImage();
 }

}

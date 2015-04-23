/*********************************************************************
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
* Author: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/victim_vj_detector.h"

namespace pandora_vision
{
  /**
  @brief Class Constructor
  Initializes cascade_name and constants
  and allocates memory for sequence of elements
  @param cascade_path [std::string] : the name of
  the cascade to be loaded
  @param model_path [std::string] : the path to the model
  to be loaded
  @param bufferSize [int] number : of frames in the frame buffer
  @return void
  **/
  VictimVJDetector::VictimVJDetector(const std::string& ns, sensor_processor::AbstractHandler* handler) :
    Processor<VictimCVMatStamped, POIsStamped>(ns, handler), params_(ns), imageTransport_(this->accessPublicNh())
  {
    trained_cascade.load(VictimParameters::cascade_path);
    //~ ROS_ERROR("%s", cascade_path.c_str());
    if(trained_cascade.empty())
    {
      ROS_ERROR("[Face Detector]: Cannot load cascade classifier");
      exit(0);
    }
    trained_model = cv::createFisherFaceRecognizer();
    trained_model->load(VictimParameters::model_path);
    
    RgbSystemValidator::initialize(  // FROM DETECTION
      VictimParameters::rgb_classifier_path);

    DepthSystemValidator::initialize(  // FROM DETECTION
      VictimParameters::depth_classifier_path);
      
    _debugVictimsPublisher = imageTransport_.advertise  // FROM DETECTION
      (VictimParameters::victimDebugImg, 1, true);
    _interpolatedDepthPublisher = imageTransport_.advertise  // FROM DETECTION
      (VictimParameters::interpolatedDepthImg, 1, true);
  }

  /**
  @brief Class Destructor
  Deallocates all memory used for storing sequences of faces,
  matrices and images
  **/
  VictimVJDetector::~VictimVJDetector()
  {
  }


  /**
  @brief Detects number of faces found in current frame.
  The image buffer contributs to probability.
  @param frame [cv::Mat] The frame to be scanned for faces
  @return Integer of the sum of faces found in all
  rotations of the frame
  **/
  std::vector<DetectedVictim> VictimVJDetector::findFaces(cv::Mat frame)
  {
    cv::Mat tmp;
    tmp = cv::Mat::zeros(frame.size().width, frame.size().height , CV_8UC1);

    //! Clear vector of faces before using it for the current frame
    faces_total.clear();

    std::vector<DetectedVictim> candidateVictim;
    std::vector<float> preds = detectFace(frame);
    std::vector<float> probs = predictionToProbability(preds);
    std::vector<BoundingBox> keypoints = getAlertKeypoints();
    
    if(probs.size() != keypoints.size())
    {
      ROS_FATAL("[PANDORA_VICTIM] Something went terribly wrong");
    }
    
    for(unsigned int i = 0 ; i < probs.size() ; i++)
    {
      DetectedVictim dv;
      dv.probability = probs[i];
      dv.keypoint = keypoints[i].keypoint;
      dv.boundingBox = keypoints[i].bounding_box;
      candidateVictim.push_back(dv);
    }
    
    return candidateVictim;
  }

  /**
  @brief Creates the continuous table of faces found that contains
  information for each face in every set of 4 values:
  @return int[] table of face positions and sizes
  **/
  std::vector<BoundingBox> VictimVJDetector::getAlertKeypoints()
  {
    std::vector<BoundingBox> table;
    for(int ii = 0; ii < faces_total.size(); ii++)
    {
      BoundingBox tbb;
      cv::Rect faceRect = faces_total.at(ii);
      cv::Point2f p (
        round( faceRect.x + faceRect.width * 0.5 ),
        round( faceRect.y + faceRect.height * 0.5 )
      );
      tbb.keypoint = p;
      tbb.bounding_box = faceRect;
      table.push_back(tbb);
    }
    return table;
  }

  /**
  @brief Returns the probability of the faces detected in the frame
  @return [float] probability value
  */  
  std::vector<float> VictimVJDetector::predictionToProbability
    (std::vector<float> prediction)
  {
    std::vector<float> p;
    for(unsigned int i = 0 ; i < prediction.size() ; i++)
    {
      //~ Normalize probability to [-1,1]
      float temp_prob = tanh(0.5 * (prediction[i] - 20.0) );
      //~ Normalize probability to [0,1]
      temp_prob = (1 + temp_prob) / 2.0;
      ROS_INFO_STREAM("Viola pred/prob :" << prediction[i] << " " << temp_prob);
      p.push_back(temp_prob);
    }
    return p;
  }

  /**
  @brief Calls detectMultiscale to scan frame for faces and drawFace
  to create rectangles around the faces found in each frame
  @param img [cv::Mat] the frame to be scaned.
  @return [int] the number of faces found in each frame
  **/
  std::vector<float> VictimVJDetector::detectFace(cv::Mat img)
  {
    std::vector<float> predictions;
    cv::Mat original(img.size().width, img.size().height, CV_8UC1);
    original = img.clone();
    cv::Mat gray(img.size().width, img.size().height, CV_8UC1);
    if(original.channels() != 1)
    {
      cvtColor(original, gray, CV_BGR2GRAY);
    }
    std::vector< cv::Rect_<int> > thrfaces;

    int im_width = VictimParameters::modelImageWidth;
    int im_height = VictimParameters::modelImageHeight;
    
    if(!trained_cascade.empty())
    {
      //! Find the faces in the frame:
      trained_cascade.detectMultiScale(gray, thrfaces);
      for(int i = 0; i < thrfaces.size(); i++)
      {
        //! Process face by face:
        cv::Rect face_i = thrfaces[i];
        cv::Mat face = gray(face_i);
        cv::Mat face_resized;
        cv::resize(face, face_resized, cv::Size(im_width, im_height), 
          1.0, 1.0, cv::INTER_CUBIC);
        double local_conf = 0.0;
        int pred_label = -1;
        trained_model->predict(face_resized, pred_label, local_conf);
        predictions.push_back(local_conf);
        rectangle(original, face_i, CV_RGB(0, 255, 0), 1);
        //! Add every element created for each frame, to the total amount of faces
        faces_total.push_back (thrfaces.at(i));
      }
    }
    thrfaces.clear();
    return predictions;
  }
  
  /**
  @brief This method check in which state we are, according to
  the information sent from hole_detector_node
  @return void
  **/
  void VictimDetection::detectVictims(
    bool depthEnabled,
    bool holesEnabled,
    const cv::Mat& rgbImage,
    const cv::Mat& depthImage,
    const pandora_vision_msgs::EnhancedHolesVectorMsg& msg
  )
  {
    if(VictimParameters::debug_img || VictimParameters::debug_img_publisher)
    {
      rgbImage.copyTo(debugImage);
      rgb_vj_keypoints.clear();
      rgb_svm_keypoints.clear();
      depth_vj_keypoints.clear();
      depth_svm_keypoints.clear();
      rgb_vj_bounding_boxes.clear();
      rgb_svm_bounding_boxes.clear();
      depth_vj_bounding_boxes.clear();
      depth_svm_bounding_boxes.clear();
      holes_bounding_boxes.clear();
      rgb_vj_p.clear();
      rgb_svm_p.clear();
      depth_vj_p.clear();
      depth_svm_p.clear();
    }

    DetectionImages imgs;
    int stateIndicator = 2 * depthEnabled + holesEnabled + 1;

    {
      EnhancedMat emat;
      rgbImage.copyTo(emat.img);
      imgs.rgb = emat;
      imgs.rgb.bounding_box = cv::Rect(0, 0, 0, 0);
      imgs.rgb.keypoint = cv::Point2f(0, 0);
    }

    DetectionMode detectionMode;
    switch(stateIndicator)
    {
      case 1:
        detectionMode = GOT_RGB;
        break;
      case 2:
        detectionMode = GOT_HOLES;
        break;
      case 3:
        detectionMode = GOT_DEPTH;
        {
          EnhancedMat emat;
          depthImage.copyTo(emat.img);
          imgs.depth = emat;
          imgs.depth.bounding_box = cv::Rect(0, 0, 0, 0);
          imgs.depth.keypoint = cv::Point2f(0, 0);
        }
        break;
      case 4:
        detectionMode = GOT_HOLES_AND_DEPTH;
        {
          EnhancedMat emat;
          depthImage.copyTo(emat.img);
          imgs.depth = emat;
          imgs.depth.bounding_box = cv::Rect(0, 0, 0, 0);
          imgs.depth.keypoint = cv::Point2f(0, 0);
        }
        break;
    }
    for(unsigned int i = 0 ; i < msg.enhancedHoles.size();
      i++)
    {
      int minx = 10000, maxx = -1, miny = 10000, maxy = -1;
      for(unsigned int j = 0 ; j < 4 ; j++)
      {
        int xx = msg.enhancedHoles[i].verticesX[j];
        int yy = msg.enhancedHoles[i].verticesY[j];
        minx = xx < minx ? xx : minx;
        maxx = xx > maxx ? xx : maxx;
        miny = yy < miny ? yy : miny;
        maxy = yy > maxy ? yy : maxy;
      }
      cv::Rect rect(minx, miny, maxx - minx, maxy - miny);
      holes_bounding_boxes.push_back(rect);

      EnhancedMat emat;
      emat.img = rgbImage(rect);
      cv::resize(emat.img, emat.img,
        cv::Size(VictimParameters::frameWidth, VictimParameters::frameHeight));
      emat.bounding_box = rect;
      emat.keypoint = cv::Point2f(
        msg.enhancedHoles[i].keypointX,
        msg.enhancedHoles[i].keypointY
      );
      imgs.rgbMasks.push_back(emat);

      if(GOT_HOLES_AND_DEPTH || GOT_DEPTH)
      {
        emat.img = depthImage(rect);
        imgs.depthMasks.push_back(emat);
      }
    }

    std::vector<DetectedVictim> final_victims =
      victimFusion(imgs, detectionMode);

    //!< Message alert creation
    for(int i = 0;  i < final_victims.size() ; i++)
    {
      if( final_victims[i].probability > 0.0001)
      {

        float x = final_victims[i].keypoint.x
          - static_cast<float>(VictimParameters::frameWidth) / 2;
        float y = static_cast<float>(VictimParameters::frameHeight) / 2
          - final_victims[i].keypoint.y;

        //!< Create message of Victim Detector
        pandora_common_msgs::GeneralAlertMsg victimMessage;

        victimMessage.header.frame_id = _frame_ids_map.find(_frame_id)->second;

        victimMessage.header.stamp = victimFrameTimestamp;

        victimMessage.yaw =
          atan(2 * x / VictimParameters::frameWidth
            * tan(VictimParameters::hfov / 2));

        victimMessage.pitch =
          atan(2 * y / VictimParameters::frameHeight
            * tan(VictimParameters::vfov / 2));

        victimMessage.probability = final_victims[i].probability;

        _victimDirectionPublisher.publish(victimMessage);
      }
      //!< Debug purposes
      if(VictimParameters::debug_img || VictimParameters::debug_img_publisher)
      {
        cv::KeyPoint kp(final_victims[i].keypoint, 10);
        cv::Rect re = final_victims[i].boundingBox;
        switch(final_victims[i].source)
        {
          case RGB_VJ:
            rgb_vj_keypoints.push_back(kp);
            rgb_vj_bounding_boxes.push_back(re);
            rgb_vj_p.push_back(final_victims[i].probability);
            break;
          case RGB_SVM:
            rgb_svm_keypoints.push_back(kp);
            rgb_svm_bounding_boxes.push_back(re);
            rgb_svm_p.push_back(final_victims[i].probability);
            break;
          case DEPTH_VJ:
            depth_vj_keypoints.push_back(kp);
            depth_vj_bounding_boxes.push_back(re);
            depth_vj_p.push_back(final_victims[i].probability);
            break;
          case DEPTH_RGB_SVM:
            depth_svm_keypoints.push_back(kp);
            depth_svm_bounding_boxes.push_back(re);
            depth_svm_p.push_back(final_victims[i].probability);
            break;
        }

      }
    }

    //! Debug image
    if(VictimParameters::debug_img || VictimParameters::debug_img_publisher)
    {
      cv::drawKeypoints(debugImage, rgb_vj_keypoints, debugImage,
        CV_RGB(0, 255, 0),
        cv::DrawMatchesFlags::DEFAULT);
      for(unsigned int i = 0 ; i < rgb_vj_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, rgb_vj_bounding_boxes[i],
          CV_RGB(0, 255, 0));
        {
          std::ostringstream convert;
          convert << rgb_vj_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            rgb_vj_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 0), 1, CV_AA);
        }
      }

      cv::drawKeypoints(debugImage, depth_vj_keypoints, debugImage,
        CV_RGB(255, 100, 0),
        cv::DrawMatchesFlags::DEFAULT);
      for(unsigned int i = 0 ; i < depth_vj_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, depth_vj_bounding_boxes[i],
          CV_RGB(255, 100, 0));
        {
          std::ostringstream convert;
          convert << depth_vj_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            depth_vj_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(255, 100, 0), 1, CV_AA);
        }
      }

      cv::drawKeypoints(debugImage, rgb_svm_keypoints, debugImage,
        CV_RGB(0, 100, 255),
        cv::DrawMatchesFlags::DEFAULT);
      for(unsigned int i = 0 ; i < rgb_svm_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, rgb_svm_bounding_boxes[i],
          CV_RGB(0, 100, 255));
        {
          std::ostringstream convert;
          convert << rgb_svm_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            rgb_svm_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 100, 255), 1, CV_AA);
        }
      }

      cv::drawKeypoints(debugImage, depth_svm_keypoints, debugImage,
        CV_RGB(0, 255, 255),
        cv::DrawMatchesFlags::DEFAULT);
      for(unsigned int i = 0 ; i < depth_svm_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, depth_svm_bounding_boxes[i],
          CV_RGB(0, 255, 255));
        {
          std::ostringstream convert;
          convert << depth_svm_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            depth_svm_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 255), 1, CV_AA);
        }
      }
      for(unsigned int i = 0 ; i < holes_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, holes_bounding_boxes[i],
          CV_RGB(0, 0, 0));
      }

      {
        std::ostringstream convert;
        convert << "RGB_VJ : "<< rgb_vj_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 20),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 0), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "DEPTH_VJ : "<< depth_vj_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 40),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(255, 100, 0), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "RGB_SVM : "<< rgb_svm_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 60),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 100, 255), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "DEPTH_SVM : "<< depth_svm_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 80),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 255), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "Holes got : "<< msg.enhancedHoles.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 100),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 0, 0), 1, CV_AA);
      }
    }
    if(VictimParameters::debug_img_publisher)
    {
      // Convert the image into a message
      cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

      msgPtr->header = msg.header;
      msgPtr->encoding = sensor_msgs::image_encodings::BGR8;
      msgPtr->image = debugImage;
      // Publish the image message
      _debugVictimsPublisher.publish(*msgPtr->toImageMsg());
    }
    if(VictimParameters::debug_img)
    {
      cv::imshow("Victim detector", debugImage);
      cv::waitKey(30);
    }
  }

  /**
  @brief Function that enables suitable subsystems, according
  to the current State
  @param [std::vector<cv::Mat>] : vector of images to be processed. Size of
  vector can be either 2 or 1, if we have both rgbd information or not
  @return void
  **/
  std::vector<DetectedVictim> VictimDetection::victimFusion(
    DetectionImages imgs,
    DetectionMode detectionMode)
  {
    std::vector<DetectedVictim> final_probabilities;

    std::vector<DetectedVictim> rgb_vj_probabilities;
    std::vector<DetectedVictim> depth_vj_probabilities;
    std::vector<DetectedVictim> rgb_svm_probabilities;
    std::vector<DetectedVictim> depth_svm_probabilities;

    DetectedVictim temp;

    ///Enable Viola Jones for rgb image
    rgb_vj_probabilities = _rgbViolaJonesDetector.findFaces(imgs.rgb.img);

    if(detectionMode == GOT_HOLES_AND_DEPTH || detectionMode == GOT_DEPTH)
    {
      depth_vj_probabilities = _rgbViolaJonesDetector.findFaces(imgs.depth.img);
    }
    if(detectionMode == GOT_HOLES || detectionMode == GOT_HOLES_AND_DEPTH )//|| detectionMode == GOT_RGB
    {
      for(int i = 0 ; i < imgs.rgbMasks.size(); i++)
      {
        temp.probability = RgbSystemValidator::calculateSvmRgbProbability(
          imgs.rgbMasks.at(i).img);
        temp.keypoint = imgs.rgbMasks[i].keypoint;
        temp.source = RGB_SVM;
        temp.boundingBox = imgs.rgbMasks[i].bounding_box;
        rgb_svm_probabilities.push_back(temp);
      }
    }
    if(detectionMode == GOT_HOLES_AND_DEPTH)
    {
      for(int i = 0 ; i < imgs.depthMasks.size(); i++)
      {
        temp.probability = DepthSystemValidator::calculateSvmDepthProbability(
          imgs.depthMasks.at(i).img);
        temp.keypoint = imgs.depthMasks[i].keypoint;
        temp.source = DEPTH_RGB_SVM;
        temp.boundingBox = imgs.depthMasks[i].bounding_box;
        depth_svm_probabilities.push_back(temp);
      }
    }

    // SVM mask merging
    // Combine rgb & depth probabilities
    if(detectionMode == GOT_HOLES_AND_DEPTH)
    {
      for(unsigned int i = 0 ; i < depth_svm_probabilities.size() ; i++)
      {
        //! Weighted mean
        temp.probability =
          (VictimParameters::depth_svm_weight *
            depth_svm_probabilities[i].probability +
          VictimParameters::rgb_svm_weight *
            rgb_svm_probabilities[i].probability) /
          (VictimParameters::depth_svm_weight +
            VictimParameters::rgb_svm_weight);

        temp.keypoint = depth_svm_probabilities[i].keypoint;
        temp.source = DEPTH_RGB_SVM;
        temp.boundingBox = depth_svm_probabilities[i].boundingBox;
        final_probabilities.push_back(temp);
      }
    }
    // Only rgb svm probabilities

    if(detectionMode == GOT_HOLES )  // || detectionMode == GOT_RGB
    {
      for(unsigned int i = 0 ; i < rgb_svm_probabilities.size() ; i++)
      {
        temp.probability = rgb_svm_probabilities[i].probability *
          VictimParameters::rgb_svm_weight;
        temp.keypoint = rgb_svm_probabilities[i].keypoint;
        temp.source = RGB_SVM;
        temp.boundingBox = rgb_svm_probabilities[i].boundingBox;
        final_probabilities.push_back(temp);
      }
    }

    // VJ mask merging (?)
    for(unsigned int i = 0 ; i < rgb_vj_probabilities.size() ; i++)
    {
      temp.probability = rgb_vj_probabilities[i].probability *
        VictimParameters::rgb_vj_weight;
      temp.keypoint = rgb_vj_probabilities[i].keypoint;
      temp.source = RGB_VJ;
      temp.boundingBox = rgb_vj_probabilities[i].boundingBox;
      final_probabilities.push_back(temp);
    }
    for(unsigned int i = 0 ; i < depth_vj_probabilities.size() ; i++)
    {
      temp.probability = depth_vj_probabilities[i].probability *
        VictimParameters::depth_vj_weight;
      temp.keypoint = depth_vj_probabilities[i].keypoint;
      temp.source = DEPTH_VJ;
      temp.boundingBox = depth_vj_probabilities[i].boundingBox;
      final_probabilities.push_back(temp);
    }

    return final_probabilities;
  }
  
  bool VictimVJDetector::process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output)
  {
    // ..................MORE HERE
    
    cv_bridge::CvImagePtr inMsgD = cv_bridge::toCvCopy(msg.depthImage, sensor_msgs::image_encodings::TYPE_8UC1);
    cv::Mat depthImage = inMsgD->image.clone();

    //! The actual victim detection
    detectVictims(
      msg.isDepth,
      msg.enhancedHoles.size() > 0,
      rgbImage,
      depthImage,
      msg
    );

    //! Interpolated depth image publishing
    {
      // Convert the image into a message
      cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

      msgPtr->header = msg.header;
      msgPtr->encoding = sensor_msgs::image_encodings::MONO8;
      depthImage.copyTo(msgPtr->image);

      // Publish the image message
      _interpolatedDepthPublisher.publish(*msgPtr->toImageMsg());
    }
    
    // .................MORE HERE
  }
}  // namespace pandora_vision

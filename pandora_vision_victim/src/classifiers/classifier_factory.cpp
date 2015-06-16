/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Authors:
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*   Vassilis Choutas <vasilis4ch@gmail.com>
*********************************************************************/

#include <string>

#include <boost/algorithm/string.hpp>

#include "pandora_vision_victim/classifiers/classifier_factory.h"
#include "pandora_vision_victim/classifiers/svm_classifier.h"
#include "pandora_vision_victim/classifiers/neural_network_classifier.h"
#include "pandora_vision_victim/classifiers/random_forests_classifier.h"

namespace pandora_vision
{
  ClassifierFactory::ClassifierFactory(const std::string& ns): nh_(ns)
  {
    ROS_INFO("[Victim_Training]: Starting Victim Training Procedure!");

    std::string imageType;
    std::string classifierType;

    if (!nh_.getParam("image_type", imageType))
    {
      ROS_ERROR("Could not retrieve the image type from the yaml file!");
      ROS_ERROR("Shutting Down now!");
      ros::shutdown();
    }

    if (!nh_.getParam("classifier_type", classifierType))
    {
      ROS_ERROR("Could not retrieve the classifier type from the yaml file!");
      ROS_ERROR("Shutting Down now!");
      ros::shutdown();
    }

    boost::shared_ptr<AbstractClassifier> victimTrainerPtr;

    victimTrainerPtr.reset(this->createClassifier(classifierType, imageType));
    if (!victimTrainerPtr)
    {
      ROS_ERROR("[Victim_Training]: Null pointer allocated for classifier Trainer!");
      ROS_ERROR("[Victim_Training]: The node will now terminate!");
      ros::shutdown();
    }

    // Structs for calculating elapsed time.
    struct timeval startwtime, endwtime;
    gettimeofday(&startwtime, NULL);

    // Train the system
    victimTrainerPtr->trainAndValidate();

    gettimeofday(&endwtime, NULL);
    double trainingTime = static_cast<double>((endwtime.tv_usec -
          startwtime.tv_usec) / 1.0e6
        + endwtime.tv_sec - startwtime.tv_sec);
    std::cout << "The training finished after " << trainingTime << " seconds" << std::endl;

    ROS_INFO("[Victim_Training]: The procedure has finished!");
  }

  AbstractClassifier* ClassifierFactory::createClassifier(const std::string& classifierType,
          const std::string& imageType)
  {
    int numFeatures;
    std::string datasetPath;

    if (!nh_.getParam("num_features", numFeatures))
    {
      ROS_ERROR("Could not retrieve the number of features for the system!");
      ros::shutdown();
    }

    if (!nh_.getParam("dataset_path", datasetPath))
    {
      ROS_ERROR("Could not get the path to the training dataset!");
      ros::shutdown();
    }

    std::string ns = nh_.getNamespace();

    AbstractClassifier* classifierPtr;

    if (boost::iequals(classifierType, "svm"))
    {
      classifierPtr = new SvmClassifier(ns, numFeatures, datasetPath, classifierType, imageType);
    }
    else if (boost::iequals(classifierType, "ann"))
    {
      classifierPtr =  new NeuralNetworkClassifier(ns, numFeatures, datasetPath, classifierType, imageType);
    }
    else if (boost::iequals(classifierType, "rf"))
    {
      classifierPtr =  new RandomForestsClassifier(ns, numFeatures, datasetPath, classifierType, imageType);
    }
    else
    {
      ROS_ERROR("[Victim_Training]: Invalid Classifier Type!");
      ROS_ERROR("[Victim Training]: The node will now shutdown!");
      return NULL;
    }

    ROS_INFO("[Victim_Training]: Created %s Classifier for %s images",
        boost::to_upper_copy<std::string>(classifierType).c_str(),
        boost::to_upper_copy<std::string>(imageType).c_str());
    return classifierPtr;
  }

  ClassifierFactory::~ClassifierFactory()
  {
  }
}  //  namespace pandora_vision

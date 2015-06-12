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
*   Vassilis Choutas  <vasilis4ch@gmail.com>
*********************************************************************/

#include <sstream>
#include <string>

#include <ros/ros.h>

#include "pandora_vision_victim/classifiers/neural_network_classifier.h"

namespace pandora_vision
{
  /**
   * @brief Constructor for the Neural Network Classifier Wrapper Class
   * @param ns[const std::string&] The namespace of the node
   * @param numFeatures[int] The number of input features to the classifier
   * @param datasetPath[const std::string&] The path to the training dataset
   * @param classifierType[const std::string&] The model used by the
   * classifier.
   * @param imageType[const std::string&] The type of input images given to
   * the classifier(RGB or Depth)
   */
  NeuralNetworkClassifier::NeuralNetworkClassifier(const std::string& ns,
      int numFeatures, const std::string& datasetPath,
      const std::string& classifierType,
      const std::string& imageType)
    : AbstractClassifier(ns, numFeatures, datasetPath, classifierType,
        imageType)
  {
    ROS_INFO("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Creating Neural "
        "Network training instance");

    XmlRpc::XmlRpcValue layers;
    if (!nh_.getParam("layers", layers))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve"
          " the number of neurons in each layer of the network!");
      ros::shutdown();
    }
    int layerNum = static_cast<int>(layers.size());

    // Iterate over the parameter YAML file to get the size for each layer
    // of the ANN.
    cv::Mat layerSizes(layerNum, 1, CV_32SC1);
    int ii = 0;

    for (int ii = 0; ii < layerNum ; ii++)
    {
      std::stringstream ss;
      ss << layers[ii];
      ss >> layerSizes.at<int>(ii);
    }

    // Get the parameters of  the sigmoid function.
    double alpha, beta;

    if (!nh_.getParam("alpha", alpha))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]:Could not retrieve" 
          " alpha parameter for sigmoid function!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]:Setting alpha value"
          " to 1!");
      alpha = 1;
    }

    if (!nh_.getParam("beta", beta))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve" 
          " beta parameter for the sigmoid function!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting beta value"
          " to 1!");
      beta = 1;
    }

    std::string trainingAlgorithm;

    if (!nh_.getParam("training_algorithm", trainingAlgorithm))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve" 
          " the type of the training algorithm for the neural Network!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Using the standard"
          " back propagation algorithm!");
      trainingAlgorithm = std::string("BackPropagation");
      NeuralNetworkParams_.train_method = cv::ANN_MLP_TrainParams::BACKPROP;
    }

    double learningRate, bpMomentScale;
    // Parse the learning rate parameter
    if (!nh_.getParam("learning_rate", learningRate))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve" 
          " the learning rate for the training procedure!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting the learning"
          " rate to 0.1");
      learningRate = 0.1;
    } 

    if (!nh_.getParam("momentum_scale", bpMomentScale))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve" 
          " the strength of the momentum term!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting the momentum"
          " term to 0(the feature will be disabled)");
      bpMomentScale = 0.1;
    }

    // Get the maximum number of iterations for the training of the network.
    int maxIter;
    if (!nh_.getParam("maximum_iter", maxIter))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve" 
          " the maximum number number of training iterations!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting its value"
          " to 1000 iterations.");
      maxIter = 1000;
    }

    // Get the maximum number of iterations for the training of the network.
    double epsilon;
    if (!nh_.getParam("epsilon", epsilon))
    {
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Could not retrieve" 
          " the epsilon value for the error change between iterations!");
      ROS_DEBUG("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Setting its value"
          " to 0.01!");
      epsilon = 0.01;
    }

    // TO DO(Vassilis Choutas): Add RLProp parameters
    // If the training algorithm is not the back propagation algorithm
    // then we must get the rest of the parameters for RPROP algorithm.
    if (trainingAlgorithm.compare("BackPropagation") != 0)
    {
      NeuralNetworkParams_.train_method= cv::ANN_MLP_TrainParams::RPROP;
    }
    else
    {
      NeuralNetworkParams_.train_method= cv::ANN_MLP_TrainParams::BACKPROP;
      NeuralNetworkParams_.bp_dw_scale = learningRate;
      NeuralNetworkParams_.bp_moment_scale = bpMomentScale;
    }
    // Initialize the training algorithm's termination criteria.
    NeuralNetworkParams_.term_crit =
      cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIter, epsilon);

    // Initialize the pointer to the Neural Network Classifier object.
    classifierPtr_.reset(new CvANN_MLP());

    // Create the Neural Network with the specified topology.
    classifierPtr_->create(layerSizes, CvANN_MLP::SIGMOID_SYM, 
        alpha, beta);

    ROS_INFO("[PANDORA_VISION_VICTIM_NEURAL_NETWORK]: Successfully created " 
        "Neural Network Classifier Object!");
  } // End of NeuralNetworkClassifier Constructor

  /**
   * @brief Destructor
   */
  NeuralNetworkClassifier::~NeuralNetworkClassifier()
  {
    ROS_DEBUG("[victim_node] : Destroying Neural Network training instance");
  }

  /**
   * @brief Function that implements the training for the subsystems
   * according to the given training sets. It applies the training algorithm for the corresponding
   * classifier tyep and extracts a suitable model.
   * @return void
   */
  void NeuralNetworkClassifier::trainSubSystem()
  {
    int numTrainingFiles = file_utilities::findNumberOfAnnotations(trainingAnnotationsFile_);
    int numTestFiles = file_utilities::findNumberOfAnnotations(testAnnotationsFile_);

    cv::Mat trainingFeaturesMat = cv::Mat::zeros(numTrainingFiles, numFeatures_, CV_64FC1);
    cv::Mat trainingLabelsMat = cv::Mat::zeros(numTrainingFiles, 1, CV_64FC1);
    cv::Mat testFeaturesMat = cv::Mat::zeros(numTestFiles, numFeatures_, CV_64FC1);
    cv::Mat testLabelsMat = cv::Mat::zeros(numTestFiles, 1, CV_64FC1);

    if (loadClassifierModel_ && file_utilities::exist(classifierFile_.c_str()))
    {
      classifierPtr_->load(classifierFile_.c_str());
    }
    else
    {
      if (file_utilities::exist(trainingFeaturesMatrixFile_.c_str()) == false ||
          trainingSetFeatureExtraction_)
      {
        std::cout << "Create necessary training matrix" << std::endl;
        std::string prefix = "training_";

        bool vocabularyNeeded = constructBagOfWordsVocabulary(trainingDirectory_,
            trainingAnnotationsFile_);

        constructFeaturesMatrix(trainingDirectory_, trainingAnnotationsFile_,
            &trainingFeaturesMat, &trainingLabelsMat);

        trainingFeaturesMat.convertTo(trainingFeaturesMat, CV_32FC1);
        trainingLabelsMat.convertTo(trainingLabelsMat, CV_32FC1);

        file_utilities::saveFeaturesInFile(trainingFeaturesMat, trainingLabelsMat,
            prefix, trainingFeaturesMatrixFile_, trainingLabelsMatrixFile_,
            imageType_);

        if (vocabularyNeeded)
        {
          std::cout << "Save bag of words vocabulary" << std::endl;
          const std::string bagOfWordsFile = imageType_ + classifierType_ + "bag_of_words.xml";
          const std::string bagOfWordsFilePath = filesDirectory_ + bagOfWordsFile;
          file_utilities::saveToFile(bagOfWordsFilePath, "bag_of_words",
              featureExtraction_->getBagOfWordsVocabulary());
        }
      }
      else
      {
        trainingFeaturesMat = file_utilities::loadFiles(
            trainingFeaturesMatrixFile_, "training_features_mat");
        trainingLabelsMat = file_utilities::loadFiles(
            trainingLabelsMatrixFile_, "training_labels_mat");
      }

      // Start Training Process
      std::cout << "Starting training process for the " << imageType_ << " images" << std::endl;

      // Structs for calculating elapsed time.
      struct timeval startwtime, endwtime;
      gettimeofday(&startwtime , NULL);
      classifierPtr_->train(trainingFeaturesMat,  trainingLabelsMat,
          cv::Mat(), cv::Mat(), NeuralNetworkParams_);
      gettimeofday(&endwtime , NULL);
      double trainingTime = static_cast<double>((endwtime.tv_usec - 
            startwtime.tv_usec) / 1.0e6 
          + endwtime.tv_sec - startwtime.tv_sec);
      std::cout << "The training finished after " << trainingTime << " seconds" << std::endl;

      classifierPtr_->save(classifierFile_.c_str());
      std::cout << "Finished training process" << std::endl;
    }
    if (file_utilities::exist(testFeaturesMatrixFile_.c_str()) == false ||
        testSetFeatureExtraction_)
    {
      std::cout << "Create necessary test matrix" << std::endl;
      std::string prefix = "test_";

      constructFeaturesMatrix(testDirectory_, testAnnotationsFile_,
          &testFeaturesMat, &testLabelsMat);

      loadNormalizationParametersAndNormalizeFeatures(&testFeaturesMat);

      testFeaturesMat.convertTo(testFeaturesMat, CV_32FC1);
      testLabelsMat.convertTo(testLabelsMat, CV_32FC1);

      file_utilities::saveFeaturesInFile(testFeaturesMat, testLabelsMat,
          prefix, testFeaturesMatrixFile_, testLabelsMatrixFile_, imageType_);
    }
    else
    {
      testFeaturesMat = file_utilities::loadFiles(
          testFeaturesMatrixFile_, "test_features_mat");
      testLabelsMat = file_utilities::loadFiles(
          testLabelsMatrixFile_, "test_labels_mat");
    }

    cv::Mat results = cv::Mat::zeros(numTestFiles, 1, CV_32FC1);

    classifierPtr_->predict(testFeaturesMat, results);

    file_utilities::saveToFile(resultsFile_, "results", results);
    evaluate(results, testLabelsMat);

  }

}  // namespace pandora_vision


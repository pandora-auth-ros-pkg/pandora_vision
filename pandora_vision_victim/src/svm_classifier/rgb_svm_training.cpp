/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors:
*   Marios Protopapas
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include "pandora_vision_victim/svm_classifier/rgb_svm_training.h"

namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  RgbSvmTraining::RgbSvmTraining(const std::string& ns, int _num_feat, const std::string& datasetPath) :
      SvmTraining(ns, _num_feat, datasetPath)
  {
    featureExtraction_ = new RgbFeatureExtraction();
    std::cout << "Created RGB SVM Training Instance" << std::endl;
  }

  /**
   * @brief Destructor
   */
  RgbSvmTraining::~RgbSvmTraining()
  {
  }

  /**
  @brief This method constructs the rgb training matrix
  to be used for the training
  @param file_name [std::string] : filename to save the extracted training matrix
  @param type [int]:  Value that indicates, if we train depth subsystem,
  or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
  @return void
  **/
  void RgbSvmTraining::constructFeaturesMatrix(const std::string& fileName,
                                               const boost::filesystem::path& directory,
                                               const std::string& prefix,
                                               cv::Mat* featuresMat,
                                               cv::Mat* labelsMat)
  {
    std::cout << "Constructing Features Matrix" << std::endl;
    cv::Mat image;

    int rowIndex = 0;
    for (boost::filesystem::recursive_directory_iterator iter(directory);
         iter != boost::filesystem::recursive_directory_iterator();
         iter++)
    {
      if (is_directory(iter->status()))
        continue;
      std::string imageAbsolutePath = iter->path().string();
      std::string imageName = iter->path().filename().string();
      image = cv::imread(imageAbsolutePath);
      if (!image.data)
      {
        std::cout << "Error reading file " << imageName << std::endl;
        continue;
      }

      // cv::Size size(640, 480);
      // cv::resize(image, image, size);

      featureExtraction_->extractFeatures(image);
      std::vector<double> featureVector;
      if (!featureVector.empty())
        featureVector.clear();
      featureVector = featureExtraction_->getFeatureVector();
      std::cout << "Feature vector of image " << imageName << " "
                << featureVector.size() << std::endl;
      /// display feature vector
      /*
       *for (int kk = 0; kk < featureVector.size(); kk++)
       *  std::cout << featureVector[kk] << " ";
       */

      for (int jj = 0; jj < featureVector.size(); jj++)
        featuresMat->at<double>(rowIndex, jj) = featureVector[jj];

      std::string checkIfPositive = "positive";
      if (boost::algorithm::contains(imageName, checkIfPositive))
        labelsMat->at<double>(rowIndex, 0) = 1.0;
      else
        labelsMat->at<double>(rowIndex, 0) = -1.0;

      rowIndex += 1;
    }

    /// Normalize the training matrix
    // zScoreNormalization(&featuresMat);

    /// Perform PCA to reduce the dimensions of the features
    // performPcaAnalysis(&trainingMat);

    std::stringstream features_mat_file_stream;
    features_mat_file_stream << package_path << "/data/" << fileName;
    std::string varName = prefix + "features_mat";
    saveToFile(features_mat_file_stream.str(), varName, *featuresMat);

    std::cout << features_mat_file_stream.str() << std::endl << " "
              << featuresMat->size() << std::endl;

    std::stringstream labels_mat_file_stream;
    labels_mat_file_stream << package_path << "/data/labels_" << fileName;
    varName = prefix + "labels_mat";
    saveToFile(labels_mat_file_stream.str(), varName, *labelsMat);

    std::cout << labels_mat_file_stream.str() << std::endl << " "
              << labelsMat->size() << std::endl;

    std::string features_matrix_csv_file;
    features_matrix_csv_file = prefix + "rgb_matrix.csv";
    features_mat_file_stream.str("");
    features_mat_file_stream << package_path << "/data/" << features_matrix_csv_file;
    saveToCSV(features_mat_file_stream.str(), *featuresMat, *labelsMat);
  }

  /**
  @brief Function that implements the training for the subsystems
  according to the given training sets. It applies svm and extracts
  a suitable model
  @param [int] type, Value that indicates, if we train depth subsystem,
  or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
  @return void
  **/
  void RgbSvmTraining::trainSubSystem()
  {
    std::string training_matrix_file_path;
    std::string test_matrix_file_path;
    std::string labels_matrix_file_path;
    std::string results_file_path;
    std::stringstream in_file_stream;
    std::stringstream in_test_file_stream;
    std::stringstream labels_mat_file_stream;
    std::stringstream test_labels_mat_file_stream;
    std::stringstream svm_file_stream;
    std::stringstream results_file_stream;
    cv::Mat results = cv::Mat::zeros(test_num_files, 1, CV_64FC1);
    float prediction;
    double A, B;

    training_matrix_file_path = "rgb_training_matrix.xml";
    test_matrix_file_path = "rgb_test_matrix.xml";
    results_file_path = "rgb_results.xml";

    in_file_stream << package_path << "/data/" << training_matrix_file_path;
    in_test_file_stream << package_path << "/data/" << test_matrix_file_path;
    labels_mat_file_stream << package_path << "/data/" << "labels_" +training_matrix_file_path;
    test_labels_mat_file_stream << package_path << "/data/" << "labels_" +test_matrix_file_path;
    svm_file_stream << package_path << "/data/" << "rgb_svm_classifier.xml";
    results_file_stream << package_path << "/data/" << results_file_path;


    std::stringstream trainingDatasetPath;
    trainingDatasetPath << path_to_samples << "/data/Training_Images";
    boost::filesystem::path trainingDirectory(trainingDatasetPath.str());
    int numTrainingFiles = countFilesInDirectory(trainingDirectory);

    std::stringstream testDatasetPath;
    testDatasetPath << path_to_samples << "/data/Test_Images";
    boost::filesystem::path testDirectory(testDatasetPath.str());
    int numTestFiles = countFilesInDirectory(testDirectory);

    cv::Mat trainingFeaturesMat = cv::Mat::zeros(numTrainingFiles, num_feat, CV_64FC1);
    cv::Mat trainingLabelsMat = cv::Mat::zeros(numTrainingFiles, 1, CV_64FC1);
    cv::Mat testFeaturesMat = cv::Mat::zeros(numTestFiles, num_feat, CV_64FC1);
    cv::Mat testLabelsMat = cv::Mat::zeros(numTestFiles, 1, CV_64FC1);

    //if(exist(in_file_stream.str().c_str()) == false)
    //{
      std::cout << "Create necessary training matrix" << std::endl;
      std::string prefix = "training_";
      constructFeaturesMatrix(training_matrix_file_path,
          trainingDirectory, prefix, &trainingFeaturesMat, &trainingLabelsMat);
    //}

    //if(exist(in_test_file_stream.str().c_str()) == false)
    //{
      std::cout << "Create necessary test matrix" << std::endl;
      //std::string
        prefix = "test_";
      constructFeaturesMatrix(test_matrix_file_path,
          testDirectory, prefix, &testFeaturesMat, &testLabelsMat);
    //}
    trainingFeaturesMat.convertTo(trainingFeaturesMat, CV_32FC1);
    trainingLabelsMat.convertTo(trainingLabelsMat, CV_32FC1);
    testFeaturesMat.convertTo(testFeaturesMat, CV_32FC1);
    testLabelsMat.convertTo(testLabelsMat, CV_32FC1);
    /*
     *loadFiles(in_file_stream.str(),
     *          labels_mat_file_stream.str(),
     *          in_test_file_stream.str(),
     *          test_labels_mat_file_stream.str());
     */

    //calcMinDistance();

    std::cout << "Starting training process for the rgb images" << std::endl;
    if (USE_OPENCV_GRID_SEARCH_AUTOTRAIN)
    {
      std::cout << "(SVM 'grid search' => may take some time!)" << std::endl;
      SVM.train_auto(trainingFeaturesMat, trainingLabelsMat, cv::Mat(), cv::Mat(),
                     params, 10, CvSVM::get_default_grid(CvSVM::C),
                     CvSVM::get_default_grid(CvSVM::GAMMA),
                     CvSVM::get_default_grid(CvSVM::P),
                     CvSVM::get_default_grid(CvSVM::NU),
                     CvSVM::get_default_grid(CvSVM::COEF),
                     CvSVM::get_default_grid(CvSVM::DEGREE),
                     true);

      params = SVM.get_params();
      std::cout << "Using optimal Parameters" << std::endl;
      std::cout << "degree=" << params.degree << std::endl;
      std::cout << "gamma=" << params.gamma << std::endl;
      std::cout << "coef0=" << params.coef0 << std::endl;
      std::cout << "C=" << params.C << std::endl;
      std::cout << "nu=" << params.nu << std::endl;
      std::cout << "p=" << params.p << std::endl;
    }
    else
    {
      SVM.train(trainingFeaturesMat, trainingLabelsMat, cv::Mat(), cv::Mat(), params);
    }
    SVM.save(svm_file_stream.str().c_str());
    std::cout << "Finished training process" << std::endl;

    ///uncomment to produce the platt probability
    //~ for (int ii = 0; ii < testFeaturesMat.rows; ii++)
    //~ {
      //~ prediction = SVM.predict(testFeaturesMat.row(ii), true);
      //~ results.at<double>(ii, 0)= prediction;
    //~ }
    //~ sigmoid_train(results, testLabelsMat, &A, &B);
    //~ std::cout << "A=" << A << std::endl;
    //~ std::cout << "B=" << B << std::endl;

    ///uncomment for ONE_CLASS SVM
    //~ for (int ii = 0; ii < results.rows; ii++)
     //~ for (int jj = 0; jj < results.cols; jj++)
      //~ if(results.at<float>(ii, jj) == 0)
          //~ results.at<float>(ii, jj) = -1;
    SVM.predict(testFeaturesMat, results);
    //std::cout << "results" << results.size() << std::endl << results <<std::endl <<std::endl;
    saveToFile(results_file_stream.str(), "results", results);
    evaluate(results, testLabelsMat);
  }
}// namespace pandora_vision



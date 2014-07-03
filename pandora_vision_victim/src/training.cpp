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

#include "pandora_vision_victim/training.h"

namespace pandora_vision
{
  /**
    @brief Constructor
  **/
  SvmTraining::SvmTraining(int type) : _nh()
  {
    
    params.svm_type = CvSVM::C_SVC;
    params.kernel_type = CvSVM::RBF; //CvSVM::RBF, CvSVM::LINEAR ...
    params.degree = 1; // for poly
    params.gamma = 5.0625000000000009e-01; // for poly/rbf/sigmoid
    params.coef0 = 0; // for poly/sigmoid
    //~ CvParamGrid CvParamGrid_C(pow(2.0,-5), pow(2.0,15), pow(2.0,2));
    //~ CvParamGrid CvParamGrid_gamma(pow(2.0,-20), pow(2.0,3), pow(2.0,2));
    //~ if (!CvParamGrid_C.check() || !CvParamGrid_gamma.check())
      //~ std::cout << "The grid is NOT VALID." << std::endl;
    params.C = 3.1250000000000000e+02; // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
    params.nu = 0.3; // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
    params.p = 0.0; // for CV_SVM_EPS_SVR
    params.class_weights = NULL; // for CV_SVM_C_SVC
    params.term_crit.type = CV_TERMCRIT_ITER+CV_TERMCRIT_EPS;
    params.term_crit.max_iter = 10000;
    params.term_crit.epsilon = 1e-6;
    
    package_path = ros::package::getPath("pandora_vision_victim");
    ROS_INFO("[victim_node] : Created Svm training instance");
    
    if(type == 1)
    {
      system = "rgb";
      num_feat=121;
    }
      
    else
    {
      system = "depth";
      num_feat=103;
    }
    char answer;
    training_mat_file = system + "_training_matrix.xml";
    test_mat_file = system + "_test_matrix.xml";
    labels_mat_file = "labels_" + training_mat_file;
    test_labels_mat_file = "labels_" + test_mat_file;
    std::stringstream in_file_stream, in_test_file_stream;
    std::stringstream labels_mat_file_stream, test_labels_mat_file_stream;
    std::stringstream svm_file_stream, results_file_stream;
    in_file_stream << package_path << "/data/" << training_mat_file;
    in_test_file_stream << package_path << "/data/" << test_mat_file;
    labels_mat_file_stream << package_path << "/data/" << labels_mat_file;
    test_labels_mat_file_stream << package_path << "/data/" <<  test_labels_mat_file;
    svm_file_stream << package_path << "/data/" << system + "_svm_classifier.xml";
    results_file_stream << package_path << "/data/" << system << "_results.xml";
    
    std::cout <<in_file_stream.str()<<std::endl;
    std::cout <<labels_mat_file_stream.str()<<std::endl;

    
    if(exist(in_file_stream.str().c_str()) && exist(labels_mat_file_stream.str().c_str()))
    {
      std::cout <<" Do you want to load the existing training files for the " << system << " system (y/n) " << std::endl;
      std::cin >> answer;
    }
      if(answer == 'y')
      {
        loadTrainFiles(in_file_stream.str(), labels_mat_file_stream.str());
        pos_files = 0;
        neg_files = 0;
        for(int ii = 0; ii < labels_mat.rows; ii++)
        {
          if(labels_mat.at<double>(ii,0) == 1.0)
            pos_files++;
          else
            neg_files++;
        }
      }
    
      else
      {      
        std::cout << "Add total number of class 1 (positive) images to be trained :" << std::endl;
        std::cin >> pos_files;
        std::cout << "Add total number of class -1 (negative) images to be trained :" << std::endl;
        std::cin >> neg_files;
        std::cout << "Add absolute path, where your samples are stored "<< std::endl;
        std::cin >> path_to_samples;
        constructTrainingMatrix(in_file_stream.str(), labels_mat_file_stream.str(), type);
      }
      std::cout << "Do you want to use svm_train_auto of opencv? (y/n) " << std::endl;
      std::cin >> answer;
      bool autotrain;
      if(answer == 'y')
        autotrain = true;
      else
        autotrain = false;
      trainSubSystem(svm_file_stream.str(),autotrain);
      
    if(exist(in_test_file_stream.str().c_str()) && exist(test_labels_mat_file_stream.str().c_str()))
    {
      std::cout <<" Do you want to load the existing test files for the " << system << " system (y/n) " << std::endl;
      std::cin >> answer;
    }
      if(answer == 'y')
      {
        loadTestFiles(in_test_file_stream.str(),test_labels_mat_file_stream.str());
        test_pos_files = 0;
        test_neg_files = 0;
        for(int ii = 0; ii < test_labels_mat.rows; ii++)
        {
          if(test_labels_mat.at<double>(ii, 0) == 1.0)
            test_pos_files++;
          else
            test_neg_files++;
        }
      }
    
      else
      {      
        std::cout << "Add total number of class 1 (positive) images to be tested :" << std::endl;
        std::cin >> test_pos_files;
        std::cout << "Add total number of class -1 (negconsative) images to be tested :" << std::endl;
        std::cin >> test_neg_files;
        if(path_to_samples.empty())
        {
          std::cout << "Add absolute path, where your samples are stored "<< std::endl;
          std::cin >> path_to_samples;
        }
        constructTestMatrix(in_test_file_stream.str(), test_labels_mat_file_stream.str(), type);
      }
      validateSubSystem(results_file_stream.str());
    
  }
  
  /**
    @brief Destructor
  */
  SvmTraining::~SvmTraining()
  {
    ROS_DEBUG("[victim_node] : Destroying Svm training instance");
  }
  
  /**
   * @brief This function extract features according to the
   * predifined features for the rgb image
   * @param inImage [cv::Mat] current rgb frame to be processed
   * @return void
  */ 
  void SvmTraining::extractRgbFeatures(const cv::Mat& inImage)
  {
    ///Extract color and statistics oriented features
    ///for rgb image
    _channelsStatisticsDetector.findChannelsStatisticsFeatures(inImage);
    
    ///Extract edge orientation features for rgb image
    _edgeOrientationDetector.findEdgeFeatures(inImage);
     
    ///Extract haralick features for rgb image 
    _haralickFeatureDetector.findHaralickFeatures(inImage);
    
    
    if(!_rgbFeatureVector.empty())
      _rgbFeatureVector.clear();
    
    setRgbFeatureVector();
    
  }
  
  /**
   * @brief This function creates feature vector according to the
   * predifined features for the rgb image
   * @return void
  */ 
  void SvmTraining::setRgbFeatureVector()
  {
    ///Append to rgbFeatureVector features according to color
    ///histogramms and other statistics
    std::vector<double> channelsStatictisFeatureVector = 
        _channelsStatisticsDetector.getRgbFeatures();
    for(int i = 0; i < channelsStatictisFeatureVector.size(); i++ )
          _rgbFeatureVector.push_back(channelsStatictisFeatureVector[i]);
    
    ///Append to rgbFeatureVector features according to edge orientation
    std::vector<double> edgeOrientationFeatureVector = 
        _edgeOrientationDetector.getFeatures();
    for(int i = 0; i < edgeOrientationFeatureVector.size(); i++ )
          _rgbFeatureVector.push_back(edgeOrientationFeatureVector[i]);   
    
    ///Append to rgbFeatureVector features according to haaralick features
    std::vector<double> haaralickFeatureVector = 
        _haralickFeatureDetector.getFeatures();
    for(int i = 0; i < haaralickFeatureVector.size(); i++ )
          _rgbFeatureVector.push_back(haaralickFeatureVector[i]);  
          
    ///Deallocate memory
    channelsStatictisFeatureVector.clear();
    _channelsStatisticsDetector.emptyCurrentFrameFeatureVector();
    
    edgeOrientationFeatureVector.clear();
    _edgeOrientationDetector.emptyCurrentFrameFeatureVector(); 
    
    haaralickFeatureVector.clear();
    _haralickFeatureDetector.emptyCurrentFrameFeatureVector();
  }
  
  std::vector<double> SvmTraining::getRgbFeatureVector()
  {
    return _rgbFeatureVector;
  }
  
  /**
   * @brief This function extract features according to the
   * predifined features for the depth image
   * @param inImage [cv::Mat] current depth frame to be processed
   * @return void
  */ 
  void SvmTraining::extractDepthFeatures(cv::Mat inImage)
  {
    ///Extract statistics oriented features for depth image
    _channelsStatisticsDetector.findDepthChannelsStatisticsFeatures(inImage);
    ///Extract edge orientation features for depth image
    _edgeOrientationDetector.findEdgeFeatures(inImage);
     
    ///Extract haralick features for depth image 
    _haralickFeatureDetector.findHaralickFeatures(inImage);
    
    if(!_depthFeatureVector.empty())
      _depthFeatureVector.clear();
    
    setDepthFeatureVector();
  }
  
  /**
   * @brief This function creates feature vector according to the
   * predifined features for the depth image
   * @return void
  */ 
  void SvmTraining::setDepthFeatureVector()
  {
    ///Append to rgbFeatureVector features according to color
    ///histogramms and other statistics
    std::vector<double> channelsStatictisFeatureVector = 
        _channelsStatisticsDetector.getDepthFeatures();
    for(int i = 0; i < channelsStatictisFeatureVector.size(); i++ )
          _depthFeatureVector.push_back(channelsStatictisFeatureVector[i]);
    
    ///Append to depthFeatureVector features according to edge orientation
    std::vector<double> edgeOrientationFeatureVector = 
        _edgeOrientationDetector.getFeatures();
    for(int i = 0; i < edgeOrientationFeatureVector.size(); i++ )
          _depthFeatureVector.push_back(edgeOrientationFeatureVector[i]);   
    
    ///Append to depthFeatureVector features according to haaralick features
    std::vector<double> haaralickFeatureVector = 
        _haralickFeatureDetector.getFeatures();
    for(int i = 0; i < haaralickFeatureVector.size(); i++ )
          _depthFeatureVector.push_back(haaralickFeatureVector[i]);  
          
    ///Deallocate memory
    channelsStatictisFeatureVector.clear();
    _channelsStatisticsDetector.emptyCurrentDepthFrameFeatureVector();
    
    edgeOrientationFeatureVector.clear();
    _edgeOrientationDetector.emptyCurrentFrameFeatureVector(); 
    
    haaralickFeatureVector.clear();
    _haralickFeatureDetector.emptyCurrentFrameFeatureVector();   
  }
  
  /**
   * @brief This function returns current feature vector according
   * to the features found in rgb image
   * @return [std::vector<double>] _rgbFeatureVector, feature vector 
   * for current rgb image
  */ 
  std::vector<double> SvmTraining::getDepthFeatureVector()
  {
    return _depthFeatureVector;
  }
  
  /**
   * @brief This method constructs the rgb training matrix 
   * to be used for the training
   * @param [std::string] training_mat_file path 
   * to save the extracted training matrix
   * @param [std::string] labels_mat_file path 
   * to save the extracted training matrix
   * @param [int] type, Value that indicates, if we train depth subsystem,
   * or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
   * @return void
  */
  void SvmTraining::constructTrainingMatrix(std::string training_mat_file,
                                            std::string labels_mat_file,
                                            int type)
  {
    cv::Mat img;
    std::vector<double> _featureVector;

    training_mat = cv::Mat::zeros(pos_files + neg_files, num_feat, CV_64FC1);
    labels_mat = cv::Mat::zeros(pos_files +neg_files, 1, CV_64FC1);

    std::stringstream img_name;
    for (int ii = 0; ii < pos_files+neg_files; ii++)
    {
      if(type == 1)
      {
      if(ii < pos_files)
        img_name << path_to_samples << "/data/" 
            << "Positive_Images/positive" << ii + 1 << ".jpg";
      else
        img_name << path_to_samples << "/data/" << "Negative_Images/negative" 
        << ii + 1 - pos_files << ".jpg";
      }
      else
      
      {
      if(ii < pos_files)
        img_name << path_to_samples << "/data/" 
            << "Depth_Positive_Images/positive" << ii + 1 << ".jpg";
      else
        img_name << path_to_samples << "/data/" 
        << "Depth_Negative_Images/negative" 
        << ii + 1 - pos_files << ".jpg";
      }
        
      std::cout << img_name.str() << std::endl;
        
      img = cv::imread(img_name.str());
      if (!img.data) 
      {
       std::cout << "Error reading file " << img_name.str() << std::endl;
       return;
      }
      
      cv::Size size(640, 480);
      cv::resize(img, img, size);
      
      if(type == 1)
      {
        extractRgbFeatures(img);
        _featureVector = getRgbFeatureVector();
      }
      
      else
      {
        extractDepthFeatures(img);
        _featureVector = getDepthFeatureVector();
      }
      
      std::cout << "Feature vector of image " << img_name.str() << " "
               << _featureVector.size() << std::endl;
          
       /// display feature vector

        for(int kk = 0; kk < _featureVector.size(); kk++)
       {
          std::cout <<  _featureVector[kk] << "  ";   

       }
              
      for (int jj = 0; jj < _featureVector.size(); jj++)
        training_mat.at<double>(ii, jj) = _featureVector[jj];

      if(ii < pos_files)
        labels_mat.at<double>(ii, 0) = 1.0;
      
      else
        labels_mat.at<double>(ii, 0) = -1.0;

        
      ///Empty stringstream for next image
      img_name.str("");   
      
    }
    
    /// Perform pca to reduce the dimensions of the features

    int nEigens = 23;

    cv::PCA pca(training_mat, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Mat mean = pca.mean.clone();
    cv::Mat eigenvalues = pca.eigenvalues.clone();
    cv::Mat eigenvectors = pca.eigenvectors.clone();
    std::cout << "EigenValues" << eigenvalues << std::endl;
    std::cout << "EigenVectors " << eigenvectors.size() << std::endl;
    //pca.project(training_mat, training_mat);
    
    /// Normalize the training matrix
    
    for (int kk = 0; kk < training_mat.rows; kk++)
    cv::normalize(training_mat.row(kk), training_mat.row(kk), -1, 1, cv::NORM_MINMAX, -1);
        
    saveToFile(training_mat_file, "training_mat", training_mat);
    
    std::cout << training_mat_file << std::endl << 
        " " << training_mat.size() << std::endl;
        
    saveToFile(labels_mat_file, "labels_mat", labels_mat);
    
    std::cout << labels_mat_file << std::endl << 
        " " << labels_mat.size() << std::endl;   
  }
  
  /**
   * @brief This method constructs the rgb test matrix 
   * to be used for validation of the training
   * @param [std::string] test_mat_file path 
   * to save the extracted test matrix
   * @param [std::string] test_labels_mat_file path 
   * to save the extracted test matrix
   * @param [int] type, Value that indicates, if we train depth subsystem,
   * or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
   * @return void
  */
  void SvmTraining::constructTestMatrix(std::string test_mat_file,
                                        std::string test_labels_mat_file,
                                        int type)
  {
    cv::Mat img;
    std::vector<double> _featureVector;

    test_mat = cv::Mat::zeros(test_pos_files + test_neg_files, num_feat, CV_64FC1);
    test_labels_mat = cv::Mat::zeros(test_pos_files + test_neg_files, 1, CV_64FC1);

    std::stringstream img_name;
    for (int ii = 0; ii < test_pos_files + test_neg_files; ii++)
    {
      if(type == 1)
      {
      if(ii < test_pos_files)
        img_name << path_to_samples << "/data/" 
            << "Test_Positive_Images/positive" << ii + 1 << ".jpg";
      else
        img_name << path_to_samples << "/data/" 
            << "Test_Negative_Images/negative" << ii + 1 - test_pos_files << ".jpg";
      }
      else
      {
        if(ii < test_pos_files)
        img_name << path_to_samples << "/data/" 
            << "Test_Depth_Positive_Images/positive" << ii + 1 << ".jpg";
      else
        img_name << path_to_samples << "/data/" 
            << "Test_Depth_Negative_Images/negative" << ii + 1 - test_pos_files << ".jpg";
      }
        
      std::cout << img_name.str() << std::endl;
        
      img = cv::imread(img_name.str());

      if (!img.data) 
      {
       std::cout << "Error reading file " << img_name.str() << std::endl;
       return;
      }
      
      cv::Size size(640, 480);
      cv::resize(img, img, size);
      
      if(type == 1)
      {
        extractRgbFeatures(img);
        _featureVector = getRgbFeatureVector();
      }
      
      else
      {
        extractDepthFeatures(img);
        _featureVector = getDepthFeatureVector();
      }
      
      std::cout << "Feature vector of image " << img_name.str() << " "
          << _featureVector.size() << std::endl;
          
      /// display feature vector

        for(int kk = 0; kk < _featureVector.size(); kk++)
       {
          std::cout <<  _featureVector[kk] << "  ";   

       }

      for (int jj = 0; jj < _featureVector.size(); jj++)
        test_mat.at<double>(ii, jj)=_featureVector[jj];
                
      if(ii < test_pos_files)
        test_labels_mat.at<double>(ii, 0) = 1.0;
      
      else
        test_labels_mat.at<double>(ii, 0) = -1.0;

      /// Empty stringstream for next image
      img_name.str("");   
      
    }
    
    /// Perform pca to reduce the dimensions of the features
    
    int nEigens = 23;
    cv::PCA pca(test_mat, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Mat mean = pca.mean.clone();
    cv::Mat eigenvalues = pca.eigenvalues.clone();
    cv::Mat eigenvectors = pca.eigenvectors.clone();
    std::cout << "EigenValues" << eigenvalues << std::endl;
    std::cout << "EigenVectors " << eigenvectors.size() << std::endl;
    //pca.project(test_mat, test_mat);
    
    /// Normalize the test matrix
    for (int kk = 0; kk < test_mat.rows; kk++)
    cv::normalize(test_mat.row(kk), test_mat.row(kk), -1, 1, cv::NORM_MINMAX, -1);
    
    saveToFile(test_mat_file, "test_mat", test_mat);
    
    std::cout << test_mat_file << std::endl << 
        " " << test_mat.size() << std::endl;
        

    saveToFile(test_labels_mat_file, "test_labels_mat", test_labels_mat);

    std::cout << test_labels_mat_file << std::endl << 
        " " << test_labels_mat.size() << std::endl;   
       
  }
  
  /**
   *@brief Function that implements the training for the subsystems
   * according to the given training sets. It applies svm and extracts
   * a suitable model
   * @param [std::string] svm_file 
   * path to save the results of the prediction
   * @return void
  */ 
  void SvmTraining::trainSubSystem(std::string svm_file, bool autotrain)
  {
        if (autotrain)
        {
          std::cout << "(SVM 'grid search' => may take some time!)" << std::endl;
          SVM.train_auto(training_mat, labels_mat, cv::Mat(), cv::Mat(), 
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
          SVM.train(training_mat, labels_mat, cv::Mat(), cv::Mat(), params);
        SVM.save(svm_file.c_str());
        std::cout << "Finished training process" << std::endl;
  }


  /**
   *@brief Function that implements the validation and the evaluation for the subsystems
   * according to the given test sets. It applies svm prediction and extracts
   * a suitable model
   * @param [std::string] results_file
   * path to save the results of the prediction
   * @return void
  */ 
  void SvmTraining::validateSubSystem(std::string results_file)
  {
    cv::Mat results = cv::Mat::zeros(test_pos_files + test_neg_files, 1, CV_64FC1);
    float prediction;
    double A, B;
    
    ///uncomment to produce the platt probability
    //~ for (int ii = 0; ii < test_mat.rows; ii++)
    //~ {
      //~ prediction = SVM.predict(test_mat.row(ii), true);
      //~ results.at<double>(ii, 0)= prediction;
    //~ }
    //~ sigmoid_train(results, test_labels_mat, &A, &B);
    //~ std::cout << "A=" << A << std::endl;
    //~ std::cout << "B=" << B << std::endl;
        
    ///uncomment for ONE_CLASS SVM
    //~ for (int ii = 0; ii < results.rows; ii++)
     //~ for (int jj = 0; jj < results.cols; jj++)
      //~ if(results.at<float>(ii, jj) == 0)
          //~ results.at<float>(ii, jj) = -1;
    SVM.predict(test_mat, results);
    //std::cout << "results" << results.size() << std::endl << results <<std::endl <<std::endl;
    saveToFile(results_file, "results", results);
    evaluate(results, test_labels_mat);
  }
  
  /**
   *@brief Function that saves a variable to a file
   * @param [std::string] file_name, name of the file to be created
   * @param [std::string] var_name, name of the variable to be saved to the file
   * @param [cv::Mat] var, variable to be saved to the file
   * @return void
  */ 
  void SvmTraining::saveToFile(std::string file_name, std::string var_name, cv::Mat var)
  {
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);

    if (!fs.isOpened())
      fs.open(file_name, cv::FileStorage::WRITE);
    
    fs << var_name << var;
    fs.release();
  }

   /**
   *@brief Function that loads the necessary files for the training
   * @param [std::string] training_mat_file, name of the file
   * that contains the training data
   * @param [std::string] labels_mat_file, name of the file
   *  that contains the labels of each class
   * of the training data
   * @return void
  */ 
  void SvmTraining::loadTrainFiles(std::string training_mat_file, 
                                   std::string labels_mat_file)
  {
    cv::FileStorage fs1, fs2;
    cv::Mat temp1, temp2;
    fs1.open(training_mat_file, cv::FileStorage::READ);
    fs1["training_mat"] >> temp1;
    fs2.open(labels_mat_file, cv::FileStorage::READ);
    fs2["labels_mat"] >> temp2;
    fs1.release();
    fs2.release();
    if (temp1.data  && temp2.data) 
      {
       std::cout << "files uploaded successfully" << std::endl; 
       std::cout << training_mat_file << temp1.size() << std::endl;;
       std::cout << labels_mat_file << temp2.size() << std::endl;
      }
    
    training_mat = temp1.clone();
    labels_mat = temp2.clone();
    training_mat.convertTo(training_mat, CV_32FC1);
    labels_mat.convertTo(labels_mat, CV_32FC1);
  }
  
   /**
   *@brief Function that loads the necessary files for the training
   * @param [std::string] test_mat_file, name of the file that contains the test data
   * @param [std::string] test_labels_mat_file, name of the file that contains the labels of each class
   * of the training data
   * @return void
  */ 
  void SvmTraining::loadTestFiles(std::string test_mat_file, 
                                   std::string test_labels_mat_file)
  {
    cv::FileStorage fs3, fs4;
    cv::Mat temp3, temp4;
    fs3.open(test_mat_file, cv::FileStorage::READ);
    fs3["test_mat"] >> temp3;
    fs4.open(test_labels_mat_file, cv::FileStorage::READ);
    fs4["test_labels_mat"] >> temp4;
    fs3.release();
    fs4.release();
    if (temp3.data  && temp4.data) 
      {
       std::cout << "files uploaded successfully" << std::endl; 
       std::cout << test_mat_file << temp3.size() << std::endl;;
       std::cout << test_labels_mat_file << temp4.size() << std::endl;
      }
      
    test_mat = temp3.clone();
    test_labels_mat = temp4.clone();
    test_mat.convertTo(test_mat, CV_32FC1);
    test_labels_mat.convertTo(test_labels_mat, CV_32FC1);
  }

  /**
   *@brief Function that checks if a file exists
   *@param [const char*] name, Name of file to check if exists 
   *@return true if the file was found, and false if not 
  */     
  bool SvmTraining::exist(const char *name)
  {
    std::ifstream file(name);
    if(!file)    //if the file was not found, then file is 0, i.e. !file=1 or true
        return false;    //the file was not found
    else         //if the file was found, then file is non-0
        return true;     //the file was found
  }
  
  /**
   *@brief Function that evaluates the training
   *@param [cv::Mat&] predicted the predicted results
   *@param [cv::Mat&] the actual results 
   *@return void
  */ 
  void SvmTraining::evaluate(const cv::Mat& predicted, const cv::Mat& actual) 
  {
    assert(predicted.rows == actual.rows);
    int tp = 0;
    int fp = 0;
    int tn = 0;
    int fn = 0;
    for(int ii = 0; ii < actual.rows; ii++) {
      float p = predicted.at<float>(ii, 0);
      float a = actual.at<float>(ii, 0);
      if((p >= 0.0 && a >= 0.0)) 
      {
        tp++;
      } 
      else if(p <= 0.0  && a <= 0.0)
      { 
        tn++;
      }
      else if(p >= 0.0  && a <= 0.0)
      { 
        fp++;
      }
      else if(p <= 0.0  && a >= 0.0)
      { 
        fn++;
      }
    }
    accuracy = ((tp +tn) * 1.0) / (tp + tn + fp + fn);
    precision = (tp * 1.0) / (tp + fp);
    recall = (tp * 1.0) / (tp + fn);
    fmeasure= (tp *2.0) / (2.0 * tp + fn + fp); 
    
    std::cout << "True Positives= " << tp << std::endl;
    std::cout << "True Negatives= " << tn << std::endl;
    std::cout << "False Positives= " << fp << std::endl;
    std::cout << "False Negatives= " << fn << std::endl;

    std::cout << "svm accuracy= " << accuracy << std::endl; 
    std::cout << "svm precision= " << precision << std::endl;
    std::cout << "svm recall= " << recall << std::endl;
    std::cout << "svm fmeasure= " << fmeasure << std::endl;
    
  }

  /**
   *@brief Function that computes the min distance between the features
   * of the 2 classes
   *@return void
  */ 
  void SvmTraining::calcMinDistance() 
  {
    double distance;
    double sum;
    double min = 100000;
    for (int ii = 0; ii < pos_files; ii++)
    {
      sum = 0;
      for(int jj = 0; jj < training_mat.cols; jj++)
          sum = sum + pow( training_mat.at<double>(ii, jj) - training_mat.at<double>(ii + pos_files), 2);
      distance = sqrt(sum);
      std::cout << "distance=" << distance <<" "<< ii << std::endl;

      if(distance < min)
      min = distance;
    }
    std::cout << "min distance=" << min << std::endl;
  }
  
    // Platt's binary SVM Probablistic Output: an improvement from Lin et al.
  /**
   *@brief Function that computes the vectors A,B necessary for the computation
   * of the probablistic output of the svm bases on platt's binary svm 
   * probablistic Output
   * @param [cv::Mat] dec_values, the distance from the hyperplane of the 
   * predicted results of the given test dataset
   * @param [cv::Mat] labels, the true labels of the dataset
   * @param [double&] A, the vector A to be computed
   * @param [double&] B, the vector B to be computed
   * @return void
  */  
  void SvmTraining::sigmoid_train(cv::Mat dec_values, cv::Mat labels, 
                                  double* A, double* B)
  {
    double prior1 = 0, prior0 = 0;

    for (int ii = 0; ii < dec_values.rows; ii++)
      if (labels.at<double>(ii, 0) > 0) 
        prior1 += 1;
      else 
        prior0+= 1;
    
    int max_iter = 100;// Maximal number of iterations
    double min_step = 1e-10;// Minimal step taken in line search
    double sigma = 1e-12;// For numerically strict PD of Hessian
    double eps = 1e-5;
    double hiTarget = (prior1 + 1.0) / (prior1 + 2.0);
    double loTarget = 1 / (prior0 + 2.0);
    double* t = new double[labels.rows];
    double fApB, p, q, h11, h22, h21, g1, g2, det, dA, dB, gd, stepsize;
    double newA, newB, newf, d1, d2, Avector, Bvector;
    int iter;
    
    // Initial Point and Initial Fun Value
    Avector = 0.0; 
    Bvector = log((prior0 + 1.0) / (prior1 + 1.0));
    double fval = 0.0;

    for (int ii = 0; ii <labels.rows; ii++)
    {
      if (labels.at<double>(ii, 0) > 0) 
        t[ii] = hiTarget;
      else 
        t[ii]=loTarget;
      fApB = dec_values.at<double>(ii, 0) * Avector + Bvector;
      if (fApB >= 0)
        fval += t[ii] * fApB + log(1 + exp(-fApB));
      else
        fval += (t[ii] - 1) * fApB + log(1 + exp(fApB));
    }
    for (iter = 0; iter < max_iter; iter++)
    {
      // Update Gradient and Hessian (use H' = H + sigma I)
      h11 = sigma; // numerically ensures strict PD
      h22 = sigma;
      h21 = 0.0;
      g1 = 0.0;
      g2 = 0.0;
      for (int ii = 0; ii < dec_values.rows; ii++)
      {
        fApB = dec_values.at<double>(ii, 0) * Avector + Bvector;
        if (fApB >= 0)
        {
          p = exp(-fApB) / (1.0 + exp(-fApB));
          q = 1.0 / (1.0 + exp(-fApB));
        }
        else
        {
          p = 1.0 / (1.0 + exp(fApB));
          q = exp(fApB) / (1.0 + exp(fApB));
        }
        d2 = p * q;
        h11 += dec_values.at<double>(ii, 0) * dec_values.at<double>(ii, 0) * d2;
        h22 += d2;
        h21 += dec_values.at<double>(ii, 0) * d2;
        d1 = t[ii] - p;
        g1 += dec_values.at<double>(ii, 0) * d1;
        g2 += d1;
      }

      // Stopping Criteria
      if (fabs(g1) < eps && fabs(g2) < eps)
        break;

      // Finding Newton direction: -inv(H') * g
      det= h11 * h22 - h21 * h21;
      dA =- (h22 * g1 - h21 * g2) / det;
      dB=-(-h21 * g1 + h11 * g2) / det;
      gd = g1 * dA + g2 * dB;


      stepsize = 1;// Line Search
      while (stepsize >= min_step)
      {
        newA = Avector + stepsize * dA;
        newB = Bvector + stepsize * dB;

        // New function value
        newf = 0.0;
        for (int ii = 0; ii < labels.rows; ii++)
        {
          fApB = dec_values.at<double>(ii, 0) * newA + newB;
          if (fApB >= 0)
            newf += t[ii] * fApB + log(1 + exp(-fApB));
          else
            newf += (t[ii] - 1) * fApB +log(1 + exp(fApB));
        }
        // Check sufficient decrease
        if (newf < fval + 0.0001 * stepsize * gd)
        {
          Avector = newA;
          Bvector = newB;
          fval = newf;
          break;
        }
        else
          stepsize = stepsize / 2.0;
      }

      if (stepsize < min_step)
      {
        std::cout << "Line search fails in two-class probability estimates" << std::endl;
        break;
      }
    }

    if (iter >= max_iter)
      std::cout << "Reaching maximal iterations in two-class probability estimates" << std::endl;
    free(t);
    *A = Avector;
    *B = Bvector;
  }
  
}// namespace pandora_vision

int main(int argc, char** argv)
{
  int type=0;
  ros::init(argc, argv, "victim_train_node");
  std::cout << "Choose the type of the training 1 rgb, 2 depth" <<std::endl;
  std::cin >> type;
  pandora_vision::SvmTraining victim_trainer(type);
  ros::spin();
  return 0;
}

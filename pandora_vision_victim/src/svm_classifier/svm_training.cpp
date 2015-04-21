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
*   Marios Protopapas
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include "pandora_vision_victim/svm_classifier/svm_training.h"

namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  SvmTraining::SvmTraining(const std::string& ns,
      int _num_feat,
      const std::string& datasetPath) :
      _nh(ns), vparams(ns)
  {
    path_to_samples = datasetPath;
    num_feat = _num_feat;
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

    featureExtraction_ = new FeatureExtraction();
  }

  /**
   * @brief Destructor
   */
  SvmTraining::~SvmTraining()
  {
    ROS_DEBUG("[victim_node] : Destroying Svm training instance");
  }

  /**
   * @brief
   * @param newMax [double]
   * @param newMin [double]
   * @param image [cv::Mat*]
   * @param minVec [std::vector<double>*]
   * @param maxVec [std::vector<double>*]
   * @return void
   */
  void SvmTraining::minMaxNormalization(double newMax,
                                        double newMin,
                                        cv::Mat* image,
                                        std::vector<double>* minVec,
                                        std::vector<double>* maxVec)
  {
    double minVal, maxVal;
    if (!minVec->empty())
      minVec->clear();
    if (!maxVec->empty())
      maxVec->clear();

    for (int ii = 0; ii < image->cols; ii++)
    {
      cv::minMaxLoc(image->col(ii), &minVal, &maxVal);
      subtract(image->col(ii), minVal, image->col(ii));
      divide(image->col(ii), maxVal - minVal, image->col(ii));
      multiply(image->col(ii), newMax - newMin, image->col(ii));
      add(image->col(ii), newMin, image->col(ii));

      minVec->push_back(minVal);
      minVec->push_back(maxVal);
    }
  }

  /**
   * @brief
   * @param image [cv::Mat*]
   * @param meanVec [std::vector<double>*]
   * @param stdDevVec [std::vector<double>*]
   * @return void
   */
  void SvmTraining::zScoreNormalization(cv::Mat* image,
                                        std::vector<double>* meanVec,
                                        std::vector<double>* stdDevVec)
  {
    cv::Scalar mu, sigma;
    if (!meanVec->empty())
      meanVec->clear();
    if (!stdDevVec->empty())
      stdDevVec->clear();

    for (int ii = 0; ii < image->cols; ii++)
    {
      meanStdDev(image->col(ii), mu, sigma);
      subtract(image->col(ii), mu.val[0], image->col(ii));
      if (sigma.val[0] != 0.0)
        divide(image->col(ii), sigma.val[0], image->col(ii));

      meanVec->push_back(mu.val[0]);
      stdDevVec->push_back(sigma.val[0]);
    }
  }

  /**
   * @brief Function that evaluates the training
   * @param predicted [const cv::Mat&] the predicted results
   * @param actual [const cv::Mat&] the actual results
   * @return void
   */
  void SvmTraining::evaluate(const cv::Mat& predicted, const cv::Mat& actual)
  {
    assert(predicted.rows == actual.rows);
    int truePositives = 0;
    int falsePositives = 0;
    int trueNegatives = 0;
    int falseNegatives = 0;
    for (int ii = 0; ii < actual.rows; ii++)
    {
      float p = predicted.at<float>(ii, 0);
      float a = actual.at<float>(ii, 0);

      if (p >= 0.0 && a >= 0.0)
        truePositives++;
      else if (p <= 0.0 && a <= 0.0)
        trueNegatives++;
      else if (p >= 0.0 && a <= 0.0)
        falsePositives++;
      else if (p <= 0.0 && a >= 0.0)
        falseNegatives++;
    }
    accuracy_ = static_cast<float>(truePositives + trueNegatives) /
              (truePositives + trueNegatives + falsePositives + falseNegatives);
    precision_ = static_cast<float>(truePositives) /
              (truePositives + falsePositives);
    recall_ = static_cast<float>(truePositives) /
              (truePositives + falseNegatives);
    fmeasure_ = (2.0 * truePositives) /
              (2.0 * truePositives + falseNegatives + falsePositives);

    std::cout << "True Positives = " << truePositives << std::endl;
    std::cout << "True Negatives = " << trueNegatives << std::endl;
    std::cout << "False Positives = " << falsePositives << std::endl;
    std::cout << "False Negatives = " << falseNegatives << std::endl;

    std::cout << "SVM Accuracy = " << accuracy_ << std::endl;
    std::cout << "SVM Precision = " << precision_ << std::endl;
    std::cout << "SVM Recall = " << recall_ << std::endl;
    std::cout << "SVM F-Measure = " << fmeasure_ << std::endl;
  }

  // Platt's binary SVM Probablistic Output: an improvement from Lin et al.
  /**
   * @brief Function that computes the vectors A,B necessary for the
   * computation of the probablistic output of the SVM bases on Platt's binary
   * SVM probablistic Output
   * @param dec_values [cv::Mat] the distance from the hyperplane of the
   * predicted results of the given test dataset
   * @param labels [cv::Mat] the true labels of the dataset
   * @param A [double*] the vector A to be computed
   * @param B [double*] the vector B to be computed
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

  void SvmTraining::extractFeatures(const cv::Mat& inImage)
  {
    featureExtraction_->extractFeatures(inImage);
  }

  /**
  @brief Function that saves a variable to a file
  @param file_name [std::string] : name of the file to be created
  @param var_name [std::string] : name of the variable to be saved to the file
  @param var [cv::Mat] : variable to be saved to the file
  @return void
  **/
  void SvmTraining::saveToFile(std::string file_name, std::string var_name, cv::Mat var)
  {
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);

    if (!fs.isOpened())
      fs.open(file_name, cv::FileStorage::WRITE);

    fs << var_name << var;
    fs.release();
  }
  
  /**
  @brief Function that saves a variable to a file
  @param file_name [std::string] : name of the file to be created
  @param training_mat [cv::Mat] : name of the mat of features to be saved to the file
  @param labels_mat [cv::Mat] : name of the mat of labels to be saved to the file
  @return void
  **/
  void SvmTraining::saveToCSV(const std::string& file_name, const cv::Mat& training_mat, const cv::Mat& labels_mat)
  {
    std::ofstream outFile;
    outFile.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);
    if (!outFile)
    {
      ROS_ERROR("cannot load file");
      return;
    }
    else
    {  
      if(outFile.is_open())
      {
        for(int kk = 0; kk < training_mat.cols; kk++)
        {
          outFile << "a" << kk;
          if(kk < training_mat.cols - 1)
             outFile <<",";
          else
            outFile <<std::endl;
        }
        for (int ii = 0; ii < training_mat.rows; ii++)
        {
          outFile << labels_mat.at<double>(ii) << ",";
          for(int jj =0; jj < training_mat.cols; jj++)
          {
            outFile << training_mat.at<double>(ii, jj);
            if(jj < training_mat.cols-1)
              outFile << ",";
            else
              outFile << std::endl;
          }
        }
      }
      outFile.close();
    }
  }

  /**
  @brief Function that loads the necessary files for the training
  @param [std::string] training_mat_file, name of the file that contains the training data
  @param [std::string] labels_mat_file, name of the file that contains the labels of each class
  of the training data
  @return void
  **/
  void SvmTraining::loadFiles(std::string training_mat_file,
                              std::string labels_mat_file,
                              std::string test_mat_file,
                              std::string test_labels_mat_file)
  {
    cv::FileStorage fs1, fs2, fs3, fs4;
    cv::Mat temp1, temp2, temp3, temp4;
    fs1.open(training_mat_file, cv::FileStorage::READ);
    fs1["training_mat"] >> temp1;
    fs2.open(labels_mat_file, cv::FileStorage::READ);
    fs2["labels_mat"] >> temp2;
    fs3.open(test_mat_file, cv::FileStorage::READ);
    fs3["test_mat"] >> temp3;
    fs4.open(test_labels_mat_file, cv::FileStorage::READ);
    fs4["test_labels_mat"] >> temp4;
    fs1.release();
    fs2.release();
    fs3.release();
    fs4.release();
    if (temp1.data && temp2.data && temp3.data && temp4.data)
      {
       std::cout << "files uploaded successfully" << std::endl;
       std::cout << training_mat_file << temp1.size() << std::endl;;
       std::cout << labels_mat_file << temp2.size() << std::endl;
       std::cout << test_mat_file << temp3.size() << std::endl;;
       std::cout << test_labels_mat_file << temp4.size() << std::endl;
      }

    trainingFeaturesMat_ = temp1.clone();
    trainingLabelsMat_ = temp2.clone();
    testFeaturesMat_ = temp3.clone();
    testLabelsMat_ = temp4.clone();
    trainingFeaturesMat_.convertTo(trainingFeaturesMat_, CV_32FC1);
    trainingLabelsMat_.convertTo(trainingLabelsMat_, CV_32FC1);
    testFeaturesMat_.convertTo(testFeaturesMat_, CV_32FC1);
    testLabelsMat_.convertTo(testLabelsMat_, CV_32FC1);
  }

  /**
  @brief Function that checks if a file exists
  @param [const char*] name, Name of file to check if exists
  @return true if the file was found, and false if not
  **/
  bool SvmTraining::exist(const char *name)
  {
    std::ifstream file(name);
    if(!file) //if the file was not found, then file is 0, i.e. !file=1 or true
        return false; //the file was not found
    else //if the file was found, then file is non-0
        return true; //the file was found
  }

  int SvmTraining::countFilesInDirectory(
      const boost::filesystem::path& directory)
  {
    int numFiles = std::count_if(
        boost::filesystem::recursive_directory_iterator(directory),
        boost::filesystem::recursive_directory_iterator(),
        boost::lambda::bind(static_cast<bool(*)(const boost::filesystem::path&)>
          (boost::filesystem::is_regular_file),
            boost::lambda::bind(&boost::filesystem::directory_entry::path,
              boost::lambda::_1)));
    return numFiles;
  }
}// namespace pandora_vision


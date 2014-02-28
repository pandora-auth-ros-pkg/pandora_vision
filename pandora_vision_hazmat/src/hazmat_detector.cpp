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
* Author:  Tsakalis Vasilis
* 		   Despoina Paschalidou
*********************************************************************/

#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include "ros/ros.h"
#include "pandora_vision_hazmat/hazmat_detector.h"
/**
 * Constructor
 */
HazmatEpsilonDetector::HazmatEpsilonDetector()
:rows(480),
 cols(640)
{
	ros::NodeHandle nh;
	if (!nh.getParam("/pandora_vision/packagepath", param_path))
	{
		ROS_ERROR("METAFILE PARAMETER NOT FOUND");
		param_path = ros::package::getPath("pandora_vision_hazmat");
	}

	setParameters();
	initDetector();
	frameNum=0;
}

/**
 * Sets hazmat node related parameters
 * @param clrVariance
 * @param votingThr
 * @param minAreaThr
 * @param maxAreaThr
 * @param sideLgth
 * @param featThr
 * @param MOThr
 */
void HazmatEpsilonDetector::setHazmatParameters(int clrVariance,float votingThr,float minAreaThr,float maxAreaThr,int sideLgth,int featThr,float MOThr){
	colorVariance = clrVariance;
	votingThreshold = votingThr;
	minAreaThreshold = minAreaThr;
	maxAreaThreshold = maxAreaThr;
	featureThreshold = featThr;
	MOThreshold = MOThr;
	sideLength = sideLgth;

    calcMinMax();
}

/**
 * Destructor
 */
HazmatEpsilonDetector::~HazmatEpsilonDetector(){
	delete[] nFeats;
	delete[] minUV;
	delete[] maxUV;
}

/**
 * Calculates min and max UV values from YUV colorspace of hazmats
 */
void HazmatEpsilonDetector::calcMinMax()
{
	minUV=new float* [nPatterns];
	maxUV=new float* [nPatterns];
	cv::Mat image;
	for (int i=0;i<nPatterns;i++)
	{
		minUV[i]=new float [2];
		maxUV[i]=new float [2];
	}
	for (int i=0;i<nPatterns;i++)
	{
        char temp_name[50];
		sprintf(temp_name,"/patterns/enter%d.png",i+1);
		std::string name=param_path+temp_name;
		
		image=cv::imread(name.c_str(),1);
		cvtColor(image, image, CV_BGR2YCrCb);
		int height,width,step,channels;
		//uchar *data;
		height    = image.size().height;
		width     = image.size().width;
		//step      = image.widthStep;
		step      = image.step;
		channels  = image.channels();
		//data      = (uchar *)image.data;
		for (int j=0;j<sideLength;j++)
		{
			for (int k=0;k<sideLength;k++)
			{
				if( (j==0) && (k==0) ){
					minUV[i][0] = ((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step + ((width/2)-(sideLength/2)+k)*channels+1];
					minUV[i][1] = ((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step + ((width/2)-(sideLength/2)+k)*channels+2];
					maxUV[i][0] = ((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step + ((width/2)-(sideLength/2)+k)*channels+1];
					maxUV[i][1] = ((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step + ((width/2)-(sideLength/2)+k)*channels+2];
				}
				else{
					if (minUV[i][0]>((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step + ((width/2)-(sideLength/2)+k)*channels+1]){
						minUV[i][0]=((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step + ((width/2)-(sideLength/2)+k)*channels+1];
					}
					if (minUV[i][1]>((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step+((width/2)-(sideLength/2)+k)*channels+2]){
						minUV[i][1]=((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step+((width/2)-(sideLength/2)+k)*channels+2];
					}
					if(maxUV[i][0]<((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step+((width/2)-(sideLength/2)+k)*channels+1]){
						maxUV[i][0]=((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step+((width/2)-(sideLength/2)+k)*channels+1];
					}
					if(maxUV[i][1]<((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step+((width/2)-(sideLength/2)+k)*channels+2]){
						maxUV[i][1]=((uchar *)image.data)[((height/2)-(sideLength/2)+j)*step+((width/2)-(sideLength/2)+k)*channels+2];
					}
				}
			}
		}
	}
}

/**
 * Calculates histograms for given hazmat signs
 */
void HazmatEpsilonDetector::calcHistograms()
{
    cv::Mat* patterns=new cv::Mat[nPatterns];
    cv::Mat image;
    char temp_name[50];
    for(int i=0;i<nPatterns;i++)
    {
	   sprintf(temp_name,"/patterns/enter%d.png",i+1);
	   std::string name = param_path+temp_name;
	   image = cv::imread(name.c_str(),1);
       cvtColor(image,image, CV_BGR2HSV);
       patterns[i]=image;
    }
    // Quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30;
    int histSize[] = {hbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    const float* ranges[] = { hranges};
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0};
    calcHist(patterns ,nPatterns, channels, cv::Mat(),patternHistog, 1, histSize, ranges,true, false );
    delete[] patterns;
}

/**
 * Sets general parameters
 */
void HazmatEpsilonDetector::setParameters()
{
	// Set parameters
	KDTREE_BBF_MAX_NN_CHKS = 200;
	NN_SQ_DIST_RATIO_THR = .49;
	char temp_name[50];
	strcpy(temp_name, "/contents");
	patternIndexPath = param_path + temp_name;
	
} 

/**
 * Calculates the area of a rectangle, using the area of its 2 fundamental triangles.
 * Irona's computation method is used
 * @param pt1
 * @param pt2
 * @param pt3
 * @param pt4
 * @return
 */
float HazmatEpsilonDetector::calculateRectangleArea(CvPoint2D64f pt1,CvPoint2D64f pt2, CvPoint2D64f pt3,CvPoint2D64f pt4)
{
	float sideA, sideB, sideC; 
	sideA = sqrt(((pt1.x - pt2.x) * (pt1.x - pt2.x)) + ((pt1.y - pt2.y) * (pt1.y - pt2.y)));
	sideB = sqrt(((pt2.x - pt3.x) * (pt2.x - pt3.x)) + ((pt2.y - pt3.y) * (pt2.y - pt3.y)));
	sideC = sqrt(((pt1.x - pt3.x) * (pt1.x - pt3.x)) + ((pt1.y - pt3.y) * (pt1.y - pt3.y)));
	float tempValue,triangleArea;
	tempValue = (sideA + sideB + sideC)/2.;
	triangleArea = sqrt( tempValue * (tempValue - sideA) * (tempValue - sideB) * (tempValue - sideC) );
	float triangleArea2;
	sideA = sqrt(((pt1.x - pt3.x) * (pt1.x - pt3.x)) + ((pt1.y - pt3.y) * (pt1.y - pt3.y)));
	sideB = sqrt(((pt1.x - pt4.x) * (pt1.x - pt4.x)) + ((pt1.y - pt4.y) * (pt1.y - pt4.y)));
	sideC = sqrt(((pt3.x - pt4.x) * (pt3.x - pt4.x)) + ((pt3.y - pt4.y) * (pt3.y - pt4.y)));
	tempValue = (sideA + sideB + sideC)/2.;
	triangleArea2 = sqrt(tempValue*(tempValue - sideA)*(tempValue - sideB)*(tempValue - sideC));
	float rectangleArea = triangleArea + triangleArea2;
	return rectangleArea;
}

/**
 * Draws detected hazmat for debuging purposes
 * @param img
 * @param target
 * @param H
 */
static void draw_xform(cv::Mat* img, cv::Mat* target, CvMat* H )
{
	CvPoint2D64f xc[4], c[4] = { { 0, 0 }, { target->size().width-1, 0 }, { target->size().width-1, target->size().height-1 }, { 0, target->size().height-1 } };
	int i;
	for( i = 0; i < 4; i++ )
		xc[i] = persp_xform_pt( c[i], H );
	for( i = 0; i < 4; i++ )
		cv::line( *img, cv::Point( xc[i].x, xc[i].y ), cv::Point( xc[(i+1)%4].x, xc[(i+1)%4].y ), CV_RGB( 0, 255, 0 ), 10, CV_AA, 0 );
}

/**
 * Initializes the hazmat/epsilon detector
 */
void HazmatEpsilonDetector::initDetector()
{
	// Read number of patterns

	FILE* contents = fopen(patternIndexPath.c_str(), "r");
	if (contents==NULL) 
		printf("Can't open contents file");
     
	char imgName[50];
	char featName[50];
	int n;
	nPatterns=0;
	while(true)
	{
		fgets(imgName, 49, contents);	

		if(imgName[0]=='\n') 
		{
			//~ printf("%d npattern imgName[0]==\n",nPatterns);
			break;
		}
		if (feof(contents)!=0) 
		{
			//~ printf("%d feof(contents)!=0\n",nPatterns);
			break;
		}

		nPatterns++;
	}		
	//Sets the position indicator associated with stream to the beginning of the file.
	rewind(contents);
	// Load features of patterns in memory
    feats=new feature* [nPatterns];
	//trees = (struct kd_node**)malloc(nPatterns * sizeof(struct kd_node*));
	nFeats=new int[nPatterns];
	for(n=0; n<nPatterns; n++)
	{
		fgets(imgName, 49, contents);
		imgName[strlen(imgName)-1] = '\0';
		sprintf(featName, "/%s.sift", imgName);

		std::string feature_name;
		feature_name=param_path+featName;

		//Returns the number of features imported from every pattern
		nFeats[n] = import_features((char*)feature_name.c_str(), FEATURE_LOWE, &feats[n]);
		//printf("%d nFeats[n]\n",nFeats[n]);
	} 
	fclose(contents);
}

/**
 * Preprocess stage. It loads trained hazmats on the detector
 */
void HazmatEpsilonDetector::preprocessHazmat()
{
	// Read table of contents
	FILE* contents = fopen(patternIndexPath.c_str(), "r");
	char imgName[50], featName[50];
	while( true )
	{
		// Get next pattern
		fgets(imgName, 49, contents);
		imgName[strlen(imgName)-1] = '\0';
		if( strlen(imgName)==0) break;
		printf("Processing hazmat \"%s\"\n", imgName);
		struct feature* feat;

		// Compute and store features
		IplImage* img = cvLoadImage(imgName, 1);
		if(!img) 
		{
			std::cout<<"No image!\n"<<std::endl;
		}
		int n = sift_features(img, &feat);
		sprintf(featName, "%s.sift", imgName);
		export_features(featName, feat, n);

		// Free memory
		cvReleaseImage(&img);
		delete feat;
	}
	fclose(contents);
}

/**
 * Searches for useful features to track
 * @param m
 * @param n
 * @param testNum
 * @param kd_root
 * @return
 */
vector <int>HazmatEpsilonDetector::findFeature(int &m,int n,int testNum,struct kd_node* kd_root)
{
	        struct feature** nbrs;	   
	        int i;
	        double d0,d1;
			vector <int> tempvector;
			for(i=0; i<testNum; i++)
			{
				int k = kdtree_bbf_knn( kd_root, feats[n] +i, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
				if( k == 2 ){
					d0 = descr_dist_sq( feats[n] +i, nbrs[0] );
					d1 = descr_dist_sq( feats[n] +i, nbrs[1] );
					if( d0 < d1 * NN_SQ_DIST_RATIO_THR ){
						feats[n][i].fwd_match = nbrs[0];
						tempvector.push_back(i);
						m++;
					}
				}
				free(nbrs);
			}
			return tempvector;
} 

/**
 * Calculates a given 4 point area. Its not necessarily a rectangle.
 * @param H
 * @param pattern_image
 */
void  HazmatEpsilonDetector::calculateArea(CvMat* H,cv::Mat pattern_image)
{
	CvPoint2D64f point1 = {0,0};
	CvPoint2D64f point2 = {0,pattern_image.size().height};
	CvPoint2D64f point3 = {pattern_image.size().width,0};
	CvPoint2D64f point4 = {pattern_image.size().width,pattern_image.size().height};
   //find the transformation of the 4 corners of the initial image
	point1 = persp_xform_pt( point1, H );
	point2 = persp_xform_pt( point2, H );
	point3 = persp_xform_pt( point3, H );
	point4 = persp_xform_pt( point4, H );
    //calculate the final area
	area = calculateRectangleArea(point1,point2,point3,point4);
}

/**
 * calculates variance, useful for later hazmat matching
 * @param SAD
 * @param SAD2
 * @param img
 * @param H
 * @param _pattern_image
 * @param n
 * @return
 */
CvPoint2D64f HazmatEpsilonDetector::defineVariance(float& SAD,float& SAD2,IplImage* img,CvMat* H,cv::Mat _pattern_image,int n)  
{ 
    IplImage* pattern_image = cvCreateImage( cvSize(cols,rows), IPL_DEPTH_8U, 3 );
    IplImage* temp = new IplImage(_pattern_image);
    pattern_image=cvCloneImage(temp);
   
    IplImage* _xformed= cvCreateImage( cvGetSize( img ), IPL_DEPTH_8U, 3 );
    cvWarpPerspective( pattern_image, _xformed, H, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,cv::Scalar( 0,0,0 ) );
    cv::Mat xformed(_xformed,false);
 
	cv::Mat color_image = cv::Mat( cv::Size(cols,rows), CV_8U, 3 );
	xformed.copyTo(color_image);
	cvtColor(color_image,color_image,CV_BGR2YCrCb);
	cv::Mat _img(img,false);
	cv::Mat img_ycrcb = cv::Mat( cvSize(cols,rows), CV_8U, 3 );
	_img.copyTo(img_ycrcb);
	cvtColor(img_ycrcb,img_ycrcb,CV_BGR2YCrCb);

	int height,width,step,channels;
	int height2,width2,step2,channels2;	
	height    = color_image.size().height;
	width     = color_image.size().width;
	step      = color_image.step;
	channels  = color_image.channels();
	height2    = img_ycrcb.size().height;
	width2     = img_ycrcb.size().width;
	step2      = img_ycrcb.step;
	channels2  = img_ycrcb.channels();
	CvPoint2D64f point={(_pattern_image.size().width)/2,(_pattern_image.size().height)/2};
	point = persp_xform_pt( point, H );
	votes = 0;
						
	//start voting
	for (int counter=0; counter<sideLength; counter++)
    {
		for (int counter2=0; counter2<sideLength; counter2++)
	    {
			if (( (point.y - (sideLength/2) +counter ) >= 0)&& ( (point.y - (sideLength/2) +counter ) < rows ))
			{
				if (( (point.x - (sideLength/2) +counter2 ) >= 0)&& ( (point.x - (sideLength/2) +counter2 ) < cols ))
			    {
					bool test1 = (((uchar*)img_ycrcb.data)[(((int)point.y)-(sideLength/2)+counter)*step + (((int)point.x)-(sideLength/2)+counter2)*channels+1] >= (minUV[n][0]-333333333333333333333)) && (((uchar*)img_ycrcb.data)[(((int)point.y)-(sideLength/2)+counter)*step + (((int)point.x)-(sideLength/2)+counter2)*channels+1] <= (maxUV[n][0]+colorVariance));
					if(test1 == true)
					{
						bool test2 = (((uchar*)img_ycrcb.data)[(((int)point.y)-(sideLength/2)+counter)*step + (((int)point.x)-(sideLength/2)+counter2)*channels+2] >= (minUV[n][1]-colorVariance)) && (((uchar*)img_ycrcb.data)[(((int)point.y)-(sideLength/2)+counter)*step + (((int)point.x)-(sideLength/2)+counter2)*channels+2] <= (maxUV[n][1]+colorVariance));
						if(test2 == true)
						{
							votes = votes + 1;
						}
					}
			//find the absolute variance between the initial and the final image
					SAD = SAD + fabsf( ((uchar*)img_ycrcb.data)[(((int)point.y)-(sideLength/2)+counter)*step2 + (((int)point.x)-(sideLength/2)+counter2)*channels2+1] - ((uchar *)color_image.data)[(((int)point.y)-(sideLength/2)+counter)*step + (((int)point.x)-(sideLength/2)+counter2)*channels+1]); 
					SAD2 = SAD2 + fabsf( ((uchar*)img_ycrcb.data)[(((int)point.y)-(sideLength/2)+counter)*step2 + (((int)point.x)-(sideLength/2)+counter2)*channels2+2] - ((uchar *)color_image.data)[(((int)point.y)-(sideLength/2)+counter)*step + (((int)point.x)-(sideLength/2)+counter2)*channels+2]); 
					}	 	
				}
		}
	}
	cvReleaseImage(&pattern_image);
	cvReleaseImage(&_xformed);
	delete temp;
	return point;
}						

/**
 * basic function of the hazmat detector
 * @param hazmatFrame
 * @return
 */
vector<HazmatEpsilon> HazmatEpsilonDetector::detectHazmat(cv::Mat hazmatFrame)
{
	vector<HazmatEpsilon> result;
    frameNum++;
	IplImage* img = cvCreateImage( cv::Size(cols,rows), IPL_DEPTH_8U, 3 );
    IplImage* temp = new IplImage(hazmatFrame);
    img=cvCloneImage(temp);
    if(frameNum=0)
    {
		initDetector();
	}
	else
	{
		result.erase(result.begin(),result.end());
	}
	// SIFT process of screenshot
	struct kd_node* kd_root;
	
	if (!img)
		cout << "Could not load image" << endl;
	struct feature* featShot;
	//int testNum;
	// Finds SIFT features in an image.
	int nShot = sift_features(img, &featShot);
	//~ printf("%d features in shot  found\n", nShot);
	if (nShot>0)
	{
		kd_root = kdtree_build( featShot, nShot );
		
		// Search screenshot for each pattern
		int n, i;
		//double d0, d1;
		int max = -1, pat;
		for(n=0; n<nPatterns; n++)
		{
			// Find nearest neighboor for each feature in screenshot
			int m=0;
			int testNum = nFeats[n];
			vector <int> tempvector;
			tempvector=findFeature(m,n,testNum,kd_root);
          
			// Run RANSAC to find out a transformation that transforms patterns into screenshot
			if( m > featureThreshold )
			{
				CvMat* H;
				H = ransac_xform( feats[n], testNum, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01, homog_xfer_err,3.0, NULL, NULL );
				if( H )
				{
					char temp_name[50];
					sprintf(temp_name,"/patterns/enter%d.png",n+1);
					std::string name;
					name = param_path + temp_name;
				
					cv::Mat pattern_image;
					pattern_image=cv::imread(name.c_str(),CV_LOAD_IMAGE_COLOR);
					if (!pattern_image.data){
						cout<< "could not load pattern image"<<endl;
					}
				    calculateArea(H,pattern_image);

					if (area >=minAreaThreshold && area <= maxAreaThreshold)
					{
						float SAD = 0;
						float SAD2 = 0;
						CvPoint2D64f point;
						point=defineVariance(SAD,SAD2,img,H,pattern_image,n);
						
						//check if the final image's results is within thresholds
						if (votes > votingThreshold){
							float MO= ( SAD + SAD2 )/2;	
							if (MO < MOThreshold)
							{
								HazmatEpsilon a;
								a.x = point.x;
								a.y = point.y;
								a.pattern_num = n+1;
								a.m=m;
								a.MO=MO;
								a.votes=votes;
								a.H = cvCloneMat(H);
								if (result.size()==0)
								{
									result.push_back(a);
								}
								else
								{
									bool flag = false;
									for (int v=0; v<result.size(); v++)
									{
										float check1 = fabsf( a.x - result[v].x);
										float check2 = fabsf( a.y - result[v].y);
										if ( (check1 <= 20) && (check2 <= 20) )
										{
											flag = true;
											float possibility1 = 0;
											float possibility2 = 0;
											//check if i already have a hazmat in the same place and check which fits best
											if ( a.m > result[v].m)
											{
												possibility1 = possibility1 + 0.4;
											}
											else
											{
												possibility2 = possibility2 +0.4;
											}
											if (a.MO < result[v].MO)
											{
												possibility1 = possibility1 + 0.3;
											}
											else{
												possibility2 = possibility2 + 0.3;
											}
											if ( a.votes > result[v].votes )
											{
												possibility1 = possibility1 + 0.3;
											}
											else if( a.votes = result[v].votes )
											{
												possibility1 = possibility1 + 0.3;
												possibility2 = possibility2 + 0.3;
											}
											else
											{
												possibility2 = possibility2 + 0.3;
											}
											if ( possibility1 >= possibility2)
											{
												result[v].x = a.x;
												result[v].y = a.y;
												result[v].pattern_num = a.pattern_num;
												result[v].m = a.m;
												result[v].MO = a.MO;
												result[v].votes = a.votes;
												result[v].H = a.H;
											}
										}
									}
									if (flag == false){
										result.push_back(a);
									}
								}
							}
						}
					}
				}

				cvReleaseMat(&H);
			}
			if (tempvector.size() > 0)
			{
				for (int l=0;l<tempvector.size(); l++)
				{
					feats[n][tempvector[l]].fwd_match=NULL;
				}
			}
			tempvector.erase(tempvector.begin(),tempvector.end());
		}
		kdtree_release(kd_root);
		free(featShot);
	}	
	   if(frameNum>1)
    {
		frameNum=0;
		delete[] feats;
	}
	delete temp;
	cvReleaseImage(&img);	
	return result;
}

/**
 * Basic general function
 * @param img
 * @return
 */
vector<HazmatEpsilon> HazmatEpsilonDetector::DetectHazmatEpsilon(cv::Mat img){
	vector<HazmatEpsilon> result;
	result=detectHazmat(img);
	return result;
}

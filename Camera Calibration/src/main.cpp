#include <iostream>
#include <stdio.h>  
#include <opencv2/opencv.hpp>  
#include <vector>  


using namespace std;
using namespace cv;

#define SQRT2 1.41
#define SQRT3 1.7321

struct normalizedData{
Mat T2D;
Mat T3D;
vector<Point2f> newPts2D;
vector<Point3f> newPts3D;
};


struct normalizedData NormalizeData(vector<Point3f> pts3D,vector<Point2f> pts2D){
    int ptsNum=pts3D.size();

	//calculate means (they will be the center of coordinate systems)
	float mean1x=0.0,mean1y=0.0,mean2x=0.0,mean2y=0.0,mean2z=0.0;
	for (int i=0;i<ptsNum;i++){
		mean1x+=pts2D[i].x;
		mean1y+=pts2D[i].y;
		mean2x+=pts3D[i].x;
		mean2y+=pts3D[i].y;
		mean2z+=pts3D[i].z;
		}
	mean1x/=ptsNum;
	mean1y/=ptsNum;
	mean2x/=ptsNum;
	mean2y/=ptsNum;
	mean2z/=ptsNum;

	float spread1x=0.0,spread1y=0.0,spread2x=0.0,spread2y=0.0,spread2z=0.0;
	
	for (int i=0;i<ptsNum;i++){
		spread1x+=(pts2D[i].x-mean1x)*(pts2D[i].x-mean1x);
		spread1y+=(pts2D[i].y-mean1y)*(pts2D[i].y-mean1y);
		spread2x+=(pts3D[i].x-mean2x)*(pts3D[i].x-mean1x);
		spread2y+=(pts3D[i].y-mean2y)*(pts3D[i].y-mean2y);
		spread2z+=(pts3D[i].z-mean2z)*(pts3D[i].z-mean2z);
		}
	
	spread1x/=ptsNum;
	spread1y/=ptsNum;
	spread2x/=ptsNum;
	spread2y/=ptsNum;
	spread2z/=ptsNum;

	Mat offs1=Mat::eye(3,3,CV_32F);
	Mat offs2=Mat::eye(4,4,CV_32F);
	Mat scale1=Mat::eye(3,3,CV_32F);
	Mat scale2=Mat::eye(4,4,CV_32F);

	offs1.at<float>(0,2)=-mean1x;
	offs1.at<float>(1,2)=-mean1y;

	offs2.at<float>(0,3)=-mean2x;
	offs2.at<float>(1,3)=-mean2y;
	offs2.at<float>(2,3)=-mean2z;

	scale1.at<float>(0,0)=SQRT2/sqrt(spread1x);
	scale1.at<float>(1,1)=SQRT2/sqrt(spread1y);

	scale2.at<float>(0,0)=SQRT3/sqrt(spread2x);
	scale2.at<float>(1,1)=SQRT3/sqrt(spread2y);
	scale2.at<float>(2,2)=SQRT3/sqrt(spread2z);



	struct normalizedData ret;
	ret.T2D=scale1*offs1;
	ret.T3D=scale2*offs2;

	for (int i=0;i<ptsNum;i++){
		Point2f p2D;
		Point3f p3D;

		p2D.x=SQRT2*(pts2D[i].x-mean1x)/sqrt(spread1x);
		p2D.y=SQRT2*(pts2D[i].y-mean1y)/sqrt(spread1y);

		p3D.x=SQRT3*(pts3D[i].x-mean2x)/sqrt(spread2x);
		p3D.y=SQRT3*(pts3D[i].y-mean2y)/sqrt(spread2y);
		p3D.z=SQRT3*(pts3D[i].z-mean2z)/sqrt(spread2z);

		ret.newPts2D.push_back(p2D);
		ret.newPts3D.push_back(p3D);
	}
	
	return ret;

}

Mat calcProjMtx(vector<Point3f> objPts,vector<Point2f> imgPts){
    const int ptsNum=objPts.size();

    struct normalizedData nd=NormalizeData(objPts,imgPts);


//Unnormalized version:
//    vector<Point3f> currPts3D=objPts;
//    vector<Point2f> currPts2D=imgPts;

//Normalized version:
    vector<Point3f> currPts3D=nd.newPts3D;
    vector<Point2f> currPts2D=nd.newPts2D;

    Mat A(2*ptsNum,12,CV_32F);

    for(int i=0;i<ptsNum;i++){

        float X=currPts3D[i].x;
        float Y=currPts3D[i].y;
        float Z=currPts3D[i].z;

        float u=currPts2D[i].x;
        float v=currPts2D[i].y;

        A.at<float>(2*i,0)=X;
        A.at<float>(2*i,1)=Y;
        A.at<float>(2*i,2)=Z;
        A.at<float>(2*i,3)=1.0f;
        A.at<float>(2*i,4)=0.0f;
        A.at<float>(2*i,5)=0.0f;
        A.at<float>(2*i,6)=0.0f;
        A.at<float>(2*i,7)=0.0f;
        A.at<float>(2*i,8)=-u*X;
        A.at<float>(2*i,9)=-u*Y;
        A.at<float>(2*i,10)=-u*Z;
        A.at<float>(2*i,11)=-u;

        A.at<float>(2*i+1,0)=0.0f;
        A.at<float>(2*i+1,1)=0.0f;
        A.at<float>(2*i+1,2)=0.0f;
        A.at<float>(2*i+1,3)=0.0f;
        A.at<float>(2*i+1,4)=X;
        A.at<float>(2*i+1,5)=Y;
        A.at<float>(2*i+1,6)=Z;
        A.at<float>(2*i+1,7)=1.0f;
        A.at<float>(2*i+1,8)=-v*X;
        A.at<float>(2*i+1,9)=-v*Y;
        A.at<float>(2*i+1,10)=-v*Z;
        A.at<float>(2*i+1,11)=-v;


        }

        Mat eVecs(12,12,CV_32F),eVals(12,12,CV_32F);
        cout <<A <<endl;
        eigen(A.t()*A, eVals, eVecs);

        cout <<eVals <<endl;
        cout <<eVecs <<endl;


        Mat P(3,4,CV_32F);
        for (int i=0;i<12;i++) P.at<float>(i/4,i%4)=eVecs.at<float>(11,i);

        cout <<P <<endl;

//For normalization:
        P=(nd.T2D.inv())*P*nd.T3D; 

        cout <<"T2D:\n"<<nd.T2D <<endl;
        cout <<"T3D:\n"<< nd.T3D <<endl;


        //Normalize:
        P=P*(1.0/P.at<float>(2,3));
        cout <<P <<endl;

        return P;
}




int main(int argc, char*argv[])
{
  // Create a synthetic projection matrix
  vector<Point3f> pts3D;
  vector<Point2f> pts2D;


  Point3f currPt3D;
  Point2f currPt2D;

  // Point #1
  currPt3D.x=1000.0f;
  currPt3D.y=1000.0;
  currPt3D.z=0.0f;

  currPt2D.x=379.0f;
  currPt2D.y=762.0f;

  pts3D.push_back(currPt3D);
  pts2D.push_back(currPt2D);

  // Point #2
  currPt3D.x=1000.0f;
  currPt3D.y=0.0;
  currPt3D.z=0.0f;

  currPt2D.x=710.0f;
  currPt2D.y=446.0f;

  pts3D.push_back(currPt3D);
  pts2D.push_back(currPt2D);

  // Point #3
  currPt3D.x=0.0f;
  currPt3D.y=0.0f;
  currPt3D.z=0.0f;

  currPt2D.x=877.0f;
  currPt2D.y=981.0f;

  pts3D.push_back(currPt3D);
  pts2D.push_back(currPt2D);

  // Point #4
  currPt3D.x=0.0f;
  currPt3D.y=1000.0f;
  currPt3D.z=0.0f;

  currPt2D.x=468.0f;
  currPt2D.y=1276.0f;

  pts3D.push_back(currPt3D);
  pts2D.push_back(currPt2D);

  // Point #5
  currPt3D.x=1000.0f;
  currPt3D.y=0.0f;
  currPt3D.z=-1000.0f;

  currPt2D.x=1104.0f;
  currPt2D.y=602.0f;

  pts3D.push_back(currPt3D);
  pts2D.push_back(currPt2D);

  // Point #6
  currPt3D.x=0.0f;
  currPt3D.y=0.0f;
  currPt3D.z=-1000.0f;

  currPt2D.x=1273.0f;
  currPt2D.y=1042.0f;

  pts3D.push_back(currPt3D);
  pts2D.push_back(currPt2D);

  // Point #7
  currPt3D.x=0.0f;
  currPt3D.y=1000.0f;
  currPt3D.z=-1000.0f;

  currPt2D.x=915.0f;
  currPt2D.y=1272.0f;

  pts3D.push_back(currPt3D);
  pts2D.push_back(currPt2D);

  vector<Mat> rvecs, tvecs;  
  Mat intrinsic_Matrix(3,3, CV_64F);  
  Mat distortion_coeffs(8,1, CV_64F);  


  Mat P=calcProjMtx(pts3D,pts2D);


  cv::Mat K(3,3,CV_32F); // intrinsic parameter matrix
  cv::Mat R(3,3,CV_32F); // rotation matrix
  cv::Mat T(4,1,CV_32F); // translation vector

  cv::decomposeProjectionMatrix(P, K, R, T);
  K=K*(1.0/K.at<float>(2,2));
  std::cout << "K: " << K << std::endl;
  std::cout << "R: " << R << std::endl;
  std::cout << "T: " << T << std::endl;

  return 0;

}



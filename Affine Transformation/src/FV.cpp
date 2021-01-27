/*
The project visualizes a 3D point cloud. The task is to improve the original projects:
- Add mouse-controlling to the project, similarly to Meshlab. The object is rotated if the left mouse button 
is pressed and the mouse position is changed (dragged). Vertical and horizontal movement changes the angles 
u and v, respectively.  
- Animate the object, independently to the user's control. Rotate the objects with a constant rotation w.r.t 
time, all the three principal axes should be used for the rotation (2%), periodically scale the object between
scale factors 0.5 and 2.0  
*/
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>
#include <ctime>
#include <time.h>


#include "MatrixReaderWriter.h"
#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;


float v=1.0;
float u=0.5;
float angle=0.5;
float rad = 100.0;
float rad2 = 1000.0;
float scale = 0.2;
float curMouseXPos, curMouseYPos, lastMouseXPos, lastMouseYPos;


int cnt = 0;
int cons = 0;


MatrixReaderWriter* mrw;
Mat resImg;

template <typename Duration, typename Function>
void timer(Duration const & d, Function const & f)
{
	std::thread([d, f]() {
		std::this_thread::sleep_for(d);
		f();
	}).detach();
}


void drawPoints(MatrixReaderWriter* mrw,float u, float v,float rad,Mat& resImg){
	int NUM=mrw->rowNum;

	
	Mat C(3,3,CV_32F);
	Mat R(3,3,CV_32F);
	Mat T(3,1,CV_32F);

	float tx=cos(u)*sin(v);
	float ty=sin(u)*sin(v);
	float tz=cos(v);

	//cout << "In draw points : " << rad << "," << u << "," << v << endl;

//Intrincic parameters

	C.at<float>(0,0)=3000.0f;
	C.at<float>(0,1)=0.0f;
	C.at<float>(0,2)=400.0f;

	C.at<float>(1,0)=0.0f;
	C.at<float>(1,1)=3000.0f;
	C.at<float>(1,2)=300.0f;

	C.at<float>(2,0)=0.0f;
	C.at<float>(2,1)=0.0f;
	C.at<float>(2,2)=1.0f;
	
	T.at<float>(0,0)=rad*tx;
	T.at<float>(1,0)=rad*ty;
	T.at<float>(2,0)=rad*tz;


//Mirror?
	int HowManyPi=(int)floor(v/3.1415);
		


//Axes:
	Point3f Z(-1.0*tx,-1.0*ty,-1.0*tz);
	Point3f X(sin(u)*sin(v),-cos(u)*sin(v),0.0f);
	if (HowManyPi%2)
		X=(1.0/sqrt(X.x*X.x+X.y*X.y+X.z*X.z))*X;
	else
		X=(-1.0/sqrt(X.x*X.x+X.y*X.y+X.z*X.z))*X;

	Point3f up=X.cross(Z);  //Axis Y

/*
	printf("%f\n",X.x*X.x+X.y*X.y+X.z*X.z);
	printf("%f\n",up.x*up.x+up.y*up.y+up.z*up.z);
	printf("%f\n",Z.x*Z.x+Z.y*Z.y+Z.z*Z.z);
	printf("(%f,%f)\n",u,v);
*/
	R.at<float>(0, 0) = X.x;
	R.at<float>(0, 1) = X.y;
	R.at<float>(0, 2) = X.z;

	R.at<float>(1, 0) = up.x;
	R.at<float>(1, 1) = up.y;
	R.at<float>(1, 2) = up.z;

	R.at<float>(2, 0) = Z.x;
	R.at<float>(2, 1) = Z.y;
	R.at<float>(2, 2) = Z.z;

	
	for (int i=0;i<NUM;i++){
		Mat vec(3,1,CV_32F);
		vec.at<float>(0,0)=mrw->data[3*i];
		vec.at<float>(1,0)=mrw->data[3*i+1];
		vec.at<float>(2,0)=mrw->data[3*i+2];
		
		int red=255;
		int green=255;
		int blue=255;
		
		Mat trVec=R*(vec-T);
		trVec=C*trVec;
		trVec=trVec/trVec.at<float>(2,0);
//		printf("(%d,%d)",(int)trVec.at<float>(0,0),(int)trVec.at<float>(1,0));
		
		circle( resImg, Point( (int)trVec.at<float>(0,0), (int)trVec.at<float>(1,0) ), 2.0, Scalar( blue, green, red ), 2, 8 );
	}
}


void drawPointsCont(MatrixReaderWriter* mrw, float angle, float u, float v, float rad, float scale, Mat& resImg) {
	int NUM = mrw->rowNum;

	Mat R = Mat::zeros(4, 4, CV_32F);
	Mat Rx = Mat::zeros(4, 4, CV_32F);
	Mat Ry = Mat::zeros(4, 4, CV_32F);
	Mat Rz = Mat::zeros(4, 4, CV_32F);
	Mat scaleTr = Mat::eye(4, 4, CV_32F);
	Mat offsetTr = Mat::eye(4, 4, CV_32F);

	//Scale
	scaleTr.at<float>(0, 0) = scale;
	scaleTr.at<float>(1, 1) = scale;
	scaleTr.at<float>(2, 2) = scale;

	//translation
	offsetTr.at<float>(0, 3) = 300.0;
	offsetTr.at<float>(1, 3) = 300.0;
	offsetTr.at<float>(2, 3) = 300.0;

	/*Rotation in x direction*/
	Rx.at<float>(0, 0) = Rx.at<float>(3, 3) = 1.0;
	Rx.at<float>(1, 1) = Rx.at<float>(2, 2) = cos(angle);
	Rx.at<float>(1, 2) = -sin(angle);
	Rx.at<float>(2, 1) = sin(angle);

	/*Rotation in y direction*/
	Ry.at<float>(1, 1) = Ry.at<float>(3, 3) = 1.0;
	Ry.at<float>(0, 0) = Ry.at<float>(2, 2) = cos(angle);
	Ry.at<float>(0, 2) = sin(angle);
	Ry.at<float>(2, 0) = -sin(angle);

	/*Rotation in z direction*/
	Rz.at<float>(2, 2) = Rz.at<float>(3, 3) = 1.0;
	Rz.at<float>(0, 0) = Rz.at<float>(1, 1) = cos(angle);
	Rz.at<float>(0, 1) = -sin(angle);
	Rz.at<float>(1, 0) = sin(angle);

	/*3D rotation*/
	R = Rx * Ry * Rz;		

	for (int i = 0; i < NUM; i++) {
		Mat vec(4, 1, CV_32F);
		vec.at<float>(0, 0) = mrw->data[3 * i];
		vec.at<float>(1, 0) = mrw->data[3 * i + 1];
		vec.at<float>(2, 0) = mrw->data[3 * i + 2];
		vec.at<float>(3, 0) = 1;

		int red = 255;
		int green = 255;
		int blue = 255;

		Mat trVec = offsetTr * R * scaleTr * vec;
		trVec = trVec / trVec.at<float>(3, 0);

		circle(resImg, Point((int)trVec.at<float>(0, 0), (int)trVec.at<float>(1, 0)), 2.0, Scalar(blue, green, red), 2, 8);
	}
}

void rotateScale() {	
	resImg = Mat::zeros(600, 800, CV_8UC3);
	angle += 0.2;
	cnt++;
	if (cnt % 10 == 0) {
		cnt = 0;
		cons++;
	}
	if (cons % 2 == 0) {
		drawPointsCont(mrw, angle, u, v, rad2, 0.5, resImg);
	}
	else {
		drawPointsCont(mrw, angle, u, v, rad2, 0.2, resImg);
	}		
	imshow("Display window", resImg);
}


void MouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	curMouseXPos = x;
	curMouseYPos = y;
	
	/*zoom in and zoom out*/
	if (event==EVENT_MOUSEWHEEL)
	{
		//cout << getMouseWheelDelta(flags) << endl;
		if (getMouseWheelDelta(flags)>0){
			rad *= (float)1.1;
		}
		else if (getMouseWheelDelta(flags)<0){
			rad /= (float)1.1;
		}
		resImg = Mat::zeros(600, 800, CV_8UC3);
		drawPoints(mrw, u, v, rad, resImg);
		imshow("Display window", resImg);
	}
	/*rotate on left click and movement of mouse*/
	if (event == EVENT_MOUSEMOVE && flags == EVENT_FLAG_LBUTTON) {
		if (curMouseYPos > lastMouseYPos) {
			v += 0.2;
		}
		else if (curMouseYPos < lastMouseYPos) {
			v -= 0.2;
		}
		else if (curMouseXPos > lastMouseXPos) {
			u += 0.2;
		}
		else if (curMouseXPos < lastMouseXPos) {
			u -= 0.2;
		}
		resImg = Mat::zeros(600, 800, CV_8UC3);
		drawPoints(mrw, u, v, rad, resImg);
		imshow("Display window", resImg);
	}
	lastMouseXPos = curMouseXPos;
	lastMouseYPos = curMouseYPos;
}

void MouseKeyBoardOptions() {
	resImg = Mat::zeros(600, 800, CV_8UC3);
	drawPoints(mrw, u, v, rad, resImg);
	imshow("Display window", resImg);

	namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
	setMouseCallback("Display window", MouseCallBackFunc, NULL);
	imshow("Display window", resImg);

	char key;
	while (true) {
		key = waitKey(0);
		if (key == 27) break;

		switch (key) {
		case 'q':
			u += 0.1;
			break;
		case 'a':
			u -= 0.1;
			break;
		case 'w':
			v += 0.1;
			break;
		case 's':
			v -= 0.1;
			break;
		case 'e':
			rad *= 1.1;
			break;
		case 'd':
			rad /= 1.1;
			break;
		}
		resImg = Mat::zeros(600, 800, CV_8UC3);
		drawPoints(mrw, u, v, rad, resImg);
		imshow("Display window", resImg);                   // Show our image inside it.
	}
}

int main(int argc, const char** argv){
	int choice;
	choice = 1;
	if (argc!=2) {printf("Usage: FV filename\n");exit(0);}
	
	mrw = new MatrixReaderWriter(argv[1]);
    
	printf("%d %d\n",mrw->rowNum,mrw->columnNum);
	
	cout << "User Menu:" << endl;
	cout << "1. Mouse and keyboard events" << endl;
	cout << "2. Animation" << endl;
	cout << "Press 1 or 2 "<< endl;
	cin >> choice;

	switch (choice) {
	case 1:
		MouseKeyBoardOptions();
		break;
	case 2:
		resImg = Mat::zeros(600, 800, CV_8UC3);
		drawPoints(mrw, u, v, rad2, resImg);
		imshow("Display window", resImg);
		while (1) {
			timer(std::chrono::milliseconds(1000), &rotateScale);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			char key = waitKey(5);
		}
		break;
	default:
		cout << "Enter right choice" << endl;
		break;
	}
	return 0;
}

//Apply Image Transformations, move, translate, rotate and find the new position, orientation of the image
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;




Mat createPerspectiveTraf(float angle,float skew,float dx,float dy,float scale1, float scale2,float persp1, float persp2){
    Mat transformation(3,3,CV_32F);

    //Offset (eltolas)
	
    Mat offsetTr=Mat::eye(3,3,CV_32F);
    offsetTr.at<float>(0,2)=dx;
    offsetTr.at<float>(1,2)=dy;


    //Skew (nyiras)
    Mat skewTr=Mat::eye(3,3,CV_32F);
    skewTr.at<float>(0,1)=skew;

    //Scale (skalazas)
    Mat scaleTr=Mat::eye(3,3,CV_32F);
    scaleTr.at<float>(0,0)=scale1;
    scaleTr.at<float>(1,1)=scale2;


    //Rotation (elforgatas)
    Mat rotTr=Mat::eye(3,3,CV_32F);
    rotTr.at<float>(0,0)=rotTr.at<float>(1,1)=cos(angle);
    rotTr.at<float>(0,1)=-sin(angle);
    rotTr.at<float>(1,0)=sin(angle);

/*
    cout <<offsetTr << endl;
    cout <<rotTr << endl;
    cout <<skewTr << endl;
    cout <<scaleTr << endl;
*/
    

    //Concatenation of transformations (trafok egymas utani elvegzese)
    transformation=offsetTr*rotTr*skewTr*scaleTr;
    cout << "Affine Traf: " <<transformation <<endl ;


    //Perspectivity (perspectiv resz):
    transformation.at<float>(2,0)=persp1;
    transformation.at<float>(2,1)=persp2;
    cout << "Perspective Traf: " <<transformation <<endl;


	return transformation;
}




//Kepeket eltranszformalja:

void transformImage(Mat origImg,Mat& newImage,Mat tr,bool isPerspective){
    Mat invTr=tr.inv();
    const int WIDTH=origImg.cols;
    const int HEIGHT=origImg.rows;


    for (int x=0;x<WIDTH;x++) for (int y=0;y<HEIGHT;y++){
        Mat pt(3,1,CV_32F);
        pt.at<float>(0,0)=x;
        pt.at<float>(1,0)=y;
        pt.at<float>(2,0)=1.0;

        Mat ptTransformed=invTr*pt;
//        cout <<pt <<endl;
//        cout <<invTr <<endl;
//        cout <<ptTransformed <<endl;
        if (isPerspective) ptTransformed=(1.0/ptTransformed.at<float>(2,0))*ptTransformed;

        int newX=round(ptTransformed.at<float>(0,0));
        int newY=round(ptTransformed.at<float>(1,0));

        if ((newX>=0)&&(newX<WIDTH)&&(newY>=0)&&(newY<HEIGHT)) newImage.at<Vec3b>(y,x)=origImg.at<Vec3b>(newY,newX);

//        printf("x:%d y:%d newX:%d newY:%d\n",x,y,newY,newY);
    }
}

int main( int argc, char** argv )
{
    if( argc != 2)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    Mat image;
    image = imread(argv[1]);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }


    Mat tr=createPerspectiveTraf(1,1,100.0,100.0,1,1,1.001,1.0002);
//    cout <<tr.inv() <<endl;

    Mat transformedImage=Mat::zeros(image.size(),image.type());
    transformImage(image,transformedImage,tr,true);

    Mat tr2=createPerspectiveTraf(0.0,0.0,400.0,400.0,1.0,1.0,0.0,0.0);
    transformImage(image,transformedImage,tr2,false);

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", transformedImage );                   // Show our image inside it.
    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}

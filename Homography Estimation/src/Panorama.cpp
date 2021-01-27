#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>

#include "MatrixReaderWriter.h"

using namespace cv;
using namespace std;


Mat calcHomography(vector<pair<Point2f, Point2f> > pointPairs) {
    const int ptsNum = pointPairs.size();
    Mat A(2 * ptsNum, 9, CV_32F);
    for (int i = 0; i < ptsNum; i++) {
        float u1 = pointPairs[i].first.x;
        float v1 = pointPairs[i].first.y;

        float u2 = pointPairs[i].second.x;
        float v2 = pointPairs[i].second.y;

        A.at<float>(2 * i, 0) = u1;
        A.at<float>(2 * i, 1) = v1;
        A.at<float>(2 * i, 2) = 1.0f;
        A.at<float>(2 * i, 3) = 0.0f;
        A.at<float>(2 * i, 4) = 0.0f;
        A.at<float>(2 * i, 5) = 0.0f;
        A.at<float>(2 * i, 6) = -u2 * u1;
        A.at<float>(2 * i, 7) = -u2 * v1;
        A.at<float>(2 * i, 8) = -u2;

        A.at<float>(2 * i + 1, 0) = 0.0f;
        A.at<float>(2 * i + 1, 1) = 0.0f;
        A.at<float>(2 * i + 1, 2) = 0.0f;
        A.at<float>(2 * i + 1, 3) = u1;
        A.at<float>(2 * i + 1, 4) = v1;
        A.at<float>(2 * i + 1, 5) = 1.0f;
        A.at<float>(2 * i + 1, 6) = -v2 * u1;
        A.at<float>(2 * i + 1, 7) = -v2 * v1;
        A.at<float>(2 * i + 1, 8) = -v2;

    }

    Mat eVecs(9, 9, CV_32F), eVals(9, 9, CV_32F);//Create eigen Vectors,Eigen Values
    cout << A << endl;
    eigen(A.t() * A, eVals, eVecs);//optimal solution is the eigenvector of A.t*A corresponding to the smallest eigenvalue
    //EVals list in descending order, in the last Evec we have the last coressponding Evalues

    cout << eVals << endl;
    cout << eVecs << endl;


    Mat H(3, 3, CV_32F);
    for (int i = 0; i < 9; i++) H.at<float>(i / 3, i % 3) = eVecs.at<float>(8, i);//last evecs,evec with index 8

    cout << H << endl;

    //Normalize:
    H = H * (1.0 / H.at<float>(2, 2));//Setting the scale as the bottom write order
    cout << H << endl;

    return H;
}

Mat normTransform(vector<Point2f> points) {
    Point2f mean(0.0f, 0.0f);
    for (const auto& p : points)
        mean += p;
    mean /= static_cast<float>(points.size());
    Point2f variance(0.0f, 0.0f);
    for (const auto& p : points) {
        variance.x += (p.x - mean.x) * (p.x - mean.x);
        variance.y += (p.y - mean.y) * (p.y - mean.y);
    }
    variance /= static_cast<float>(points.size());
    Point2f std(sqrtf(variance.x), sqrtf(variance.y));

    Mat translate = Mat::eye(3, 3, CV_32F);
    translate.at<float>(0, 2) = -mean.x;
    translate.at<float>(1, 2) = -mean.y;

    Mat scale = Mat::eye(3, 3, CV_32F);
    scale.at<float>(0, 0) = sqrtf(2) / std.x;
    scale.at<float>(1, 1) = sqrtf(2) / std.y;

    return scale * translate;
}

Mat hommTransform(vector<Point2f> first, vector<Point2f> second) {
    cv::Mat A(0, 9, CV_32F);
    for (int i = 0; i < first.size(); i++) {
        auto& p1 = first[i];
        auto& p2 = second[i];
        Mat row = (Mat_<float>(2, 9) << p1.x, p1.y, 1, 0, 0, 0, -p1.x * p2.x, -p1.y * p2.x, -p2.x,
            0, 0, 0, p1.x, p1.y, 1, -p1.x * p2.y, -p1.y * p2.y, -p2.y);
        vconcat(A, row, A);
    }
    Mat eigenvalues, eigenvector;
    eigen(A.t() * A, eigenvalues, eigenvector);
    Mat H(3, 3, CV_32F);
    for (int i = 0;i < 9;i++)
        H.at<float>(i / 3, i % 3) = eigenvector.at<float>(8, i);
    H *= 1.0 / H.at<float>(2, 2);
    return H;
}


//Tranformation of images

void transformImage(Mat origImg, Mat& newImage, Mat tr, bool isPerspective) {
    Mat invTr = tr.inv();
    const int WIDTH = origImg.cols;
    const int HEIGHT = origImg.rows;

    const int newWIDTH = newImage.cols;
    const int newHEIGHT = newImage.rows;

    //Pespectivity is important for homography

    for (int x = 0; x < newWIDTH; x++) for (int y = 0; y < newHEIGHT; y++) {
        Mat pt(3, 1, CV_32F);
        pt.at<float>(0, 0) = x;
        pt.at<float>(1, 0) = y;
        pt.at<float>(2, 0) = 1.0;

        Mat ptTransformed = invTr * pt;
        //        cout <<pt <<endl;
        //        cout <<invTr <<endl;
        //        cout <<ptTransformed <<endl;
        if (isPerspective) ptTransformed = (1.0 / ptTransformed.at<float>(2, 0)) * ptTransformed;

        int newX = round(ptTransformed.at<float>(0, 0));
        int newY = round(ptTransformed.at<float>(1, 0));

        if ((newX >= 0) && (newX < WIDTH) && (newY >= 0) && (newY < HEIGHT)) newImage.at<Vec3b>(y, x) = origImg.at<Vec3b>(newY, newX);

        //        printf("x:%d y:%d newX:%d newY:%d\n",x,y,newY,newY);
    }
}

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        cout << " Usage: point_file img1 img2" << endl;
        return -1;
    }//3arguments + name of the project

    MatrixReaderWriter mtxrw(argv[1]);

    if ((mtxrw.rowNum != 4) || (mtxrw.columnNum == 0))
    {
        cout << "Point file format error" << std::endl;
        return -1;
    }


    int r = mtxrw.rowNum;
    int c = mtxrw.columnNum;

    //Convert the coordinates:
    vector<pair<Point2f, Point2f> > pointPairs;
    for (int i = 0; i < mtxrw.columnNum; i++) {
        pair<Point2f, Point2f> currPts;
        currPts.first = Point2f((float)mtxrw.data[i], (float)mtxrw.data[c + i]);//u, v coordinates, for first photo
        currPts.second = Point2f((float)mtxrw.data[2 * c + i], (float)mtxrw.data[3 * c + i]);//for second photo
        pointPairs.push_back(currPts);
    }

    Mat H = calcHomography(pointPairs);

    //Create Resulting image
    Mat image1;
    image1 = imread(argv[2]);   // Read the file

    if (!image1.data)                              // Check for invalid input
    {
        cout << "Could not open or find the image" << argv[2] << std::endl;
        return -1;
    }

    Mat image2;
    image2 = imread(argv[3]);   // Read the file

    if (!image2.data)                              // Check for invalid input
    {
        cout << "Could not open or find the image" << argv[3] << std::endl;
        return -1;
    }


    //    cout <<tr.inv() <<endl;

    Mat transformedImage = Mat::zeros(1.5 * image1.size().height, 2.0 * image1.size().width, image1.type());//Size of resulting image is bigger of course than the 1st image
    transformImage(image2, transformedImage, Mat::eye(3, 3, CV_32F), true);

    //    transformImage(image2,transformedImage,tr2,false);
    transformImage(image1, transformedImage, H, true);

    imwrite("res.png", transformedImage);

    namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
    imshow("Display window", transformedImage);                   // Show our image inside it.
    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
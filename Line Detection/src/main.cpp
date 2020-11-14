#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include "main.h"

using namespace std;
using namespace cv;

// Apply RANSAC to fit points to a 2D line
void FitLineRANSAC(
	vector<Point> points_,
	double threshold_,
	int maximum_iteration_number_,
	Mat image_)
{
	srand(time(NULL));
	// The current number of iterations
	int iterationNumber = 0;
	// The indices of the inliers of the current best model
	vector<int> inliers;
	inliers.reserve(points_.size());
	// The sample size, i.e., 2 for 2D lines
	constexpr int kSampleSize = 2;
	std::vector<int> sample(kSampleSize);
	bool shouldDraw = image_.data != nullptr;
	// RANSAC:
	// 1. Select a minimal sample, i.e., in this case, 2 random points.
	// 2. Fit a line to the points.
	// 3. Count the number of inliers, i.e., the points closer than the threshold.
	// 4. Store the inlier number and the line parameters.
	while (iterationNumber++ < maximum_iteration_number_)
	{
		// 1. Select a minimal sample, i.e., in this case, 2 random points.
		for (size_t sampleIdx = 0; sampleIdx < kSampleSize; ++sampleIdx)
		{
			do
			{	// Generate a random index between [0, pointNumber]
				sample[sampleIdx] =
					round((points_.size() - 1) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX));
				// If the first point is selected we don't have to check if
				// that particular index had already been selected.
				if (sampleIdx == 0) 
					break;
				// If the second point is being generated,
				// it should be checked if the index had been selected beforehand. 
				if (sampleIdx == 1 &&
					sample[0] != sample[1])
					break;
			} while (true);
		}
		// 2. Fit a line to the points.
		const Point2d& p1 = points_[sample[0]]; // First point selected
		const Point2d& p2 = points_[sample[1]]; // Second point select		
		Point2d v = p2 - p1; // Direction of the line
		v = v / cv::norm(v);
		// Rotate v by 90° to get n.
		Point2d n;
		n.x = -v.y;
		n.y = v.x;
		// To get c use a point from the line.
		long double a = n.x;
		long double b = n.y;
		long double c = -(a * p1.x + b * p1.y);

		// 3. Count the number of inliers, i.e., the points closer than the threshold.
		inliers.clear();
		for (size_t pointIdx = 0; pointIdx < points_.size(); ++pointIdx)
		{
			const Point2d& point = points_[pointIdx];
			const long double distance =
				abs(a * point.x + b * point.y + c);

			if (distance < threshold_)
			{
				inliers.emplace_back(pointIdx);
			}
		}
		// 4. Store the inlier number and the line parameters 
		if (inliers.size() > 200 && shouldDraw)
		{
			size_t pointIdx = inliers.size() -1;
 			while (pointIdx > 0)
			{
				points_.erase(points_.begin() + inliers[pointIdx]);
				--pointIdx;
			}
			inliers.clear();
			inliers.resize(0);
			cv::line(image_,
				p1,
				p2,
				cv::Scalar(0, 0, 255),
				1);
		}
	}
}



int main()
{
	Mat img = imread("C:/Users/haris/source/repos/LineDetection/data/right.jpg", 1);

	if (!img.data) {
		std::cerr << "No image data" << endl;
		return EXIT_FAILURE;
	}
	Mat bin;
	Mat edge_blur;
	Mat blur_bin;
	cvtColor(img, bin, CV_BGR2GRAY);
	
	 
	GaussianBlur(bin, blur_bin, Size(3,3), 0);
	Canny(blur_bin, edge_blur, 150, 200, 3);
	Mat image = Mat::zeros(edge_blur.rows, edge_blur.cols, CV_8UC3);
	vector<Point> points;
	findNonZero(edge_blur, points);
	FitLineRANSAC(points, 1, 100000, img);

	imshow("Grey blurred image", blur_bin);
	waitKey(0);
	imshow("Edge blurred image", edge_blur);
	waitKey(0);
	imshow("Result Line Fitting", img);
	waitKey(0);
	
	return 0;
}
 
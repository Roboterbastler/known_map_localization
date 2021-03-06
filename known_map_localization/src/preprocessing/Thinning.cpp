/*
 * Thinning.cpp
 *
 *  Created on: 24.10.2016
 *      Author: jacob
 */

#include <opencv2/imgproc/imgproc.hpp>

/**
 * # Thinning algorithms
 *
 * Implementations of two thinning algorithms by Nashruddin Amin.
 *
 * Code from:
 * http://opencv-code.com/quick-tips/implementation-of-guo-hall-thinning-algorithm/
 * http://opencv-code.com/quick-tips/implementation-of-thinning-algorithm-in-opencv/
 */

namespace kml {

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * @param  im    Binary image with range = 0-1
 * @param  iter  0=even, 1=odd
 */
void thinningGuoHallIteration(cv::Mat& im, int iter) {
	cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

	for (int i = 1; i < im.rows; i++) {
		for (int j = 1; j < im.cols; j++) {
			uchar p2 = im.at<uchar>(i - 1, j);
			uchar p3 = im.at<uchar>(i - 1, j + 1);
			uchar p4 = im.at<uchar>(i, j + 1);
			uchar p5 = im.at<uchar>(i + 1, j + 1);
			uchar p6 = im.at<uchar>(i + 1, j);
			uchar p7 = im.at<uchar>(i + 1, j - 1);
			uchar p8 = im.at<uchar>(i, j - 1);
			uchar p9 = im.at<uchar>(i - 1, j - 1);

			int C = (!p2 & (p3 | p4)) + (!p4 & (p5 | p6)) + (!p6 & (p7 | p8))
					+ (!p8 & (p9 | p2));
			int N1 = (p9 | p2) + (p3 | p4) + (p5 | p6) + (p7 | p8);
			int N2 = (p2 | p3) + (p4 | p5) + (p6 | p7) + (p8 | p9);
			int N = N1 < N2 ? N1 : N2;
			int m = iter == 0 ? ((p6 | p7 | !p9) & p8) : ((p2 | p3 | !p5) & p4);

			if (C == 1 && (N >= 2 && N <= 3) & m == 0)
				marker.at<uchar>(i, j) = 1;
		}
	}

	im &= ~marker;
}

void thinningGuoHall(cv::Mat& im) {
	im /= 255;

	cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
	cv::Mat diff;

	do {
		thinningGuoHallIteration(im, 0);
		thinningGuoHallIteration(im, 1);
		cv::absdiff(im, prev, diff);
		im.copyTo(prev);
	} while (cv::countNonZero(diff) > 0);

	im *= 255;
}

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * @param  im    Binary image with range = 0-1
 * @param  iter  0=even, 1=odd
 */
void thinningZhangSuenIteration(cv::Mat& im, int iter) {
	cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

	for (int i = 1; i < im.rows - 1; i++) {
		for (int j = 1; j < im.cols - 1; j++) {
			uchar p2 = im.at<uchar>(i - 1, j);
			uchar p3 = im.at<uchar>(i - 1, j + 1);
			uchar p4 = im.at<uchar>(i, j + 1);
			uchar p5 = im.at<uchar>(i + 1, j + 1);
			uchar p6 = im.at<uchar>(i + 1, j);
			uchar p7 = im.at<uchar>(i + 1, j - 1);
			uchar p8 = im.at<uchar>(i, j - 1);
			uchar p9 = im.at<uchar>(i - 1, j - 1);

			int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1)
					+ (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1)
					+ (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1)
					+ (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
			int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
			int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
			int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				marker.at<uchar>(i, j) = 1;
		}
	}

	im &= ~marker;
}

void thinningZhangSuen(cv::Mat& im) {
	im /= 255;

	cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
	cv::Mat diff;

	do {
		thinningZhangSuenIteration(im, 0);
		thinningZhangSuenIteration(im, 1);
		cv::absdiff(im, prev, diff);
		im.copyTo(prev);
	} while (cv::countNonZero(diff) > 0);

	im *= 255;
}

}

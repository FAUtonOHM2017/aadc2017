/**
 * Copyright (c)
 * Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
 * 4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************
 * $Author:: schoen $   $Date:: 2015-12-28 #$
 **********************************************************************/

#include "stdafx.h"
#include "CameraEstimator.h"

#include <cmath>

const float CameraEstimator::READJUSTMENT_RATE = 0.1;
const int CameraEstimator::ADJUSTMENT_ITERATIONS_FOR_FINE_FIX = 60;

const float CameraEstimator::MIN_PITCH = -20;
const float CameraEstimator::MAX_PITCH = 0;

/**
 * use camera position x = 0.22m
 * average camera pitch = -10°
 * so => alpha_AP = -10°
 * z = 0.1273m * alpha + 2.52m
 * z = 25.46px * alpha + 504px
 * for 0.03° z changes about 0.76 pixel
 */
const float CameraEstimator::MIN_CHANGEABLE_PITCH_DIFFERENCE = 0.03;

CameraEstimator::CameraEstimator() {
	Init();
}

CameraEstimator::~CameraEstimator() {
}

tVoid CameraEstimator::Init() {
	binaryImageSize = Rect(0, 0, 300, 300);
	stopLine = Mat::zeros(30, 10, CV_8UC1);
	rectangle(stopLine, Point(0,10), Point(10, 19), Scalar(255), CV_FILLED);

	yDiff = 0;
	numberOfEstimates_yDiff = 0;

	currentPitch = 0;
	lastPitch = 0;
	failCount = 0;

	adjustmentIteration_2 = 0;
}

tVoid CameraEstimator::DrawHoughLine(cv::Mat& image, tFloat32 rho, tFloat32 theta) {
	if (theta < 0) {
		theta += 180;
		if (rho > 0) {
			rho = -rho;
		}
	}

	theta = theta * CV_PI / 180;

	Point pt1, pt2;
	tFloat32 a = cos(theta), b = sin(theta);
	tFloat32 x0 = a * rho, y0 = b * rho;
	pt1.x = cvRound(x0 + 1000 * (-b));
	pt1.y = cvRound(y0 + 1000 * (a));
	pt2.x = cvRound(x0 - 1000 * (-b));
	pt2.y = cvRound(y0 - 1000 * (a));
	line(image, pt1, pt2, Scalar(0, 255, 0), 2, CV_AA);
}

tBool CameraEstimator::DetectPitch(cv::Mat& in, cv::Mat &debugImage) {
	double rightTheta = 0;
	double rightWeight = 0;
	foundLines.clear();

	Rect rect(150, 0, 100, 300);
	Mat image = in(rect);
	HoughLinesP(image, lines, 1, CV_PI / 180 /2, 10, 30, 5);

	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];

		Point2f p0(l[0] + rect.x, l[1] + rect.y);
		Point2f p1(l[2] + rect.x, l[3] + rect.y);

		l[0] = p0.x;
		l[1] = p0.y;
		l[2] = p1.x;
		l[3] = p1.y;

		Point2d temp;
		if (p1.y > p0.y) {
			temp = p0 - p1;
		} else {
			temp = p1 - p0;
		}

		double currentWeight = norm(temp);
		double currentTheta = atan(temp.x / abs(temp.y)) * 180 / CV_PI;

		if (currentTheta < -15 || currentTheta > 15) {
			continue;
		}

		rightWeight += currentWeight;
		rightTheta += currentTheta * currentWeight;

		foundLines.push_back(l);
	}

	if (rightWeight > 0) {
		rightTheta /= rightWeight;
	}

	double leftWeight = 0;
	double leftTheta = 0;
	rect = Rect(50, 0, 100, 300);
	image = in(rect);
	HoughLinesP(image, lines, 1, CV_PI / 180 / 2, 10, 20, 5);

	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];

		Point2f p0(l[0] + rect.x, l[1] + rect.y);
		Point2f p1(l[2] + rect.x, l[3] + rect.y);

		l[0] = p0.x;
		l[1] = p0.y;
		l[2] = p1.x;
		l[3] = p1.y;

		Point2d temp;
		if (p1.y > p0.y) {
			temp = p0 - p1;
		} else {
			temp = p1 - p0;
		}

		double currentWeight = norm(temp);
		double currentTheta = atan(temp.x / abs(temp.y)) * 180 / CV_PI;

		if (currentTheta < -15 || currentTheta > 15) {
			continue;
		}

		leftWeight += currentWeight;
		leftTheta += currentTheta * currentWeight;

		foundLines.push_back(l);
	}

	if (leftWeight > 0) {
		leftTheta /= leftWeight;
	}

	lastPitch = currentPitch;

	if (leftWeight > 50 && rightWeight > 100) {
		currentPitch += (leftTheta - rightTheta) / 2;

		failCount--;
	} else if(leftWeight + rightWeight < 100) {
		failCount++;
	}

	if (abs(lastPitch - currentPitch) > MIN_CHANGEABLE_PITCH_DIFFERENCE) {
		currentPitch = LimitPitch(currentPitch);

		lastPitch = currentPitch;
		return tTrue;
	} else {
		currentPitch = lastPitch;
	}

	return tFalse;
}

tBool CameraEstimator::DetectPitch2(cv::Mat& in, cv::Mat &debugImage) {
	adjustmentIteration_2++;

	double rightTheta = 0;
	double rightWeight = 0;
	foundLines.clear();

	Rect rect(170, 100, 60, 200);
	Mat image = in(rect);
	HoughLinesP(image, lines, 1, CV_PI / 180 / 2, 10, 40, 5);

	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];

		Point2f p0(l[0] + rect.x, l[1] + rect.y);
		Point2f p1(l[2] + rect.x, l[3] + rect.y);

		l[0] = p0.x;
		l[1] = p0.y;
		l[2] = p1.x;
		l[3] = p1.y;

		Point2d temp;
		if (p1.y > p0.y) {
			temp = p0 - p1;
		} else {
			temp = p1 - p0;
		}

		double currentWeight = norm(temp);
		double currentTheta = atan(temp.x / abs(temp.y)) * 180 / CV_PI;

		if (currentTheta < -5 || currentTheta > 5) {
			continue;
		}

		rightWeight += currentWeight;
		rightTheta += currentTheta * currentWeight;

		foundLines.push_back(l);
	}

	if (rightWeight > 0) {
		rightTheta /= rightWeight;
	}

	double leftWeight = 0;
	double leftTheta = 0;
	rect = Rect(70, 100, 60, 200);
	image = in(rect);
	HoughLinesP(image, lines, 1, CV_PI / 180 / 2, 10, 20, 5);

	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];

		Point2f p0(l[0] + rect.x, l[1] + rect.y);
		Point2f p1(l[2] + rect.x, l[3] + rect.y);

		l[0] = p0.x;
		l[1] = p0.y;
		l[2] = p1.x;
		l[3] = p1.y;

		Point2d temp;
		if (p1.y > p0.y) {
			temp = p0 - p1;
		} else {
			temp = p1 - p0;
		}

		double currentWeight = norm(temp);
		double currentTheta = atan(temp.x / abs(temp.y)) * 180 / CV_PI;

		if (currentTheta < -5 || currentTheta > 5) {
			continue;
		}

		leftWeight += currentWeight;
		leftTheta += currentTheta * currentWeight;

		foundLines.push_back(l);
	}

	if (leftWeight > 0) {
		leftTheta /= leftWeight;
	}

	if (leftWeight > 10 && rightWeight > 100) {

		failCount--;
		if (adjustmentIteration_2 > ADJUSTMENT_ITERATIONS_FOR_FINE_FIX) {
			currentPitch += READJUSTMENT_RATE * (leftTheta - rightTheta) / 2;
		} else {
			currentPitch += (leftTheta - rightTheta) / 2;
		}
	}

	if (abs(lastPitch - currentPitch) > MIN_CHANGEABLE_PITCH_DIFFERENCE) {
		currentPitch = LimitPitch(currentPitch);

		lastPitch = currentPitch;

		return tTrue;
	} else {
		currentPitch = lastPitch;
	}

	return tFalse;
}

tBool CameraEstimator::DetectPitch3_StopLine(cv::Mat &in, cv::Mat &debugImage, cv::Point2f imageStopPosition) {
	foundLines.clear();

	Mat result;
	double maxVal = 0, minVal = 0;
	Point minLoc, maxLoc;
	int yCoord = 0;
	int count = 0;

	for (int i = 0; i < 60; i+=10) {
		Rect rect(imageStopPosition + Point2f(-30 + i, -50), Size(10, 100));
		rect &= binaryImageSize;
		Mat image = in(rect);
		matchTemplate(image, stopLine, result, CV_TM_SQDIFF);
		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		if(minVal < 10 * 2 * 255 * 255) {
			yCoord += minLoc.y + rect.y;
			count++;
			circle(debugImage, minLoc + Point(rect.x + 5, rect.y + 15), 3.0, Scalar(0, 255, 0), 1, 8);
		}

		rectangle(debugImage, minLoc + Point(rect.x, rect.y), minLoc + Point(rect.x + 10, rect.y + 30), Scalar(0,0,255), 1, 8, 0);
	}

	float estimatedYCoord = 0;
	if (count > 2) {
		estimatedYCoord = yCoord / count + 10;
	} else {
		return tFalse;
	}

	line(debugImage, Point(0, yCoord), Point(300, yCoord), Scalar(255, 0, 0), 2);

	//alpha_AP = -10°
	//z = 0.1273m * alpha + 2.52m
	//z = 25.46px * alpha + 504px
	int yDiff = imageStopPosition.y - estimatedYCoord;

	//set some limits
	if (yDiff < -20) {
		yDiff = -20;
	} else if (yDiff > 20) {
		yDiff = 20;
	}

	this->yDiff += yDiff;
	numberOfEstimates_yDiff++;

	return tTrue;
	
	/*
	currentPitch -= yDiff / 25.46;

	if(abs(currentPitch - lastPitch) > MIN_CHANGEABLE_PITCH_DIFFERENCE) {
		currentPitch = LimitPitch(currentPitch);

		lastPitch = currentPitch;

		return tTrue;
	}

	currentPitch = lastPitch;

	return tFalse;
	*/
}

tFloat32 CameraEstimator::LimitPitch(tFloat32 pitch) {
	if (pitch > MAX_PITCH) {
		return MAX_PITCH;
	} else if (pitch < MIN_PITCH) {
		return MIN_PITCH;
	} else {
		return pitch;
	}
}

tVoid CameraEstimator::DrawDebugData(cv::Mat& debugImage) {
	for (size_t i = 0; i < foundLines.size(); i++) {
		Vec4i l = foundLines[i];

		Point2f p0(l[0], l[1]);
		Point2f p1(l[2], l[3]);

		line(debugImage, p0, p1, Scalar(255, 0, 0), 2);
	}
}



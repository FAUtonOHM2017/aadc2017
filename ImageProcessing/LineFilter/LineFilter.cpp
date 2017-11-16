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
 * $Author:: schoen $   $Date:: 2015-12-03 #$
 **********************************************************************/

#include "stdafx.h"
#include "LineFilter.h"
#include "ImageProcessingUtils.h"
#include "CVMath.h"
#include <sstream>
#include <iostream>

using namespace cv;

const float LineFilter::SIZE_X = 1.5;
const float LineFilter::SIZE_Y = 1.5;

LineFilter::LineFilter() {
	//Some default parameters
	this->_near = 0.7;
	this->_far = 2.2;

	this->_pitch = 0;
	this->_yaw = 0;
	this->_roll = 0;
	this->_cameraOffsetX = -0.015;
	this->_cameraOffsetZ = 0.22;

	this->_cameraWidth = 1280;
	this->_cameraHeight = 720;

	_adaptiveThresholdConstant = 21;
	_enableAdaptiveThresholding = true;

	_maskWorkRegion = 0;

	_enableMedianFiltering = false;
	_isInitialized = false;
}

LineFilter::~LineFilter() {
}

void LineFilter::Init(float near, cv::Mat intrinsics, cv::Mat distorsion, float pitch, float cameraPositionX,
		float cameraPositionZ, float adaptiveThresholdConstant, int adaptiveThresholdWidth, int cameraWidth, int cameraHeight) {
	this->_near = near;
	this->_far = near + SIZE_Y;

	_intrinsics = intrinsics;
	_distorsion = distorsion;

	this->_pitch = pitch;
	this->_cameraOffsetX = cameraPositionX;
	this->_cameraOffsetZ = cameraPositionZ;

	this->_cameraWidth = cameraWidth;
	this->_cameraHeight = cameraHeight;

	this->_adaptiveThresholdConstant = adaptiveThresholdConstant;
	this->_adaptiveThresholdWidth = adaptiveThresholdWidth;
	this->_enableAdaptiveThresholding = true;

	_maskWorkRegion = 0;

	_gray = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));
	_validPixels = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));
	_warp = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC3, Scalar(0, 0, 0));
	_affineMask = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));

	_isInitialized = true;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::EnableAdaptiveThresholding(bool enable) {
	this->_enableAdaptiveThresholding = enable;
}

void LineFilter::EnableMedianFiltering(bool enable) {
	this->_enableMedianFiltering = enable;
}

void LineFilter::SetNearFar(float near) {
	this->_near = near;
	this->_far = near + SIZE_Y;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetRPY(float roll, float pitch, float yaw) {
	this->_roll = roll;
	this->_pitch = pitch;
	this->_yaw = yaw;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetCameraPosition(float cameraPositionX, float cameraPositionZ) {
	this->_cameraOffsetX = cameraPositionX;
	this->_cameraOffsetZ = cameraPositionZ;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetCameraSize(int cameraWidth, int cameraHeight) {
	this->_cameraWidth = cameraWidth;
	this->_cameraHeight = cameraHeight;

	CalculatePerspectiveTransformationMatrix();
}

void LineFilter::SetAdaptiveThresholdConstant(float adaptiveThresholdConstant) {
	this->_adaptiveThresholdConstant = adaptiveThresholdConstant;
}

void LineFilter::SetValidPixels(cv::Mat &mask, Size imageSize, bool enable) {
	if(enable) {
		int cropWidth = 20;
		int maskWidth = mask.cols-cropWidth;
		int outHeight = (imageSize.height >> 1) - _maskWorkRegion;
		//int outWidth = 0.95 * maskWidth;//measured 0.95
		int outWidth = (imageSize.width >> 1);

		mask(Rect(0, _maskWorkRegion, mask.cols, mask.rows - _maskWorkRegion)).copyTo(_validPixels);


		Mat element = getStructuringElement( MORPH_RECT, Size( 4, 3));
		cv::morphologyEx(_validPixels, _validPixels, cv::MORPH_CLOSE, element);
		cv::erode(_validPixels, _validPixels, element);

		if(!_lastMask.empty()) {
			bitwise_or(_validPixels, _lastMask, _buffer);
			if(!_lastlastMask.empty()) {
				bitwise_or(_buffer, _lastlastMask, _buffer);
			}

			//imwrite("/tmp/buffer.jpg", _buffer);

			resize(_buffer(Rect(0, 0, maskWidth, _buffer.rows)), _buffer, Size(outWidth, outHeight));

			_lastMask.copyTo(_lastlastMask);
//			imwrite("/tmp/_lastMask.jpg", _lastMask);
//			imwrite("/tmp/_lastlastMask.jpg", _lastlastMask);
		} else {
			resize(_validPixels(Rect(0, 0, maskWidth, _validPixels.rows)), _buffer, Size(outWidth, outHeight));
		}

		_validPixels.copyTo(_lastMask);

		if(outWidth != (imageSize.width >> 1)) {
			copyMakeBorder(_buffer, _buffer, 0, 0, 0, (imageSize.width >> 1) - outWidth, BORDER_REPLICATE);
		}

		if(0) {
			//Mat element = getStructuringElement(MORPH_RECT, Size( 12, 2));
			//cv::erode(buffer, validPixels, element, Point(-1,-1), 1);

			blur(_buffer, _validPixels, Size(15, 15));
			//imwrite("/tmp/maske.jpg", _validPixels);
		} else {
			_buffer.copyTo(_validPixels);
		}

		if(!_lastValidPixels.empty()){
			multiply(_validPixels, _lastValidPixels, _validPixels, 1.0/255);
		}

		if(1) {
			cv::dilate(_validPixels, _lastValidPixels, element, Point(-1,-1), 1);
		}

		copyMakeBorder(_validPixels, _validPixels, _maskWorkRegion, 0, 0, 0, BORDER_CONSTANT);
//		imwrite("/tmp/border.jpg", _validPixels);
		resize(_validPixels, _validPixels, imageSize);

		//imwrite("/tmp/final.jpg", _validPixels);
	} else {
		_validPixels = Mat::ones(imageSize, CV_8UC1) * 255;
	}
}

tResult LineFilter::GetTopDownView(Mat &in, Mat& out, bool use_CV_rgb2gray) {
	if(!_isInitialized) {
		RETURN_AND_LOG_ERROR(0);
	}
	//imwrite("/tmp/in.jpg", in);

	if(use_CV_rgb2gray) {
		cvtColor(in,in,COLOR_RGB2GRAY);
	} else {
		cvtColor(in,in,COLOR_BGR2GRAY);
	}

	//undistort first for better results
	//cv::undistort(in, _buffer, _intrinsics, _distorsion);
	//multiply(_buffer, _validPixels, _buffer, 1.0/255);
	multiply(in, _validPixels, _buffer, 1.0/255);

	if(1) {
		if(_enableAdaptiveThresholding) {
			warpPerspective(_buffer, _warp, _topDownMatrix, _warp.size(), INTER_LINEAR, BORDER_REPLICATE);
			adaptiveThreshold(_warp, _gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, _adaptiveThresholdWidth, _adaptiveThresholdConstant);
			_gray.copyTo(out, _affineMask);
		} else {
			warpPerspective(_buffer, _warp, _topDownMatrix, _warp.size(), INTER_LINEAR, BORDER_REPLICATE);
			//warp = warp - 32;
			//out.setTo(128);
			_warp.copyTo(out, _affineMask);
		}
	} else {
		warpPerspective(_validPixels, _warp, _topDownMatrix, _warp.size(), INTER_LINEAR, BORDER_REPLICATE);
		_warp.copyTo(out, _affineMask);
	}

	if(_enableMedianFiltering) {
		medianBlur(out, out, 3);
	}

	RETURN_NOERROR;
}

Point2f LineFilter::GetTopLeftWorldCoordinates() {
	return Point2f(_far, SIZE_X * 0.5);
}

void LineFilter::CalculatePerspectiveTransformationMatrix() {
	Point2f sourcePoints[4] = { Point2f(0, 0), Point2f(0, 0), Point2f(0, 0), Point2f(0, 0) };
	CalculateSourcePoints(sourcePoints);

	_maskWorkRegion = (sourcePoints[0].y) / 2;
	if(_maskWorkRegion < 0) {
		_maskWorkRegion = 0;
	}

	Point2f destPoints[4] = { Point2f(100, 0), Point2f(200, 0), Point2f(200, 300), Point2f(100, 300) };
	_topDownMatrix = getPerspectiveTransform(sourcePoints, destPoints);

	try {
		Mat white = Mat(this->_cameraHeight, this->_cameraWidth, CV_8UC1, Scalar(255));
		warpPerspective(white, _affineMask, _topDownMatrix, _affineMask.size(), INTER_LINEAR, BORDER_CONSTANT, 0);

	} catch (cv::Exception &e) {
		std::cout << e.msg << std::endl;
	}
}


void LineFilter::CalculateSourcePoints(cv::Point2f* sourcePoints) {
	if(!_isInitialized) {
		return;
	}
	float _fovVertical = 43;
	float _fovHorizontal = 70;

	float vertical = tan(_fovVertical / 180 * CV_PI / 2);
	float horizontal = tan(_fovHorizontal / 180 * CV_PI / 2);

	//need 4 points for transformation
	//field with 1m width and 1.5 m length
	//one column is a 3d point

	cv::Mat points = (Mat_<float>(3,4) <<
			//top left				top right				bottom right			bottom left
			-0.25 - _cameraOffsetX,	0.25 - _cameraOffsetX, 	0.25 - _cameraOffsetX, -0.25 - _cameraOffsetX,
			_cameraOffsetZ,         _cameraOffsetZ,         _cameraOffsetZ,        _cameraOffsetZ,
			_far,                   _far,                   _near,                 _near);

	cv::Mat rVec = (Mat_<float>(1,3) <<  -_pitch, -_yaw, -_roll);

	cv::Mat R;
	cv::Rodrigues(rVec, R);

	//rotate to fit camera rotation
	points = R * points;

	for (int i = 0; i < 4; ++i) {
		Point2f point;

		// camera_width       vectorFarLeft.x * (camera_width / 2)
		// ------------  +   -------------------------------------
		//       2                       (far * horizontal)

		//get corresponding pixel to real world coordinate
		//todo: use camera matrix instead of fov


		//fx 0 cx
		//0 fy cy
		//0 0 0 1

		LOG_WARNING(cString::Format("worldpoint[%d] x, y, z: %f, %f, %f",i, points.at<float>(0, i), points.at<float>(1, i), points.at<float>(2, i) ));
		point.x = points.at<float>(0, i) / points.at<float>(2, i) * _intrinsics.at<double>(0,0) +  _intrinsics.at<double>(0,2);
		point.y = points.at<float>(1, i) / points.at<float>(2, i) * _intrinsics.at<double>(1,1) +  _intrinsics.at<double>(1,2);

		//point.x = (_cameraWidth / 2) + points.at<float>(0, i) * (_cameraWidth / 2) / (points.at<float>(2, i) * horizontal) ;
		//point.y = (_cameraHeight / 2) + points.at<float>(1, i) * (_cameraHeight / 2) / (points.at<float>(2, i) * vertical);
		sourcePoints[i] = point;
	}

	for (int i = 0; i < 4; ++i) {
		LOG_WARNING(cString::Format("SourcePoint[%d] x, y: %f, %f",i, sourcePoints[i].x, sourcePoints[i].y ));
	}
}

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

#ifndef CAMERAESTIMATOR_H_
#define CAMERAESTIMATOR_H_

class CameraEstimator {
	/**
	 * in pixel
	 */
	Rect binaryImageSize;

	/**
	 * templates for stop line matching
	 */
	Mat stopLine;

	/**
	 * error between estimated and real stop line distance
	 */
	float yDiff;

	int numberOfEstimates_yDiff;

	/**
	 * in degree
	 */
	float lastPitch;

	/**
	 * in degree
	 */
	float currentPitch;

	int failCount;

	int adjustmentIteration_2;

	//static const int ADJUSTMENT_ITERATIONS_FOR_FINE_FIX;

	static const float READJUSTMENT_RATE;
	static const int ADJUSTMENT_ITERATIONS_FOR_FINE_FIX;

	static const float MIN_CHANGEABLE_PITCH_DIFFERENCE;

	static const float MIN_PITCH;
	static const float MAX_PITCH;

	vector<Vec4i> lines;
	vector<Vec4i> foundLines;
public:
	CameraEstimator();
	tVoid Init();

	virtual ~CameraEstimator();

	tVoid SetCurrentPitch(tFloat32 value) {

		if(abs(currentPitch - value) > 0.001) {
			// last pitch angle was not transmitted?
			failCount++;
		}
		currentPitch = value;
	}

	tFloat32 GetCurrentYDiff() {
		if(numberOfEstimates_yDiff == 0) {
			return 0;
		}
		return yDiff / numberOfEstimates_yDiff;
	}

	tFloat32 GetCurrentPitch() {
		return currentPitch;
	}

	tInt32 GetFailCount() {
		return failCount;
	}

	tBool DetectPitch(cv::Mat &image, cv::Mat &debugImage);

	/**
	 *
	 * @param in
	 * @param debugImage
	 * @return true if pitch angle was changed
	 */
	tBool DetectPitch2(cv::Mat &image, cv::Mat &debugImage);

	tBool DetectPitch3_StopLine(cv::Mat &image, cv::Mat &debugImage, cv::Point2f imageStopPosition);

	inline tFloat32 LimitPitch(tFloat32 pitch);

	tVoid DrawHoughLine(cv::Mat &image, tFloat32 rho, tFloat32 theta);
	tVoid DrawDebugData(cv::Mat &debugImage);
};

#endif /* CAMERAESTIMATOR_H_ */

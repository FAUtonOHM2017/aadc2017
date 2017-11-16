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

#ifndef _LaneTracking_FILTER_HEADER_
#define _LaneTracking_FILTER_HEADER_

#define OID_ADTF_CameraEstimator  "adtf.aadc.camPitchEstimator"

#include "CameraEstimator.h"
#include "TSignalValue.h"
#include "TBoolSignalValue.h"

class CameraEstimatorAdapter: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_CameraEstimator, "CamPitch", OBJCAT_DataFilter, "CamPitch", 0, 2, 0, "FAUtonOHM");

protected:
	//Input of the RGB image
	cVideoPin videoInputPin;

	/**
	 * pin for pitch angle
	 */
	cOutputPin pitchOutput;

	cOutputPin failOutput;

	cOutputPin successOutput;

	/**
	 * colored debug information is plotted on this output image
	 */
	cVideoPin videoDebugOutputPin;

	/**
	 * filename where pitch angle and offset ist stored
	 */
	cFilename filePitch;

	TSignalValue tSignalValuePitch;

	TBoolSignalValue tBoolSignalValueFailed;
	TBoolSignalValue tBoolSignalValueSuccessful;

	// image processing

	/**
	 * indicates the first frame.
	 */
	tBool firstFrame;

	/**
	 * input video format
	 */
	tBitmapFormat inputFormat;

	CameraEstimator *cameraEstimator;
	tFloat32 maxAdjustmentTime_1;
	tFloat32 maxAdjustmentTime_2;
	tFloat32 maxAdjustmentTime_3;

	cv::Point2f stopLineMarker;

	tInt32 MAX_FAILED_DETECTIONS;
	tInt32 continueIfNoFix;
	tBool calibrationFailed;
	tBool calibrationSucceeded;

	tBool writeToFile;



	struct Sync {
		/**
		 * Synchronization of OnPinEvent
		 */
		cCriticalSection criticalSection_OnPinEvent;

		bool skipNextFrame;
		int bufferCount;
		bool log_warning;
		bool log_console;
		const char* name;

		Sync() {
			skipNextFrame = false;
			bufferCount = 0;
			log_warning = false;
			log_console = false;
			name = "CamPitch";
		}

		//helper
		inline void PrintCleared() {
			if(log_warning)
				LOG_WARNING(cString::Format("%s: cleared image queue", name));
			if(log_console)
				std::cout << name << ":cleared image queue" << std::endl;
		}

		inline void DecreaseBufferCount() {
			bufferCount--;
			if(log_warning)
				LOG_WARNING(cString::Format("%s: processed image buffer count = %d", name, bufferCount));

			if(log_console)
				std::cout << name << ": processed image buffer count = " << bufferCount << std::endl;

			if(bufferCount < 0) {
				bufferCount = 0;
				LOG_WARNING(cString::Format("%s: image queue buffer count < 0", name));
			}
		}

		inline void IncreaseBufferCount() {
			bufferCount++;
			if(log_warning)
				LOG_WARNING(cString::Format("%s: image queue buffer count = %d", name, bufferCount));

			if(log_console)
				std::cout << name << ": image queue buffer count = " << bufferCount << std::endl;
		}
	} sync;
public:
	CameraEstimatorAdapter(const tChar*);
	virtual ~CameraEstimatorAdapter();

	// implements cFilter
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

	tResult OnPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* mediaSample);
	tResult OnAsyncPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* mediaSample);

private:
	tResult ProcessInputFrame(IMediaSample* sample);

	tResult ConvertInputImage(cv::Mat &image, IMediaSample* mediaSample);

	tResult ReadAsFloatAndClear (cv::Mat &image, tUInt32 bytePosition, tFloat32 *value);

	tResult StoreToFile();
};

#endif 

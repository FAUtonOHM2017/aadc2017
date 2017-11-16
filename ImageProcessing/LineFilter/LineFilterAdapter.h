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

#ifndef _LaneTracking_FILTER_HEADER_
#define _LaneTracking_FILTER_HEADER_

#define OID_ADTF_LineFilter  "adtf.aadc.lineFilter"

#include "TSignalValue.h"
#include "TPoseStruct.h"
#include <deque>

class LineFilter;

class LineFilterAdapter: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LineFilter, "LineFilter", OBJCAT_DataFilter, "LineFilter", 0, 2, 0, "FAUtonOHM");

protected:
	//Input of the RGB image
	cVideoPin inputVideoPin;

	//Input of the valid image
	struct Mask {
		cVideoPin inputPin;
		cv::Mat image;
		bool firstFrame;
		tBitmapFormat dInputFormat;

		Mask() {
			firstFrame = true;
		}
	}mask;

	tFloat32 _pitch;
	tFloat32 _roll;

	cv::Mat beforeMasking;

	//output video
	cVideoPin outputVideoPin;

	/**
	 * pin for x,y and yaw angle of the car
	 */
	cInputPin poseInput;

	/**
	 * filename where pitch angle and offset is stored
	 */
	cFilename filePitch;

	/**
	 *  descriptor for tSignalValue data
	 */
	TSignalValue tSignalValue;

	/**
	 *  descriptor for tPoseStruct data
	 */
	TPoseStruct tPoseStruct;

	struct CarPose {
		cv::Point2f position;
		float yaw;
		float pitch;
		float roll;
		tTimeStamp adtfTime;
		bool estimated;

		CarPose () {
			this->yaw = 0;
			this->pitch = 0;
			this->roll = 0;
			this->adtfTime = 0;
			this->estimated = false;
		}

		CarPose (cv::Point2f pose, float yaw, float pitch, float roll, tTimeStamp adtfTime, bool estimated) {
			this->position = pose;
			this->yaw = yaw;
			this->pitch = pitch;
			this->roll = roll;
			this->adtfTime = adtfTime;
			this->estimated = estimated;
		}
	};

	std::deque<CarPose> carPose;
	tTimeStamp timeOfPrevFrame;

	/**
	 * currently used line filter
	 */
	LineFilter * lineFilter;

	struct Cache {
		/**
		 * local output buffer
		 */
		Mat out;

		/**
		 * local input buffer
		 */
		Mat image;
	} cache;

	/*! bitmap format of input pin */
	tBitmapFormat m_sInputFormat;


	struct Sync {
		/**
		 * Synchronization of OnPinEvent
		 */
		cCriticalSection criticalSection_OnPinEvent;

		int bufferCount;
		bool log_warning;
		bool log_console;
		const char* name;

		Sync() {
			bufferCount = 0;
			log_warning = false;
			log_console = false;
			name = "LineFilter";
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

	/**
	 * image error offset in meter of the transformed output picture.
	 */
	tFloat32 xEstimationErrorOffset;

	/**
	 * camera position measured from the rear axle
	 */
	tFloat32 xCameraOffset;

	tFloat32  nearPlane;

public:
	LineFilterAdapter(const tChar*);
	virtual ~LineFilterAdapter();

	// implements cFilter
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! This Function is always called when any property has changed. This should be the only place
    *    to read from the properties itself and store their values in a member.
    *
    *    \param [in] strName the name of the property that has changed.
    *    \
    *    \return   Returns a standard result code.
    */
    tResult PropertyChanged(const tChar* strName);

private:
	tResult ProcessVideo(IMediaSample* sample);

	tResult ConvertInputImageRGB(cv::Mat &image, IMediaSample* mediaSample);
	tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

	tResult ReadFromFile(tFloat32 *pitch, tFloat32 *yOffset);

	tResult WriteAsFloat(cv::Mat& image, tUInt32 bytePosition, tFloat32 value);


	bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);
};

#endif

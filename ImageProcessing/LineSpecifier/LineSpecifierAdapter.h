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
 * $Author:: schoen $   $Date:: 2016-03-11 #$ $Rev:: 0.2.0   $
 **********************************************************************/

#ifndef LINESPECIFIERADAPTER_H_
#define LINESPECIFIERADAPTER_H_

#define OID_ADTF_LINE_SPECIFIER "adtf.aadc.lineSpecifier"

class LineSpecifier;
class StopLineDetector;

#include "stdafx.h"
#include "TRoadSignStructExt.h"

using namespace adtf;

class PoseCache;

//*************************************************************************************************
class LineSpecifierAdapter: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LINE_SPECIFIER, "LineSpecifier", OBJCAT_DataFilter, "LineSpecifier", 0, 2, 0, "FAUtonOHM");

protected:
	//pins
	/**
	 * pin for a binary image
	 */
	cVideoPin _inputBinaryVideoPin;

	/**
	 * local input buffer
	 */
	cv::Mat image;
	tBool _onLeftLane;

	tBitmapFormat outputFormat;

	/**
	 * colored debug information is plotted on this output image
	 */
	cVideoPin videoDebugOutputPin;

	/**
	 * pin for a recommended steering angle
	 */
    cOutputPin steeringAngleOutput;

    /**
     * center of the stop line. x,y coordinates are written if stop line is detected
     */
    cOutputPin localGoalOutput;

	/**
	 * center of the stop line. x,y coordinates are written if stop line is detected
	 */
	cOutputPin speedOutput;

	/**
	 *
	 */
	cOutputPin nullOutput;

    /**
      * action pin
      */
    cInputPin actionInput;

    /**
      * feedback pin
      */
    cOutputPin feedbackOutput;

    /**
     * pin for road signs from filter RoadSign
     */
    cInputPin trafficSignInput;

	/**
	 * pin for x,y and yaw angle of the car
	 */
	cInputPin poseInput;

    /**
     *  descriptor for tSignalValue data
     */
    TSignalValue tSignalValueSteering;

    /**
     *  descriptor for tSignalValue data
     */
    TSignalValue tSignalValueSpeed;

    /**
     *  descriptor for tSignalValue data
     */
    TSignalValue tSignalValueNull;

    /**
      * descriptor for tActionStruct
      */
    TActionStruct tActionStruct;

    /**
      * descriptor for tActionStruct
      */
    TFeedbackStruct tFeedbackStruct;

    /**
      *  descriptor for TPoseStruct data
      */
    TPoseStruct tPoseStructGoal;

    /**
      *  descriptor for TPoseStruct data
      */
    TPoseStruct tPoseStructIn;

    /**
     *
     */
    TRoadSignStructExt _tTrafficSign;

    struct Offset {
		/**
		 * car position in the image frame. in Meter.
		 */
		cv::Point2f steering;

		cv::Point2f steeringLeft;

		/**
		 * stopline offset
		 */
		cv::Point2f stopline;
		tFloat32 yawScale;
		tFloat32 maxXYRation_Stopline;

		/**
		 * parking offset
		 */
		cv::Point2f parkingTrans;

		/**
		 * parking offset
		 */
		cv::Point2f parkingLong;

		cv::Point2f backwards;

		tFloat32 pullOutXOffset;

		tFloat32 stopTransmissionDistance;
    } offset;
    /**
     * a factor which scales the output steering angle
     */
    tFloat32 steeringAngleScaleLeft;
    tFloat32 steeringAngleScaleRight;

	/**
	 * pointer to the class which implements the lane tracker
	 */
	LineSpecifier *lineSpecifier;

	/**
	 * for speed controller
	 */
	tFloat32 currentSteeringAngle;

	tTimeStamp _lastVideoTime;

	PoseCache *carPoses;

	struct Sign {
		float distance;
		bool haveWay;

		Sign() {
			distance = -1;
			haveWay = false;
		}
	} _prioritySign;

	/**
	  * pointer to the class which implements the stop line detector
	  */
	StopLineDetector *stopLineDetector;

	/**
	  * pointer to the class which implements the stop line detector
	  */
	ParkingSpotDetector *parkingDetector;
	tBool firstParkingDetection;

	IntersectionDetector *intersectionDetector;

	/**
	 * set true if debug output image pin is connected
	 */
	tBool debugEnabled;
	tBool debugEnabledLogEnabled;

	tBool saveToHardDisk;

	TActionStruct::ActionSub _actionSub;

	struct LastIntersection {
		TFeedbackStruct::Data feedback;
		tTimeStamp time;
		CarPose onEnterIntersection;
		tBool processHint;

		tFloat32 yawIntegral;


		tBool badCrossingType;

		LastIntersection() {
			time = -1;
			processHint = tFalse;
			yawIntegral = 0;
			badCrossingType = tFalse;
		}

		tBool Valid() {
			if(this->time != -1) {
				return tTrue;
			} else {
				return tFalse;
			}
		}

		tVoid Clear() {
			time = -1;
		}

		tBool ProcessHint(tBool value) {
			tBool last = processHint;
			if(Valid()) {
				processHint = value;
			} else {
				processHint = tFalse;
			}
			return last;
		}

		tVoid Save(CarPose intersectionPoint, tTimeStamp time) {
			onEnterIntersection = intersectionPoint;
			this->time = time;
		}

	private:
		tInt32 CheckTurn(tFloat32 currentYaw) {
			//tFloat32 diff = onEnterIntersection.yaw - currentYaw;
			//TODO:
			return 0;
		}
	} _lastIntersection;

	SpeedRecommender _speedController;

	struct Sync {
		/**
		 * Synchronization of OnPinEvent
		 */
		cCriticalSection criticalSection_OnPinEvent;

		ucom::cObjectPtr<IMediaSample> lastActionMediaSample;

		float checkRoadSign;
		TRoadSignStructExt::Data trafficSign;

		/**
		 *
		 */
		tBitmapFormat inputFormat;

		int bufferCount;
		bool log_warning;
		bool log_console;
		const char* name;

		Sync() {
			checkRoadSign = false;
			bufferCount = 0;
			log_warning = false;
			log_console = false;
			name = "LineSpecifier";
			lastActionMediaSample = 0;
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
	} _sync;
public:
	LineSpecifierAdapter(const tChar* __info);
	virtual ~LineSpecifierAdapter();

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Shutdown(tInitStage eStage, __exception);

	tResult CreateOutputPins(__exception);
	tResult OnPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* mediaSample) ;
	tResult OnAsyncPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* mediaSample);

private:
	tResult ProcessAction(IMediaSample* mediaSample);
	tResult ProcessVideo(IMediaSample* mediaSample);

	tResult ProcessStoplineDetection(cv::Mat &debugImage, cv::Point2f frameCoord, cv::Point2f image_destination, tFloat32 nearPlane, tTimeStamp outputTime);

	tResult ProcessParkingDetection(cv::Mat &debugImage, cv::Point2f frameCoord, tTimeStamp outputTime);

	tResult ProcessPullOut(cv::Mat &debugImage, cv::Point2f frameCoord, tTimeStamp outputTime);

	tResult ProcessIntersection(cv::Mat &debugImage, cv::Point2f frameCoord, tFloat32 nearPlane, tTimeStamp outputTime);

	TPoseStruct::Data DetermineWorldStopPoint(Point2f stopPosition, Point2f image_yawDirection);

	tResult ProcessBackwards(cv::Mat &debugImage, Point2f frameCoord, cv::Point2f image_destination, tTimeStamp outputTime);

    tResult PropertyChanged(const tChar* strName);


	/**
	 * convert adtf image to opencv image
	 * @param image
	 * @param mediaSample
	 * @return
	 */
	tResult ConvertInputImage(IMediaSample* mediaSample);

	tResult ReadAsFloatAndClear (cv::Mat &image, tUInt32 bytePosition, tFloat32 *value);

	/**
	 * needed for Record(Mat)
	 */
	tInt32 _imageNumber;

    VideoWriter *writer;
	/**
	 * writes input images to hard drive
	 * @param in
	 */
	tVoid Record(Mat &in, Mat &debug) ;
};

//*************************************************************************************************
#endif // LINESPECIFIERADAPTER_H_

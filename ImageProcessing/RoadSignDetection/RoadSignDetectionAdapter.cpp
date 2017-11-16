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
 * $Author:: schoen $   $Date:: 2016-02-17 #$
 * $Author:: fink $   $Date:: 2016-03-11 #$
 **********************************************************************/

#include "stdafx.h"
#include "RoadSignDetectionAdapter.h"

#include <roadSign_enums.h>
#include <iostream>


#include "CVMath.h"
#include <set>

ADTF_FILTER_PLUGIN("RoadSign", OID_ADTF_ROADSIGNEDETECTIONFILTER, RoadSignDetectionAdapter)

#define RSD_PITCH "Camera position/rotation::pitch"
#define RSD_PITCH_STORAGE "Camera position/rotation::pitch angle file"

#define RSD_MARKER_ANGLE_LIMIT_XY "Marker::angle limit X Y (degree)"
#define RSD_MARKER_ANGLE_LOWER_LIMIT_Z "Marker::angle lower limit Z (degree)"
#define RSD_MARKER_ANGLE_UPPER_LIMIT_Z "Marker::angle upper limit Z (degree)"
#define RSD_MARKER_LIMIT_Y "Marker::limit Y"
#define RSD_MARKER_STEERING_LIMIT "Marker::Steering angle limit"
#define RSD_MARKER_DISTANCE_LIMIT "Marker::distance search limit"
#define RSD_MARKER_REDUCE_RESOLUTION "Marker::reduce image resolution"

#define RSD_START_COMMAND "Debug::start action command"
#define RSD_DEBUG_TO_CONSOLE "Debug::output to console"
#define RSD_EXECTIME_TO_CONSOLE "Debug::print execution time to console"

#define RSD_ARUCO_DICTIONARY "Aruco::Dictionary File For Markers"
#define RSD_ARUCO_CAMERA_CALIBRATION "Aruco::Calibration File for used Camera"
#define RSD_ARUCO_SIZE_OF_MARKERS "Aruco::Size of road sign markers"

RoadSignDetectionAdapter::RoadSignDetectionAdapter(const tChar* __info) :
	cAsyncDataTriggeredFilter(__info) {
	SetPropertyFloat(RSD_PITCH, 0);
	SetPropertyFloat(RSD_PITCH NSSUBPROP_MAX, 0);
	SetPropertyFloat(RSD_PITCH NSSUBPROP_MIN, -20);
	SetPropertyStr(RSD_PITCH NSSUBPROP_DESCRIPTION, "default angle of the camera in degree. This will be overwritten by the pitch angle file");

	SetPropertyStr(RSD_PITCH_STORAGE, "../../../../../Camera_Pitch.txt");
	SetPropertyBool(RSD_PITCH_STORAGE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(RSD_PITCH_STORAGE NSSUBPROP_DESCRIPTION, "file which is used for storing the pitch angle");

	SetPropertyFloat(RSD_MARKER_ANGLE_LIMIT_XY, 30);
	SetPropertyFloat(RSD_MARKER_ANGLE_LIMIT_XY NSSUBPROP_MAX, 45);
	SetPropertyFloat(RSD_MARKER_ANGLE_LIMIT_XY NSSUBPROP_MIN, 0);
	SetPropertyStr(RSD_MARKER_ANGLE_LIMIT_XY NSSUBPROP_DESCRIPTION, "in degree");

	SetPropertyFloat(RSD_MARKER_ANGLE_LOWER_LIMIT_Z, 30);
	SetPropertyFloat(RSD_MARKER_ANGLE_LOWER_LIMIT_Z NSSUBPROP_MAX, 90);
	SetPropertyFloat(RSD_MARKER_ANGLE_LOWER_LIMIT_Z NSSUBPROP_MIN, -90);
	SetPropertyStr(RSD_MARKER_ANGLE_LOWER_LIMIT_Z NSSUBPROP_DESCRIPTION, "in degree");

	SetPropertyFloat(RSD_MARKER_ANGLE_UPPER_LIMIT_Z, 70);
	SetPropertyFloat(RSD_MARKER_ANGLE_UPPER_LIMIT_Z NSSUBPROP_MAX, 90);
	SetPropertyFloat(RSD_MARKER_ANGLE_UPPER_LIMIT_Z NSSUBPROP_MIN, 0);
	SetPropertyStr(RSD_MARKER_ANGLE_UPPER_LIMIT_Z NSSUBPROP_DESCRIPTION, "in degree");

	SetPropertyFloat(RSD_MARKER_LIMIT_Y, -0.2);
	SetPropertyFloat(RSD_MARKER_LIMIT_Y NSSUBPROP_MAX, 0);
	SetPropertyFloat(RSD_MARKER_LIMIT_Y NSSUBPROP_MIN, -1);
	SetPropertyStr(RSD_MARKER_LIMIT_Y NSSUBPROP_DESCRIPTION, "in meter top is negative");

	SetPropertyFloat(RSD_MARKER_STEERING_LIMIT, 10);
	SetPropertyFloat(RSD_MARKER_STEERING_LIMIT NSSUBPROP_MAX, 15);
	SetPropertyFloat(RSD_MARKER_STEERING_LIMIT NSSUBPROP_MIN, -15);
	SetPropertyStr(RSD_MARKER_STEERING_LIMIT NSSUBPROP_DESCRIPTION, "Steering angle limits for RoadSign Detection");

	SetPropertyFloat(RSD_MARKER_DISTANCE_LIMIT, 2.95);
	SetPropertyFloat(RSD_MARKER_DISTANCE_LIMIT NSSUBPROP_MAX, 10);
	SetPropertyFloat(RSD_MARKER_DISTANCE_LIMIT NSSUBPROP_MIN, 0);

	SetPropertyBool(RSD_MARKER_REDUCE_RESOLUTION, tTrue);
	SetPropertyStr(RSD_MARKER_REDUCE_RESOLUTION NSSUBPROP_DESCRIPTION, "Reduce resolution of input video stream before detecting aruco markers");

	SetPropertyInt(RSD_START_COMMAND, 0);
	SetPropertyStr(RSD_START_COMMAND NSSUBPROP_DESCRIPTION,
			"If not  0, this is the start command");

	SetPropertyBool(RSD_DEBUG_TO_CONSOLE, tFalse);
	SetPropertyStr(RSD_DEBUG_TO_CONSOLE NSSUBPROP_DESCRIPTION,
			"If enabled additional debug information is printed to the console");

	SetPropertyBool(RSD_EXECTIME_TO_CONSOLE, tFalse);
	SetPropertyStr(RSD_EXECTIME_TO_CONSOLE NSSUBPROP_DESCRIPTION,
			"If enabled execution time of aruco marker detection is printed to the console");

	SetPropertyStr(RSD_ARUCO_DICTIONARY, "../../../../configuration_files/roadsign.yml");
	SetPropertyBool(RSD_ARUCO_DICTIONARY NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(RSD_ARUCO_DICTIONARY NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");
	SetPropertyStr(RSD_ARUCO_DICTIONARY NSSUBPROP_DESCRIPTION,
			"Here you have to set the dictionary file which holds the marker IDs and their content");

	SetPropertyStr(RSD_ARUCO_CAMERA_CALIBRATION, "../../../../configuration_files/xtionIntrinsicCalib.yml");
	SetPropertyBool(RSD_ARUCO_CAMERA_CALIBRATION NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(RSD_ARUCO_CAMERA_CALIBRATION NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");
	SetPropertyStr(RSD_ARUCO_CAMERA_CALIBRATION NSSUBPROP_DESCRIPTION,
			"Here you have to set the file with calibration parameters of the used camera");

	SetPropertyFloat(RSD_ARUCO_SIZE_OF_MARKERS, 0.1f);
	SetPropertyStr(RSD_ARUCO_SIZE_OF_MARKERS NSSUBPROP_DESCRIPTION, "Size (length of one side) of markers in meter");
}

RoadSignDetectionAdapter::~RoadSignDetectionAdapter() {
}

tResult RoadSignDetectionAdapter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst)
	{
		tTrafficSign.StageFirst(__exception_ptr);
		tSignalValueSteering.StageFirst(__exception_ptr);
		tSignalValueSpeed.StageFirst(__exception_ptr);
		tActionStruct.StageFirst(__exception_ptr);
		tFeedbackStruct.StageFirst(__exception_ptr);

		RETURN_IF_FAILED(steeringInput.Create("steering", tSignalValueSteering.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&steeringInput));

		RETURN_IF_FAILED(speedInput.Create("speed", tSignalValueSpeed.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&speedInput));

		RETURN_IF_FAILED(actionInput.Create("action", tActionStruct.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&actionInput));

		RETURN_IF_FAILED(trafficSignOutput.Create("sign", tTrafficSign.GetMediaType(), 0));
		RETURN_IF_FAILED(RegisterPin(&trafficSignOutput));

		RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct.GetMediaType(), 0));
		RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

		RETURN_IF_FAILED(videoInput.Create("Video_RGB",IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&videoInput));

		RETURN_IF_FAILED(videoOutput.Create("RGB_Debug", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&videoOutput));
	}
	else if (eStage == StageNormal)
	{
		// debugging things
		debugMode = tFalse;
		debugExecutionTime = tFalse;
		debugModeEnabled = GetPropertyBool(RSD_DEBUG_TO_CONSOLE);

		thresholds.COS_ANGLE_LIMIT_XY = cos(GetPropertyFloat(RSD_MARKER_ANGLE_LIMIT_XY) / 180 *CV_PI);
		thresholds.COS_ANGLE_UPPER_LIMIT_Z = cos(GetPropertyFloat(RSD_MARKER_ANGLE_UPPER_LIMIT_Z) / 180 *CV_PI);
		thresholds.COS_ANGLE_LOWER_LIMIT_Z = cos(GetPropertyFloat(RSD_MARKER_ANGLE_LOWER_LIMIT_Z) / 180 *CV_PI);
		thresholds.MAX_DISTANCE = GetPropertyFloat(RSD_MARKER_DISTANCE_LIMIT);
		thresholds.STEERING_LIMIT = GetPropertyFloat(RSD_MARKER_STEERING_LIMIT) / 180 * CV_PI;
		m_bReduceImageResolution = GetPropertyBool(RSD_MARKER_REDUCE_RESOLUTION);
		debugExecutionTime = GetPropertyBool(RSD_EXECTIME_TO_CONSOLE);
		thresholds.CAMERA_Y_LIMIT = GetPropertyFloat(RSD_MARKER_LIMIT_Y);

		// camera calibration
		cameraPitch = GetPropertyFloat(RSD_PITCH);
		filePitch = GetPropertyStr(RSD_PITCH_STORAGE);
		ReadFromFile(&cameraPitch);


		// for trajectory estimation
		lastSpeed = 0;
		lastSteeringAngle = 0;

		lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
		lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_NONE;

		tInt32 startCommand = GetPropertyInt(RSD_START_COMMAND);
		if(3000 <= startCommand && startCommand < 4000) {
			actionSub.enabled = tTrue;
			actionSub.started = tTrue;
			actionSub.command = startCommand;
		}

		cameraParamsLoaded = tFalse;
		markerSize = static_cast<tFloat32>(GetPropertyFloat(RSD_ARUCO_SIZE_OF_MARKERS));

		//Get path of marker configuration file
		cFilename fileConfig = GetPropertyStr(RSD_ARUCO_DICTIONARY);
		//create absolute path for marker configuration file
		ADTF_GET_CONFIG_FILENAME(fileConfig);
		fileConfig = fileConfig.CreateAbsolutePath(".");

		//check if marker configuration file exits
		if (fileConfig.IsEmpty() || !(cFileSystem::Exists(fileConfig))) {
			LOG_ERROR("RoadSign: Dictionary File for Markers not found");
			RETURN_ERROR(ERR_INVALID_FILE);
		} else {
			//try to read the marker configuration file
			if(dictionary.fromFile(std::string(fileConfig))==false) {
				LOG_ERROR("RoadSign: Dictionary File for Markers could not be read");
				RETURN_ERROR(ERR_INVALID_FILE);

			} if(dictionary.size()==0) {
				RETURN_ERROR(ERR_INVALID_FILE);
				LOG_ERROR("RoadSign: Dictionary File does not contain valid markers or could not be read sucessfully");
			}
			//set marker configuration file to highlyreliable markers class
			if (!(HighlyReliableMarkers::loadDictionary(dictionary)==tTrue)) {
				LOG_ERROR("RoadSign: Dictionary File could not be read for highly reliable markers");
			}
		}

		//Get path of calibration file with camera paramters
		cFilename fileCalibration = GetPropertyStr(RSD_ARUCO_CAMERA_CALIBRATION);

		//Get path of calibration file with camera paramters
		ADTF_GET_CONFIG_FILENAME(fileCalibration);
		fileCalibration = fileCalibration.CreateAbsolutePath(".");
		//check if calibration file with camera paramters exits
		if (fileCalibration.IsEmpty() || !(cFileSystem::Exists(fileCalibration))) {
			LOG_ERROR("RoadSign: Calibration File for camera not found");

		} else {
			// read the calibration file with camera paramters exits and save to member variable
			cameraParameters.readFromXMLFile(fileCalibration.GetPtr());
			cv::FileStorage camera_data (fileCalibration.GetPtr(),cv::FileStorage::READ);
			camera_data ["camera_matrix"] >> intrinsics;
			camera_data ["distortion_coefficients"] >> distortion;
			cameraParamsLoaded = tTrue;

			// Reduce Image resolution: 640x480 -> 320x240 (change camera matrix)
	        if(m_bReduceImageResolution) {
	        	cameraParameters_downsampled = cameraParameters;
	        	cameraParameters_downsampled.CameraMatrix.at<tFloat32>(0,0) = cameraParameters.CameraMatrix.at<tFloat32>(0,0)/2;
	        	cameraParameters_downsampled.CameraMatrix.at<tFloat32>(0,2) = cameraParameters.CameraMatrix.at<tFloat32>(0,2)/2;
	        	cameraParameters_downsampled.CameraMatrix.at<tFloat32>(1,1) = cameraParameters.CameraMatrix.at<tFloat32>(1,1)/2;
	        	cameraParameters_downsampled.CameraMatrix.at<tFloat32>(1,2) = cameraParameters.CameraMatrix.at<tFloat32>(1,2)/2;
	        	cameraParameters_downsampled.CamSize.height = cameraParameters.CamSize.height/2;
	        	cameraParameters_downsampled.CamSize.width = cameraParameters.CamSize.width/2;
	        	//LOG_WARNING(cString::Format("RoadSign: Orig fx: %f, cx: %f, fy: %f, cy: %f",cameraParameters.CameraMatrix.at<tFloat32>(0,0),cameraParameters.CameraMatrix.at<tFloat32>(0,2),cameraParameters.CameraMatrix.at<tFloat32>(1,1),cameraParameters.CameraMatrix.at<tFloat32>(1,2)));
	        	//LOG_WARNING(cString::Format("RoadSign: Conv fx: %f, cx: %f, fy: %f, cy: %f",cameraParameters_downsampled.CameraMatrix.at<tFloat32>(0,0),cameraParameters_downsampled.CameraMatrix.at<tFloat32>(0,2),cameraParameters_downsampled.CameraMatrix.at<tFloat32>(1,1),cameraParameters_downsampled.CameraMatrix.at<tFloat32>(1,2)));
	        }
		}
	}
	else if (eStage == StageGraphReady)
	{
		tTrafficSign.StageGraphReady();
		tSignalValueSteering.StageGraphReady();
		tSignalValueSpeed.StageGraphReady();
		tActionStruct.StageGraphReady();
		tFeedbackStruct.StageGraphReady();

		// get the image format of the input video pin
		cObjectPtr<IMediaType> pType;
		RETURN_IF_FAILED(videoInput.GetMediaType(&pType));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

		// set the image format of the input video pin
		UpdateInputImageFormat(pTypeVideo->GetFormat());

		// set the image format of the output video pin; reduced resolution is also handled in this case!
		UpdateOutputImageFormat(pTypeVideo->GetFormat());

		// set paramenter for the marker detector class
		// same parameters as in aruco sample aruco_hrm_test.cpp
		// effects of parameters has to be tested
		markerDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
		//markerDetector.setThresholdParams( 21, 7);
		markerDetector.setThresholdParams( 10.5, 7.0);
		markerDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
		//markerDetector.setThresholdMethod(aruco::MarkerDetector::CANNY);
		markerDetector.setWarpSize((dictionary[0].n()+2)*8); // default: *8
		markerDetector.setMinMaxSize(0.005f, 0.3f); // default: min 0.005f, max 0.5f

		// Debugging: measure execution time of aruco marker detection (enabled with property)
		ms_sum = 0; // Debugging: overall execution time (aruco detection)
		number_of_signs = 0; // Debugging: count number of detected markers
		frame_counter = 0; // Debugging: received video frame counter

		same_sign_counter = 0; // sign has to be detected multiple times to be valid
		prev_closest_sign = -1;  // save id of last detected nearest sign

		parking_sign_received = tFalse; // save parking sign status

		if (videoOutput.IsConnected()) {
			debugMode = tTrue;
		} else {
			debugMode = tFalse;
		}
	}
	RETURN_NOERROR;
}

tResult RoadSignDetectionAdapter::Shutdown(tInitStage eStage, __exception) {
	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult RoadSignDetectionAdapter::OnPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* mediaSample) {
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);

	adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (source == &videoInput) {
			if(sync.bufferCount > 2) {
				sync.skipNextFrame = true;
			}

			sync.IncreaseBufferCount();

			return cAsyncDataTriggeredFilter::OnPinEvent(source, nEventCode, nParam1, nParam2, mediaSample);
		} else if (source == &actionInput){
			sync.lastActionMediaSample = mediaSample;

			return cAsyncDataTriggeredFilter::OnPinEvent(source, nEventCode, nParam1, nParam2, mediaSample);
		} else if (source == &steeringInput) {
			TSignalValue::Data data;
			tSignalValueSteering.Read(mediaSample, &data);
			lastSteeringAngle = (data.f32_value - 90) / 180 * CV_PI;

			if (abs(lastSteeringAngle) > 35) {
				LOG_WARNING("RoadSignDetection: SteeringAngle out of bounds");
			}
		} else if (source == &speedInput) {
			TSignalValue::Data data;
			tSignalValueSpeed.Read(mediaSample, &data);
			lastSpeed = data.f32_value;

			if (abs(lastSpeed) > 5.0) {
				LOG_WARNING("RoadSignDetection: Speed out of bounds");
			}
		}
	} else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
		if (source == &videoInput) {
			//the input format was changed, so the imageformat has to changed in this filter also
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(videoInput.GetMediaType(&pType));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

			UpdateInputImageFormat(videoInput.GetFormat());
			// reduced resolution is also handled by this function-call !
		}
	}
	RETURN_NOERROR;
}
tResult RoadSignDetectionAdapter::OnAsyncPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* mediaSample) {
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// a new image was received so the processing is started
		if (source == &actionInput)  {
			ProcessAction(mediaSample);
		} else if (source == &videoInput) {
			{
				adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

				sync.DecreaseBufferCount();

				if(sync.skipNextFrame) {
					sync.skipNextFrame = false;
					RETURN_NOERROR;
				}
			}
			if (actionSub.enabled && actionSub.started) {
				if(actionSub.command == AC_RS_START || actionSub.command == AC_RS_START_PARKING_SIGN_LOGGING || actionSub.command == AC_RS_START_PRIORITY_SIGN_LOGGING) {
					ProcessVideo(mediaSample);
				}
			}
		}
	}
	RETURN_NOERROR;
}

tResult RoadSignDetectionAdapter::ProcessAction(IMediaSample* mediaSample) {
	TActionStruct::ActionSub currentAction = tActionStruct.Read_Action(sync.lastActionMediaSample, F_ROAD_SIGNS);

	if(currentAction.enabled == tFalse || currentAction.started == tFalse) {
		lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
		lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_NONE;
	} else if(currentAction.command == AC_RS_GET_CROSSING_SIGN) {
		tFeedbackStruct.Transmit(&feedbackOutput, lastPrioritySign, _clock->GetStreamTime());
	} else if(currentAction.command == AC_RS_GET_PARKING_SIGN_RECEIVED) {
		TFeedbackStruct::Data feedback;
		feedback.ui32_filterID = F_ROAD_SIGNS;
		if(parking_sign_received) {
			parking_sign_received = tFalse;
			feedback.ui32_status = FB_RS_PARKING_SIGN_RECEIVED;
			tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
		} else {
			feedback.ui32_status = FB_RS_NO_PARKING_SIGN_RECEIVED;
			tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
		}
	} else if (currentAction.command == AC_RS_RESET_PARKING_SIGN) {
		parking_sign_received = tFalse; // also reset parking status
	} else if (currentAction.command == AC_RS_RESET_CROSSING_SIGN) {
		lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
		lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_NONE;

		TFeedbackStruct::Data feedback;
		feedback.ui32_filterID = F_ROAD_SIGNS;
		feedback.ui32_status = FB_RS_CROSSING_SIGN_RESET;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
	}

	actionSub = currentAction;

	RETURN_NOERROR;

}

tResult RoadSignDetectionAdapter::ProcessVideo(adtf::IMediaSample* inputSample) {
	//creating new pointer for input data
	const tVoid* srcBuffer;
	//creating matrix for input image
	Mat inputImage;
	Mat outputImage;

	// new Image with reduced resolution
	Mat reduced_Image;
	reduced_Image = Mat (cv::Size(320,240), CV_8UC3);

	//receiving data from input sample, and saving to InputImage
	if (IS_OK(inputSample->Lock(&srcBuffer))) {
		try {
			//convert to mat
			inputImage = Mat(inputFormat.nHeight, inputFormat.nWidth, CV_8UC3, (tVoid*) srcBuffer,
					inputFormat.nBytesPerLine);

			tTimeStamp starttime = _clock->GetStreamTime();

			// doing the detection of markers in image
	        if(m_bReduceImageResolution) { // reduced image resolution
				resize(inputImage,reduced_Image,reduced_Image.size());
	        	markerDetector.detect(reduced_Image(Rect(0, 0, static_cast<tUInt32>(0.85*reduced_Image.cols), static_cast<tUInt32>(0.65*reduced_Image.rows))), cacheMarkers, cameraParameters_downsampled, static_cast<float>(markerSize));
				//LOG_WARNING(cString::Format("RoadSign: cols (width) orig: %d, new: %d; rows (height): orig: %d, new: %d", reduced_Image.cols, static_cast<tUInt32>(0.65*reduced_Image.cols), reduced_Image.rows, static_cast<tUInt32>(0.75*reduced_Image.rows)));
	        } else {
	        	markerDetector.detect(inputImage(Rect(0, 0, 640, 480)), cacheMarkers, cameraParameters, static_cast<float>(markerSize));
	        }

	        // Debug Execution time
	        if(debugExecutionTime) {
				tTimeStamp timediff = _clock->GetStreamTime() - starttime;
				tFloat32 difftime_ms = timediff / 1000;
				ms_sum += difftime_ms;
				frame_counter++;

				if(frame_counter % 30 == 0) {
					LOG_WARNING(cString::Format("RoadSign: execution time in ms: %f, overall time: %f, number of signs: %d", difftime_ms, ms_sum, number_of_signs));
					frame_counter =  0;
				}
	        }
		} catch (cv::Exception &e) {
			LOG_ERROR(cString::Format("RoadSign Exception: %s", e.msg.c_str()));
			inputSample->Unlock(srcBuffer);
			RETURN_ERROR(ERR_EXCEPTION_RAISED);
		}
	}
	inputSample->Unlock(srcBuffer);

	if (debugMode) {
		// do a deep copy of the image, otherwise the original frame is modified
		if(m_bReduceImageResolution) {
			outputImage = reduced_Image.clone();
		}
		else{
			outputImage = inputImage.clone();
		}
	}

	RoadSign closestSign;
	closestSign.distance = std::numeric_limits<tFloat32>::max();
	RoadSign prevClosestSign;

	cv::Point3f m_x;
	cv::Point3f m_y;
	tFloat32 angle = 0;

	for (unsigned int i = 0; i < cacheMarkers.size(); i++) {
		if (norm(cacheMarkers[i].Tvec) < closestSign.distance) {

			cv::Mat rot;
			cv::Rodrigues(cacheMarkers[i].Rvec, rot);
			cv::Mat pitchRot = CVMath::RotationX_3D_Degree (cameraPitch);
			rot = pitchRot * rot;

			cv::Point3f e_x = Point3f(rot.at<tFloat32>(0, 0), rot.at<tFloat32>(1, 0), rot.at<tFloat32>(2, 0));
			cv::Point3f e_y = Point3f(rot.at<tFloat32>(0, 1), rot.at<tFloat32>(1, 1), rot.at<tFloat32>(2, 1));
			cv::Point3f e_z = Point3f(rot.at<tFloat32>(0, 2), rot.at<tFloat32>(1, 2), rot.at<tFloat32>(2, 2));

			//Point3f exp_e_z = Point3f(cos(60.0 / 180 *CV_PI), 0, -sin(60.0 / 180 *CV_PI));
			Point3f translation = Point3f(cacheMarkers[i].Tvec);
			Mat t = pitchRot * cacheMarkers[i].Tvec;
			translation = Point3f(t);
			float distance = cv::norm(translation);

			if(distance > thresholds.MAX_DISTANCE || translation.y < thresholds.CAMERA_Y_LIMIT) {
				continue;
			}

			// Debug Execution time
			if(debugExecutionTime) {
				number_of_signs++; // count number of detected signs
			}

			Point3f exp_e_z = - translation / distance;
			Point3f exp_e_x = Point3f(0, 1, 0);
			//Point3f exp_e_y = Point3f(1, 0, 0);

			e_z.y = 0;
			e_z = e_z / cv::norm(e_z);
			//const tFloat32 x_dot_x = exp_e_x.dot(e_x);
			//const tFloat32 x_dot_y = exp_e_x.dot(e_y);


			float y = e_z.z *exp_e_z.x - e_z.x * exp_e_z.z;
			/*
			if((exp_e_z.dot(e_z) > thresholds.COS_ANGLE_UPPER_LIMIT_Z && y < 0) ||
					(exp_e_z.dot(e_z) > thresholds.COS_ANGLE_LOWER_LIMIT_Z && y > 0)) {
				//const tFloat32 y_dot_y = exp_e_y.dot(e_y);

				//if(x_dot_x > thresholds.COS_ANGLE_LIMIT_XY && y_dot_y > thresholds.COS_ANGLE_LIMIT_XY) {
				//} else if(- x_dot_y > thresholds.COS_ANGLE_LIMIT_XY && exp_e_y.dot(e_x) > thresholds.COS_ANGLE_LIMIT_XY) {
				//} else if(- x_dot_x > thresholds.COS_ANGLE_LIMIT_XY && - y_dot_y > thresholds.COS_ANGLE_LIMIT_XY) {
				//} else if(x_dot_y  > thresholds.COS_ANGLE_LIMIT_XY && - x_dot_x > thresholds.COS_ANGLE_LIMIT_XY) {
				//}

				if ((-x_dot_x > thresholds.COS_ANGLE_LIMIT_XY && e_y.z < 0)
						|| (x_dot_x > thresholds.COS_ANGLE_LIMIT_XY && e_y.z > 0)
						|| (-x_dot_y > thresholds.COS_ANGLE_LIMIT_XY && e_x.z > 0)
						|| (x_dot_y > thresholds.COS_ANGLE_LIMIT_XY && e_x.z < 0)) {

				} else {
					//continue; // TODO tmp disabled, new sign check method
				}
			} else {
				//continue; TODO tmp disabled, new sign check method
			}
			*/

			// z-axis limits
			if((exp_e_z.dot(e_z) > thresholds.COS_ANGLE_UPPER_LIMIT_Z && y < 0) ||
					(exp_e_z.dot(e_z) > thresholds.COS_ANGLE_LOWER_LIMIT_Z && y > 0)) {
			} else {
				if(cacheMarkers[i].id != MARKER_ID_PARKINGAREA) {
					continue;
				}
			}

			// x-axis limit
			if((e_x.dot(exp_e_x) <= thresholds.COS_ANGLE_LIMIT_XY)) {
				continue;
			}

			// skip sign after s-curve
			if( (fabsf(lastSteeringAngle) > thresholds.STEERING_LIMIT && cacheMarkers[i].id != MARKER_ID_PARKINGAREA)) {
				if(debugModeEnabled) {
					LOG_WARNING(cString::Format("RoadSign: skipped sign with ID %d because steering angle (%f) > limit (%f)", cacheMarkers[i].id, (lastSteeringAngle/CV_PI)*180, (thresholds.STEERING_LIMIT/CV_PI)*180 ));
				}
				continue;
			}

			if (debugModeEnabled) {
				LOG_WARNING(
						cString::Format("RoadSign: ID %d, counter: %d, x: %f, y: %f, z: %f, x_dot_x: %f, z_dot_z: %f, cam pitch: %f, steering: %f", cacheMarkers[i].id, same_sign_counter, translation.x, translation.y,
								translation.z, e_x.dot(exp_e_x), exp_e_z.dot(e_z), cameraPitch, (lastSteeringAngle/CV_PI)*180.0f ));
			}

			m_x = e_x;
			m_y = e_y;

			prevClosestSign = closestSign;
			Mat closestRot = ((pitchRot * cacheMarkers[i].Rvec));
			closestSign = RoadSign(translation, Point3f(closestRot), cacheMarkers[i].id, e_z, inputSample->GetTime());
			closestSign.index = i;
			angle = exp_e_z.dot(e_z);

		}
	}

	// received valid sign
	if (closestSign.id != -1) {

		// check nearest sign: same as last received nearest sign -> increase counter
		if(prev_closest_sign == -1) {
			same_sign_counter = 0;
		} else {
			if(prev_closest_sign == closestSign.id) {
				same_sign_counter++;
			} else {
				same_sign_counter = 0;
			}
		}

		// save last received nearest sign
		prev_closest_sign = closestSign.id;

		// received same sign three times in a row
		if(same_sign_counter >= 2) {

			// reset same sign counter
			same_sign_counter = 0;
			prev_closest_sign = -1;

			if(debugModeEnabled) {
				LOG_WARNING(cString::Format("RoadSign: Sign with id %d detected three times as nearest sign in a row", closestSign.id));
			}

			//evaluate signs
			tBool near = MatchToTrajectory(closestSign, lastSteeringAngle, lastSpeed);
			tBool far = MatchToTrajectory(prevClosestSign, lastSteeringAngle, lastSpeed);

			if(far && near == tFalse) {
				closestSign = prevClosestSign;
			}

			// reset parking_sign_received
			parking_sign_received = tFalse;

			if (far || near) {
				TTrafficSign::Data data;
				data.ui32_signID = closestSign.id;
				data.f32_distance = closestSign.distance;

				if(actionSub.command == AC_RS_START || actionSub.command == AC_RS_START_PRIORITY_SIGN_LOGGING) {
					if(actionSub.command == AC_RS_START) {
						if(data.ui32_signID != MARKER_ID_PARKINGAREA) {
							// replace Roundabout sign with giveway sign
							if(data.ui32_signID == MARKER_ID_ROUNDABOUT) {
								data.ui32_signID = MARKER_ID_GIVEWAY;
							}
							tTrafficSign.Transmit(&trafficSignOutput, data, inputSample->GetTime());
						}
					}

					switch (closestSign.id) {
						case MARKER_ID_ROUNDABOUT: {
							lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
							lastPrioritySign.ui32_status = FB_RS_ROUNDABOUT;
							break;			}
						case MARKER_ID_UNMARKEDINTERSECTION:
						case MARKER_ID_STOPANDGIVEWAY:
						case MARKER_ID_GIVEWAY: {
							lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
							lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_GIVEWAY;

							break;			}
						case MARKER_ID_HAVEWAY: {
							lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
							lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_PRIORITY;

							break;			}
						case MARKER_ID_PARKINGAREA: {
							parking_sign_received = tTrue; // set parking sign received
							TFeedbackStruct::Data response;
							response.ui32_filterID = F_ROAD_SIGNS;
							response.ui32_status = 3000 + closestSign.id;
							tFeedbackStruct.Transmit(&feedbackOutput, response, inputSample->GetTime());
						}
					}
				} else {  //AC_RS_START_PARKING_SIGN_LOGGING
					if (closestSign.id == MARKER_ID_PARKINGAREA) {
						parking_sign_received = tTrue; // set parking sign received
					}
				}

				//if(debugModeEnabled) {
					//LOG_WARNING(cString::Format("RoadSign: Marker found %d", closestSign.id));
				//}

				if (debugMode) {
					// vectors x and y are wrong printed
					putText(outputImage, cv::String(cString::Format("%.2f %.2f %.2f %.2f", m_x.x, m_y.x, closestSign.z_vec.x, closestSign.translation.x).GetPtr()),
							Point2f(0, 25), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
					putText(outputImage, cv::String(cString::Format("%.2f %.2f %.2f %.2f", m_x.y, m_y.y, closestSign.z_vec.y, closestSign.translation.y).GetPtr()),
							Point2f(0, 50), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));
					putText(outputImage, cv::String(cString::Format("%.2f %.2f %.2f %.2f %.2f", m_x.z, m_y.z, closestSign.z_vec.z, closestSign.translation.z, angle).GetPtr()),
							Point2f(0, 75), CV_FONT_NORMAL, 0.35, Scalar(0, 0, 255));

					// draw the marker in the image
					cacheMarkers[closestSign.index].draw(outputImage, Scalar(0, 0, 255), 1);

					// draw cube in image
					//CvDrawingUtils::draw3dCube(TheInputImage,m_TheMarkers[i],m_TheCameraParameters);

					// draw 3d axis in image if the intrinsic params were loaded
					if (cameraParamsLoaded){
						if(m_bReduceImageResolution) {
							CvDrawingUtils::draw3dAxis(outputImage, cacheMarkers[closestSign.index], cameraParameters_downsampled);
						}
						else{
							CvDrawingUtils::draw3dAxis(outputImage, cacheMarkers[closestSign.index], cameraParameters);
						}
					}
				}
			}
		}
	}

	//update new media sample with image data if something has to be transmitted
	if (debugMode ) {
		cObjectPtr<IMediaSample> sample;
		if (IS_OK(AllocMediaSample(&sample))) {
			if(m_bReduceImageResolution) {

				sample->Update(inputSample->GetTime(), outputImage.data, tInt32(reducedOutputFormat.nSize), 0);

				videoOutput.SetFormat(&reducedOutputFormat, NULL);
				videoOutput.Transmit(sample);
			}
			else{
				sample->Update(inputSample->GetTime(), outputImage.data, tInt32(outputFormat.nSize), 0);

				videoOutput.SetFormat(&outputFormat, NULL);
				videoOutput.Transmit(sample);
			}
		}
	}

	RETURN_NOERROR;
}

tResult RoadSignDetectionAdapter::UpdateInputImageFormat(const tBitmapFormat* pFormat) {
	if (pFormat != NULL) {
		inputFormat = (*pFormat);

		LOG_INFO(
				adtf_util::cString::Format(
						"Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
						inputFormat.nWidth, inputFormat.nHeight, inputFormat.nBytesPerLine, inputFormat.nSize,
						inputFormat.nPixelFormat));

	}

	RETURN_NOERROR;
}

tResult RoadSignDetectionAdapter::UpdateOutputImageFormat(const tBitmapFormat* pFormat) {
	if (pFormat != NULL) {
		outputFormat = (*pFormat);
		// reduced image resolution
		if(m_bReduceImageResolution){
			reducedOutputFormat = outputFormat;
			reducedOutputFormat.nWidth = outputFormat.nWidth/2;
			reducedOutputFormat.nHeight = outputFormat.nHeight/2;
			reducedOutputFormat.nBytesPerLine = outputFormat.nBytesPerLine/2;
			reducedOutputFormat.nSize = reducedOutputFormat.nBytesPerLine * reducedOutputFormat.nHeight;
		}

		LOG_INFO(
				adtf_util::cString::Format(
						"Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d",
						outputFormat.nWidth, outputFormat.nHeight, outputFormat.nBytesPerLine, outputFormat.nSize,
						outputFormat.nPixelFormat));
		if(m_bReduceImageResolution){
			videoOutput.SetFormat(&reducedOutputFormat, NULL);
		}
		else{
			videoOutput.SetFormat(&outputFormat, NULL);
		}
	}

	RETURN_NOERROR;
}

tResult RoadSignDetectionAdapter::ReadFromFile(tFloat32 *pitch) {
	if (filePitch.GetLength() < 2){
		LOG_WARNING("RoadSignDetection: No filename for the pitch angle is given");
		RETURN_NOERROR;
	}

	// create absolute path
	ADTF_GET_CONFIG_FILENAME(filePitch);
	cFilename absFilename = filePitch.CreateAbsolutePath(".");

	if (cFileSystem::Exists(filePitch) == tFalse) {
		LOG_ERROR("RoadSignDetection: File for stored pitch angle does not exist");
		RETURN_NOERROR;
	}

	cFile file;
	file.Open(absFilename.GetPtr(), cFile::OM_Read);

	cString tmp;
	file.ReadLine(tmp); // comment line
	file.ReadLine(tmp);

	if(tmp.GetLength() >= 1) {
		*pitch = tmp.AsFloat64();
	} else {
		LOG_ERROR(cString::Format("RoadSignDetection: Something went wrong with opening file %s. Using default pitch angle", absFilename.GetPtr()));
	}

	RETURN_NOERROR;
}

tBool RoadSignDetectionAdapter::MatchToTrajectory(RoadSign sign, tFloat32 steeringAngle, tFloat32 speed) {
	const tFloat32 carLength = 0.6;
	const tFloat32 timestep = 0.03333333;

	if(sign.id < 0 || sign.index < 0) {
		return tFalse;
	}

	//TODO disabled because it does not work properly
	return tTrue;

	if(speed < 0.5) {
		speed = 0.5;
	}

	//std::cout << "steering " << steeringAngle << " speed " << speed << std::endl;
	tFloat32 psi_point = tan(steeringAngle) * ( speed / carLength );

	tFloat psi = psi_point;
	Point2f position = Point2f(0,0);

	for (tFloat32 absDistance = 0; absDistance < 5;) {
		Point2f steering = Point2f (sin(psi), cos(psi));

		absDistance += speed * timestep;
		position += steering * speed * timestep;
		psi += psi_point * timestep;

		Point3f dir = Point3f(steering);
		Point3f signDir = Point3f(sign.translation.x, sign.translation.z, 0) - Point3f(position);

		if(dir.dot(signDir) > 0) {
			// angle between dir and signDir bigger than 90 degree
			//sign behind the car

			Point3f z = dir.cross(signDir);
			if(z.z > 0) {
				//sign on left hand side

				if(Point3f(sign.z_vec.x, sign.z_vec.z, 0).dot(dir) < cos(150.0 / 180 * CV_PI)) {
					return tTrue;
				}
				std::cout << "return false1" << "position " << position.x << " "<< position.y << "dir " << dir.x << " "<< dir.y << "z_vec " << sign.z_vec.x << " " << sign.z_vec.y << std::endl;
			
			}
			return tFalse;
		}
	}

			std::cout << "return false2" << std::endl;
	return tFalse;
}

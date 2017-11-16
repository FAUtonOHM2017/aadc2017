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
#include "CameraEstimatorAdapter.h"
#include "ImageProcessingUtils.h"

ADTF_FILTER_PLUGIN("CamPitch", OID_ADTF_CameraEstimator, CameraEstimatorAdapter);

#define CPM_MAX_ADJUSTMENT_TIME_1 "max adjustment time::1 aprox estimation"
#define CPM_MAX_ADJUSTMENT_TIME_2 "max adjustment time::2 exact estimation"
#define CPM_MAX_ADJUSTMENT_TIME_3 "max adjustment time::3 stop line position estimation"
#define CPM_CONTINUE_ADJUSTMENT_IF_NO_FIX "max adjustment time::extend time if no fix"

#define CPM_PITCH_STORAGE "use as Start Calibration::pitch angle file"
#define CPM_WRITE_TO_FILE "use as Start Calibration::activate success fail pin and write to file"
#define CPM_STOP_LINE_DISTANCE "use as Start Calibration::stop line distance"


CameraEstimatorAdapter::CameraEstimatorAdapter(const tChar* __info) :
		cAsyncDataTriggeredFilter (__info) {

	SetPropertyFloat(CPM_MAX_ADJUSTMENT_TIME_1, 1.0);
	SetPropertyStr(CPM_MAX_ADJUSTMENT_TIME_1 NSSUBPROP_DESCRIPTION, "maximum estimation time of the camera parameters in seconds");

	SetPropertyFloat(CPM_MAX_ADJUSTMENT_TIME_2, 5.0);
	SetPropertyStr(CPM_MAX_ADJUSTMENT_TIME_2 NSSUBPROP_DESCRIPTION, "maximum estimation time of the camera parameters in seconds. Includes time 1.");

	SetPropertyFloat(CPM_MAX_ADJUSTMENT_TIME_3, 5.0);
	SetPropertyStr(CPM_MAX_ADJUSTMENT_TIME_3 NSSUBPROP_DESCRIPTION, "maximum estimation time of the camera parameters in seconds. Includes time 2.");

	SetPropertyStr(CPM_PITCH_STORAGE, "");
	SetPropertyBool(CPM_PITCH_STORAGE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(CPM_PITCH_STORAGE NSSUBPROP_DESCRIPTION,"file which is used for storing the pitch angle");

	SetPropertyBool(CPM_WRITE_TO_FILE, tFalse);
	SetPropertyStr(CPM_WRITE_TO_FILE NSSUBPROP_DESCRIPTION, "stores the pitch angle in the given file and sends success or failed over the pins");

	SetPropertyFloat(CPM_STOP_LINE_DISTANCE, 1.0);
	SetPropertyFloat(CPM_STOP_LINE_DISTANCE NSSUBPROP_MAX, 2.0);
	SetPropertyFloat(CPM_STOP_LINE_DISTANCE NSSUBPROP_MIN, 0.3);
	SetPropertyStr(CPM_STOP_LINE_DISTANCE NSSUBPROP_DESCRIPTION, "write here the distance from the camera to the center of the stopline in meter");

	SetPropertyInt(CPM_CONTINUE_ADJUSTMENT_IF_NO_FIX, 1);
 	SetPropertyStr(CPM_CONTINUE_ADJUSTMENT_IF_NO_FIX NSSUBPROP_VALUELISTNOEDIT, "1@no|2@yes");
	SetPropertyStr(CPM_CONTINUE_ADJUSTMENT_IF_NO_FIX NSSUBPROP_DESCRIPTION,
			"defines whether the maximum adjustment time should be extended if camera parameter could not be estimated properly");
}

CameraEstimatorAdapter::~CameraEstimatorAdapter() {

}

tResult CameraEstimatorAdapter::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst) {
		tSignalValuePitch.StageFirst(__exception_ptr);
		tBoolSignalValueFailed.StageFirst(__exception_ptr);
		tBoolSignalValueSuccessful.StageFirst(__exception_ptr);
		firstFrame = tTrue;
		calibrationFailed = tFalse;

		//Pitch Output Pin
		RETURN_IF_FAILED(pitchOutput.Create("Pitch", tSignalValuePitch.GetMediaType(), 0));
		RETURN_IF_FAILED(RegisterPin(&pitchOutput));

		RETURN_IF_FAILED(successOutput.Create("Success", tBoolSignalValueFailed.GetMediaType(), 0));
		RETURN_IF_FAILED(RegisterPin(&successOutput));

		RETURN_IF_FAILED(failOutput.Create("Fail", tBoolSignalValueSuccessful.GetMediaType(), 0));
		RETURN_IF_FAILED(RegisterPin(&failOutput));

		// Video Input
		RETURN_IF_FAILED(videoInputPin.Create("Binary_Image", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&videoInputPin));

		RETURN_IF_FAILED(videoDebugOutputPin.Create("RGB_DebugVideo", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&videoDebugOutputPin));

	} else if (eStage == StageNormal) {
		firstFrame = tTrue;
		calibrationFailed = tFalse;
		calibrationSucceeded = tFalse;

		maxAdjustmentTime_1 = GetPropertyFloat(CPM_MAX_ADJUSTMENT_TIME_1);

		MAX_FAILED_DETECTIONS = maxAdjustmentTime_1 * 30;
		maxAdjustmentTime_2 = GetPropertyFloat(CPM_MAX_ADJUSTMENT_TIME_2);
		continueIfNoFix = GetPropertyInt(CPM_CONTINUE_ADJUSTMENT_IF_NO_FIX);

		maxAdjustmentTime_3 = GetPropertyFloat(CPM_MAX_ADJUSTMENT_TIME_3);

		stopLineMarker = Point2f(GetPropertyFloat(CPM_STOP_LINE_DISTANCE), 0);


		writeToFile = GetPropertyBool(CPM_WRITE_TO_FILE);
		filePitch = GetPropertyStr(CPM_PITCH_STORAGE);
		cameraEstimator = new CameraEstimator();
		cameraEstimator->Init();

	} else if (eStage == StageGraphReady) {
		tSignalValuePitch.StageGraphReady();
		tBoolSignalValueFailed.StageGraphReady();
		tBoolSignalValueSuccessful.StageGraphReady();
	}
	RETURN_NOERROR;
}

tResult CameraEstimatorAdapter::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr) {
	if (eStage == StageNormal) {
		delete cameraEstimator;
	}

	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}


tResult CameraEstimatorAdapter::OnPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* mediaSample) {
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);

	{
		adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

		if(sync.bufferCount > 5) {
			sync.bufferCount = 0;
			ClearAsyncQueue();
			sync.PrintCleared();
		}
	}

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (source == &videoInputPin) {
			{
				adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

				sync.IncreaseBufferCount();
			}

			return cAsyncDataTriggeredFilter::OnPinEvent(source, nEventCode, nParam1, nParam2, mediaSample);
		}
	}

	RETURN_NOERROR;
}

tResult CameraEstimatorAdapter::OnAsyncPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* mediaSample) {
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (source == &videoInputPin) {
			adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(&(sync.criticalSection_OnPinEvent));

			sync.DecreaseBufferCount();

			if(sync.skipNextFrame) {
				sync.skipNextFrame = false;
				LOG_WARNING(cString::Format("CamPitch: image queue buffer count %i. Skipped frame.", sync.bufferCount));

				RETURN_NOERROR;
			}
			if(sync.bufferCount > 2) {
				sync.skipNextFrame = true;
			}

			//Videoformat
			if (firstFrame) {
				cObjectPtr<IMediaType> type;
				RETURN_IF_FAILED(videoInputPin.GetMediaType(&type));

				cObjectPtr<IMediaTypeVideo> typeVideo;
				RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&typeVideo));

				const tBitmapFormat* format = typeVideo->GetFormat();
				if (format == NULL) {
					LOG_ERROR("LineSpecifier: No Bitmap information found on input pin Binary_Image");
					RETURN_ERROR(ERR_NOT_SUPPORTED);
				}

				inputFormat.nPixelFormat = format->nPixelFormat;
				inputFormat.nWidth = format->nWidth;
				inputFormat.nHeight = format->nHeight;
				inputFormat.nBitsPerPixel = format->nBitsPerPixel;
				inputFormat.nBytesPerLine = format->nBytesPerLine;
				inputFormat.nSize = format->nSize;
				inputFormat.nPaletteSize = format->nPaletteSize;
				firstFrame = tFalse;

				tBoolSignalValueSuccessful.Transmit(&successOutput, tFalse, 0, mediaSample->GetTime());
				tBoolSignalValueFailed.Transmit(&failOutput, tFalse, 0, mediaSample->GetTime());

			}

			RETURN_IF_FAILED(ProcessInputFrame(mediaSample));
		}
	}

	RETURN_NOERROR;
}

tResult CameraEstimatorAdapter::ProcessInputFrame(IMediaSample* sample) {
	if (_clock->GetStreamTime() > maxAdjustmentTime_3 * 1000 * 1000) {
		RETURN_NOERROR;
	} else if(writeToFile) {
		if (calibrationFailed == tFalse && calibrationSucceeded == tTrue) {
			StoreToFile();
			LOG_WARNING(cString::Format("CameraPitchEstimator: pitch angle fixed: %f degree. YOffset %f", cameraEstimator->GetCurrentPitch(), cameraEstimator->GetCurrentYDiff() / 200));
		
			tBoolSignalValueSuccessful.Transmit(&successOutput, tTrue, 0, sample->GetTime());
		} else {
			//if (calibrationFailed == tTrue) {
				LOG_WARNING(cString::Format("CameraPITCHEstimator: NO pitch angle estimated and saved", cameraEstimator->GetCurrentPitch()));
			//}
			tBoolSignalValueFailed.Transmit(&failOutput, tTrue, 0, sample->GetTime());
		}
	}

	Mat image(Size(this->inputFormat.nWidth, this->inputFormat.nHeight), CV_8UC1);
	Mat edgeImage(Size(this->inputFormat.nWidth, this->inputFormat.nHeight), CV_8UC1);
	RETURN_IF_FAILED(ConvertInputImage(image, sample));

	tFloat32 pitch;
	RETURN_IF_FAILED(ReadAsFloatAndClear(image, 0, &pitch));
	cameraEstimator->SetCurrentPitch(pitch);

	//read frame position, stored in the image
	Point2f frameCoord;
	ReadAsFloatAndClear(image, 4, &frameCoord.x);
	ReadAsFloatAndClear(image, 8, &frameCoord.y);

	medianBlur(image, image, 5);

	Mat element = getStructuringElement( MORPH_RECT, Size( 4,15));
	cv::dilate(image, edgeImage, element, Point(-1,-1), 3);

	element = getStructuringElement( MORPH_RECT, Size( 3,10));
	cv::erode(edgeImage, edgeImage, element, Point(-1,-1), 3);


	Canny(edgeImage, edgeImage, 50, 200, 3);

	Mat debugImage(Size(this->inputFormat.nWidth, this->inputFormat.nHeight), CV_8UC3);
	cvtColor(edgeImage, debugImage, CV_GRAY2RGB);

	tFloat32 currentPitch = 0;
	if (_clock->GetStreamTime() < maxAdjustmentTime_1 * 1000 * 1000) {
		if(cameraEstimator->DetectPitch(edgeImage, debugImage)) {
			currentPitch = cameraEstimator->GetCurrentPitch();

			tSignalValuePitch.Transmit(&pitchOutput, currentPitch, 0, sample->GetTime());

			if (_clock->GetStreamTime() > (maxAdjustmentTime_1 - 0.1) * 1000 * 1000) {
				LOG_WARNING(cString::Format("CameraPitchEstimator: First approximated pitch angle: %f", cameraEstimator->GetCurrentPitch()));
			}
		} else if(continueIfNoFix == 2){
			//TODO: increase adjustment times with timestamps from the media samples
			maxAdjustmentTime_1 += 1.0/30 + 0.0001;
			maxAdjustmentTime_2 += 1.0/30 + 0.0001;

			if(cameraEstimator->GetFailCount() > MAX_FAILED_DETECTIONS) {
				//something went wrong. Reset and try again
				LOG_WARNING(cString::Format("CameraPitchEstimator: could not detect camera pitch after %d iterations. Now reseting and trying again. ", cameraEstimator->GetFailCount()));
				cameraEstimator->Init();

				tSignalValuePitch.Transmit(&pitchOutput, 0.0, 0, sample->GetTime());
			}
		}
	} else if (_clock->GetStreamTime() < maxAdjustmentTime_2 * 1000 * 1000) {
		if(cameraEstimator->GetFailCount() > MAX_FAILED_DETECTIONS && calibrationFailed == tFalse) {
			//something went wrong.
			LOG_WARNING(cString::Format("CameraPitchEstimator: could not detect camera pitch after %d iterations. Calibration failed", cameraEstimator->GetFailCount()));
			calibrationFailed = tTrue;
		}

		if(cameraEstimator->DetectPitch2(edgeImage, debugImage)) {
			currentPitch = cameraEstimator->GetCurrentPitch();
			tSignalValuePitch.Transmit(&pitchOutput, currentPitch, 0, sample->GetTime());
			calibrationSucceeded = tTrue;
		} 
	} else {
		Point2f imageCoord = ImageUtils::ConvertToImageCoordinates(stopLineMarker, frameCoord);
		try {
			if (cameraEstimator->DetectPitch3_StopLine(image, debugImage, imageCoord)) {
				currentPitch = cameraEstimator->GetCurrentPitch();
				tSignalValuePitch.Transmit(&pitchOutput, currentPitch, 0, sample->GetTime());
				LOG_WARNING(cString::Format("CameraPitchEstimator: refined pitch angle with stop line. pitch: %f", currentPitch));
			}
		} catch (cv::Exception &e) {
			LOG_ERROR(e.msg.c_str());
		}
	}

	cameraEstimator->DrawDebugData(debugImage);

	cObjectPtr<IMediaSample> newSample;
	if (IS_OK(AllocMediaSample(&newSample))) {
		tTimeStamp outputTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();

		tBitmapFormat outputFormat = inputFormat;
		outputFormat.nWidth = debugImage.cols;
		outputFormat.nHeight = debugImage.rows;
		outputFormat.nBitsPerPixel = debugImage.channels() * 8;
		outputFormat.nBytesPerLine = debugImage.cols * debugImage.channels();
		outputFormat.nSize = outputFormat.nBytesPerLine * debugImage.rows;
		outputFormat.nPixelFormat = cImage::PF_RGB_888;

		newSample->Update(outputTime, debugImage.data, tInt32(outputFormat.nSize), 0);

		videoDebugOutputPin.SetFormat(&outputFormat, NULL);

		sync.criticalSection_OnPinEvent.Leave();
		videoDebugOutputPin.Transmit(newSample);
		sync.criticalSection_OnPinEvent.Enter();
	}

	RETURN_NOERROR;
}

tResult CameraEstimatorAdapter::ConvertInputImage(cv::Mat &image, IMediaSample* mediaSample) {
	cImage oImage;
	const tVoid* buffer;
	if (IS_OK(mediaSample->Lock(&buffer))) {
		if (inputFormat.nPixelFormat != cImage::PF_GREYSCALE_8) {
			mediaSample->Unlock(buffer);
			RETURN_AND_LOG_ERROR_STR(ERR_NOT_SUPPORTED, "CameraPitchEstimator: Only Greyscale images are implemented");
		}
		oImage.SetBits((tUInt8*) buffer, &inputFormat);
		mediaSample->Unlock(buffer);
	}
	memcpy(image.data, oImage.GetBitmap(), inputFormat.nSize);

	RETURN_NOERROR;
}

tResult CameraEstimatorAdapter::ReadAsFloatAndClear(cv::Mat& image, tUInt32 bytePosition, tFloat32 *value) {
	if(bytePosition > image.dataend - image.datastart) {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, "CameraEstimatorAdapter::ReadAsFloatAndClear bytePosition out of bounds");
	}

	union {
	     tUInt8 bytes[4];
	     tFloat32 f;
	} byteToFloat;

	byteToFloat.bytes[0] = image.data[bytePosition];
	byteToFloat.bytes[1] = image.data[bytePosition+1];
	byteToFloat.bytes[2] = image.data[bytePosition+2];
	byteToFloat.bytes[3] = image.data[bytePosition+3];

	*value = byteToFloat.f;

	image.data[bytePosition] = 0;
	image.data[bytePosition+1] = 0;
	image.data[bytePosition+2] = 0;
	image.data[bytePosition+3] = 0;

	RETURN_NOERROR;
}


tResult CameraEstimatorAdapter::StoreToFile() {
	// create absolute path

	ADTF_GET_CONFIG_FILENAME(filePitch);
	cFilename absFilename = filePitch.CreateAbsolutePath(".");

	if (cFileSystem::Exists(filePitch)) {
		cFileSystem::DelFile(filePitch);
	}

	LOG_WARNING(cString::Format("CameraPitch: stored to file %s", absFilename.GetPtr()));
	cFile file;

	file.Open(absFilename.GetPtr(), cFile::OM_Write);

	file.WriteLine(cString::Format("pitch angle in degree"));
	file.WriteLine(cString::Format("%f", cameraEstimator->GetCurrentPitch()));

	file.WriteLine(cString::Format("top down transformation x offset in meter"));
	file.WriteLine(cString::Format("%f", cameraEstimator->GetCurrentYDiff() / 200));

	RETURN_NOERROR;
}

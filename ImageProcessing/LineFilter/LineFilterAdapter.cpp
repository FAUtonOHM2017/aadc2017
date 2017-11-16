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
#include "LineFilterAdapter.h"
#include "ImageProcessingUtils.h"

#include <iostream>

ADTF_FILTER_PLUGIN("LineFilter", OID_ADTF_LineFilter, LineFilterAdapter);

#define LF_NEAR "Field of View::near"
#define LF_FAR "Field of View::far"

#define LF_FOV_H "Field of View::Horizontal"
#define LF_FOV_V "Field of View::Vertical"

#define LF_PITCH "Camera position/rotation::pitch"
#define LF_ROLL "Camera position/rotation::roll"
#define LF_YAW "Camera position/rotation::yaw"

#define LF_CAMERA_X_OFFSET "Camera position/rotation::X"
#define LF_CAMERA_Y_OFFSET "Camera position/rotation::Y"
#define LF_CAMERA_Z_OFFSET "Camera position/rotation::Z"

#define LF_ADAPTIVE_THRESHOLD_CONSTANT "Image filtering::adaptive threshold constant"
#define LF_ADAPTIVE_THRESHOLD_WIDTH "Image filtering::adaptive threshold width"
#define LF_ENABLE_ADAPTIVE_THRESHOLD "Image filtering::enable adaptive thresholding"
#define LF_ENABLE_MEDIAN_FILTER "Image filtering::median filter size 3x3"

#define LS_PITCH_STORAGE "Camera position/rotation::pitch angle file"

#define EXPERIMENTAL_FEATURES 0

tTimeStamp totaltime = 0;
int frameCount = 0;

LineFilterAdapter::LineFilterAdapter(const tChar* __info) :
		cAsyncDataTriggeredFilter(__info) {
	lineFilter = nullptr;

	SetPropertyFloat(LF_NEAR, 0.6);
	SetPropertyStr(LF_NEAR NSSUBPROP_DESCRIPTION, "near distance to transform");
	SetPropertyBool(LF_NEAR NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(LF_FOV_H, 59);
	SetPropertyStr(LF_FOV_H NSSUBPROP_DESCRIPTION, "field of view in degree");
	SetPropertyBool(LF_FOV_H NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(LF_FOV_V, 45);
	SetPropertyStr(LF_FOV_V NSSUBPROP_DESCRIPTION, "field of view in degree");
	SetPropertyBool(LF_FOV_V NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(LF_PITCH, 0);
	SetPropertyFloat(LF_PITCH NSSUBPROP_MAX, 10);
	SetPropertyFloat(LF_PITCH NSSUBPROP_MIN, -30);
	SetPropertyStr(LF_PITCH NSSUBPROP_DESCRIPTION,
			"pitch angle of the camera in degree");
	SetPropertyBool(LF_PITCH NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(LF_ROLL, 0);
	SetPropertyFloat(LF_ROLL NSSUBPROP_MAX, 10);
	SetPropertyFloat(LF_ROLL NSSUBPROP_MIN, -30);
	SetPropertyStr(LF_ROLL NSSUBPROP_DESCRIPTION,
			"roll angle of the camera in degree");
	SetPropertyBool(LF_ROLL NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(LF_YAW, 0);
	SetPropertyFloat(LF_YAW NSSUBPROP_MAX, 10);
	SetPropertyFloat(LF_YAW NSSUBPROP_MIN, -30);
	SetPropertyStr(LF_YAW NSSUBPROP_DESCRIPTION,
			"yaw angle of the camera in degree");
	SetPropertyBool(LF_YAW NSSUBPROP_ISCHANGEABLE, tTrue);

	//car is at (0,0,0) (left= +y)(up= +z)(front= +x)
	SetPropertyFloat(LF_CAMERA_X_OFFSET, 0.285);
	SetPropertyStr(LF_CAMERA_X_OFFSET NSSUBPROP_DESCRIPTION,
			"RGB camera x_position from rear axis");
	SetPropertyBool(LF_CAMERA_X_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(LF_CAMERA_Y_OFFSET, 0.015);
	SetPropertyStr(LF_CAMERA_Y_OFFSET NSSUBPROP_DESCRIPTION,
			"RGB camera y_position from rear axis");
	SetPropertyBool(LF_CAMERA_Y_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(LF_CAMERA_Z_OFFSET, 0.22);
	SetPropertyStr(LF_CAMERA_Z_OFFSET NSSUBPROP_DESCRIPTION,
			"RGB camera z_position from rear axis");
	SetPropertyFloat(LF_CAMERA_Z_OFFSET NSSUBPROP_MIN, -1);
	SetPropertyFloat(LF_CAMERA_Z_OFFSET NSSUBPROP_MAX, +1);
	SetPropertyBool(LF_CAMERA_Z_OFFSET NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat(LF_ADAPTIVE_THRESHOLD_CONSTANT, -20);
	SetPropertyBool(LF_ADAPTIVE_THRESHOLD_CONSTANT NSSUBPROP_ISCHANGEABLE,
			tTrue);

	SetPropertyInt(LF_ADAPTIVE_THRESHOLD_WIDTH, 21);
	SetPropertyBool(LF_ADAPTIVE_THRESHOLD_WIDTH NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(LF_ENABLE_ADAPTIVE_THRESHOLD, tTrue);
	SetPropertyBool(LF_ENABLE_ADAPTIVE_THRESHOLD NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool(LF_ENABLE_MEDIAN_FILTER, tTrue);
	SetPropertyBool(LF_ENABLE_MEDIAN_FILTER NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyStr(LS_PITCH_STORAGE, "../../../../../Camera_Pitch.txt");
	SetPropertyBool(LS_PITCH_STORAGE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(LS_PITCH_STORAGE NSSUBPROP_DESCRIPTION,
			"file which is used for storing the pitch angle");


	SetPropertyStr("Calibration File for used Camera", "");
	SetPropertyBool("Calibration File for used Camera" NSSUBPROP_FILENAME,
	tTrue);
	SetPropertyStr(
			"Calibration File for used Camera" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER,
			"YML Files (*.yml)");
	SetPropertyStr("Calibration File for used Camera" NSSUBPROP_DESCRIPTION,
			"Here you have to set the file with calibration paraemters of the used camera");
}

LineFilterAdapter::~LineFilterAdapter() {

}

tResult LineFilterAdapter::Init(tInitStage eStage, __exception) {
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst)
	{

		LOG_INFO(cString::Format("LineFilter: StageFirst"));
		tSignalValue.StageFirst(__exception_ptr);
		tPoseStruct.StageFirst(__exception_ptr);

		RETURN_IF_FAILED(poseInput.Create("car_pose", tPoseStruct.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&poseInput));

		// Video Input
		RETURN_IF_FAILED(inputVideoPin.Create("Video_RGB", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&inputVideoPin));

		RETURN_IF_FAILED(mask.inputPin.Create("Mask_8Bit", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&mask.inputPin));

		RETURN_IF_FAILED(outputVideoPin.Create("Binary_Image", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&outputVideoPin));
	} else if (eStage == StageNormal) {

		lineFilter = new LineFilter();
		xEstimationErrorOffset = 0;
		timeOfPrevFrame = 0;

		tBool enableMedianFiltering = GetPropertyBool(LF_ENABLE_MEDIAN_FILTER);
		tBool enableAdaptiveThresholding = GetPropertyBool(LF_ENABLE_ADAPTIVE_THRESHOLD);

		nearPlane = GetPropertyFloat(LF_NEAR);
		tFloat32 fov_h = GetPropertyFloat(LF_FOV_H);
		tFloat32 fov_v = GetPropertyFloat(LF_FOV_V);
		xCameraOffset = GetPropertyFloat(LF_CAMERA_X_OFFSET);
		tFloat32 camera_y_offset = GetPropertyFloat(LF_CAMERA_Y_OFFSET);
		tFloat32 camera_z_offset = GetPropertyFloat(LF_CAMERA_Z_OFFSET);
		tFloat32 th_constant = GetPropertyFloat(LF_ADAPTIVE_THRESHOLD_CONSTANT);
		tInt32 th_width = GetPropertyInt(LF_ADAPTIVE_THRESHOLD_WIDTH);

		filePitch = GetPropertyStr(LS_PITCH_STORAGE);
		//ReadFromFile(&_pitch, &xEstimationErrorOffset);




		//Get path of calibration file with camera paramters
		cFilename fileCalibration = GetPropertyStr("Calibration File for used Camera");

		if (fileCalibration.IsEmpty())
		{
			THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not set");
		}

		//Get path of calibration file with camera paramters
		ADTF_GET_CONFIG_FILENAME(fileCalibration);
		fileCalibration = fileCalibration.CreateAbsolutePath(".");
		//check if calibration file with camera paramters exits
		if (!(cFileSystem::Exists(fileCalibration)))
		{
			THROW_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
		}


	    /*! CAMERA MATRIX FROM CALIBRATION FILE */
	    cv::Mat intrinsics;

	    /*! DISTORSION COEFFICIENTS FROM CALIBRATION FILE */
	    cv::Mat distorsion;

		// read the calibration file with camera paramters exits and save to member variable
		readCameraParameters(fileCalibration.GetPtr(), intrinsics, distorsion);
	
		lineFilter->Init(nearPlane, intrinsics, distorsion, _pitch/180*CV_PI, camera_y_offset, camera_z_offset, th_constant, th_width, 1920, 1080);
		lineFilter->EnableMedianFiltering(enableMedianFiltering);
		lineFilter->EnableAdaptiveThresholding(enableAdaptiveThresholding);
		
		
		PropertyChanged(LF_PITCH);
		PropertyChanged(LF_CAMERA_Y_OFFSET);

		tBitmapFormat m_sOutputFormat;
		m_sOutputFormat.nWidth = BINARY_IMAGE_WIDTH;
		m_sOutputFormat.nHeight = BINARY_IMAGE_HEIGHT;
		m_sOutputFormat.nBitsPerPixel = 8;
		m_sOutputFormat.nBytesPerLine = BINARY_IMAGE_WIDTH;
		m_sOutputFormat.nSize = m_sOutputFormat.nBytesPerLine * BINARY_IMAGE_HEIGHT;
		m_sOutputFormat.nPixelFormat = cImage::PF_GREYSCALE_8;
		m_sOutputFormat.nPaletteSize = 0;

		outputVideoPin.SetFormat(&m_sOutputFormat, NULL);

		cache.out = Mat (BINARY_IMAGE_HEIGHT, BINARY_IMAGE_WIDTH, CV_8UC1, Scalar(0));


		LOG_ERROR(cString::Format("LineFilter: StageNormal finish"));

	} else if (eStage == StageGraphReady) {
		tSignalValue.StageGraphReady();
		tPoseStruct.StageGraphReady();

		// get the image format of the input video pin
		cObjectPtr<IMediaType> pType;
		RETURN_IF_FAILED(inputVideoPin.GetMediaType(&pType));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

		// set the image format of the input video pin
		const tBitmapFormat* format = inputVideoPin.GetFormat();
		if (format != NULL) {
			m_sInputFormat = (*format);
			LOG_INFO(cString::Format(
							"LineFilter: StageGraphReady:  Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
							m_sInputFormat.nWidth, m_sInputFormat.nHeight, m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize,
							m_sInputFormat.nPixelFormat));
		} else {
			RETURN_AND_LOG_ERROR_STR(ERR_UNEXPECTED, "LineFilter: StageGraphReady: No InputFormat read from RGB_Video pin");
		}
		/*
		 if (m_sInputFormat.nWidth <= 0 || m_sInputFormat.nWidth > 2000 || m_sInputFormat.nHeight <= 0
		 || m_sInputFormat.nHeight > 2000) {

		 RETURN_AND_LOG_ERROR_STR(ERR_UNEXPECTED, "LineFilter: unexpected image size");
		 }
		 */
		lineFilter->SetCameraSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight);
	}
	RETURN_NOERROR;
}

bool LineFilterAdapter::readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


tResult LineFilterAdapter::PropertyChanged(const tChar* strName) {

	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::PropertyChanged(strName));
	if (lineFilter != nullptr) {

		if (cString::IsEqual(strName, LF_PITCH)
				|| cString::IsEqual(strName, LF_ROLL)
				|| cString::IsEqual(strName, LF_YAW)) {
				
				/*
			if(filePitch.GetLength() < 2){
			}
			else {
				LOG_WARNING("Using pitch from file. Property pitch will be ignored!");
			}*/
			
				_pitch = GetPropertyFloat(LF_PITCH);
			_roll = GetPropertyFloat(LF_ROLL);
			tFloat yaw = GetPropertyFloat(LF_YAW);


			LOG_WARNING(cString::Format("LineFilter: y,p,r:%f, %f, %f",yaw, _pitch, _roll));

			lineFilter->SetRPY(_roll / 180 * CV_PI, _pitch / 180 * CV_PI, yaw / 180 * CV_PI);

		} else if (cString::IsEqual(strName, LF_NEAR))
			lineFilter->SetNearFar(GetPropertyFloat(LF_NEAR));
		//else if (cString::IsEqual(strName, LF_FOV_H)
		//		|| (cString::IsEqual(strName, LF_FOV_V)))
			//lineFilter->SetFOV(GetPropertyFloat(LF_FOV_H),
			//		GetPropertyFloat(LF_FOV_V));

		else if (cString::IsEqual(strName, LF_CAMERA_Y_OFFSET)
				|| cString::IsEqual(strName, LF_CAMERA_Z_OFFSET))
			lineFilter->SetCameraPosition(GetPropertyFloat(LF_CAMERA_Y_OFFSET),
					GetPropertyFloat(LF_CAMERA_Z_OFFSET));
		else if (cString::IsEqual(strName, LF_ADAPTIVE_THRESHOLD_CONSTANT))
			lineFilter->SetAdaptiveThresholdConstant(
					GetPropertyFloat( LF_ADAPTIVE_THRESHOLD_CONSTANT));

		else if (cString::IsEqual(strName, LF_ADAPTIVE_THRESHOLD_WIDTH))
			lineFilter->SetAdaptiveThresholdWidth(
					GetPropertyInt(LF_ADAPTIVE_THRESHOLD_WIDTH));
		else if (cString::IsEqual(strName, LF_ENABLE_ADAPTIVE_THRESHOLD))

			lineFilter->EnableAdaptiveThresholding(
					GetPropertyBool(LF_ENABLE_ADAPTIVE_THRESHOLD));
		else if (cString::IsEqual(strName, LF_ENABLE_MEDIAN_FILTER))
			lineFilter->EnableMedianFiltering(
					GetPropertyBool(LF_ENABLE_MEDIAN_FILTER));
	}

	RETURN_NOERROR;
}

tResult LineFilterAdapter::Shutdown(tInitStage eStage,
		ucom::IException** __exception_ptr) {
	if (eStage == StageNormal) {
		delete lineFilter;
	}

	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult LineFilterAdapter::OnPinEvent(IPin* source, tInt nEventCode,
		tInt nParam1, tInt nParam2, IMediaSample* mediaSample) {
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);

	adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(
			&(sync.criticalSection_OnPinEvent));

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (source == &inputVideoPin) {
			{
				if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN) {
					RETURN_IF_FAILED(
							UpdateInputImageFormat(inputVideoPin.GetFormat()));
				}

				if (sync.bufferCount > 1) {
					sync.bufferCount = 0;
					ClearAsyncQueue();
					sync.PrintCleared();
				}

				sync.IncreaseBufferCount();
			}
			return cAsyncDataTriggeredFilter::OnPinEvent(source, nEventCode,
					nParam1, nParam2, mediaSample);
		} else if (source == &poseInput) {

			TPoseStruct::Data data;
			tPoseStruct.Read(mediaSample, &data);

			CarPose current(Point2f(data.f32_x, data.f32_y), data.f32_yaw,
					data.f32_pitch, data.f32_roll, mediaSample->GetTime(),
					false);

                        carPose.push_front(current);
                        //static int imuCounter = 0;
                        //imuCounter++;
                        //if(imuCounter % 20 == 0){
                        //    //set pitch form imu
                        //    LOG_WARNING(cString::Format("CAR POSE: roll %f pitch %f ", current.roll, current.pitch));
                        //    lineFilter->SetRPY(_roll / 180 * CV_PI,
                        //                _pitch / 180 * CV_PI + current.pitch*10, 0);
                        //}
		} else if (source == &mask.inputPin) {
			if (mask.firstFrame) {
				cObjectPtr<IMediaType> type;
				RETURN_IF_FAILED(mask.inputPin.GetMediaType(&type));

				cObjectPtr<IMediaTypeVideo> typeDepthImage;
				RETURN_IF_FAILED(
						type->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&typeDepthImage));

				const tBitmapFormat* format = typeDepthImage->GetFormat();
				if (format == NULL) {
					LOG_ERROR(
							"ObstacleDetec: No Bitmap information found on DepthImagePin.");
					RETURN_ERROR(ERR_NOT_SUPPORTED);
				}
				/* assign format of received input video to variable 'format' */
				mask.dInputFormat.nPixelFormat = format->nPixelFormat;
				mask.dInputFormat.nWidth = format->nWidth;
				mask.dInputFormat.nHeight = format->nHeight;
				mask.dInputFormat.nBitsPerPixel = format->nBitsPerPixel;
				mask.dInputFormat.nBytesPerLine = format->nBytesPerLine;
				mask.dInputFormat.nSize = format->nSize;
				mask.dInputFormat.nPaletteSize = format->nPaletteSize;

				LOG_WARNING(
						cString::Format("width %d, height %d",
								mask.dInputFormat.nWidth,
								mask.dInputFormat.nHeight));

				/* check for format errors */
				/*
				if (mask.dInputFormat.nWidth <= 0
						|| mask.dInputFormat.nWidth > 800
						|| mask.dInputFormat.nHeight <= 0
						|| mask.dInputFormat.nHeight > 800) {
					RETURN_AND_LOG_ERROR_STR(ERR_UNEXPECTED,
							cString::Format(
									"LineFilter: Unexpected depth image size, since format with width: %d, height: %d is not supported!",
									m_sInputFormat.nWidth,
									m_sInputFormat.nHeight));
				}
				*/

				//* if receiving the input format information and processing of first frame was successful, flag is set to false */
				mask.firstFrame = tFalse;
			}

			const tVoid* srcBuffer;
			//receiving data from input sample
			if (IS_OK(mediaSample->Lock(&srcBuffer))) {
				try {
					Mat temp = Mat(mask.dInputFormat.nHeight,
							mask.dInputFormat.nWidth, CV_8UC1,
							(tVoid*) srcBuffer,
							mask.dInputFormat.nBytesPerLine);
					temp.copyTo(mask.image);
				} catch (cv::Exception &e) {
					LOG_ERROR(e.msg.c_str());
				}
			}
			mediaSample->Unlock(srcBuffer);
		}
	} else if (nEventCode == IPinEventSink::PE_MediaTypeChanged) {
		if (source == &inputVideoPin) {
			//the input format was changed, so the imageformat has to changed in this filter also
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(inputVideoPin.GetMediaType(&pType));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(
					pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

			const tBitmapFormat* format = inputVideoPin.GetFormat();
			if (format != NULL) {
				m_sInputFormat = (*format);
				LOG_WARNING(
						cString::Format(
								"LineFilter: PE_MediaTypeChanged: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
								m_sInputFormat.nWidth, m_sInputFormat.nHeight,
								m_sInputFormat.nBytesPerLine,
								m_sInputFormat.nSize,
								m_sInputFormat.nPixelFormat));
			}
		}
	}

	RETURN_NOERROR;
}

tResult LineFilterAdapter::OnAsyncPinEvent(IPin* source, tInt nEventCode,
		tInt nParam1, tInt nParam2, IMediaSample* mediaSample) {
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);
	{
		adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(
				&(sync.criticalSection_OnPinEvent));
		sync.DecreaseBufferCount();
	}
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		if (source == &inputVideoPin) {
			if (m_sInputFormat.nPixelFormat == IImage::PF_UNKNOWN) {
				RETURN_IF_FAILED(
						UpdateInputImageFormat(inputVideoPin.GetFormat()));
			}

			RETURN_IF_FAILED(ProcessVideo(mediaSample));
		}
	}

	RETURN_NOERROR;
}

tResult LineFilterAdapter::ProcessVideo(IMediaSample* sample) {
	RETURN_IF_FAILED(ConvertInputImageRGB(cache.image, sample));

	//tTimeStamp time = _clock->GetStreamTime();
	{
		adtf_util::cSynchronizer _sync_criticalSection_OnPinEvent(
				&(sync.criticalSection_OnPinEvent));

		lineFilter->SetValidPixels(mask.image, cache.image.size(),
				!mask.firstFrame);

	}
	/*
	 totaltime += (_clock->GetStreamTime() - time);
	 frameCount++;
	 std::cout << totaltime << " " << (totaltime / frameCount) << std::endl;
	 */


	try {
		RETURN_IF_FAILED(
				lineFilter->GetTopDownView(cache.image, cache.out,
						m_sInputFormat.nPixelFormat == cImage::PF_BGR_888));
	} catch (cv::Exception &e) {
		LOG_ERROR(e.msg.c_str());
	}

	if (carPose.size() > 0) {
		CarPose pose = carPose.front();
		putText(cache.out,
				cv::String(
						cString::Format("%.2f %.2f %.0f", pose.position.x,
								pose.position.y, 180 / CV_PI * pose.yaw).GetPtr()),
				Point2f(0, 17), CV_FONT_NORMAL, 0.4, Scalar(155));
#if (EXPERIMENTAL_FEATURES == 0)
		carPose.clear();
#endif
	}

	timeOfPrevFrame = sample->GetTime();

	tFloat32 pitch = lineFilter->GetPitch();
	WriteAsFloat(cache.out, 0, pitch);
	cv::Point2f frameCoord = lineFilter->GetTopLeftWorldCoordinates();

	WriteAsFloat(cache.out, 4,
			frameCoord.x + xCameraOffset - xEstimationErrorOffset);
	WriteAsFloat(cache.out, 8, frameCoord.y);
	WriteAsFloat(cache.out, 12, nearPlane);

	//generate output
	cObjectPtr<IMediaSample> rgbSample;
	if (IS_OK(AllocMediaSample(&rgbSample))) {
		rgbSample->Update(sample->GetTime(), cache.out.data,
				tInt32(cache.out.cols * cache.out.rows), 0);

		outputVideoPin.Transmit(rgbSample);
	}

	RETURN_NOERROR;
}

tResult LineFilterAdapter::ConvertInputImageRGB(cv::Mat &image,
		IMediaSample* sample) {

	if (m_sInputFormat.nPixelFormat != cImage::PF_RGB_888
			&& m_sInputFormat.nPixelFormat != cImage::PF_BGR_888) {
		//keep in mind that it is confusing that the data is actually stored in the order RED GREEN BLUE, but is called BGR in ADTF
		RETURN_AND_LOG_ERROR_STR(ERR_NOT_SUPPORTED,
				"LineFilter: Only RGB/BGR images are supported");
	}

	const tVoid* srcBuffer;

	//receiving data from input sample, dont copying input image at all
	if (IS_OK(sample->Lock(&srcBuffer))) {
		try {
			image = Mat(m_sInputFormat.nHeight, m_sInputFormat.nWidth, CV_8UC3,
					(tVoid*) srcBuffer, m_sInputFormat.nBytesPerLine);
		} catch (cv::Exception &e) {
			LOG_ERROR(e.msg.c_str());
		}
	}
	sample->Unlock(srcBuffer);

	RETURN_NOERROR;
}

tResult LineFilterAdapter::ReadFromFile(tFloat32 *pitch, tFloat32 *yOffset) {
	if (filePitch.GetLength() < 2) {
		LOG_WARNING("LineFilter: No filename for the pitch angle is given");
		RETURN_NOERROR;
	}

	// create absolute path
	ADTF_GET_CONFIG_FILENAME(filePitch);
	cFilename absFilename = filePitch.CreateAbsolutePath(".");

	if (cFileSystem::Exists(filePitch) == tFalse) {
		LOG_ERROR("LineFilter: File for stored pitch angle does not exist");
		RETURN_NOERROR;
	}

	cFile file;
	file.Open(absFilename.GetPtr(), cFile::OM_Read);

	cString tmp;
	file.ReadLine(tmp); // comment line
	file.ReadLine(tmp);

	if (tmp.GetLength() >= 1) {
		*pitch = tmp.AsFloat64();
	} else {
		LOG_ERROR(
				cString::Format(
						"LineFilter: Something went wrong while reading file %s. Using default pitch angle",
						absFilename.GetPtr()));
	}

	file.ReadLine(tmp); // comment line
	file.ReadLine(tmp);
	if (tmp.GetLength() >= 1) {
		*yOffset = tmp.AsFloat64();
	} else {
		LOG_ERROR(
				cString::Format(
						"LineFilter: Something went wrong while reading file %s. Using default pitch angle",
						absFilename.GetPtr()));
	}

	RETURN_NOERROR;
}

tResult LineFilterAdapter::WriteAsFloat(cv::Mat& image, tUInt32 bytePosition,
		tFloat32 value) {
	if (bytePosition > image.dataend - image.datastart) {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX,
				"LineFilter: ReadAsFloatAndClear bytePosition out of bounds");
	}

	union {
		tUInt8 bytes[4];
		tFloat32 f;
	} floatToByte;

	floatToByte.f = value;

	image.data[bytePosition] = floatToByte.bytes[0];
	image.data[bytePosition + 1] = floatToByte.bytes[1];
	image.data[bytePosition + 2] = floatToByte.bytes[2];
	image.data[bytePosition + 3] = floatToByte.bytes[3];

	RETURN_NOERROR;
}

tResult LineFilterAdapter::UpdateInputImageFormat(
		const tBitmapFormat* pFormat) {
	if (pFormat != NULL) {
		//update member variable
		//  m_sInputFormat = (*pFormat);
		cMemoryBlock::MemCopy(&m_sInputFormat, pFormat, sizeof(tBitmapFormat));
		LOG_INFO(
				cString::Format(
						"Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
						m_sInputFormat.nWidth, m_sInputFormat.nHeight,
						m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize,
						m_sInputFormat.nPixelFormat));

		lineFilter->SetCameraSize(m_sInputFormat.nWidth,
				m_sInputFormat.nHeight);
		//create the input matrix
		//  RETURN_IF_FAILED(BmpFormat2Mat(m_sInputFormat, m_inputImage));
	}
	RETURN_NOERROR;
}

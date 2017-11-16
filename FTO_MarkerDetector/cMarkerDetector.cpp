/**
 Copyright (c)
 Audi Autonomous Driving Cup. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
 4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


 **********************************************************************
 * $Author:: kuckal  $  $Date:: 2017-05-22 10:14:17#$ $Rev:: 63667   $
 **********************************************************************/

#include "stdafx.h"
#include "aadc_roadSign_enums.h"
#include "cMarkerDetector.h"

ADTF_FILTER_PLUGIN("FTO_Marker Detector Plugin", OID_ADTF_MARKERDETECTFILTER,
		cMarkerDetector)

cMarkerDetector::cMarkerDetector(const tChar* __info) :cAsyncDataTriggeredFilter(__info)  {
	SetPropertyInt("ROI::XOffset", 500);
	SetPropertyStr("ROI::XOffset" NSSUBPROP_DESCRIPTION,
			"X Offset for Region of Interest Rectangular");
	SetPropertyBool("ROI::XOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::YOffset", 350);
	SetPropertyStr("ROI::YOffset" NSSUBPROP_DESCRIPTION,
			"Y Offset for Region of Interest Rectangular");
	SetPropertyBool("ROI::YOffset" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::Width", 460);
	SetPropertyStr("ROI::Width" NSSUBPROP_DESCRIPTION,
			"Width of the Region of Interest Rectangular");
	SetPropertyBool("ROI::Width" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyInt("ROI::Height", 360);
	SetPropertyStr("ROI::Height" NSSUBPROP_DESCRIPTION,
			"Height of the Region of Interest Rectangular");
	SetPropertyBool("ROI::Height" NSSUBPROP_ISCHANGEABLE, tTrue);



	SetPropertyFloat("ROI::MaxDistance", 1.5);
	SetPropertyStr("ROI::MaxDistance" NSSUBPROP_DESCRIPTION,
			"Max Distance of the Region of Interest Rectangular");
	SetPropertyBool("ROI::MaxDistance" NSSUBPROP_ISCHANGEABLE, tTrue);


	SetPropertyInt("DropFrameCounter", 10);
	SetPropertyStr("DropFrameCounter" NSSUBPROP_DESCRIPTION,
			"Only process every x frame");
	SetPropertyBool("DropFrameCounter" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Tvec::XMin", 0);
	SetPropertyStr("Tvec::XMin" NSSUBPROP_DESCRIPTION, "Translation X Min");
	SetPropertyBool("Tvec::XMin" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Tvec::YMin", -1);
	SetPropertyStr("Tvec::YMin" NSSUBPROP_DESCRIPTION, "Translation Y Min");
	SetPropertyBool("Tvec::YMin" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Tvec::ZMin", 0);
	SetPropertyStr("Tvec::ZMin" NSSUBPROP_DESCRIPTION, "Translation Z Min");
	SetPropertyBool("Tvec::ZMin" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Tvec::XMax", 1);
	SetPropertyStr("Tvec::XMax" NSSUBPROP_DESCRIPTION, "Translation X Max");
	SetPropertyBool("Tvec::XMax" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Tvec::YMax", 1);
	SetPropertyStr("Tvec::YMax" NSSUBPROP_DESCRIPTION, "Translation Y Max");
	SetPropertyBool("Tvec::YMax" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Tvec::ZMax", 2);
	SetPropertyStr("Tvec::ZMax" NSSUBPROP_DESCRIPTION, "Translation Z Max");
	SetPropertyBool("Tvec::ZMax" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Rvec::XMin", 2.6);
	SetPropertyStr("Rvec::XMin" NSSUBPROP_DESCRIPTION, "Rotation X Min");
	SetPropertyBool("Rvec::XMin" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Rvec::YMin", -0.1);
	SetPropertyStr("Rvec::YMin" NSSUBPROP_DESCRIPTION, "Rotation Y Min");
	SetPropertyBool("Rvec::YMin" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Rvec::ZMin", 0.1);
	SetPropertyStr("Rvec::ZMin" NSSUBPROP_DESCRIPTION, "Rotation Z Min");
	SetPropertyBool("Rvec::ZMin" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Rvec::XMax", 3.3);
	SetPropertyStr("Rvec::XMax" NSSUBPROP_DESCRIPTION, "Rotation X Max");
	SetPropertyBool("Rvec::XMax" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Rvec::YMax", 0.9);
	SetPropertyStr("Rvec::YMax" NSSUBPROP_DESCRIPTION, "Rotation Y Max");
	SetPropertyBool("Rvec::YMax" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyFloat("Rvec::ZMax", 0.9);
	SetPropertyStr("Rvec::ZMax" NSSUBPROP_DESCRIPTION, "Rotation Z Max");
	SetPropertyBool("Rvec::ZMax" NSSUBPROP_ISCHANGEABLE, tTrue);

	SetPropertyBool("Debug Output to Console", tFalse);
	SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION,
			"If enabled additional debug information is printed to the console (Warning: decreases performance).");

	SetPropertyStr("Calibration File for used Camera", "");
	SetPropertyBool("Calibration File for used Camera" NSSUBPROP_FILENAME,
	tTrue);
	SetPropertyStr(
			"Calibration File for used Camera" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER,
			"YML Files (*.yml)");
	SetPropertyStr("Calibration File for used Camera" NSSUBPROP_DESCRIPTION,
			"Here you have to set the file with calibration paraemters of the used camera");

	SetPropertyStr("Detector Paramater File", "");
	SetPropertyBool("Detector Paramater File" NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(
			"Detector Paramater File" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER,
			"YML Files (*.yml)");
	SetPropertyStr("Detector Paramater File" NSSUBPROP_DESCRIPTION,
			"Here you have to set the file with the parameters with the detector params");

	SetPropertyFloat("Size of Markers", 0.117f);
	SetPropertyStr("Size of Markers" NSSUBPROP_DESCRIPTION,
			"Size (length of one side) of markers in m");

	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::Start",
			m_oProcessStart);
	UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::End",
			m_oProcessEnd);
	UCOM_REGISTER_TIMING_SPOT(
			cString(OIGetInstanceName()) + "::MarkerDetection::Start",
			m_oPreMarkerDetect);
	UCOM_REGISTER_TIMING_SPOT(
			cString(OIGetInstanceName()) + "::MarkerDetection::End",
			m_oPostMarkerDetect);
	UCOM_REGISTER_TIMING_SPOT(
			cString(OIGetInstanceName()) + "::PoseEstimation::Start",
			m_oPrePoseEstimation);
	UCOM_REGISTER_TIMING_SPOT(
			cString(OIGetInstanceName()) + "::PoseEstimation::End",
			m_oPostPoseEstimation);
}

cMarkerDetector::~cMarkerDetector() {
}

tResult cMarkerDetector::PropertyChanged(const tChar* strName) {
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::PropertyChanged(strName));
	//associate the properties to the member
	if (cString::IsEqual(strName, "ROI::Width"))
		m_filterProperties.ROIWidth = GetPropertyInt("ROI::Width");
	else if (cString::IsEqual(strName, "ROI::Height"))
		m_filterProperties.ROIHeight = GetPropertyInt("ROI::Height");
	else if (cString::IsEqual(strName, "ROI::XOffset"))
		m_filterProperties.ROIOffsetX = GetPropertyInt("ROI::XOffset");
	else if (cString::IsEqual(strName, "ROI::YOffset"))
		m_filterProperties.ROIOffsetY = GetPropertyInt("ROI::YOffset");
	else if (cString::IsEqual(strName, "Rvec::ZMin"))
		m_filterProperties.RvecOffsetZMin = GetPropertyInt("Rvec::ZMin");
	else if (cString::IsEqual(strName, "Rvec::YMin"))
		m_filterProperties.RvecOffsetYMin = GetPropertyInt("Rvec::YMin");
	else if (cString::IsEqual(strName, "Rvec::XMin"))
		m_filterProperties.RvecOffsetXMin = GetPropertyInt("Rvec::XMin");
	else if (cString::IsEqual(strName, "Tvec::ZMin"))
		m_filterProperties.TvecOffsetZMin = GetPropertyInt("Tvec::ZMin");
	else if (cString::IsEqual(strName, "Tvec::YMin"))
		m_filterProperties.TvecOffsetYMin = GetPropertyInt("Tvec::YMin");
	else if (cString::IsEqual(strName, "Tvec::XMin"))
		m_filterProperties.TvecOffsetXMin = GetPropertyInt("Tvec::XMin");
	else if (cString::IsEqual(strName, "Rvec::ZMax"))
		m_filterProperties.RvecOffsetZMax = GetPropertyInt("Rvec::ZMax");
	else if (cString::IsEqual(strName, "Rvec::YMax"))
		m_filterProperties.RvecOffsetYMax = GetPropertyInt("Rvec::YMax");
	else if (cString::IsEqual(strName, "Rvec::XMax"))
		m_filterProperties.RvecOffsetXMax = GetPropertyInt("Rvec::XMax");
	else if (cString::IsEqual(strName, "Tvec::ZMax"))
		m_filterProperties.TvecOffsetZMax = GetPropertyInt("Tvec::ZMax");
	else if (cString::IsEqual(strName, "Tvec::YMax"))
		m_filterProperties.TvecOffsetYMax = GetPropertyInt("Tvec::YMax");
	else if (cString::IsEqual(strName, "Tvec::XMax"))
		m_filterProperties.TvecOffsetXMax = GetPropertyInt("Tvec::XMax");
	else if (cString::IsEqual(strName, "ROI::MaxDistance"))
		m_filterProperties.MaxDistance = GetPropertyFloat("ROI::MaxDistance");
	else if (cString::IsEqual(strName, "DropFrameCounter"))
		m_filterProperties.DropFrameCounter = GetPropertyInt("DropFrameCounter");



	RETURN_NOERROR;
}

tResult cMarkerDetector::Init(tInitStage eStage, __exception)
{
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));
	if (eStage == StageFirst)
	{
		tActionStruct.StageFirst(__exception_ptr);
		tFeedbackStruct.StageFirst(__exception_ptr);
		_tRoadSignExtStruct.StageFirst(__exception_ptr);


		RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct.GetMediaType(), 0));
		RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

		RETURN_IF_FAILED(actionInput.Create("action", tActionStruct.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&actionInput));

		RETURN_IF_FAILED(tTrafficSignOutput.Create("tTraficSignOutput", _tRoadSignExtStruct.GetMediaType(), static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&tTrafficSignOutput));

		// create the extended road sign OutputPin
		RETURN_IF_FAILED(m_oPinRoadSignExt.Create("RoadSign_ext", _tRoadSignExtStruct.GetMediaType(), this));
		RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSignExt));



		//create the video rgb input pin
		RETURN_IF_FAILED(m_oPinInputVideo.Create("Video_RGB_input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

		//create the video rgb output pin
		RETURN_IF_FAILED(m_oPinOutputVideo.Create("Video_RGB_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
		RETURN_IF_FAILED(RegisterPin(&m_oPinOutputVideo));

		// create the description manager
		cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

		// create the description for the road sign pin
		tChar const * strDesc = pDescManager->GetMediaDescription("tRoadSign");
		RETURN_IF_POINTER_NULL(strDesc);
		cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSign", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

		// create the road sign OutputPin
		RETURN_IF_FAILED(m_oPinRoadSign.Create("RoadSign", pType, this));
		RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSign));
		// set the description for the road sign pin
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSign));


		//*** create and register the output pin 1***//
		cObjectPtr<IMediaType> pTypeOut_GCL;
		RETURN_IF_FAILED(AllocMediaType((tVoid**)&pTypeOut_GCL,
						MEDIA_TYPE_COMMAND,
						MEDIA_SUBTYPE_COMMAND_GCL,
						NULL, NULL, __exception_ptr));
		RETURN_IF_FAILED(m_outputPinGCL.Create("GCL_Markers", pTypeOut_GCL, static_cast<IPinEventSink*> (this)));
		RETURN_IF_FAILED(RegisterPin(&m_outputPinGCL));

	}
	else if (eStage == StageNormal)
	{
		// get the propeerties
		m_f32MarkerSize = static_cast<tFloat32>(GetPropertyFloat("Size of Markers"));
		m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
		m_bCamaraParamsLoaded = tFalse;

		//Get path of detector parameter file
		cFilename fileDetectorParameter = GetPropertyStr("Detector Paramater File");
		if (fileDetectorParameter.IsEmpty())
		{
			THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter File for Markers not set");
		}
		//create absolute path for marker configuration file
		ADTF_GET_CONFIG_FILENAME(fileDetectorParameter);
		fileDetectorParameter = fileDetectorParameter.CreateAbsolutePath(".");
		//check if marker configuration file exits
		if (!(cFileSystem::Exists(fileDetectorParameter)))
		{
			THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file for Markers not found");
		}
		//create the detector params
		m_detectorParams = aruco::DetectorParameters::create();
		if (!(readDetectorParameters(fileDetectorParameter.GetPtr(), m_detectorParams)))
		{
			THROW_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file not valid");
		}

		m_Dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

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

		// read the calibration file with camera paramters exits and save to member variable
		readCameraParameters(fileCalibration.GetPtr(), m_Intrinsics, m_Distorsion);
		m_bCamaraParamsLoaded = tTrue;


		lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
		lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_NONE;

	}
	else if (eStage == StageGraphReady)
	{
		tActionStruct.StageGraphReady();
		tFeedbackStruct.StageGraphReady();
		_tRoadSignExtStruct.StageGraphReady();

		// get the image format of the input video pin
		cObjectPtr<IMediaType> pType;
		RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

		cObjectPtr<IMediaTypeVideo> pTypeVideo;
		RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

		// set the image format of the input video pin
		UpdateInputImageFormat(pTypeVideo->GetFormat());

		// set the image format of the output video pin
		UpdateOutputImageFormat(pTypeVideo->GetFormat());

		// IDs were not set yet
		m_bIDsRoadSignSet = tFalse;

		_frameCount = 0;

	}
	RETURN_NOERROR;
}

tResult cMarkerDetector::Shutdown(tInitStage eStage, __exception)
{
	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}


tResult cMarkerDetector::OnPinEvent(IPin* pSource, tInt nEventCode,
		tInt nParam1, tInt nParam2, IMediaSample* pMediaSample) {
	switch (nEventCode) {
	case IPinEventSink::PE_MediaSampleReceived:
		if (pSource == &actionInput){
			sync.lastActionMediaSample = pMediaSample;
			return cAsyncDataTriggeredFilter::OnPinEvent(pSource, nEventCode, nParam1, nParam2, pMediaSample);
		}

		// a new image was received so the processing is started
		else if (pSource == &m_oPinInputVideo){
			UCOM_TIMING_SPOT(m_oProcessStart);
			ProcessVideo(pMediaSample);
			UCOM_TIMING_SPOT(m_oProcessEnd);
			return cAsyncDataTriggeredFilter::OnPinEvent(pSource, nEventCode, nParam1, nParam2, pMediaSample);
		}

		break;
	case IPinEventSink::PE_MediaTypeChanged:
		if (pSource == &m_oPinInputVideo) {
			//the input format was changed, so the imageformat has to changed in this filter also
			cObjectPtr<IMediaType> pType;
			RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

			cObjectPtr<IMediaTypeVideo> pTypeVideo;
			RETURN_IF_FAILED(
					pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

			UpdateInputImageFormat(m_oPinInputVideo.GetFormat());
			UpdateOutputImageFormat(m_oPinInputVideo.GetFormat());
		}
		break;
	default:
		break;
	}
	RETURN_NOERROR;
}

tResult cMarkerDetector::OnAsyncPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* mediaSample) {
	RETURN_IF_POINTER_NULL(mediaSample);
	RETURN_IF_POINTER_NULL(source);

	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
		// a new image was received so the processing is started
		if (source == &actionInput)  {
			ProcessAction(mediaSample);
		}

		else if (source == &m_oPinInputVideo) {
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

tResult cMarkerDetector::ProcessVideo(adtf::IMediaSample* pISample) {
	RETURN_IF_POINTER_NULL(pISample);

	_frameCount++;

	//Only process every 10 frame
	if(_frameCount < m_filterProperties.DropFrameCounter){
		RETURN_NOERROR;
	}
	else{
		_frameCount = 0;
	}

	//creating new pointer for input data
	const tVoid* l_pSrcBuffer;
	//creating matrix for input image
	Mat inputImage;
	// the results from aruco detection
	vector<int> ids;
	vector<vector<Point2f> > corners, rejected;
	vector<Vec3d> rvecs, tvecs;
	//receiving data from input sample, and saving to inputImage
	if (IS_OK(pISample->Lock(&l_pSrcBuffer))) {
		//convert to mat
		inputImage = Mat(m_sInputFormat.nHeight, m_sInputFormat.nWidth, CV_8UC3,
				(tVoid*) l_pSrcBuffer, m_sInputFormat.nBytesPerLine);

		UCOM_TIMING_SPOT(m_oPreMarkerDetect);
		// cv::Rect roi(300, 300, 400, 400);
		//cv::Rect roi(m_filterProperties.ROIOffsetX, m_filterProperties.ROIOffsetY, m_filterProperties.ROIWidth, m_filterProperties.ROIHeight);
		// cv::Mat roiImage(inputImage, roi);
		//cv::Mat roiImage(inputImage, roi);

		// doing the detection of markers in image
		aruco::detectMarkers(inputImage, m_Dictionary, corners, ids,
				m_detectorParams, rejected);
		//aruco::detectMarkers(roiImage, m_Dictionary, corners, ids, m_detectorParams, rejected);

		UCOM_TIMING_SPOT(m_oPostMarkerDetect);
		UCOM_TIMING_SPOT(m_oPrePoseEstimation);

		// if we have the camera pararmeter available we calculate the pose
		if (m_bCamaraParamsLoaded && ids.size() > 0)
			aruco::estimatePoseSingleMarkers(corners, m_f32MarkerSize,
					m_Intrinsics, m_Distorsion, rvecs, tvecs);

		//aruco::drawDetectedMarkers(inputImage, corners, ids);
		//aruco::drawDetectedMarkers(roiImage, corners, ids);

		UCOM_TIMING_SPOT(m_oPostPoseEstimation);

		pISample->Unlock(l_pSrcBuffer);
		transmitGCL(ids, corners);
	} else {
		RETURN_NOERROR;
	}


	Mat outputImage;
	// 1: nothing is drawn, 2: results are drawn so we need the have copy of the frame otherwise the orginal mediasample is modified
	if (m_oPinOutputVideo.IsConnected()) {
		// do a deep copy of the image, otherwise the orginal frame is modified
		outputImage = inputImage.clone();

		// draw the marker in the image
		aruco::drawDetectedMarkers(outputImage, corners, ids);
		//aruco::drawDetectedMarkers(roiImage, corners, ids);
		//aruco::drawDetectedMarkers(inputImage, corners, ids);
		// cv::Rect roi(m_filterProperties.ROIOffsetX, m_filterProperties.ROIOffsetY, m_filterProperties.ROIWidth, m_filterProperties.ROIHeight);
		// cv::Mat roiImage(inputImage, roi);

		if (m_bCamaraParamsLoaded && ids.size() > 0) {
			for (unsigned int i = 0; i < ids.size(); i++) {
				aruco::drawAxis(outputImage, m_Intrinsics, m_Distorsion,
						rvecs[i], tvecs[i], m_f32MarkerSize * 0.5f);
			}
		}
		//creating new media sample for output
		cObjectPtr<IMediaSample> pNewSample;
		RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pNewSample));
		pNewSample->Update(pISample->GetTime(), outputImage.data,
				m_sOutputFormat.nSize, 0);
		m_oPinOutputVideo.Transmit(pNewSample);
	}

	//print marker info and draw the markers in image
	for (unsigned int i = 0; i < ids.size(); i++) {
		if (corners.at(i).at(0).x > m_filterProperties.ROIOffsetX
				&& corners.at(i).at(0).x
						< m_filterProperties.ROIOffsetX
								+ m_filterProperties.ROIWidth
				&& corners.at(i).at(0).y > m_filterProperties.ROIOffsetY
				&& corners.at(i).at(0).y
						< m_filterProperties.ROIOffsetY
								+ m_filterProperties.ROIHeight
				&& sqrt(tvecs[i][0]*tvecs[i][0]+tvecs[i][2]*tvecs[i][2]) < m_filterProperties.MaxDistance) {

			// call the function to transmit a road sign sample with the detected marker
			sendRoadSignStruct(static_cast<tInt16>(ids[i]), getMarkerArea(corners[i]), pISample->GetTime());



			// call the function to transmit a extended road sign sample with the detected marker if the Tvec in the marker was correctly set
			if (m_bCamaraParamsLoaded) {
			sendRoadSignStructExt(static_cast<tInt16>(ids[i]),
						getMarkerArea(corners[i]), pISample->GetTime(),	tvecs[i], rvecs[i]);


				if(actionSub.command == AC_RS_START || actionSub.command == AC_RS_START_PRIORITY_SIGN_LOGGING) {
					if(actionSub.command == AC_RS_START) {
						if(ids[i] != roadsignIDs::MARKER_ID_PARKINGAREA) {
							// replace Roundabout sign with giveway sign
							if(ids[i] == roadsignIDs::MARKER_ID_ROUNDABOUT) {
								ids[i] = roadsignIDs::MARKER_ID_GIVEWAY;
							}

							sendTrafficSign(static_cast<tInt16>(ids[i]),
									getMarkerArea(corners[i]), pISample->GetTime(),
									tvecs[i], rvecs[i]);
						}
					}

					switch (ids[i]) {
					case roadsignIDs::MARKER_ID_ROUNDABOUT: {
						lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
						lastPrioritySign.ui32_status = FB_RS_ROUNDABOUT;
						break;			}
					case roadsignIDs::MARKER_ID_UNMARKEDINTERSECTION:
					case roadsignIDs::MARKER_ID_STOPANDGIVEWAY:
					case roadsignIDs::MARKER_ID_GIVEWAY: {
						lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
						lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_GIVEWAY;

						break;			}
					case roadsignIDs::MARKER_ID_HAVEWAY: {
						lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
						lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_PRIORITY;

						break;			}
                    case roadsignIDs::MARKER_ID_PEDESTRIANCROSSING: {
                    	if(tvecs[i][2] < 0.5) {
                     	   	lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
	                        lastPrioritySign.ui32_status = FB_RS_PEDESTRIANCROSSING;
                        	tFeedbackStruct.Transmit(&feedbackOutput, lastPrioritySign, pISample->GetTime());
                    	}
                    	break;
                    }
					case roadsignIDs::MARKER_ID_PARKINGAREA: {
						parking_sign_received = tTrue; // set parking sign received
						TFeedbackStruct::Data response;
						response.ui32_filterID = F_ROAD_SIGNS;
						response.ui32_status = 3000 + ids[i];

						tFeedbackStruct.Transmit(&feedbackOutput, response, pISample->GetTime());
						break;
					}
					}
				} else {  //AC_RS_START_PARKING_SIGN_LOGGING
					if(ids[i] == roadsignIDs::MARKER_ID_PARKINGAREA) {
						parking_sign_received = tTrue; // set parking sign received
					}
				}
			}
		}
	}

	RETURN_NOERROR;
}

tResult cMarkerDetector::UpdateInputImageFormat(const tBitmapFormat* pFormat) {
	if (pFormat != NULL) {
		m_sInputFormat = (*pFormat);

//                ////LOG_INFO(
//				adtf_util::cString::Format(
//						"Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
//						m_sInputFormat.nWidth, m_sInputFormat.nHeight,
//						m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize,
//						m_sInputFormat.nPixelFormat));

	}

	RETURN_NOERROR;
}

tResult cMarkerDetector::UpdateOutputImageFormat(const tBitmapFormat* pFormat) {
	if (pFormat != NULL) {
		m_sOutputFormat = (*pFormat);

//                //LOG_INFO(
//				adtf_util::cString::Format(
//						"Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d",
//						m_sOutputFormat.nWidth, m_sOutputFormat.nHeight,
//						m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize,
//						m_sOutputFormat.nPixelFormat));

		m_oPinOutputVideo.SetFormat(&m_sOutputFormat, NULL);
	}

	RETURN_NOERROR;
}

tResult cMarkerDetector::sendRoadSignStruct(const tInt16 &i16ID,
		const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame) {
	// create new media sample
	cObjectPtr<IMediaSample> pMediaSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pMediaSample));

	// get the serializer
	cObjectPtr<IMediaSerializer> pSerializer;
	m_pDescriptionRoadSign->GetMediaSampleSerializer(&pSerializer);
	tInt nSize = pSerializer->GetDeserializedSize();

	// alloc the buffer memory
	RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

	{
		// focus for sample write lock
		//write date to the media sample with the coder of the descriptor
		__adtf_sample_write_lock_mediadescription(m_pDescriptionRoadSign,
				pMediaSample, pCoder);

		// get IDs
		if (!m_bIDsRoadSignSet) {
			pCoder->GetID("i16Identifier", m_szIDRoadSignI16Identifier);
			pCoder->GetID("f32Imagesize", m_szIDRoadSignF32Imagesize);
			m_bIDsRoadSignSet = tTrue;
		}

		pCoder->Set(m_szIDRoadSignI16Identifier, (tVoid*) &i16ID);
		pCoder->Set(m_szIDRoadSignF32Imagesize, (tVoid*) &f32MarkerSize);

		pMediaSample->SetTime(timeOfFrame);
	}

	//doing the transmit
	RETURN_IF_FAILED(m_oPinRoadSign.Transmit(pMediaSample));

	//print debug info if activated
//	if (m_bDebugModeEnabled)
//                //LOG_INFO(
//				cString::Format("Sign ID %d detected. Area is: %f", i16ID,
//						f32MarkerSize));

    //LOG_INFO(cString::Format("Sign ID %d detected. Area is: %f", i16ID,f32MarkerSize));

	RETURN_NOERROR;
}

tResult cMarkerDetector::sendRoadSignStructExt(const tInt16 &i16ID,
		const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame,
		const Vec3d &Tvec, const Vec3d &Rvec) {

	if (Tvec[0] > m_filterProperties.TvecOffsetXMin
			and Tvec[0] < m_filterProperties.TvecOffsetXMax
			and Tvec[1] > m_filterProperties.TvecOffsetYMin
			and Tvec[1] < m_filterProperties.TvecOffsetYMax
			and Tvec[2] > m_filterProperties.TvecOffsetZMin
			and Tvec[2] < m_filterProperties.TvecOffsetZMax
//				and Rvec[0] > m_filterProperties.RvecOffsetXMin
//				and Rvec[0] < m_filterProperties.RvecOffsetXMax
//				and Rvec[1] > m_filterProperties.RvecOffsetYMin
//				and Rvec[1] < m_filterProperties.RvecOffsetYMax
//				and Rvec[2] > m_filterProperties.RvecOffsetZMin
//				and Rvec[2] < m_filterProperties.RvecOffsetZMax
					) {
		TRoadSignStructExt::Data data;

		//pCoder->Set(m_szIDRoadSignExtI16Identifier, (tVoid*) &i16ID);
		//pCoder->Set(m_szIDRoadSignExtF32Imagesize, (tVoid*) &f32MarkerSize);
		//convert from cv::Vec3D to array

		data.i16Identifier = i16ID;
		data.f32Imagesize = f32MarkerSize;

		for (int i = 0; i < 3; ++i) {
			data.af32RVec[i] = Rvec[i];
			data.af32TVec[i] = Tvec[i];
		}
		//pCoder->Set("af32TVec", (tVoid*) &tvecFl32array[0]);
		//pCoder->Set("af32RVec", (tVoid*) &rvecFl32array[0]);

		_tRoadSignExtStruct.Transmit(&m_oPinRoadSignExt, data, timeOfFrame);
	}

//	//print debug info if activated
//	if (m_bDebugModeEnabled)
//                //LOG_INFO(
//				cString::Format(
//						"Sign ID %d detected, translation is: %f, %f, %f",
//						i16ID, Tvec[0], Tvec[1], Tvec[2]));

	RETURN_NOERROR;
}

tResult cMarkerDetector::transmitGCL(const vector<int>& ids,
		const vector<vector<Point2f> >& corners) {
	IDynamicMemoryBlock* pGCLCmdDebugInfo;

	if ((ids.size() != corners.size()) || ids.size() == 0
			|| corners.size() == 0) {

		cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_CLEAR);
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_END);
	} else {
		cGCLWriter::GetDynamicMemoryBlock(pGCLCmdDebugInfo);

		//set color
		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FGCOL,
				cColor::Red.GetRGBA());

		//iterate through all corners
		vector<int>::const_iterator itIds = ids.begin();
		for (vector<vector<Point2f> >::const_iterator it = corners.begin();
				it != corners.end(); it++, itIds++) {
			// add the ID as text to middle of marker
			tInt centerMarkerX = 0;
			tInt centerMarkerY = 0;
			for (int p = 0; p < 4; p++) {
				centerMarkerX += tInt(it->at(p).x);
				centerMarkerY += tInt(it->at(p).y);
			}
			centerMarkerX = centerMarkerX / 4;
			centerMarkerY = centerMarkerY / 4;
			cString idTest = cString::FromInt(*itIds);
			cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_TEXT_SIZE_HUGE);
			cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_TEXT,
					centerMarkerX, centerMarkerY, idTest.GetLength());
			cGCLWriter::StoreData(pGCLCmdDebugInfo, idTest.GetLength(),
					idTest.GetPtr());

			// draw marker sides as lines
			for (int j = 0; j < 4; j++) {
				Point2i p0, p1;
				p0 = it->at(j);
				p1 = it->at((j + 1) % 4);
				cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_DRAWLINE,
						p0.x, p0.y, p1.x, p1.y);

			}
			//// draw first corner as big circle to check rotation
			Point2i pFirst;
			pFirst = it->at(0);
			cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_FILLCIRCLE,
					pFirst.x, pFirst.y, 10, 10);
		}

		cGCLWriter::StoreCommand(pGCLCmdDebugInfo, GCL_CMD_END);
	}

	//alloc media sample and transmit it over output pin
	cObjectPtr<IMediaSample> pSample;
	RETURN_IF_FAILED(AllocMediaSample((tVoid** )&pSample));
	RETURN_IF_FAILED(
			pSample->Update(_clock->GetStreamTime(), pGCLCmdDebugInfo->GetPtr(),
					(tInt )pGCLCmdDebugInfo->GetSize(), IMediaSample::MSF_None));
	RETURN_IF_FAILED(m_outputPinGCL.Transmit(pSample));

	cGCLWriter::FreeDynamicMemoryBlock(pGCLCmdDebugInfo);

	RETURN_NOERROR;
}

tResult cMarkerDetector::ProcessAction(IMediaSample* mediaSample) {
	TActionStruct::ActionSub currentAction = tActionStruct.Read_Action(
			sync.lastActionMediaSample, F_ROAD_SIGNS);

	//LOG_INFO(cString::Format("currentAction.enabled = %d ", currentAction.enabled ));
	//LOG_INFO(cString::Format("currentAction.started = %d ", currentAction.started ));
	//LOG_INFO(cString::Format("currentAction.command = %d ", currentAction.command ));

	if (currentAction.enabled == tFalse || currentAction.started == tFalse) {
		lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
		lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_NONE;
	}


	else if (currentAction.command == AC_RS_GET_CROSSING_SIGN) {
		tFeedbackStruct.Transmit(&feedbackOutput, lastPrioritySign, _clock->GetStreamTime());
	}

	else if (currentAction.command == AC_RS_GET_PARKING_SIGN_RECEIVED) {
		TFeedbackStruct::Data feedback;
		feedback.ui32_filterID = F_ROAD_SIGNS;


		if (parking_sign_received) {
			parking_sign_received = tFalse;
			feedback.ui32_status = FB_RS_PARKING_SIGN_RECEIVED;
			tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
		}
		else {
			feedback.ui32_status = FB_RS_NO_PARKING_SIGN_RECEIVED;
			tFeedbackStruct.Transmit(&feedbackOutput, feedback,_clock->GetStreamTime());

		}
	} else if (currentAction.command == AC_RS_RESET_PARKING_SIGN) {
		parking_sign_received = tFalse; // also reset parking status
	} else if (currentAction.command == AC_RS_RESET_CROSSING_SIGN) {
		lastPrioritySign.ui32_filterID = F_ROAD_SIGNS;
		lastPrioritySign.ui32_status = FB_RS_CROSSING_SIGN_NONE;

		TFeedbackStruct::Data feedback;
		feedback.ui32_filterID = F_ROAD_SIGNS;
		feedback.ui32_status = FB_RS_CROSSING_SIGN_RESET;
		tFeedbackStruct.Transmit(&feedbackOutput, feedback,
				_clock->GetStreamTime());
	}

	actionSub = currentAction;

	RETURN_NOERROR;

}

tResult cMarkerDetector::sendTrafficSign(const tInt16 &i16ID,
		const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame,
		const Vec3d &Tvec, const Vec3d &Rvec){

	TRoadSignStructExt::Data data;
	data.i16Identifier = i16ID;
	data.f32Imagesize = f32MarkerSize;


	for (int i = 0; i < 3; ++i) {
		data.af32RVec[i] = Rvec[i];
		data.af32TVec[i] = Tvec[i];
	}

	//data.ui32_signID = i16ID;
	//data.f32_distance = float(sqrt(Tvec[0]*Tvec[0]+Tvec[2]*Tvec[2]));
	_tRoadSignExtStruct.Transmit(&tTrafficSignOutput, data, timeOfFrame);

	RETURN_NOERROR;
}

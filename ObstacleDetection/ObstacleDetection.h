/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Filter implements an obstacle detection based on a received depth image from xtion camera
**********************************************************************
* $Author:: mahill $   $Date:: 2016-03-17 23:47:07#$ $Rev:: 0.4.0   $
**********************************************************************/

#ifndef _OBSTACLE_DETECTION_FILTER_HEADER_
#define _OBSTACLE_DETECTION_FILTER_HEADER_


//#define DEBUG_TO_FILE_POINTCLOUD // log second generated point cloud to file /tmp/point_cloud.dat
//#define DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED // log second generated point cloud to file /tmp/point_cloud_transformed.dat


#define OID_ADTF_OBSTACLE_DETECTION  "adtf.aadc.obstacledetection"

/* enables the debug mode, therefore the depth-image-data is NOT preprocessed, all values are taken into account;
 * decreases performance, but allows printing of a debug output depth video */

//#define DEBUG_MODE_OUTPUT_VIDEO

#include "stdafx.h"


class ObstacleDetection: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_OBSTACLE_DETECTION, "ObstacleDetection", OBJCAT_DataFilter, "ObstacleDetection", 0, 2, 0, "FAUtonOHM");

protected:
	//Input of the depth image
	cVideoPin _inputDepthImagePin;


	#ifdef DEBUG_MODE_OUTPUT_VIDEO
	// output debug depth video
	cVideoPin _outputVideoPin;
	#endif

	cOutputPin _oGCLOutput;

	cVideoPin _outputVideoValidPin;
	cVideoPin _outputDebugOptACCPin;

	cInputPin		_actionInput;
	cOutputPin		_feedbackOutput;

	cInputPin		_targetSpeedInput;
	cInputPin		_steeringAngleInput;
	cOutputPin		_targetSpeedOutput;

	TFeedbackStruct	_tFeedbackStruct_object;
	TActionStruct	_tActionStruct_object;

	TSignalValue _tSignalValueSteeringInput_object;
	TSignalValue _tSignalValueSpeedInput_object;
	TSignalValue _tSignalValueOutput_object;



private:
	/* location of setting-file where pitch angle and offset are stored,
	 * previously created by 'pitch-estimation' */
	cFilename _filePitch;
	/* flag indicating whether pitch is to be read from file */
	tBool _usePitchFromFile;
	/* variable to save pitch correction factor, accounting for offset/deviation
	 *  between rgb-cam (used for calibration) and depth-cam*/
	tFloat32 _pitchCorrection;

	/* Offset parameter for camera position */
	tFloat32 _cameraOffsetX;
	tFloat32 _cameraOffsetY;
	tFloat32 _cameraOffsetZ;

	tFloat32 _camera_DepthToRgbOffest;

	/* properties for manipulated view */
	tFloat32 _carWidth;
	tFloat32 _distanceThresholdMax;
	tFloat32 _distanceThresholdMin;
	tFloat32 _heightThresholdMax;
	tFloat32 _heightThresholdMin;
	tFloat32 _heightThresholdValidmask;


	#ifdef DEBUG_TO_FILE_POINTCLOUD
	fstream file_pointcloud;
	#endif

	#ifdef DEBUG_TO_FILE_POINTCLOUD_TRANSFORMED
	fstream file_pointcloud_trans;
	#endif

	/* global variable to temporarily save the previously received actionSub-structure;
	 *  use get and set methods to access data */
	TActionStruct::ActionSub _actionSub;

	/* variable for temporarily saving current steering angle */
	TSignalValue::Data _curSteeringAngle;

	/* variable for temporarily saving current targetSpeedLimit */
	tFloat32 _curTargetSpeedLimit;


	/* status of optical ACC (has to be disabled for parking) */
	enum operationalStatus{
		OPT_ACC_ENABLED = 1,
		OPT_ACC_DISABLED = 2
	};

	operationalStatus OACC_operational_status;

	/* variables and parameter for OpticACC */
	tFloat32 OACC_wheelbase;
	tFloat32 OACC_carWidth;
	tFloat32 OACC_frontAxisToFrontBumper;
	tFloat32 OACC_obliqueAngleFront;
	tFloat32 OACC_circSegmentDistance;
	tFloat32 OACC_ySearchTolerance;
	tFloat32 OACC_obstacleHeightThreshold_min;
	tFloat32 OACC_obstacleHeightThreshold_max;

	tUInt32 OACC_GridCellOccupiedCounterThreshold;
	tUInt32 OACC_GridCellResolutionIndicator;
	tUInt32 OACC_GridCellResolution;

	tBool debug_occupancyGrid;

	/* name of xml-file for parking mode of ACC */
	cFilename xml_configFileOpticalACC;

	/*! holds the yValues for the supporting points*/
	vector<tFloat32> oAcc_xml_xValues;
	/*! holds the xValues for the supporting points*/
	vector<tFloat32> oAcc_xml_yValues;

	/*! enable/disable warning at reached borders of the xml-file */
	tBool xml_BorderWarningModeEnabled;
	tBool xml_PrintInitialtable;


	tFloat32 _sinDynRange;
	tFloat32 _cosDynRange;

	/* flag indicating whether input video format has to be read */
	tBool _firstFrame;


	/* flags used for logging and debug mode */
	tBool pointcloud_to_file;
	tBool transformed_pointcloud_to_file;
	tBool transformed_logging_completed;
	tBool transformed_pointcloud_to_file_one;
	tBool transformed_logging_completed_one;
	tBool debugModeEnabled;
	tBool extendeddebugModeEnabled;
	tUInt32 _debugLogEventToggle;
	#ifdef DEBUG_MODE_OUTPUT_VIDEO
	tUInt32 debugType;
	#endif
	tBool _showGCLDebug;
	tBool showGCLDebug_extendedLog;
	tBool debugACCObstaclepixelcount;
	tBool _showOaccBinaryImageDebug;

	/* variables to store cosinus and sinus values of pitch */
	tFloat32 _cosPitch;
	tFloat32 _sinPitch;

	tBitmapFormat _inputFormat;

	tBool _runningState;

	struct ProcessedCloudData {
		tFloat32 targetSpeedLimit;
		cv::Mat cv_validPixelDepthImage;
		ProcessedCloudData(){
			targetSpeedLimit = 0.0f;
		}
	};

	struct GridcellElement{
		tUInt32 elem_counter;
		cv::Point2f CoordinateSum;

		GridcellElement(){
			elem_counter = 0;
		}
	};

	struct GridcellElement_GCLdebug{
			tUInt32 elem_counter;
			cv::Point2f coordinateSum;
			cv::Point2f max_valueCoord;
			cv::Point2f min_valueCoord;

			GridcellElement_GCLdebug(){
				elem_counter = 0;
			}
		};

		struct IntrinsicData {
			tFloat32 fx;
			tFloat32 fy;
			tFloat32 cx;
			tFloat32 cy;
		} _depthIntrinsicData, _colorIntrinsicData;

		struct Transformation {
			tFloat32 rotation[9];
			tFloat32 translation[3];
		} _depthToColor;

	/* structure for region of interest, to be checked for obstacles */
	struct roi {
		cv::Point2f bottomleft_corner;
		tFloat32 roi_y_width;
		tFloat32 roi_x_height;

		roi(){
			bottomleft_corner.x = 0.0f;
			bottomleft_corner.y = 0.0f;
			roi_y_width = 0.0f;
			roi_x_height = 0.0f;
		}
	};

	/* status of checked region */
	enum occupancy {
		INITIALILZED = -2,
		ERROR = -1,
		FREE_SPACE = 0,
		OCCUPIED_SPACE = 1,
		STATE_PENDING = 2,
		OCCUPIED_STATIC = 3,
	};

	enum regionType {
		REGIONTYPE_INTRSC_CROSSRIGHT = 0,
		REGIONTYPE_INTRSC_ONCOMING = 1,
		REGIONTYPE_OVRTK_ONCOMING = 2,
		REGIONTYPE_OVRTK_OWNLANE = 3,
		REGIONTYPE_CRS_PRKNG_ONCOMING = 4,
		REGIONTYPE_CRS_PRKNG_CROSSTRAFIC = 5
	};

	/* specific regions of interest that need to be checked */
	/* for checking at intersections */
	roi roi_intersection_onComingTraffic;
	roi roi_intersection_crossTrafficRight;

	/* for checking during 'on-the-road' mode*/
	roi roi_overtake_onComingTraffic;
	roi roi_overtake_originalLane;
	roi roi_overtake_ownLaneStraight;
	roi roi_overtake_ownLaneStraight_lefthalf;

	/* for checking before and after parking maneuvers */
	roi roi_parking_cross_onComing;
	roi roi_parking_cross_crossTraffic; //before pull-out

	/* counter indicating the number of free frames that occurred in a non-interrupted sequence */
	tUInt32 _freeFrameCounter;
	tUInt32 _occupiedFrameCounter;



	tUInt32 _occupiedCoordinateCounterUpperBound[6];
	tUInt32 _occupiedFrameCounterLowerBound[6];
	tUInt32 _freeFrameCounterLowerBound[6];

	/* intersection mode */
	tUInt32 intrsc_freeFrameCounterLowerBound;
	tUInt32 intrsc_occupiedFrameCounterLowerBound;

	/* overtaking mode */
	/* bound indicating the number of free frames that occurred in a non-interrupted sequence */
	//tUInt32 ovrtk_freeFrameCounterLowerBound_oncoming;
	/* bound indicating occupied pixel */
	//tUInt32 ovrtk_OccupiedCoordinateCounterUpperBound_onComing;

	/* bound indicating the number of free frames that occurred in a non-interrupted sequence */
	//tUInt32 ovrtk_freeFrameCounterLowerBound_ownLaneStraight;
	/* bound indicating occupied pixel */
	//tUInt32 ovrtk_OccupiedCoordinateCounterUpperBound_ownLaneStraight;
	//tUInt32 overtake_ownlane_occupiedFrameCounterLowerBound;
	//tUInt32 overtake_oncoming_occupiedFrameCounterLowerBound;


	/* parking mode */
	//tUInt32 crsPrkng_OccupiedCoordinateCounterUpperBound_crossRight;
	//tUInt32 crsPrkng_freeFrameCounterLowerBound_crossRight;
	//tUInt32 crsPrkng_OccupiedCoordinateCounterUpperBound_onComing;
	//tUInt32 crsPrkng_freeFrameCounterLowerBound_oncoming;
	//tUInt32 crsPrkng_occupiedFrameCounterLowerBound;


public:
	ObstacleDetection(const tChar*);
	virtual ~ObstacleDetection();

	tResult CreateInputPins(__exception);
	tResult CreateOutputPins(__exception);
	// implements cFilter
	tResult Init(tInitStage eStage, __exception = NULL);
	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

	tResult PropertyChanged(const tChar* strName);

	tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

private:
	#ifdef DEBUG_MODE_OUTPUT_VIDEO
	cv::Mat generateDebugDepthFromTransformedCloud(const std::vector<cv::Point3f> &m_cloud);
	#endif

	/* Method printing objects into RGB_image, delivered by referenceSegmentLine to GCL */
	tResult CreateAndTransmitGCL(const std::vector<cv::Point2i> &referenceSegmentLine);

	/* Method printing the dynamic calculated driving lane - check area */
	tResult CreateAndTransmitGCL_dynamicDrive(const TSignalValue::Data steeringAngle);

	/* Deprecated Method, has been included into 'ProcessOpticalAccAndGenerateValidationImage */
	//cv::Mat generateValidPixelDepthFromTransformedCloud(const std::vector<cv::Point3f> &m_cloud);

	tResult ProcessVideo(IMediaSample* sample);

	tResult GetInputDepthImage(cv::Mat &image, IMediaSample* mediaSample);

	tResult generatePointCloudFromDepthImage(const cv::Mat& depthImage, vector<Point3f>& pointCloud);
	
	tResult transformPointCloudToCarPerspective(const std::vector<cv::Point3f> &cloud, std::vector<cv::Point3f> &transformedCloud);
	
	occupancy CheckROIforObstacles(roi regionToCheck, regionType type, const std::vector<cv::Point3f> &cloud);
	occupancy CheckROIforObstacles(roi regionToCheck_OnComing, roi regionToCheck_CrossRight, const std::vector<cv::Point3f> &cloud);
	tUInt32 CheckROI(roi regionToCheck, const std::vector<cv::Point3f> &cloud);
	tBool checkROI(const cv::Point3f &point, const roi &roiToCheck) const;

	/* Methods checks for Obstacles in current field of view, returns a reduced speed depending on position of recognized obstacles */
	ProcessedCloudData ProcessOpticalAccAndGenerateValidationImage(const TSignalValue::Data steeringAngle, const std::vector<cv::Point3f> &transformedCloud, const std::vector<cv::Point3f> &orgCloud);
	cv::Mat generateDebugOutputOpticACC(TSignalValue::Data steeringAngle, const std::vector<cv::Point3f> &cloud);

	tResult ProcessAction(TActionStruct::ActionSub actionSub);

	tResult TransmitFeedbackNoObstacle();
	tResult TransmitFeedbackStaticObstacle();
	tResult TransmitTargetSpeed(TSignalValue::Data newTargetSpeed);

	TActionStruct::ActionSub GetCurrActionSub();
	tResult SetCurrActionSub(TActionStruct::ActionSub actionSub);

	tBool GetRunningState();
	tResult SetRunningState(tBool state);

	TSignalValue::Data GetSteeringAngle();
	tResult SetSteeringAngle(TSignalValue::Data steeringAngle);

	tFloat32 GetTargetSpeedLimit();
	tResult SetTargetSpeedLimit(tFloat32 targetSpeedLimit);

	operationalStatus GetOperationalStatus();
	tResult SetOperationalStatus(operationalStatus opStatus);

	tResult ReadFromFile(tFloat32 *pitch);

	/* */
	/** Necessary code for loading an using an xml-data file **/
	/*! reads the xml file which is set in the filter properties */
	tResult LoadConfigurationData(cFilename& m_fileConfig, vector<tFloat32>& m_xValues, vector<tFloat32>& m_yValues);
	/*! checks the loaded configuration data; checks if the x-values are in increasing order*/
	tResult CheckConfigurationData(cFilename m_fileConfig, vector<tFloat32> m_xValues);

	/*! doing the linear interpolation
	 @param fl32InputValue the value which should be interpolated
	 */
	tFloat32 GetLinearInterpolatedValue(tFloat32 fl32InputValue, vector<tFloat32> m_xValues, vector<tFloat32> m_yValues);


	tUInt32 GetResolutionFromIndicator(tUInt32 OACC_GridCellResolutionIndicator);


    Point3f transformDepthToColor(const Point3f& point, const Transformation& transformation) const;
    Point2f projectPointToPixel(const Point3f& point, const IntrinsicData& intrinsic) const;


	std::vector<cv::Point3f> _pointCloud;

	tFloat32 _pitchInDegree;
	tFloat32 _pitchInRad;
	tFloat32 _depthImageScaleFactor;


	/**
	 * synchronisation of OnPinEvent
	 */
	cCriticalSection criticalSection_VideoDataAccess;
	cCriticalSection criticalSection_ActionSubAccess;
	cCriticalSection criticalSection_OperationalStatusAccess;
	cCriticalSection criticalSection_SteeringAngleAccess;
	cCriticalSection criticalSection_TargetSpeedLimitAccess;
	cCriticalSection criticalSection_RunningStateAccess;
	cCriticalSection criticalSection_TransmitFeedback;
	cCriticalSection criticalSection_TransmitSpeed;

		};

#endif // _OBSTACLE_DETECTION_FILTER_HEADER_
#ifndef FTO_OBJECT_SPECIFIER_H
#define FTO_OBJECT_SPECIFIER_H

#define OID_ADTF_OBJECT_SPECIFIER "adtf.fto.object_specifier"

#include "stdafx.h"
#include "ADTF_OpenCV_helper.h"
#include <map>
#include <vector>
#include <deque>
#include <set>

#include "TActionStruct.h"
#include "TFeedbackStruct.h"
#include "TSignalValue.h"

#include "ExtService.h"
#include "ExtIf_types.h"
#include "aadc_classification_structs.h"


class ObjectSpecifier : public adtf::cFilter, public adtf::IKernelThreadFunc
{
    ADTF_FILTER(OID_ADTF_OBJECT_SPECIFIER, "FTO_ObjectSpecifier", adtf::OBJCAT_Tool);

protected:
	// input pins
	cInputPin m_actionInput;
	TActionStruct m_actionPort;
	TActionStruct::ActionSub m_actionStruct;
	bool m_sendClassificationResults;

	cVideoPin m_videoInput;
	tBitmapFormat m_videoInputFormat;
	cv::Mat m_inputFrame;

	cInputPin m_carSpeedInput;
	TSignalValue m_carSpeedInputPort;
	TSignalValue::Data m_carSpeedInputValue;

	cInputPin m_distanceDrivenInput;
	TSignalValue m_distanceDrivenPort;
	TSignalValue::Data m_distanceDrivenValue;

	// output pins
	cOutputPin m_feedbackOutput;
	TFeedbackStruct m_feedback;

	cVideoPin m_videoOutput;
	tBitmapFormat m_videoOutputFormat;
	cv::Mat m_outputFrame;

	cOutputPin m_carSpeedOutput;
	TSignalValue m_carSpeedOutputPort;

	// struct for ROIs
	struct ROI
	{
		// dimensions of the ROI
		int x, y, width, height;

		// probabilites of detectable objects for this ROI;
		// the vector contains the last N probabilities
		std::map<cString, std::deque<double> > probabilities;
		static const int MAX_NUM_PROBABILITIES = 3;

		ROI()
		{
			ROI(0, 0, 100, 100);
		}

		ROI(int px, int py, int pwidth, int pheight)
		{
			this->x = px;
			this->y = py;
			this->width = pwidth;
			this->height = pheight;
		}
	};

	// ROIs; create maps here to be able to find specific entries faster (by name)
	std::map<cString, ROI> m_ROIsNormalDrive;
	std::map<cString, ROI> m_ROIsCrossing;
	std::map<cString, ROI> m_ROIsZebraCrossing;
	std::set<cString> m_ROIsToCareAbout;

	enum DETECTION_MODE
	{
		DETECT_NORMAL_DRIVE,
		DETECT_CROSSING,
		DETECT_ZEBRA_CROSSING,
		DETECT_NOTHING
	};

	DETECTION_MODE m_detectionMode;
	bool m_createDebugImage;

	cv::Scalar m_debugClassificationTextColor;
	double m_debugClassificationTextFontScale;
	int m_debugClassificationTextThickness;
	int m_debugClassificationTextLineHeight;

	// thresholds for object detection
	std::map<cString, double> m_detectionThresholds;
	// save overall distance from car_pose, when an object is detected,
	// to keep track when the car is allowed accelerate again
	double m_detectionStartingPoint;
	// distance the car has to drive to be allowed to accelerate again
	double m_distanceToDrivePastDetection;
	// critical section
	cCriticalSection m_critSectionSpeed;

	// python server communication
	cString m_thriftIP;
	tInt m_thriftPort;

	/*! the thread for the server */
    cKernelThread m_thriftClientThread;
    /*! the external interface thrift client */
    boost::shared_ptr<ext_iface::ExtServiceClient> m_thriftClient;
    /*! critical section for send to thrift */
    cCriticalSection m_critSectionSendToPython;
    /*! buffer for received data from inputPin and send to thrift */
    ext_iface::TDataRaw m_thriftRawMessageBuffer;
    /*! buffer for received data from inputPin and send to thrift */
    ext_iface::TImageParams m_thriftImageParamsBuffer;

private:
	// internal variables
	int m_frameCount;

	const double MAX_SPEED = 0.33;

	static const int CLASSIFICATION_IMG_WIDTH = 100;
	static const int CLASSIFICATION_IMG_HEIGHT = 100;

public:
    ObjectSpecifier(const tChar* __info);
    virtual ~ObjectSpecifier();

protected:
	tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);
	tResult Start(ucom::IException** __exception_ptr);
	tResult Stop(ucom::IException** __exception_ptr);
	tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr);

	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	tResult PropertyChanged(const tChar* pPropertyName);
	tResult ThreadFunc(cKernelThread* pThread, tVoid* pvUserData, tSize szUserData);

private:
	tResult ProcessVideoInput(IMediaSample* pMediaSample);
	tResult ProcessCarSpeed(IMediaSample* pMediaSample);
	tResult ProcessDistanceDriven(IMediaSample* pMediaSample);
	tResult ProcessAction(IMediaSample* pMediaSample);

	tResult ProcessClassificationResults(ext_iface::TDataResultList serverResponse);
	tResult StoreClassificationValue(std::deque<double>& list, double probability);
	tResult ClearClassificationValues(std::map<cString, ROI>& usedMap);
	tResult TransmitClassificationResults();

	bool HasDetectedObject(std::map<cString, ROI>& usedMap, cString objectName);
	bool HasDetectedAdult(std::map<cString, ROI>& usedMap);
	bool HasDetectedChild(std::map<cString, ROI>& usedMap);
	bool HasDetectedCar(std::map<cString, ROI>& usedMap);

	void PaintROIsOnDebugFrame();
	void PaintLabelsOnDebugFrame(ext_iface::TROI roi, std::map<cString, double> labels);

	tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);
	tResult UpdateOutputImageFormat(const cv::Mat& outputImage);
};

#endif // FTO_OBJECT_SPECIFIER_H

/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: kuckal  $  $Date:: 2017-05-22 09:58:28#$ $Rev:: 63664   $
**********************************************************************/

#ifndef _MARKERDETECTFILTER_HEADER
#define _MARKERDETECTFILTER_HEADER
#define OID_ADTF_MARKERDETECTFILTER "adtf.aadc.FTO_markerDetector"

#include "stdafx.h"
#include "aruco_helpers.h"
#include "TActionStruct.h"
#include "TFeedbackStruct.h"
#include "ScmCommunication.h"
#include "TTrafficSign.h"
#include "TRoadSignStructExt.h"



/*! @defgroup MarkerDetector Marker Detector
*  @{
*
* This filter searches for Aruco markers in the given frame on the input video pin. It uses mainly the Aruco lib to find the markers and get the marker Id and their additional parameters and mark them optionally in the video frame. If one or more markers are detected in the frame samples on the pins RoadSign and RoadSign_ext are generated containing the parameters of the sign.
The samples of the pin RoadSign include the marker identifier and the image size of the marker. The samples of the pint RoadSign_ext include the marker identifier, the image size of the marker and the rotation and translation vector. For further description of the samples refer to chapter 3.5.

For the calculation of the rotation and translation vector the intrinsic parameters of the camera has to be known. These parameters can be get with the camera calibration filter and the calibration file generated by that filter loaded with in the  property Calibration File for used Camera.
For the correct assignment of the marker IDs the dictionary file has to be set in the properties as well. The dictionary file includes the assignment of the bit codes of the markers to the predefined ID and has the following content:
*
*  \image html MarkerDetector.PNG "Plugin Marker Detector"
*
* \b Dependencies \n
* This plugin needs the following libraries:
* \li OpenCV  v.3.2.0
*
*
* <b> Filter Properties</b>
* <table>
* <tr><th>Property<th>Description<th>Default
* <tr><td>Debug Output to Console<td>If enabled additional debug information is printed to the console (Warning: decreases performance).<td>tFalse
* <tr><td>Calibration File for used Camera<td>Here you have to set the file with calibration paraemters of the used camera<td>true
* <tr><td>Detector Paramater File<td>Here you have to set the file with the parameters with the detector params<td>
* <tr><td>Size of Markers<td>Size (length of one side) of markers in m"<td>0.117f
* </table>
*
* <b> Output Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_RGB_output<td>Video with detected markers<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* <tr><td>GCL_Markers<td>GCL with detected marker borders<td>MEDIA_TYPE_COMMAND<td>MEDIA_SUBTYPE_COMMAND_GCL
* </table>
* <table>
* <tr><th>Pin<th>Description<th>Media Description
* <tr><td>RoadSign<td>roadsigns<td>tRoadSign
* <tr><td>RoadSign_ext<td>roadsigns with extended information<td>tRoadSignExt
** </table>
*
* <b> Input Pins</b>
* <table>
* <tr><th>Pin<th>Description<th>MajorType<th>SubType
* <tr><td>Video_RGB_input<td>video from camera<td>MEDIA_TYPE_VIDEO<td>MEDIA_SUBTYPE_VIDEO_UNCOMPRESSED
* </table>
*
* <b>Plugin Details</b>
* <table>
* <tr><td>Path<td>src/aadcDemo/marker/AADC_MarkerDetection
* <tr><td>Filename<td>aadc_marker_detector.plb
* <tr><td>Version<td>1.0.1
* </table>
*
*/



/*!
This is the main class of the marker detector filter
*/
class cMarkerDetector : public cAsyncDataTriggeredFilter
{

    /*! declare the filter version */
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_MARKERDETECTFILTER, "FTO_AADC Marker Detector", adtf::OBJCAT_Tool, "FTO_AADC Marker Detection Filter", 1, 0, 1, "");
public:

    /*! default constructor
    * \param __info info argument for filter
    */
    cMarkerDetector(const tChar* __info);

    /*! default destructor */
    virtual ~cMarkerDetector();

    /*! Implements the default cFilter state machine call. It will be
    *    called automatically by changing the filters state and needs
    *    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *    If not using the cException smart pointer, the interface has to
    *    be released by calling Unref().
    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*! This Function will be called by all pins the filter is registered to.
    *    \param [in] pSource Pointer to the sending pin's IPin interface.
    *    \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *    \param [in] nParam1 Optional integer parameter.
    *    \param [in] nParam2 Optional integer parameter.
    *    \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *    \return   Returns a standard result code.
    *    \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *    You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(adtf::IPin* pSource, tInt nEventCode,tInt nParam1,tInt nParam2,adtf::IMediaSample* pMediaSample);

    /*! Implements the default cFilter state machine call. It will be
    *    called automatically by changing the filters state and needs
    *    to be overwritten by the special filter.
    *    Please see  page_filter_life_cycle for further information on when the state of a filter changes.
    *    \param  eStage [in]    The Init function will be called when the filter state changes as follows:\n
    *    \param __exception_ptr  [inout] An Exception pointer where exceptions will be put when failed.
    *    If not using the cException smart pointer, the interface has to
    *    be released by calling Unref().
    *
    *    \return Standard Result Code.
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    
    

protected:
    /*! input Pin for video */
    cVideoPin m_oPinInputVideo;
    /*! output Pin for video */
    cVideoPin m_oPinOutputVideo;
    /*! output Pin for detected Sign as tInt */
    cOutputPin m_oPinRoadSign;
    /*! output Pin for detected Sign as tInt with extended information */
    cOutputPin m_oPinRoadSignExt;
    /*! the gcl output pin */
    cOutputPin m_outputPinGCL;

    /*! */
    cInputPin actionInput;

    cOutputPin tTrafficSignOutput;

    /*! */
    cOutputPin feedbackOutput;

    /*!  */
    UCOM_DECLARE_TIMING_SPOT(m_oProcessStart)
    /*!  */
    UCOM_DECLARE_TIMING_SPOT(m_oProcessEnd)

	UCOM_DECLARE_TIMING_SPOT(m_oPreMarkerDetect)
	UCOM_DECLARE_TIMING_SPOT(m_oPostMarkerDetect)

	UCOM_DECLARE_TIMING_SPOT(m_oPrePoseEstimation)
	UCOM_DECLARE_TIMING_SPOT(m_oPostPoseEstimation)

private:
    /*! bitmapformat of input image */
    tBitmapFormat      m_sInputFormat;

    /*! bitmapformat of output image */
    tBitmapFormat      m_sOutputFormat;

    /*! */
    TActionStruct tActionStruct;

    TActionStruct::ActionSub actionSub;

    /*! */
    TFeedbackStruct tFeedbackStruct;
    TRoadSignStructExt _tRoadSignExtStruct;

    /*! indicates wheter information is printed to the console or not */
    tBool m_bDebugModeEnabled;

    /*! indicates wheter the camara parameters are loaded or not */
    tBool m_bCamaraParamsLoaded;

    /**
     * last detected priority sign
     */
    TFeedbackStruct::Data lastPrioritySign;

    /*! Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSign;
    /*! the id for the i16Identifier of the media description for output pin */
    tBufferID m_szIDRoadSignI16Identifier;
    /*! the id for the f32Imagesize of the media description for output pin */
    tBufferID m_szIDRoadSignF32Imagesize;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsRoadSignSet;

    tBool parking_sign_received; // save parking sign status


	int _frameCount;

	struct Sync {
		/**
		 * Synchronization of OnPinEvent
		 */
		cCriticalSection criticalSection_OnPinEvent;

		ucom::cObjectPtr<IMediaSample> lastActionMediaSample;

		int bufferCount;
		bool skipNextFrame;
		bool log_warning;
		bool log_console;
		const char* name;

		Sync() {
			bufferCount = 0;
			skipNextFrame = false;
			log_warning = false;
			log_console = false;
			name = "RoadSign";
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
				//LOG_WARNING(cString::Format("%s: image queue buffer count < 0", name));
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

    /*! function to process the video data
    * \param pSample the new media sample to be processed
    * \return standard ADTF error code
    */
    tResult ProcessVideo(IMediaSample* pSample);

    /*! function to set the m_sProcessFormat and the  m_sInputFormat variables
    * \param pFormat the new format for the input and input pin
    * \return standard ADTF error code
    */
    tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

    /*! function to set the m_output image format
    * \param pFormat the new format for the output pin
    * \return standard ADTF error code
    */
    tResult UpdateOutputImageFormat(const tBitmapFormat* pFormat);
    
    
    tResult PropertyChanged(const tChar* strName);

    /*! function to transmit a detected road sign
    * \param i16ID ID of the sign
    * \param f32MarkerSize size of the markers sides in meters
    * \param timeOfFrame the timestamp of the frame where the sign was detected
    * \return standard ADTF error code
    */
    tResult sendRoadSignStruct(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame);

    /*! function to transmit a detected road sign with extedend info
    * \param i16ID ID of the sign
    * \param f32MarkerSize size of the markers sides in meters
    * \param timeOfFrame the timestamp of the frame where the sign was detected
    * \param Tvec the translation vector
    * \param Rvec the rotation vector
    * \return standard ADTF error code
    */
    tResult sendRoadSignStructExt(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame, const Vec3d &Tvec, const Vec3d &Rvec);

    /*! function to create a gcl with the marker information
     *\param ids vector with valid ids
     *\param corners corresponding corners of markers
     *\return standard error code
     */
    tResult transmitGCL(const vector< int >& ids,  const vector< vector< Point2f > >& corners);

    //Process Action from RoadSign 2016
    tResult ProcessAction(IMediaSample* mediaSample);

    tResult OnAsyncPinEvent(IPin* source, tInt nEventCode, tInt nParam1, tInt nParam2,
    		IMediaSample* mediaSample);
    tResult sendTrafficSign(const tInt16 &i16ID,
    		const tFloat32 &f32MarkerSize, const tTimeStamp &timeOfFrame,
    		const Vec3d &Tvec, const Vec3d &Rvec);

    /*! the aruco elements: */
    /*! the aruco detector for the markers*/
    Ptr<aruco::DetectorParameters> m_detectorParams;

    /*! size of the markers*/
    tFloat32 m_f32MarkerSize;

    /*! the dictionary for the aruco lib*/
    Ptr<aruco::Dictionary> m_Dictionary;

    /*! camera matrix from calibration file */
    cv::Mat m_Intrinsics;

    /*! distorsion coefficients from calibration file */
    cv::Mat m_Distorsion;
    
    struct filterProperties
    {
        /*! Offset of the ROI in the Stream*/
        int ROIOffsetX;
        /*! Offset of the ROI in the Stream*/
        int ROIOffsetY;
        /*! Width of the ROI*/
        int ROIWidth;
        /*! Height of the ROI*/
        int ROIHeight;

        /*Region of Interest - Rotation*/
        float RvecOffsetZMin;
        float RvecOffsetYMin;
        float RvecOffsetXMin;

        float RvecOffsetZMax;
        float RvecOffsetYMax;
        float RvecOffsetXMax;

        /*Region of Interest - Translation*/
        float TvecOffsetZMin;
        float TvecOffsetYMin;
        float TvecOffsetXMin;

        float TvecOffsetZMax;
        float TvecOffsetYMax;
        float TvecOffsetXMax;

        float MaxDistance;
        int DropFrameCounter;
    }
    /*! the filter properties of this class */
    m_filterProperties;
};

/** @} */ // end of group
#endif //_MARKERDETECTFILTER_HEADER

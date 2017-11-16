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
* $Author:: kuckal  $  $Date:: 2017-05-22 09:58:28#$ $Rev:: 63664    $
**********************************************************************/

#ifndef _MARKERPOS_H_
#define _MERKERPOS_H_

#include "stdafx.h"
#include "TTrafficSignMap.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG static_cast<tFloat32>(180.0/M_PI)
#define DEG2RAD static_cast<tFloat32>(M_PI/180.0)

#define OID_ADTF_FILTER_DEF                "FTO.MarkerPos"
#define ADTF_FILTER_DESC                   "FTO_MarkerPositioning"
#define ADTF_FILTER_VERSION_SUB_NAME       "FTO_MarkerPositioning"
#define ADTF_FILTER_VERSION_ACCEPT_LABEL   "FTO_MarkerPos"
#define ADTF_FILTER_VERSION_STRING         "1.0.0"
#define ADTF_FILTER_VERSION_Major          1
#define ADTF_FILTER_VERSION_Minor          0
#define ADTF_FILTER_VERSION_Build          0
#define ADTF_FILTER_VERSION_LABEL          "Optical Marker Positioning for ADTF from FtO."
#define ADTF_CATEGORY OBJCAT_DataFilter

/*! Storage structure for the road sign data */
typedef struct _roadSign
    {
        /*! road sign */
        tInt16 u16Id;

        /*! location */
        tFloat32 f32X;
        tFloat32 f32Y;

        /*! sign search radius */
        tFloat32 f32Radius;

        /*! direction (heading) of the road sign */
        tFloat32 f32Direction;

        tInt u16Cnt;

        tTimeStamp u32ticks;/*! measurement ticks*/

    } roadSign;

struct IDs {
        tBool set;

        tBufferID i16Identifier;
        tBufferID f32Imagesize;
        tBufferID af32RVec[3];
        tBufferID af32TVec[3];

        IDs() {
            set = tFalse;

            i16Identifier = 0;
            f32Imagesize = 0;
        }
    } ids;

struct Data {
        tInt16 i16Identifier;
        tFloat32 f32Imagesize;
        tFloat32 af32RVec[3];
        tFloat32 af32TVec[3];
    }data;

// struct RS {
//         tInt16 i16_id_map;
//         tFloat32 f32x_value_map;
//         tFloat32 f32y_value_map;
//         tFloat32 f32_angle_map;
//     }rosi;






class cMarkerPos : public adtf::cFilter
{
    /*! This macro does all the plugin setup stuff
	* Warning: This macro opens a "protected" scope see UCOM_IMPLEMENT_OBJECT_INFO(...) in object.h
	*/
	ADTF_FILTER_VERSION(
		OID_ADTF_FILTER_DEF,
		ADTF_FILTER_DESC,
		ADTF_CATEGORY,
		ADTF_FILTER_VERSION_SUB_NAME,
		ADTF_FILTER_VERSION_Major,
		ADTF_FILTER_VERSION_Minor,
		ADTF_FILTER_VERSION_Build,
		ADTF_FILTER_VERSION_LABEL);


    public:
        cMarkerPos(const tChar* __info);
        virtual ~cMarkerPos();

    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
        //tResult PropertyChanged(const char* strProperty);

    private:
        /*! creates all the output pins
        @param __exception the exception pointer
        */
        tResult CreateOutputPins(__exception = NULL);
        tResult CreateInputPins(__exception = NULL);


	/*! Descriptor */
        cObjectPtr<IMediaTypeDescription> m_pDescPosition;

        cObjectPtr<IMediaTypeDescription> m_pMapDescription;


        tResult LoadConfiguration();
        tResult ProcessRoadSignStructExt(IMediaSample* pMediaSampleIn);
        tResult ProcessDistance();
        tResult ProcessPosition(IMediaSample* pMediaSampleIn);
        tResult TransmitRoadSign(const tInt16 &ID, const tFloat32 &f32x_map, const tFloat32 &f32y_map, const tFloat32 &f32_angle);

        tResult TransmitRoadSignStructExt(const tInt16 &i16ID, const tFloat32 &f32MarkerSize, const Vec3d &Tvec, const Vec3d &Rvec);



	//EIN UND AUSGÄNGE
      	cInputPin m_inputRoadSign;
        cOutputPin m_outputRoadSign;
        cOutputPin m_mapOutputPin;
        cInputPin m_InputPosition;

        /*!.whether prints has to made to the console */
        tBool m_bDebugModeEnabled;

        /*! the critical section for the on pin events */
        cCriticalSection m_critSecOnPinEvent;
        cCriticalSection m_oSendPositionCritSection;

        tUInt32 m_ui32ArduinoTimestamp;

	/*! support functions */
        tTimeStamp GetTime();

        /*! storage for the roadsign data */
        vector<roadSign> m_roadSigns;

        FILE*m_log; // debug file

        /*! Descriptor   ROADSIGNEXT */
        cObjectPtr<IMediaTypeDescription> m_pDescriptionRoadSignExt;
        /*! the id for the i16Identifier of the media description for output pin */
        tBufferID m_szIDRoadSignExtI16Identifier;
        /*! the id for the f32Imagesize of the media description for output pin */
        tBufferID m_szIDRoadSignExtF32Imagesize;
        /*! the id for the af32TVec of the media description for output pin */
        tBufferID m_szIDRoadSignExtAf32TVec;
        /*! the id for the af32RVec of the media description for output pin */
        tBufferID m_szIDRoadSignExtAf32RVec;
        /*! indicates if bufferIDs were set */
        tBool m_bIDsRoadSignExtSet;
        tBool m_bIDsRoadSignExtGet;


        /*! currently processed road-sign */
        tInt16 m_i16ID;
        tFloat32 m_f32MarkerSize;
        Mat m_Tvec; /*! translation vector */
        Mat m_Rvec; /*! rotation vector */

        int _frameCount;



        /*! Descriptor */
        /*! the id for the f32x of the media description for output pin */
        tBufferID m_szIDPositionF32X;
        /*! the id for the f32y of the media description for output pin */
        tBufferID m_szIDPositionF32Y;
        /*! the id for the af32radius of the media description for output pin */
        tBufferID m_szIDPositionF32Radius;
        /*! the id for the af32speed of the media description for output pin */
        tBufferID m_szIDPositionF32Speed;
        /*! the id for the af32heading of the media description for output pin */
        tBufferID m_szIDPositionF32Heading;
        /*! indicates if bufferIDs were set */
        tBool m_bIDsPositionSet;

        /*! speed estimate */
        tFloat32 m_f32Speed;

        tFloat32 f32X;
        tFloat32 f32Y;
        tFloat32 f32Radius;
        tFloat32 f32Speed;
        tFloat32 f32Heading;

        tBool getposition;
        tBool getroodsign;

        TTrafficSignMap::Data rosi;
        TTrafficSignMap tTrafficSignMap;

};

#endif // _MARKERPOS_H_

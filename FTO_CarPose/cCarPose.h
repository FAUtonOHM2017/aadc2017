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
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _CAR_POSE_H_
#define _CAR_POSE_H_

#define OID_ADTF_CAR_POSE "adtf.aadc.FTO_CAR_POSE"


#include "TSignalValue.h"
#include "TPoseStruct.h"
#include "TWheelData.h"
#include "TInerMeasureData.h"


class cCarPose : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_CAR_POSE, "FTO_Car_Pose_Filter", adtf::OBJCAT_DataFilter);


private:
    //properties
    tFloat32 f32_PropertyWheelBase, f32_PropertyTrackWidth, f32_PropertyWheelCircumference;
    tFloat32 f32_PropertySteeringCorrection, f32_PropertyOdometerTicks;
    int initCounter;
    tBool resetPose;

protected:


    /*! the input pins */
    cInputPin speed_controller_input;
    TSignalValue speedController;

    cInputPin iner_measure_input;
    TInerMeasureData inerMeasure;

    cInputPin left_wheel_input;
    TWheelData leftWheelInput;

    cInputPin right_wheel_input;
    TWheelData rightWheelInput;

    cInputPin reset_relative_pose;
    TSignalValue resetRelativePose;


    /*! the output pins*/
    cOutputPin car_pose_output;
    TPoseStruct carPoseOut;

    cOutputPin car_speed_output;
    TSignalValue carSpeedOut;

    cOutputPin overall_distance_output;
    TSignalValue overallDistanceOut;

    cOutputPin car_yaw_output;
    TSignalValue carYawOutput;


    /*! The Data structs */
    TSignalValue::Data lastSpeedControllerInput;
    TWheelData::Data lastLeftWheelData, lastRightWheelData;
    TSignalValue::Data overallDistanceRight, overallDistanceLeft;
    tFloat32 *leftTireSpeed, *rightTireSpeed;
    tUInt32 vectorSize, leftTireSpeedCounter, rightTireSpeedCounter, offsetCarOrienationCounter;
    tFloat32 latestSpeed;



    TInerMeasureData::Data lastInerMeasureData;
    TInerMeasureData::Data lastCarOrientation, offsetCarOrientation, offsetCarOrientationTmp;


    TPoseStruct::Data last_carPose;

    tBool firstCallOffsetCarOrientation, initializedYaw, initializedDistance, firstCallWheelLeft, firstCallWheelRight, initializedInerMeasure;
    tBool displayPose;

    cCriticalSection criticalSectionTransmit, criticalSectionPoseAccess, criticalSectionWheelTach, criticalSectionOverallDistance;

public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cCarPose(const tChar* __info);

    /*! default destructor */
    virtual ~cCarPose();

protected:
    /*! Implements the default cFilter state machine call. It will be
    *	    called automatically by changing the filters state and needs
    *	    to be overwritten by the special filter.
    *    Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *    \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *        If not using the cException smart pointer, the interface has to
    *        be released by calling Unref().
    *    \param  [in] eStage The Init function will be called when the filter state changes as follows:\n
    *    \return Standard Result Code.
    */
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    /*!
    *   Implements the default cFilter state machine call. It will be
    *   called automatically by changing the filters state and needs
    *   to be overwritten by the special filter.
    *   Please see page_filter_life_cycle for further information on when the state of a filter changes.
    *
    *   \param [in,out] __exception_ptr   An Exception pointer where exceptions will be put when failed.
    *                                   If not using the cException smart pointer, the interface has to
    *                                   be released by calling Unref().
    *   \param  [in] eStage The Init function will be called when the filter state changes as follows:\n   *
    *   \return Returns a standard result code.
    *
    */
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    /*! This Function will be called by all pins the filter is registered to.
    *   \param [in] pSource Pointer to the sending pin's IPin interface.
    *   \param [in] nEventCode Event code. For allowed values see IPinEventSink::tPinEventCode
    *   \param [in] nParam1 Optional integer parameter.
    *   \param [in] nParam2 Optional integer parameter.
    *   \param [in] pMediaSample Address of an IMediaSample interface pointers.
    *   \return   Returns a standard result code.
    *   \warning This function will not implement a thread-safe synchronization between the calls from different sources.
    *   You need to synchronize this call by your own. Have a look to adtf_util::__synchronized , adtf_util::__synchronized_obj .
    */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    tResult CreateInputPins(__exception = NULL);
    tResult CreateOutputPins(__exception = NULL);
    tResult TransmitPose(tTimeStamp );
    TPoseStruct::Data GetCarPose();
    tResult TransmitSpeed(tTimeStamp);
    tResult ProcessCarSpeedInput(IMediaSample *);
    tResult ProcessRightWheelInput(IMediaSample *);
    tResult ProcessLeftWheelInput(IMediaSample *);
    tResult ProcessInerMeasureInput(IMediaSample *);
    tResult PropertyChanged(const tChar*);
    tResult ProcessResetRelativePose(IMediaSample *);
    tResult TransmitOverallDistance(tTimeStamp);
    tFloat32 DeriveSpeedFromOdometer(TWheelData::Data *, TWheelData::Data *, tFloat32);
    tResult CalculatePitchRollYaw(TInerMeasureData::Data);
};

//*************************************************************************************************
#endif // _CAR_POSE_H_

/*!
*@}
*/

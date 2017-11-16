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
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/
#ifndef _OUTPUT_LIMITER
#define _OUTPUT_LIMITER

#define OID_ADTF_OUTPUT_LIMITER "adtf.aadc.FTO_OUTPUT_LIMITER"


#include "TSignalValue.h"


class cOutputLimiter : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_OUTPUT_LIMITER, "FTO_Output_Limiter", adtf::OBJCAT_DataFilter);


private:
    //properties
    tFloat32 f32_PropertyOdometerTicks;
    int initCounter;
    tBool resetPose;

protected:


    /*! the input pins */
    cInputPin speed_controller_input;
    TSignalValue speedController;

    cInputPin steering_controller_input;
    TSignalValue steeringController;


    /*! the output pins*/

    cOutputPin car_speed_output;
    TSignalValue carSpeedOut;

    /*! The Data structs */
    TSignalValue::Data speedOut;

    tFloat32 limitSpeedControllerOutput;

    cCriticalSection criticalSectionTransmit, criticalSectionPoseAccess, criticalSectionWheelTach, criticalSectionOverallDistance;

public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cOutputLimiter(const tChar* __info);

    /*! default destructor */
    virtual ~cOutputLimiter();

protected:


    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);

    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);


    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    tResult CreateInputPins(__exception = NULL);
    tResult CreateOutputPins(__exception = NULL);


    tResult TransmitSpeed(tTimeStamp);
    tResult ProcessCarSpeedInput(IMediaSample *);

};

//*************************************************************************************************
#endif // _CAR_POSE_H_

/*!
*@}
*/

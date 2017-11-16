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

#include <math.h>
#include "stdafx.h"
#include "cOutputLimiter.h"

//#define DEBUG
/// Create filter shell
ADTF_FILTER_PLUGIN("FTO_Output_Limiter", OID_ADTF_OUTPUT_LIMITER, cOutputLimiter);


cOutputLimiter::cOutputLimiter(const tChar* __info):cFilter(__info)
{
    SetPropertyFloat("Max_Speed_Controller_Out", limitSpeedControllerOutput = 16);
    SetPropertyBool("Max_Speed_Controller_Out" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr("Max_Speed_Controller_Out" NSSUBPROP_DESCRIPTION, "Safe a filter by setting speed controller out in this filter");

}

cOutputLimiter::~cOutputLimiter()
{

}

tResult cOutputLimiter::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        //inputs
        RETURN_IF_FAILED(speedController.StageFirst(__exception_ptr));

        //outputs
        RETURN_IF_FAILED(carSpeedOut.StageFirst(__exception_ptr));

        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));


    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.

        //Inputs
        RETURN_IF_FAILED(speedController.StageGraphReady());

        //Outputs
        RETURN_IF_FAILED(carSpeedOut.StageGraphReady());
    }

    RETURN_NOERROR;
}

tResult cOutputLimiter::CreateOutputPins(__exception){

    RETURN_IF_FAILED(car_speed_output.Create("speed_controller_out", carSpeedOut.GetMediaType(), NULL));
    RETURN_IF_FAILED(RegisterPin(&car_speed_output));

    RETURN_NOERROR;
}

tResult cOutputLimiter::CreateInputPins(__exception){

    RETURN_IF_FAILED(speed_controller_input.Create("speed_controller_in", speedController.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&speed_controller_input));


    RETURN_NOERROR;
}

tResult cOutputLimiter::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {
    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}



tResult cOutputLimiter::ProcessCarSpeedInput(IMediaSample * pMediaSample){

    TSignalValue::Data speed;
    speedController.Read(pMediaSample, &speed);


    speedOut = speed;
    RETURN_NOERROR;
}

tResult cOutputLimiter::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {
            if(pSource == &speed_controller_input){
                ProcessCarSpeedInput(pMediaSample);
                TransmitSpeed(pMediaSample->GetTime());

            }
        }
    }
    RETURN_NOERROR;
}


tResult cOutputLimiter::TransmitSpeed(tTimeStamp inputTime){
    __synchronized_obj(criticalSectionTransmit);

    TSignalValue::Data tmp_speed;
    tmp_speed.f32_value = - speedOut.f32_value;
    tFloat32 maxControllerOut = GetPropertyFloat("Max_Speed_Controller_Out", 12);

    if(tmp_speed.f32_value > maxControllerOut){
        tmp_speed.f32_value = maxControllerOut;
    }else if(tmp_speed.f32_value < -maxControllerOut){
        tmp_speed.f32_value = -maxControllerOut;
    }else if(fabs(tmp_speed.f32_value) < maxControllerOut){
        //do nothging
    }else{
         LOG_WARNING(cString::Format("cOutputLimiterTransmit::unspecified value for speedInput %f", tmp_speed.f32_value));
         tmp_speed.f32_value = 0;
    }
    //tmp_speed.f32_value = tmp_speed.f32_value;



    carSpeedOut.Transmit(&car_speed_output, tmp_speed, inputTime);
    RETURN_NOERROR;
}

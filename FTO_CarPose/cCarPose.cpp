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
#include "cCarPose.h"
#include <math.h>

#define DEBUG
/// Create filter shell
ADTF_FILTER_PLUGIN("FTO_Car_Pose", OID_ADTF_CAR_POSE, cCarPose);


cCarPose::cCarPose(const tChar* __info):cFilter(__info)
{
        initializedYaw = false, initializedDistance = false, firstCallWheelLeft = true, firstCallWheelRight = true;
        firstCallOffsetCarOrientation = true;
        lastSpeedControllerInput.f32_value = 0;
        overallDistanceRight.f32_value = 0, overallDistanceLeft.f32_value = 0;
        vectorSize = 10;
        leftTireSpeed = new tFloat32 [vectorSize];
        rightTireSpeed = new tFloat32 [vectorSize];
        resetPose = false;
        overallDistanceLeft.f32_value = 0;
        overallDistanceRight.f32_value = 0;
        last_carPose.f32_x = 0;
        last_carPose.f32_y = 0;
        last_carPose.f32_yaw = 0;
        offsetCarOrientation.f32roll = 0;
        offsetCarOrientation.f32pitch =0;
        displayPose = false;



        f32_PropertyWheelCircumference = 0.34;
        f32_PropertyWheelBase = 0.36;
        f32_PropertyOdometerTicks = 60; //amount of ticks for one wheel turn
        f32_PropertyTrackWidth = 0.30; //this has to be measured and corrected
//        f32_PropertySteeringCorrection = 0.35; //when steering controller set to 100 steering angle is ~45 deg


//        SetPropertyFloat("Steering_Correction", f32_PropertySteeringCorrection);
//        SetPropertyStr("Steering_Correction" NSSUBPROP_DESCRIPTION, "when steering controller set to 100 steering angle is ~45 deg");
//        SetPropertyBool("Steering_Correction" NSSUBPROP_ISCHANGEABLE, tTrue);


        SetPropertyBool("DisplayPose", displayPose = false);
        SetPropertyStr("DisplayPose" NSSUBPROP_DESCRIPTION, "displays Pose in console");
        SetPropertyBool("DisplayPose" NSSUBPROP_ISCHANGEABLE, tTrue);


        SetPropertyFloat("Wheel_Base", f32_PropertyWheelBase);
        SetPropertyStr("Wheel_Base"NSSUBPROP_DESCRIPTION, "Wheel Base used to calculate Pose");

        SetPropertyFloat("Track_Width", f32_PropertyTrackWidth);
        SetPropertyStr("Track_Width"NSSUBPROP_DESCRIPTION, "Track Width used to calculate Pose");

        SetPropertyFloat("Wheel_Circumference", f32_PropertyWheelCircumference);
        SetPropertyStr("Wheel_Circumference" NSSUBPROP_DESCRIPTION, "Wheel Circumference used to calculate distance and Pose");

        SetPropertyFloat("Odometer_Ticks", f32_PropertyOdometerTicks);
        SetPropertyStr("Odometer_Ticks" NSSUBPROP_DESCRIPTION, "amount of ticks for one wheel turn");

        SetPropertyBool("Reset_Pose", resetPose = false);
        SetPropertyStr("Reset_Pose" NSSUBPROP_DESCRIPTION, "set this to true to reset the pose values");
        SetPropertyBool("Reset_Pose" NSSUBPROP_ISCHANGEABLE, tTrue);

        SetPropertyInt("MedianFilterSize", vectorSize);
        SetPropertyStr("MedianFilterSize" NSSUBPROP_DESCRIPTION, "vector size for median filter. used to smooth out speed output");




}

cCarPose::~cCarPose()
{

}

tResult cCarPose::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        //inputs
        RETURN_IF_FAILED(speedController.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(leftWheelInput.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(rightWheelInput.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(inerMeasure.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(resetRelativePose.StageFirst(__exception_ptr));


        //outputs
        RETURN_IF_FAILED(overallDistanceOut.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(carPoseOut.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(carSpeedOut.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(carYawOutput.StageFirst(__exception_ptr));


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
        RETURN_IF_FAILED(leftWheelInput.StageGraphReady());
        RETURN_IF_FAILED(rightWheelInput.StageGraphReady());
        RETURN_IF_FAILED(speedController.StageGraphReady());
        RETURN_IF_FAILED(inerMeasure.StageGraphReady());
        RETURN_IF_FAILED(resetRelativePose.StageGraphReady());

        //Outputs
        RETURN_IF_FAILED(carSpeedOut.StageGraphReady());
        RETURN_IF_FAILED(carPoseOut.StageGraphReady());
        RETURN_IF_FAILED(overallDistanceOut.StageGraphReady());
        RETURN_IF_FAILED(carYawOutput.StageGraphReady());

    }

    RETURN_NOERROR;
}

tResult cCarPose::CreateOutputPins(__exception){

    RETURN_IF_FAILED(car_pose_output.Create("car_pose", carPoseOut.GetMediaType(), NULL));
    RETURN_IF_FAILED(RegisterPin(&car_pose_output));

    RETURN_IF_FAILED(car_speed_output.Create("car_speed", carSpeedOut.GetMediaType(), NULL));
    RETURN_IF_FAILED(RegisterPin(&car_speed_output));

    RETURN_IF_FAILED(overall_distance_output.Create("overall_distance", overallDistanceOut.GetMediaType(), NULL));
    RETURN_IF_FAILED(RegisterPin(&overall_distance_output));

    RETURN_IF_FAILED(car_yaw_output.Create("car_yaw", carYawOutput.GetMediaType(), NULL));
    RETURN_IF_FAILED(RegisterPin(&car_yaw_output));

    RETURN_NOERROR;
}

tResult cCarPose::CreateInputPins(__exception){

    RETURN_IF_FAILED(iner_measure_input.Create("iner_measure_data", inerMeasure.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&iner_measure_input));

    RETURN_IF_FAILED(speed_controller_input.Create("speed_controller", speedController.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&speed_controller_input));

    RETURN_IF_FAILED(left_wheel_input.Create("left_wheel_data", leftWheelInput.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&left_wheel_input));

    RETURN_IF_FAILED(right_wheel_input.Create("right_wheel_data", rightWheelInput.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&right_wheel_input));

    RETURN_IF_FAILED(reset_relative_pose.Create("reset_relative_pose", resetRelativePose.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&reset_relative_pose));

    RETURN_NOERROR;
}

tResult cCarPose::Shutdown(tInitStage eStage, __exception)
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

tResult cCarPose::PropertyChanged(const tChar* strName){
    if(cString::IsEqual(strName, "Reset_Pose")){
        RETURN_IF_FAILED(resetPose = cFilter::GetPropertyFloat("Reset_Pose", false));
    }else if(cString::IsEqual(strName, "DisplayPose")){
        RETURN_IF_FAILED(displayPose = cFilter::GetPropertyBool("DisplayPose", false));
    }
    RETURN_NOERROR;
}

tResult cCarPose::ProcessCarSpeedInput(IMediaSample * pMediaSample){

    TSignalValue::Data speed;
    speedController.Read(pMediaSample, &speed);

    speed.f32_value = speed.f32_value;
    lastSpeedControllerInput = speed;
    RETURN_NOERROR;
}

tResult cCarPose::ProcessLeftWheelInput(IMediaSample * pMediaSample){
    __synchronized_obj(criticalSectionWheelTach);
    TWheelData::Data tmpLeft;
    leftWheelInput.Read(pMediaSample, &tmpLeft);

    //first call of function, reset of pose, last input was over 0,5s ago
    if(firstCallWheelLeft || resetPose){
        lastLeftWheelData = tmpLeft;
        firstCallWheelLeft = false;
        leftTireSpeedCounter = 0;
        RETURN_NOERROR;
    }

    /*! calculate distance */
    tFloat32 tmp1 = (float)(tmpLeft.ui32_wheelTach - lastLeftWheelData.ui32_wheelTach);
    tmp1 = tmp1 *  (f32_PropertyWheelCircumference/f32_PropertyOdometerTicks) * 0.5;

    if(lastSpeedControllerInput.f32_value < 0){
        tmp1 = -tmp1;
    }

    /*! calculate / update pose*/
    last_carPose.f32_x += tmp1 * cos(lastCarOrientation.f32yaw);
    last_carPose.f32_y += tmp1 * sin(lastCarOrientation.f32yaw);


    /*! calculate speed */

    tFloat tmp2 = DeriveSpeedFromOdometer(&tmpLeft, &lastLeftWheelData, tmp1);
    leftTireSpeed[leftTireSpeedCounter] = tmp2;
    //LOG_WARNING(cString::Format("cCarPose::tmpLeftt_time: %d, lastLeftWheelData: %d, tmp1 %f, tmp2: %f", tmpLeft.ui32_arduinoTimestamp, lastLeftWheelData.ui32_arduinoTimestamp, tmp1, tmp2));

    leftTireSpeedCounter++;
    if(leftTireSpeedCounter >= vectorSize){
       leftTireSpeedCounter = 0;
     }
    overallDistanceLeft.f32_value += tmp1;
    lastLeftWheelData = tmpLeft;

    RETURN_NOERROR;
}



tResult cCarPose::ProcessRightWheelInput(IMediaSample * pMediaSample){

    TWheelData::Data tmpRight;
    rightWheelInput.Read(pMediaSample, &tmpRight);

    if(firstCallWheelRight || resetPose){
        lastRightWheelData = tmpRight;
        overallDistanceRight.f32_value = 0;
        last_carPose.f32_x = 0;
        last_carPose.f32_y = 0;
        last_carPose.f32_yaw = 0;
        firstCallWheelRight = false;
        rightTireSpeedCounter = 0;
        RETURN_NOERROR;
    }

    /*! calculate distance */
    tFloat32 tmp1 = (float)(tmpRight.ui32_wheelTach - lastRightWheelData.ui32_wheelTach);
    tmp1 = tmp1 *  (f32_PropertyWheelCircumference/f32_PropertyOdometerTicks) * 0.5;

    if(lastSpeedControllerInput.f32_value < 0){
        tmp1 = -tmp1;
    }

    /*! calculate / update pose*/
    last_carPose.f32_x += tmp1 * cos(lastCarOrientation.f32yaw);
    last_carPose.f32_y += tmp1 * sin(lastCarOrientation.f32yaw);


    /*! calculate speed */
     tFloat tmp2 = DeriveSpeedFromOdometer(&tmpRight, &lastRightWheelData, tmp1);
    //LOG_WARNING(cString::Format("cCarPose::tmpRight_time: %d, lastRightWheelData: %d, tmp1 %f, tmp2: %f", tmpRight.ui32_arduinoTimestamp, lastRightWheelData.ui32_arduinoTimestamp, tmp1, tmp2));

    rightTireSpeed[rightTireSpeedCounter] = tmp2;
    rightTireSpeedCounter++;
      if(rightTireSpeedCounter >= vectorSize){
      rightTireSpeedCounter = 0;
    }
    overallDistanceRight.f32_value += tmp1;
    lastRightWheelData = tmpRight;

    RETURN_NOERROR;
}


tFloat32 cCarPose::DeriveSpeedFromOdometer(TWheelData::Data * newData, TWheelData::Data * oldData, tFloat32 distanceTraveled){


    tFloat32 speed = 0;
    speed = (tFloat32) (newData->ui32_arduinoTimestamp - oldData->ui32_arduinoTimestamp);
    if(speed == 0){
       LOG_WARNING("cCarPose:: timedifference is 0 -> divison with 0");
       return 0;
    }
    speed = distanceTraveled * 1000000/ speed;

    return speed;
}


tResult cCarPose::ProcessInerMeasureInput(IMediaSample * pMediaSample){
    __synchronized_obj(criticalSectionWheelTach);

    TInerMeasureData::Data tmpData;
    inerMeasure.Read(pMediaSample, &tmpData);

    if(!initializedInerMeasure || resetPose){
        lastCarOrientation.f32yaw = 0;
        lastCarOrientation.f32pitch = 0;
        lastCarOrientation.f32roll = 0;
        lastInerMeasureData = tmpData;
        initializedInerMeasure = true;
    }

    CalculatePitchRollYaw(tmpData);

    lastInerMeasureData = tmpData;

    RETURN_NOERROR;
}

tResult cCarPose::CalculatePitchRollYaw(TInerMeasureData::Data tmpData){
    tFloat32 timeDiff = (tFloat32) (tmpData.ui32_arduinoTimestamp - lastInerMeasureData.ui32_arduinoTimestamp);

    //if car ist not moving dont add gyro values to limit drift
    if(fabs(latestSpeed)  > 0){
       tFloat32 timeDiff = (tFloat32) (tmpData.ui32_arduinoTimestamp - lastInerMeasureData.ui32_arduinoTimestamp);
       lastCarOrientation.f32yaw += ((timeDiff/100000) * (tmpData.f32G_z * 3250/32000))*M_PI/180;
    }
    //Pitch and Roll (sensor data fusion of gyro and acc data)
    //http://www.pieter-jan.com/node/11
    tFloat32 tmp = tmpData.f32A_x / (pow(tmpData.f32A_x,2) + pow(tmpData.f32A_z,2) );
    tFloat32 PitchAcc = atan(tmp) - offsetCarOrientation.f32pitch;
    tmp = tmpData.f32A_y / (pow(tmpData.f32A_y,2) + pow(tmpData.f32A_z,2) );
    tFloat32 RollAcc = atan(tmp) - offsetCarOrientation.f32roll;
    //LOG_WARNING(cString::Format("cCarPose:: RollAcc = %f, PitchAcc = %f", RollAcc* 180/M_PI, PitchAcc* 180/M_PI));
    lastCarOrientation.f32roll = lastCarOrientation.f32roll + ((timeDiff/100000) * (tmpData.f32G_x * 3250/32000))*M_PI/180;
    lastCarOrientation.f32pitch = lastCarOrientation.f32pitch + ((timeDiff/100000) * (tmpData.f32G_y * 3250/32000))*M_PI/180;

    //complementary filter (kalman filter with simple weights)
    lastCarOrientation.f32roll = (0.98 * lastCarOrientation.f32roll + 0.02 * RollAcc);
    lastCarOrientation.f32pitch = (0.90 * lastCarOrientation.f32pitch + 0.10 * PitchAcc);

    if(offsetCarOrienationCounter >= 50 && firstCallOffsetCarOrientation){
        offsetCarOrientation.f32roll  = offsetCarOrientationTmp.f32roll / 50;
        offsetCarOrientation.f32pitch = offsetCarOrientationTmp.f32pitch / 50;
        firstCallOffsetCarOrientation = false;

    }else if(firstCallOffsetCarOrientation){
        offsetCarOrientationTmp.f32roll += RollAcc;
        offsetCarOrientationTmp.f32pitch += PitchAcc;
        offsetCarOrienationCounter++;
    }
    //LOG_WARNING(cString::Format("cCarPose:: offset.f32roll = %f, offset.f32pitch = %f", offsetCarOrientation.f32roll* 180/M_PI, offsetCarOrientation.f32pitch* 180/M_PI));

    //LOG_WARNING(cString::Format("cCarPose:: lastCarOrient.f32roll = %f, lastCarOrient.f32pitch = %f", lastCarOrientation.f32roll* 180/M_PI, lastCarOrientation.f32pitch* 180/M_PI));
    RETURN_NOERROR;
}

tResult cCarPose::ProcessResetRelativePose(IMediaSample * pMediaSample){

    last_carPose.f32_x = 0;
    last_carPose.f32_y = 0;
    last_carPose.f32_yaw = 0;
    last_carPose.f32_car_speed = 0;
    lastCarOrientation.f32yaw = 0;
    lastCarOrientation.f32pitch = 0;
    lastCarOrientation.f32roll = 0;
    last_carPose.f32_roll = 0;
    last_carPose.f32_pitch = 0;
    last_carPose.f32_radius = 0;


    RETURN_NOERROR;

}


tResult cCarPose::OnPinEvent(IPin* pSource,
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
            }else if(pSource == &left_wheel_input){
                ProcessLeftWheelInput(pMediaSample);
            }else if(pSource == &right_wheel_input){
                ProcessRightWheelInput(pMediaSample);
                TransmitOverallDistance(pMediaSample->GetTime());
                TransmitSpeed(pMediaSample->GetTime());
                TransmitPose(pMediaSample->GetTime());
            }else if(pSource == &iner_measure_input){
                ProcessInerMeasureInput(pMediaSample);
            }else if(pSource == &reset_relative_pose){
                ProcessResetRelativePose(pMediaSample);
            }
        }
    }
    RETURN_NOERROR;
}


TPoseStruct::Data cCarPose::GetCarPose() {
        __synchronized_obj(criticalSectionPoseAccess);
        if(firstCallOffsetCarOrientation){
            last_carPose.f32_yaw = 0;
            last_carPose.f32_pitch = 0;
            last_carPose.f32_roll = 0;
        }else{
            last_carPose.f32_yaw = lastCarOrientation.f32yaw;
            last_carPose.f32_pitch = lastCarOrientation.f32pitch;
            last_carPose.f32_roll = lastCarOrientation.f32roll;
        }
        last_carPose.f32_radius = 0;
        return last_carPose;
}

tResult cCarPose::TransmitSpeed(tTimeStamp inputTime){
    __synchronized_obj(criticalSectionTransmit);

    TSignalValue::Data tmp_speed;
    for (tUInt i = 0; i < vectorSize; i++){
        tmp_speed.f32_value += leftTireSpeed[i] + rightTireSpeed[i];
    }
    tmp_speed.f32_value = tmp_speed.f32_value / vectorSize;
    latestSpeed = tmp_speed.f32_value;

    if(fabs(tmp_speed.f32_value) < 10){
        //tmp_speed.f32_value = tmp_speed.f32_value;
        carSpeedOut.Transmit(&car_speed_output, tmp_speed, inputTime);
        last_carPose.f32_car_speed = tmp_speed.f32_value;
    }else{
        LOG_WARNING(cString::Format("cCarPose::Error while computing car_speed!! %f -> was set to 0", tmp_speed.f32_value));
        tmp_speed.f32_value = 0;
        carSpeedOut.Transmit(&car_speed_output, tmp_speed, inputTime);
    }
    RETURN_NOERROR;
}

tResult cCarPose::TransmitOverallDistance(tTimeStamp inputTime){
    TSignalValue::Data tmp_distance;
    tmp_distance.f32_value = overallDistanceRight.f32_value + overallDistanceLeft.f32_value;

    overallDistanceOut.Transmit(&overall_distance_output, tmp_distance, inputTime);
    RETURN_NOERROR;
}

tResult cCarPose::TransmitPose(tTimeStamp inputTime){
   __synchronized_obj(criticalSectionTransmit);

   TPoseStruct::Data tmp_pose;
   tmp_pose = GetCarPose();
   if(displayPose)
       LOG_INFO(cString::Format("cCarPose::X %f, Y: %f, Yaw(in Deg): %f, CarSpeed: %f", tmp_pose.f32_x, tmp_pose.f32_y, tmp_pose.f32_yaw*180/M_PI, tmp_pose.f32_car_speed));

   carPoseOut.Transmit(&car_pose_output, tmp_pose, inputTime);

   TSignalValue::Data tmpYaw;
   tmpYaw.f32_value = tmp_pose.f32_yaw * 180/M_PI;
   tmpYaw.ui32_arduinoTimestamp = tmp_pose.ui32_arduinoTimestamp;
   carYawOutput.Transmit(&car_yaw_output, tmpYaw, inputTime);

   RETURN_NOERROR;
}

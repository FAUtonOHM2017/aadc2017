  /**
 *
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved. Team FAUtonOHM.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ACC Functionality based on Ultrasonic Sensors, car of team FAUtonOHM
**********************************************************************
* $Author:: mahill $   $Date:: 2016-02-15 10:49:07#$ $Rev:: 1.0.1   $
**********************************************************************/

#include "math.h"
#include "UltrasonicACC.h"

#define USACC_CONFIG_DRIVING_FILE "Interpolation::Driving::Configuration File For Driving Mode ACC"
#define USACC_CONFIG_PARKING_FILE "Interpolation::Parking::Configuration File For Parking Mode ACC"
#define USACC_DEBUG_XML_BORDER_WARNING "Debug::XML Border Warnings to Console"
#define USACC_DEBUG_PRINT_INITIAL_TABLE "Debug::Print initial tables to Console"
#define USACC_DEBUG_OUTPUT_TO_CONSOLE "Debug::Debug Output to Console"
#define USACC_EXT_DEBUG_OUTPUT_TO_CONSOLE "Debug::Extended Debug Output to Console"
#define USACC_COUNTER_THRESHOLD_NOMOVE "Movement-related parameters::Counter::no movement threshold"
#define USACC_COUNTER_THRESHOLD_MOVEAGAIN "Movement-related parameters::Counter::moving again threshold"
#define USACC_LOWERBOUND_SPEED "Movement-related parameters::Lower Bound::speed seen as 'no movement'"
#define USACC_SENSOR_SCALING_FRONT "Sensors::Front US scaling factor for sensor-weighting"
#define USACC_SENSOR_SCALING_REAR "Sensors::Rear US scaling factor for sensor-weighting"
#define USACC_SENSOR_FRONT_CHECK_LIMIT "Sensors::Front Sensors Check Limit"
#define USACC_SET_ULRASONIC_VALUES_MAX "Deactivates ultrasonic acc by setting all ultrasonic values to max"
/* sensor angles */
#define USACC_SENSOR_FRONT_LEFT -100
#define USACC_SENSOR_FRONT_CENTERLEFT -50
#define USACC_SENSOR_FRONT_CENTER 0
#define USACC_SENSOR_FRONT_CENTERRIGHT 50
#define USACC_SENSOR_FRONT_RIGHT 100
#define USACC_SENSOR_REAR_LEFT -100
#define USACC_SENSOR_REAR_CENTER 0
#define USACC_SENSOR_REAR_RIGHT +100

// Create filter shell
ADTF_FILTER_PLUGIN("Ultrasonic ACC", OID_ADTF_ULTRASONIC_ACC, UltrasonicACC);

UltrasonicACC::UltrasonicACC(const tChar* __info) : cFilter(__info) {


	/* create the filter properties */
        SetPropertyStr(USACC_CONFIG_DRIVING_FILE, "../../../utilities/UltrasonicACC/UltrasonicACC.xml");
	SetPropertyBool(USACC_CONFIG_DRIVING_FILE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(USACC_CONFIG_DRIVING_FILE NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
	SetPropertyStr(USACC_CONFIG_DRIVING_FILE NSSUBPROP_DESCRIPTION, "The XML defining the behaviour for the DRIVING MODE of the ACC has to be set here");

        SetPropertyStr(USACC_CONFIG_PARKING_FILE, "../../../utilities/UltrasonicACC/UltrasonicACC_Parking.xml");
	SetPropertyBool(USACC_CONFIG_PARKING_FILE NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr(USACC_CONFIG_PARKING_FILE NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
	SetPropertyStr(USACC_CONFIG_PARKING_FILE NSSUBPROP_DESCRIPTION, "The XML defining the behaviour for the PARKING MODE of the ACC has to be set here");

        SetPropertyBool(USACC_DEBUG_XML_BORDER_WARNING, tFalse);
        SetPropertyStr(USACC_DEBUG_XML_BORDER_WARNING NSSUBPROP_DESCRIPTION, "If enabled a warning is printed to console each time the border points of the given xml are reached");


        SetPropertyBool(USACC_SET_ULRASONIC_VALUES_MAX, deactivateACCFilter = tFalse);
        SetPropertyBool(USACC_SET_ULRASONIC_VALUES_MAX NSSUBPROP_ISCHANGEABLE, true);
        SetPropertyStr(USACC_SET_ULRASONIC_VALUES_MAX NSSUBPROP_DESCRIPTION, "Deactivates ultrasonic acc by setting all ultrasonic values to max");

	SetPropertyBool(USACC_DEBUG_PRINT_INITIAL_TABLE, tFalse);
	SetPropertyStr(USACC_DEBUG_PRINT_INITIAL_TABLE NSSUBPROP_DESCRIPTION, "If enabled the loaded points of the interpolation table of the XML are printed to console");

	SetPropertyInt(USACC_COUNTER_THRESHOLD_NOMOVE,3);//noMovementThreshold
	SetPropertyStr(USACC_COUNTER_THRESHOLD_NOMOVE NSSUBPROP_DESCRIPTION, "Defines the approximate time in seconds that should be waited until Feedback 'NoMovement' is sent");

	SetPropertyInt(USACC_COUNTER_THRESHOLD_MOVEAGAIN,1);//MovementAgainThreshold
	SetPropertyStr(USACC_COUNTER_THRESHOLD_MOVEAGAIN NSSUBPROP_DESCRIPTION, "Defines the approximate time in seconds that should be waited until Feedback 'MovingAgain' is sent");

	SetPropertyFloat(USACC_LOWERBOUND_SPEED,0.05); //modifiedSpeedLowerBound
	SetPropertyStr(USACC_LOWERBOUND_SPEED NSSUBPROP_DESCRIPTION,"Defines the remaining value of speed that is seen as 'no movement'.");

        SetPropertyFloat(USACC_SENSOR_SCALING_FRONT,8);
        SetPropertyFloat(USACC_SENSOR_SCALING_FRONT NSSUBPROP_MIN,0);
        SetPropertyFloat(USACC_SENSOR_SCALING_FRONT NSSUBPROP_MAX,100);
        SetPropertyBool(USACC_SENSOR_SCALING_FRONT NSSUBPROP_ISCHANGEABLE, true);
	SetPropertyStr(USACC_SENSOR_SCALING_FRONT NSSUBPROP_DESCRIPTION, "With value 30, the sensor pointing in direction of current steering angle is weighted with '1', while the two sensors next to it are weighted with '0.5'; "
			"DECREASING this value leads to steeper slope of weighting function, that means sensors next to 'main'-sensor are weighted LESS, e.g. with 15: only main sensor is taken into account!");

        SetPropertyFloat(USACC_SENSOR_SCALING_REAR,8);
        SetPropertyFloat(USACC_SENSOR_SCALING_REAR NSSUBPROP_MIN,0);
        SetPropertyFloat(USACC_SENSOR_SCALING_REAR NSSUBPROP_MAX,100);
        SetPropertyBool(USACC_SENSOR_SCALING_REAR NSSUBPROP_ISCHANGEABLE, true);
	SetPropertyStr(USACC_SENSOR_SCALING_REAR NSSUBPROP_DESCRIPTION, "With value 30, the sensor pointing in direction of current steering angle is weighted with '1', while the two sensors next to it are weighted with '0.5'; "
			"DECREASING this value leads to steeper slope of weighting function, that means sensors next to 'main'-sensor are weighted LESS, e.g. with 15: only main sensor is taken into account!");

	SetPropertyFloat(USACC_SENSOR_FRONT_CHECK_LIMIT, 0.4);
	SetPropertyFloat(USACC_SENSOR_FRONT_CHECK_LIMIT NSSUBPROP_MIN,0.0);
	SetPropertyFloat(USACC_SENSOR_FRONT_CHECK_LIMIT NSSUBPROP_MAX,1.5);
        SetPropertyBool(USACC_SENSOR_FRONT_CHECK_LIMIT NSSUBPROP_ISCHANGEABLE, true);
	SetPropertyStr(USACC_SENSOR_FRONT_CHECK_LIMIT NSSUBPROP_DESCRIPTION, "US values greater this property are ignored, area should then be checked with 3D camera (values in meter)");

	SetPropertyBool(USACC_DEBUG_OUTPUT_TO_CONSOLE,tFalse);
        SetPropertyBool(USACC_DEBUG_OUTPUT_TO_CONSOLE NSSUBPROP_ISCHANGEABLE, true);
	SetPropertyStr(USACC_DEBUG_OUTPUT_TO_CONSOLE NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)");

	SetPropertyBool(USACC_EXT_DEBUG_OUTPUT_TO_CONSOLE,tFalse);
	SetPropertyStr(USACC_EXT_DEBUG_OUTPUT_TO_CONSOLE NSSUBPROP_DESCRIPTION, "If enabled all US values are printed to the console (Warning: decreases performance)");


	m_bdebugModeEnabled = tFalse;
	m_bextendedDebugModeEnabled = tFalse;
	/* initialize to neutral mid-setting, corresponds to an angle of zero degree */
	tmp_steeringAngle.f32_value = 90;
}

tResult UltrasonicACC::PropertyChanged(const tChar* strName)
{
          if(cString::IsEqual(strName, USACC_SENSOR_SCALING_FRONT)){
            RETURN_IF_FAILED_AND_LOG_ERROR_STR(scalingFactorUSFront = cFilter::GetPropertyFloat(USACC_SENSOR_SCALING_FRONT, 8), "Error while fetching Scaling front value");
    }else if(cString::IsEqual(strName, USACC_SENSOR_SCALING_REAR)){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(scalingFactorUSRear = cFilter::GetPropertyFloat(USACC_SENSOR_SCALING_REAR, 8), "Error while fetching Scaling front value");
    }else if(cString::IsEqual(strName, USACC_SENSOR_FRONT_CHECK_LIMIT)){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(frontSensorsCheckLimit = cFilter::GetPropertyFloat(USACC_SENSOR_FRONT_CHECK_LIMIT, 0.4), "Error while fetching Scaling front value");
    }else if(cString::IsEqual(strName, USACC_DEBUG_OUTPUT_TO_CONSOLE)){
              RETURN_IF_FAILED_AND_LOG_ERROR_STR(m_bdebugModeEnabled = cFilter::GetPropertyBool(USACC_DEBUG_OUTPUT_TO_CONSOLE, false), "Error while fetching Scaling front value");
    }else if(cString::IsEqual(strName, USACC_SET_ULRASONIC_VALUES_MAX)){
              RETURN_IF_FAILED_AND_LOG_ERROR_STR(deactivateACCFilter = cFilter::GetPropertyBool(USACC_SET_ULRASONIC_VALUES_MAX, false), "Error while fetching Set Ultrasonic Values MAx");
    }
    RETURN_NOERROR;
}


UltrasonicACC::~UltrasonicACC() {

}

tResult UltrasonicACC::CreateInputPins(__exception)
{

	/* Create input pin for targetSpeed */
	RETURN_IF_FAILED(targetSpeed_input.Create("Target_Speed", tSignalValueSpeedInput_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&targetSpeed_input));

	/* Create input pin for original targetSpeed */
	RETURN_IF_FAILED(originalTargetSpeed_input.Create("Orig_Target_Speed", tSignalValueOriginalSpeedInput_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&originalTargetSpeed_input));

	/* Create of the input pin for steeringAngle */
	RETURN_IF_FAILED(steeringAngle_input.Create("Steering_Angle", tSignalValueSteeringInput_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&steeringAngle_input));

	/* Create input pin for ultrasonic sensor */
	RETURN_IF_FAILED(ultrasonicStruct_input.Create("Ultrasonic_Struct", tUltrasonicStructInput_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&ultrasonicStruct_input));

	/* Create action input for SCM communication */
	RETURN_IF_FAILED(actionInput.Create("action", tActionStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionInput));

	RETURN_NOERROR;
}

tResult UltrasonicACC::CreateOutputPins(__exception)
{

	/* create output pin for speed controller */
	RETURN_IF_FAILED(targetSpeed_output.Create("Target_Speed_acc", tSignalValueOutput_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&targetSpeed_output));

	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	RETURN_NOERROR;
}


tResult UltrasonicACC::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		RETURN_IF_FAILED(tUltrasonicStructInput_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueSteeringInput_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueSpeedInput_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueOriginalSpeedInput_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tSignalValueOutput_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tActionStruct_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct_object.StageFirst(__exception_ptr));

		// create and register the input pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		timeoutCounter = 0;

		runningstate = tFalse;
		runningstate_moveagain = tFalse;

		ObstacleNoMovementCounter = 0;
		ObstacleMovementAgainCounter = 0;

		noMovementThreshold = GetPropertyInt(USACC_COUNTER_THRESHOLD_NOMOVE);
		MovementAgainThreshold = GetPropertyInt(USACC_COUNTER_THRESHOLD_MOVEAGAIN);
		modifiedSpeedLowerBound = GetPropertyFloat(USACC_LOWERBOUND_SPEED);

		//Get path of configuration file for "driving-mode-ACC"
		m_fileDrivingModeACC = GetPropertyStr(USACC_CONFIG_DRIVING_FILE);
		//Get path of configuration file for "parking-mode-ACC"
		m_fileParkingModeACC = GetPropertyStr(USACC_CONFIG_PARKING_FILE);

		/* get scaling factor for weighting-function for US-Sensors */
		scalingFactorUSFront = GetPropertyInt(USACC_SENSOR_SCALING_FRONT);
		scalingFactorUSRear = GetPropertyInt(USACC_SENSOR_SCALING_REAR);

		/* get front sensors check limit */
		frontSensorsCheckLimit = GetPropertyFloat(USACC_SENSOR_FRONT_CHECK_LIMIT);

		m_bBorderWarningModeEnabled = GetPropertyBool(USACC_DEBUG_XML_BORDER_WARNING);
		m_bdebugModeEnabled = GetPropertyBool(USACC_DEBUG_OUTPUT_TO_CONSOLE,tFalse);
		m_bextendedDebugModeEnabled = GetPropertyBool(USACC_EXT_DEBUG_OUTPUT_TO_CONSOLE,tFalse);
		m_bPrintInitialtable = GetPropertyBool(USACC_DEBUG_PRINT_INITIAL_TABLE);

		/* load xml files for linear interpolation, vor defined oparational modes */
		THROW_IF_FAILED(LoadConfigurationData(m_fileDrivingModeACC, m_xValues_driving, m_yValues_driving, DRIVING_MODE_ACTIVE));
		THROW_IF_FAILED(LoadConfigurationData(m_fileParkingModeACC, m_xValues_parking, m_yValues_parking, PARKING_MODE_ACTIVE));


		/* static Feedbacks to be sent*/
		feedback_atRest.ui32_filterID = F_ULTRASONIC_ACC;
		feedback_atRest.ui32_status = FB_UA_NO_MOVEMENT;
		feedback_movingAgain.ui32_filterID = F_ULTRASONIC_ACC;
		feedback_movingAgain.ui32_status = FB_UA_MOVING_AGAIN;

		/* ACC is per default in driving mode */
		operationalMode = DRIVING_MODE_ACTIVE;
		if(m_bdebugModeEnabled) LOG_WARNING(cString::Format("UltrasonicACC: Starting in default mode DRIVING_MODE."));

	}
	else if (eStage == StageGraphReady)
	{
		// get size of media samples that has to be assigned later
		RETURN_IF_FAILED(tUltrasonicStructInput_object.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueSpeedInput_object.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueOriginalSpeedInput_object.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueSteeringInput_object.StageGraphReady());
		RETURN_IF_FAILED(tSignalValueOutput_object.StageGraphReady());
		RETURN_IF_FAILED(tActionStruct_object.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct_object.StageGraphReady());

	}

	RETURN_NOERROR;
}

tResult UltrasonicACC::TransmitFeedback(TFeedbackStruct::Data feedback){
	__synchronized_obj(m_oCriticalSectionTransmitFeedback);
	RETURN_IF_FAILED(tFeedbackStruct_object.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime()));

	RETURN_NOERROR;
}

tResult UltrasonicACC::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult UltrasonicACC::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

tResult UltrasonicACC::Shutdown(tInitStage eStage, __exception)
{

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult UltrasonicACC::IncreaseTimeoutCounter(){
	__synchronized_obj(m_oCriticalSectionTimeoutCounterAccess);
	timeoutCounter ++;
	RETURN_NOERROR;
}

tResult UltrasonicACC::ResetTimeoutCounter(){
	__synchronized_obj(m_oCriticalSectionTimeoutCounterAccess);
	timeoutCounter = 0;
	RETURN_NOERROR;
}

tUInt32 UltrasonicACC::GetTimeoutCounter(){
	__synchronized_obj(m_oCriticalSectionTimeoutCounterAccess);
	return timeoutCounter;
}

tResult UltrasonicACC::ProcessUSInput(TUltrasonicStruct::Data tmp_USdata){
	__synchronized_obj(m_oCriticalSectionUSdataAccess);

        /* Convert vom cm to m and discard any values that are smaller than 0.03m*/
        tmp_USdata.FrontLeft.f32_value /= 100;
        if(tmp_USdata.FrontLeft.f32_value < 0.03) tmp_USdata.FrontLeft.f32_value = 4.0;

        tmp_USdata.FrontCenterLeft.f32_value /= 100;
        if(tmp_USdata.FrontCenterLeft.f32_value < 0.03) tmp_USdata.FrontCenterLeft.f32_value = 4.0;

        tmp_USdata.FrontCenter.f32_value /= 100;
        if(tmp_USdata.FrontCenter.f32_value < 0.03) tmp_USdata.FrontCenter.f32_value = 4.0;

        tmp_USdata.FrontCenterRight.f32_value /= 100;
        if(tmp_USdata.FrontCenterRight.f32_value < 0.03) tmp_USdata.FrontCenterRight.f32_value = 4.0;

        tmp_USdata.FrontRight.f32_value /= 100;
        if(tmp_USdata.FrontRight.f32_value < 0.03) tmp_USdata.FrontRight.f32_value = 4.0;

        tmp_USdata.SideLeft.f32_value /= 100;
        if(tmp_USdata.SideLeft.f32_value < 0.03) tmp_USdata.SideLeft.f32_value = 4.0;

        tmp_USdata.SideRight.f32_value /= 100;
        if(tmp_USdata.SideRight.f32_value < 0.03) tmp_USdata.SideRight.f32_value = 4.0;

        tmp_USdata.RearLeft.f32_value /= 100;
        if(tmp_USdata.RearLeft.f32_value < 0.03) tmp_USdata.RearLeft.f32_value = 4.0;

        tmp_USdata.RearCenter.f32_value /= 100;
        if(tmp_USdata.RearCenter.f32_value < 0.03) tmp_USdata.RearCenter.f32_value = 4.0;

        tmp_USdata.RearRight.f32_value /= 100;
        if(tmp_USdata.RearRight.f32_value < 0.03) tmp_USdata.RearRight.f32_value = 4.0;

        //set ultrasonic values to 3m
        if(deactivateACCFilter){
            tmp_USdata.FrontLeft.f32_value = 3.0;
            tmp_USdata.FrontCenterLeft.f32_value = 3.0;
            tmp_USdata.FrontCenter.f32_value = 3.0;
            tmp_USdata.FrontCenterRight.f32_value = 3.0;
            tmp_USdata.FrontRight.f32_value = 3.0;
            tmp_USdata.SideLeft.f32_value = 3.0;
            tmp_USdata.SideRight.f32_value = 3.0;
            tmp_USdata.RearLeft.f32_value = 3.0;
            tmp_USdata.RearCenter.f32_value = 3.0;
            tmp_USdata.RearRight.f32_value = 3.0;

        }

        tmp_US_struct = tmp_USdata;
        RETURN_NOERROR;
}
TUltrasonicStruct::Data UltrasonicACC::GetUSInput(){
	__synchronized_obj(m_oCriticalSectionUSdataAccess);
	return tmp_US_struct;
}

tResult UltrasonicACC::ProcessSteeringAngleInput(TSignalValue::Data tmp_steeringData){
	__synchronized_obj(m_oCriticalSectionStrgAngledataAccess);

	/* Save temporary data to member variable */
	tmp_steeringAngle = tmp_steeringData;
	RETURN_NOERROR;
}

TSignalValue::Data UltrasonicACC::GetSteeringAngleInput(){
	__synchronized_obj(m_oCriticalSectionStrgAngledataAccess);
	return tmp_steeringAngle;
}

/* calculates temporary sensor weight value */
tFloat32 UltrasonicACC::CalcTmpWeight(tInt32 sensorAngleSetting, tFloat32 tmp_steeringAngle, tUInt32 scalingFactor){
	tFloat32 tmp_weight = -1.0f;
        tmp_weight = 1.0f - (fabsf(tmp_steeringAngle - static_cast<tFloat32>(sensorAngleSetting)) * (static_cast<tFloat32>(scalingFactor)) / 1000.0);
	/* if sensor is too far away, tmp_weight will get negative */
	if(tmp_weight < 0.0f){
		tmp_weight = 0.0f;
        }else if(tmp_weight > 1.0f){
            tmp_weight = 1.0;
        }
	return tmp_weight;
}

/* calculated the weights for the front US sensors */
UltrasonicACC::USweightsFront UltrasonicACC::GetSensorWeightsFront(TSignalValue::Data steeringAngleData){
	/* car is driving forwards */
	USweightsFront frontWeightsStruct;
	frontWeightsStruct.usFrontLeft = CalcTmpWeight(USACC_SENSOR_FRONT_LEFT,steeringAngleData.f32_value, scalingFactorUSFront);
	frontWeightsStruct.usFrontCenterLeft = CalcTmpWeight(USACC_SENSOR_FRONT_CENTERLEFT,steeringAngleData.f32_value, scalingFactorUSFront);
	frontWeightsStruct.usFrontCenter = CalcTmpWeight(USACC_SENSOR_FRONT_CENTER,steeringAngleData.f32_value, scalingFactorUSFront);
	frontWeightsStruct.usFrontCenterRight = CalcTmpWeight(USACC_SENSOR_FRONT_CENTERRIGHT,steeringAngleData.f32_value, scalingFactorUSFront);
	frontWeightsStruct.usFrontRight = CalcTmpWeight(USACC_SENSOR_FRONT_RIGHT,steeringAngleData.f32_value, scalingFactorUSFront);

        if(debugLogCounter % 10 == 0 && m_bdebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: usFrontLeft %f, usFrontCenterLeft %f, usFrontCenter %f, usFrontCenterRight %f, usFrontRight %f",
                                   frontWeightsStruct.usFrontLeft, frontWeightsStruct.usFrontCenterLeft, frontWeightsStruct.usFrontCenter, frontWeightsStruct.usFrontCenterRight, frontWeightsStruct.usFrontRight));


	return frontWeightsStruct;
}

/* calculated the weights for the rear US sensors */
UltrasonicACC::USweightsRear UltrasonicACC::GetSensorWeightsRear(TSignalValue::Data steeringAngleData){
	/* car is driving backwards */
	USweightsRear rearWeightsStruct;
        rearWeightsStruct.usRearLeft = CalcTmpWeight((USACC_SENSOR_REAR_LEFT),steeringAngleData.f32_value, scalingFactorUSRear);
	rearWeightsStruct.usRearCenter = CalcTmpWeight(USACC_SENSOR_REAR_CENTER,steeringAngleData.f32_value, scalingFactorUSRear);
        rearWeightsStruct.usRearRight = CalcTmpWeight((USACC_SENSOR_REAR_RIGHT),steeringAngleData.f32_value, scalingFactorUSRear);

        if(debugLogCounter % 10 == 0 && m_bdebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: usRearLeft %f, usRearCenter %f, usRearRight %f",
                                   rearWeightsStruct.usRearLeft, rearWeightsStruct.usRearCenter, rearWeightsStruct.usRearRight));


	return rearWeightsStruct;
}



tResult UltrasonicACC::SetOperationalMode(currentOperationalMode new_mode){
	__synchronized_obj(m_oCriticalSectionOperationalMode);
	operationalMode = new_mode;
	RETURN_NOERROR;
}

UltrasonicACC::currentOperationalMode UltrasonicACC::GetOperationalMode(){
	__synchronized_obj(m_oCriticalSectionOperationalMode);
	return operationalMode;
}

tBool UltrasonicACC::GetRunningState(){
	__synchronized_obj(m_oCriticalSectionRunningstateAccess);
	return runningstate;
}

tResult UltrasonicACC::SetRunningState(tBool new_state){
	__synchronized_obj(m_oCriticalSectionRunningstateAccess);
	runningstate = new_state;
	RETURN_NOERROR;
}

tBool UltrasonicACC::GetRunningStateMoveAgain(){
	__synchronized_obj(m_oCriticalSectionRunningstateAccessMove);
	return runningstate_moveagain;
}

tResult UltrasonicACC::SetRunningStateMoveAgain(tBool new_state){
	__synchronized_obj(m_oCriticalSectionRunningstateAccessMove);
	runningstate_moveagain = new_state;
	RETURN_NOERROR;
}

tResult UltrasonicACC::ProcessAction(TActionStruct::ActionSub actionSub){
/* Filter gets command to start the timer */
	if(actionSub.enabled && actionSub.started) {
            SetOperationalMode(DRIVING_MODE_ACTIVE);
                if(actionSub.command == AC_UA_DRIVING_MODE){
                        SetOperationalMode(DRIVING_MODE_ACTIVE);
                        deactivateACCFilter = false;
                        if(m_bdebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: changed to DRIVING MODE, based on action command: %d",actionSub.command));
                }
                else if(actionSub.command == AC_UA_PARKING_MODE){
                        SetOperationalMode(PARKING_MODE_ACTIVE);
                        deactivateACCFilter = true;
                        if(m_bdebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: changed to PARKING MODE, based on action command: %d",actionSub.command));
                }
                else if(actionSub.command == AC_UA_CHECK_NO_MOVEMENT){
                        if(!GetRunningState()){
                                ResetObstacleNoMovementCounter();
                                SetRunningState(tTrue);
                                /* UA has to track speed and check if targetspeed-input is != zero, but output stays on zero for a longer time! */
                                if(m_bdebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: checking no-movement, based on action command: %d",actionSub.command));
                        }
                        else{
                                if(m_bdebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: command received to check no-movement, but already running in that mode."));
                        }
                }
                else if(actionSub.command == AC_UA_CHECK_MOVING_AGAIN){
                        if(!GetRunningStateMoveAgain()){
                                ResetObstacleMovementAgainCounter();
                                SetRunningStateMoveAgain(tTrue);
                                /* UA has to track speed and check if targetspeed-input is != zero, but output stays on zero for a longer time! */
                                if(m_bdebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: checking moveAgain, based on action command: %d",actionSub.command));
                        }
                        else{
                                if(m_bdebugModeEnabled)LOG_WARNING(cString::Format("UltrasonicACC: command received to check moveAgain, but already running in that mode."));
                        }
                }
                else{
                        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("UltrasonicACC: ActionInput could not be processed, unknown command: %d",actionSub.command));
                }
	}
	/* in case of disable command, but still running no-movement check -> stop,disable */
	if(!actionSub.enabled && GetRunningState()) {
		SetRunningState(tFalse);
	}
	/* in case of disable command, but still running moveAgain check -> stop,disable */
	if(!actionSub.enabled && GetRunningStateMoveAgain()) {
		SetRunningStateMoveAgain(tFalse);
	}
	RETURN_NOERROR;
}

tResult UltrasonicACC::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample) {

	// so we received a media sample, so this pointer better be valid.
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	// first check what kind of event it is (and if output exists?)
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {

		if (pSource == &actionInput) {

			TActionStruct::ActionSub actionSub;
			actionSub = tActionStruct_object.Read_Action(pMediaSample, F_ULTRASONIC_ACC);

			ProcessAction(actionSub);
		}

		/* received an ultrasonicStruct, save sample */
		else if (pSource == &ultrasonicStruct_input) {
			/* safety measure if no speed signal is transmitted to ACC, timeout will occur and target speed will be set to zero*/
			if(GetTimeoutCounter() >= 5){
				TSignalValue::Data zero_TargetSpeed;
				zero_TargetSpeed.f32_value = 0.0f;
				zero_TargetSpeed.ui32_arduinoTimestamp = _clock->GetStreamTime();
				/** transmit target speed with value zero to stop immediately **/
				TransmitTargetSpeed(zero_TargetSpeed);
			}
			else{
				/* increase timeout counter; if everything works as planned, counter will be reset at receiving input-speed */
				IncreaseTimeoutCounter();
			}
                        /* if runningState NoMovement is active, speed should be tracked for possible obstacles and for period of time, provided threshold */
                        if(GetRunningState()){
                            if((GetLastOriginalTargetSpeed() != 0.0f) && (fabsf(GetLastModifiedTargetSpeed()) <= modifiedSpeedLowerBound)){
                                IncreaseObstacleNoMovementCounter();
                            } else {
                                ResetObstacleNoMovementCounter();
                            }
                            if(GetObstacleNoMovementCounter() >= (noMovementThreshold*20)){ 	// times 20 since rate of US is 20Hz
                                                                SetRunningState(tFalse);
                                                                ResetObstacleNoMovementCounter();
                                                                TransmitFeedback(feedback_atRest);
                            }
                        }
                        /* if runningState MoveAgain is active, speed should be tracked for possible moving */
                        if(GetRunningStateMoveAgain()){
                            if((GetLastOriginalTargetSpeed() != 0.0f) && (fabsf(GetLastModifiedTargetSpeed()) >= modifiedSpeedLowerBound)){
                                IncreaseObstacleMovementAgainCounter();
                            } else {
                                ResetObstacleMovementAgainCounter();
                            }
                            if(GetObstacleMovementAgainCounter() >= (MovementAgainThreshold*20)){ // times 20 since rate of US is 20Hz
                                SetRunningStateMoveAgain(tFalse);
                                ResetObstacleMovementAgainCounter();
                                TransmitFeedback(feedback_movingAgain);
                            }
                        }
                        TUltrasonicStruct::Data tmp_USdata;
			RETURN_IF_FAILED(tUltrasonicStructInput_object.Read(pMediaSample,&tmp_USdata));
			RETURN_IF_FAILED(ProcessUSInput(tmp_USdata));

		/* received steering angle input sample */
		} else if (pSource == &steeringAngle_input) {

                        TSignalValue::Data tmp_steeringData;
			RETURN_IF_FAILED(tSignalValueSteeringInput_object.Read(pMediaSample,&tmp_steeringData));
                        ProcessSteeringAngleInput(tmp_steeringData);

		/* received speed controller input sample */
		} else if (pSource == &originalTargetSpeed_input) {

			/* variable to temporarily save current value of original target_speed */
			TSignalValue::Data original_targetSpeed;
			RETURN_IF_FAILED(tSignalValueOriginalSpeedInput_object.Read(pMediaSample,&original_targetSpeed));

			/* save received value to global variable */
			SetLastOriginalTargetSpeed(original_targetSpeed.f32_value);

		/* received speed controller input sample */
		} else if (pSource == &targetSpeed_input) {

			ResetTimeoutCounter(); 
			/* variable to temporarily save current value of target_speed */
			TSignalValue::Data last_rec_targetSpeed;
			RETURN_IF_FAILED(tSignalValueSpeedInput_object.Read(pMediaSample,&last_rec_targetSpeed));

			/* bool value expressing whether car is driving forwards(true) or backwards(backwards) */
			tBool driving_forwards;
			if(last_rec_targetSpeed.f32_value >= 0.0f){ // car is driving in forwards direction
				driving_forwards = tTrue;
			}else{
				driving_forwards = tFalse;	// car is driving in backwards direction
			}

			/* save input value before modification to global variable */
			SetLastInputTargetSpeed(last_rec_targetSpeed.f32_value);

			/* finding the depending value of the percentage in the data of the activated xml-file */
			currentOperationalMode tmp_operationalMode = GetOperationalMode();

                        /* get current steering angle */
			TSignalValue::Data steeringAngleData = GetSteeringAngleInput();

			/* get weighted speed of all active sensors, depending on driving direction  */

			/* variable to save new target speed limit (absolute) based on sensor values and XML */
			TSignalValue::Data mod_targetSpeed;

			if(driving_forwards){ // car moving forwards
				TUltrasonicStruct::Data USStruct = GetUSInput();
				USweightsFront frontWeights = GetSensorWeightsFront(steeringAngleData);
                                USStruct = ComputeWeightedDistanceFront(USStruct, frontWeights);
				RedPercentFront redPercFront = GetRedPerscentFront(USStruct, tmp_operationalMode);
                                mod_targetSpeed = last_rec_targetSpeed;
                                mod_targetSpeed = ChooseOutputSpeedFront(redPercFront, mod_targetSpeed);
                                if(debugLogCounter % 10 == 0 && m_bdebugModeEnabled) LOG_INFO(cString::Format("UltrasonicACC [Ext Forwards]: Left: %f, CenLeft: %f, Cent: %f, CenRight: %f, Right: %f \n", USStruct.FrontLeft.f32_value, USStruct.FrontCenterLeft.f32_value, USStruct.FrontCenter.f32_value, USStruct.FrontCenterRight.f32_value, USStruct.FrontRight.f32_value));
                                if(debugLogCounter % 10 == 0 && m_bdebugModeEnabled) LOG_INFO(cString::Format("UltrasonicACC: receivedTargetSpeed: %f, modifiedSpeed: %f",last_rec_targetSpeed.f32_value, mod_targetSpeed.f32_value));

			}
			else{
				TUltrasonicStruct::Data USStruct = GetUSInput();
				USweightsRear rearWeights = GetSensorWeightsRear(steeringAngleData);
                                USStruct = ComputeWeightedDistanceRear(USStruct, rearWeights);
				RedPercentRear redPercRear= GetRedPerscentRear(USStruct, tmp_operationalMode);
                                mod_targetSpeed = last_rec_targetSpeed;
                                mod_targetSpeed = ChooseOutputSpeedRear(redPercRear, mod_targetSpeed);
                                if(debugLogCounter % 10 == 0 && m_bdebugModeEnabled) LOG_INFO(cString::Format("UltrasonicACC [Ext Backwards]: Left: %f, Center: %f, Right: %f \n", USStruct.RearLeft.f32_value, USStruct.RearCenter.f32_value, USStruct.RearRight.f32_value));
                                if(debugLogCounter % 10 == 0 && m_bdebugModeEnabled) LOG_INFO(cString::Format("UltrasonicACC: receivedTargetSpeed: %f, modifiedSpeed: %f",last_rec_targetSpeed.f32_value, mod_targetSpeed.f32_value));

			}


                        //LOG_INFO(cString::Format("UltrasonicACC: org. speed: %f", last_rec_targetSpeed.f32_value));

			/* limit new output speed to absolute value based on US sensor values and XML */
			if(driving_forwards) {
				if(fabsf(last_rec_targetSpeed.f32_value) > fabsf(mod_targetSpeed.f32_value)) {
					last_rec_targetSpeed.f32_value = mod_targetSpeed.f32_value;
				}
			} else {
				if(fabsf(last_rec_targetSpeed.f32_value) > fabsf(mod_targetSpeed.f32_value)) {
					last_rec_targetSpeed.f32_value = -mod_targetSpeed.f32_value;
				}
			}

			/* save modified value to global variable */
			SetLastModifiedTargetSpeed(last_rec_targetSpeed.f32_value);
			/** transmit target speed **/
			TransmitTargetSpeed(last_rec_targetSpeed);
                        debugLogCounter++;
		}
	}

	RETURN_NOERROR;
}
/**
*@brief weight all the usStruct distances with the weights of how much u trust the value of each sensor if u dont trust it at all set it to 1000
*
*/
TUltrasonicStruct::Data UltrasonicACC::ComputeWeightedDistanceFront(TUltrasonicStruct::Data USStruct, USweightsFront frontWeights){

    if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC::ComputedWeightedDistanceFront before USStuct.FrontLeft %f, USStuct.FrontCenterLeft %f, USStuct.FrontCenter %f, USStuct.FrontCenterRight %f, USStuct.FrontRight %f",
                               USStruct.FrontLeft.f32_value, USStruct.FrontCenterLeft.f32_value, USStruct.FrontCenter.f32_value, USStruct.FrontCenterRight.f32_value, USStruct.FrontRight.f32_value));

    if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: usFrontLeft %f, usFrontCenterLeft %f, usFrontCenter %f, usFrontCenterRight %f, usFrontRight %f",
                               frontWeights.usFrontLeft, frontWeights.usFrontCenterLeft, frontWeights.usFrontCenter, frontWeights.usFrontCenterRight, frontWeights.usFrontRight));


    if(frontWeights.usFrontLeft != 0.0){
        USStruct.FrontLeft.f32_value /=  frontWeights.usFrontLeft;
    }else{
        USStruct.FrontLeft.f32_value = 1000;
    }
    if(frontWeights.usFrontCenterLeft != 0.0){
        USStruct.FrontCenterLeft.f32_value /=  frontWeights.usFrontCenterLeft;
    }else{
        USStruct.FrontCenterLeft.f32_value = 1000;
    }
    if(frontWeights.usFrontCenter != 0.0){
        USStruct.FrontCenter.f32_value /=  frontWeights.usFrontCenter;
    }else{
        USStruct.FrontCenter.f32_value = 1000;
    }
    if(frontWeights.usFrontCenterRight != 0.0){
        USStruct.FrontCenterRight.f32_value /=  frontWeights.usFrontCenterRight;
    }else{
        USStruct.FrontCenterRight.f32_value = 1000;
    }
    if(frontWeights.usFrontRight != 0.0){
        USStruct.FrontRight.f32_value /=  frontWeights.usFrontRight;
    }else{
        USStruct.FrontRight.f32_value = 1000;
    }

    if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC::ComputedWeightedDistanceFront after USStuct.FrontLeft %f, USStuct.FrontCenterLeft %f, USStuct.FrontCenter %f, USStuct.FrontCenterRight %f, USStuct.FrontRight %f",
                               USStruct.FrontLeft.f32_value, USStruct.FrontCenterLeft.f32_value, USStruct.FrontCenter.f32_value, USStruct.FrontCenterRight.f32_value, USStruct.FrontRight.f32_value));
    return USStruct;
}

TUltrasonicStruct::Data UltrasonicACC::ComputeWeightedDistanceRear(TUltrasonicStruct::Data USStruct, USweightsRear rearWeights){

    if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC::ComputedWeightedDistanceFront before USStuct.RearLeft %f, USStuct.RearCenter %f, USStuct.RearRight %f",
                               USStruct.RearLeft.f32_value, USStruct.RearCenter.f32_value, USStruct.RearRight.f32_value));

    if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: usRearLeft %f, usRearCenter %f, usRearRight %f",
                               rearWeights.usRearLeft, rearWeights.usRearCenter, rearWeights.usRearRight));


    if(rearWeights.usRearLeft != 0.0){
        USStruct.RearLeft.f32_value /=  rearWeights.usRearLeft;
    }else{
        USStruct.RearLeft.f32_value = 1000;
    }
    if(rearWeights.usRearCenter != 0.0){
        USStruct.RearCenter.f32_value /=  rearWeights.usRearCenter;
    }else{
        USStruct.RearCenter.f32_value = 1000;
    }
    if(rearWeights.usRearRight != 0.0){
        USStruct.RearRight.f32_value /=  rearWeights.usRearRight;
    }else{
        USStruct.RearRight.f32_value = 1000;
    }
    if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC::ComputedWeightedDistanceFront after USStuct.RearLeft %f, USStuct.RearCenter %f, USStuct.RearRight %f",
                               USStruct.RearLeft.f32_value, USStruct.RearCenter.f32_value, USStruct.RearRight.f32_value));

    return USStruct;
}

TSignalValue::Data UltrasonicACC::ChooseOutputSpeedFront(RedPercentFront redPercent, TSignalValue::Data targetSpeed){

    if(redPercent.RedPercFrontCenter < targetSpeed.f32_value) targetSpeed.f32_value = redPercent.RedPercFrontCenter;
    if(redPercent.RedPercFrontCenterLeft < targetSpeed.f32_value) targetSpeed.f32_value = redPercent.RedPercFrontCenterLeft;
    if(redPercent.RedPercFrontCenterRight < targetSpeed.f32_value) targetSpeed.f32_value = redPercent.RedPercFrontCenterRight;
    if(redPercent.RedPercFrontLeft < targetSpeed.f32_value) targetSpeed.f32_value = redPercent.RedPercFrontLeft;
    if(redPercent.RedPercFrontRight < targetSpeed.f32_value) targetSpeed.f32_value = redPercent.RedPercFrontRight;

    return targetSpeed;

}

TSignalValue::Data UltrasonicACC::ChooseOutputSpeedRear(RedPercentRear redPercent, TSignalValue::Data targetSpeed){

    if(- redPercent.RedPercRearLeft > targetSpeed.f32_value) targetSpeed.f32_value = - redPercent.RedPercRearLeft;
    if(- redPercent.RedPercRearCenter > targetSpeed.f32_value) targetSpeed.f32_value = - redPercent.RedPercRearCenter;
    if(- redPercent.RedPercRearRight > targetSpeed.f32_value) targetSpeed.f32_value = - redPercent.RedPercRearRight;

    return targetSpeed;

}


UltrasonicACC::RedPercentFront UltrasonicACC::GetRedPerscentFront(TUltrasonicStruct::Data USStuct,currentOperationalMode tmp_operationalMode){
	RedPercentFront redPercFront;

         if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC::USStuct.FrontLeft %f, USStuct.FrontCenterLeft %f, USStuct.FrontCenter %f, USStuct.FrontCenterRight %f, USStuct.FrontRight %f",
                                    USStuct.FrontLeft.f32_value, USStuct.FrontCenterLeft.f32_value, USStuct.FrontCenter.f32_value, USStuct.FrontCenterRight.f32_value, USStuct.FrontRight.f32_value));

	// US values greater check limit are ignored (set to max range), area should then be checked with 3D camera
	if(USStuct.FrontLeft.f32_value > frontSensorsCheckLimit) {
		USStuct.FrontLeft.f32_value = 5.0f;
	}
	if(USStuct.FrontCenterLeft.f32_value > frontSensorsCheckLimit) {
		USStuct.FrontCenterLeft.f32_value = 5.0f;
	}
	if(USStuct.FrontCenter.f32_value > frontSensorsCheckLimit) {
		USStuct.FrontCenter.f32_value = 5.0f;
	}
	if(USStuct.FrontCenterRight.f32_value > frontSensorsCheckLimit) {
		USStuct.FrontCenterRight.f32_value = 5.0f;
	}
	if(USStuct.FrontRight.f32_value > frontSensorsCheckLimit) {
		USStuct.FrontRight.f32_value = 5.0f;
	}

	redPercFront.RedPercFrontLeft = GetReductionPercentage(USStuct.FrontLeft.f32_value,tmp_operationalMode);
	redPercFront.RedPercFrontCenterLeft = GetReductionPercentage(USStuct.FrontCenterLeft.f32_value,tmp_operationalMode);
	redPercFront.RedPercFrontCenter = GetReductionPercentage(USStuct.FrontCenter.f32_value,tmp_operationalMode);
	redPercFront.RedPercFrontCenterRight = GetReductionPercentage(USStuct.FrontCenterRight.f32_value,tmp_operationalMode);
	redPercFront.RedPercFrontRight = GetReductionPercentage(USStuct.FrontRight.f32_value,tmp_operationalMode);
        if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: redPerfFrontLeft %f, redPerfFrontCenterLeft %f, redPercFronCenter %f, redPercFrontCenterRight %f, redPercFrontRight %f",
                                   redPercFront.RedPercFrontLeft, redPercFront.RedPercFrontCenterLeft, redPercFront.RedPercFrontCenter, redPercFront.RedPercFrontCenterRight, redPercFront.RedPercFrontRight ));
	return redPercFront;
}

UltrasonicACC::RedPercentRear UltrasonicACC::GetRedPerscentRear(TUltrasonicStruct::Data USStuct, currentOperationalMode tmp_operationalMode){
	RedPercentRear redPercRear;
	redPercRear.RedPercRearLeft = GetReductionPercentage(USStuct.RearLeft.f32_value,tmp_operationalMode);
	redPercRear.RedPercRearCenter= GetReductionPercentage(USStuct.RearCenter.f32_value,tmp_operationalMode);
	redPercRear.RedPercRearRight = GetReductionPercentage(USStuct.RearRight.f32_value,tmp_operationalMode);

        if(debugLogCounter % 10 == 0 && m_bextendedDebugModeEnabled) LOG_WARNING(cString::Format("cUltrasonicACC:: RedPercRearLeft %f,  RedPercRearCenter %f,  RedPercRearRight %f",
                                   redPercRear.RedPercRearLeft, redPercRear.RedPercRearCenter, redPercRear.RedPercRearRight));

	return redPercRear;
}

tFloat32 UltrasonicACC::GetReductionPercentage(tFloat32 distance, currentOperationalMode tmp_operationalMode){
	tFloat32 reduction_percentage = 0.0f;
	switch(tmp_operationalMode){
		case DRIVING_MODE_ACTIVE:
				reduction_percentage = GetLinearInterpolatedValue(distance,m_xValues_driving,m_yValues_driving);
				break;
		case PARKING_MODE_ACTIVE:
				reduction_percentage = GetLinearInterpolatedValue(distance,m_xValues_parking,m_yValues_parking);
				break;
		default:
				RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE,cString::Format("UltrasonicACC: invalid state due to undefined operational mode: %d. Only '1' and '2' are currently defined.",operationalMode));
	}
	return reduction_percentage;
}

tResult UltrasonicACC::IncreaseObstacleMovementAgainCounter(){
	__synchronized_obj(m_oCriticalSectionMovementAgainCounterAccess);
	ObstacleMovementAgainCounter++;
	RETURN_NOERROR;
}
tResult UltrasonicACC::ResetObstacleMovementAgainCounter(){
	__synchronized_obj(m_oCriticalSectionMovementAgainCounterAccess);
	ObstacleMovementAgainCounter = 0;
	RETURN_NOERROR;
}
tUInt32 UltrasonicACC::GetObstacleMovementAgainCounter(){
	__synchronized_obj(m_oCriticalSectionMovementAgainCounterAccess);
	return ObstacleMovementAgainCounter;
}
tResult UltrasonicACC::IncreaseObstacleNoMovementCounter(){
	__synchronized_obj(m_oCriticalSectionNoMovementCounterAccess);
	ObstacleNoMovementCounter++;
	RETURN_NOERROR;
}
tResult UltrasonicACC::ResetObstacleNoMovementCounter(){
	__synchronized_obj(m_oCriticalSectionNoMovementCounterAccess);
	ObstacleNoMovementCounter = 0;
	RETURN_NOERROR;
}
tUInt32 UltrasonicACC::GetObstacleNoMovementCounter(){
	__synchronized_obj(m_oCriticalSectionNoMovementCounterAccess);
	return ObstacleNoMovementCounter;
}

tResult UltrasonicACC::TransmitTargetSpeed(TSignalValue::Data mod_targetSpeed){
	__synchronized_obj(m_oCriticalSectionTransmitTargetSpeed);
	RETURN_IF_FAILED(tSignalValueOutput_object.Transmit(&targetSpeed_output,mod_targetSpeed,_clock->GetStreamTime()));

	RETURN_NOERROR;
}

tResult UltrasonicACC::SetLastInputTargetSpeed(tFloat32 inputTargetSpeed){
	__synchronized_obj(m_oCriticalSectionLastInputSpeedAccess);
	last_input_targetSpeed = inputTargetSpeed;
	RETURN_NOERROR;
}

tFloat32 UltrasonicACC::GetLastInputTargetSpeed(){
	__synchronized_obj(m_oCriticalSectionLastInputSpeedAccess);
	return last_input_targetSpeed;
}

tResult UltrasonicACC::SetLastOriginalTargetSpeed(tFloat32 inputOriginalTargetSpeed){
	__synchronized_obj(m_oCriticalSectionOriginalTargetSpeedAccess);
	last_input_originalTargetSpeed = inputOriginalTargetSpeed;
	RETURN_NOERROR;
}

tFloat32 UltrasonicACC::GetLastOriginalTargetSpeed(){
	__synchronized_obj(m_oCriticalSectionOriginalTargetSpeedAccess);
	return last_input_originalTargetSpeed;
}

tResult UltrasonicACC::SetLastModifiedTargetSpeed(tFloat32 modifiedTargetSpeed){
	__synchronized_obj(m_oCriticalSectionLastModifiedSpeedAccess);
	last_mod_targetSpeed = modifiedTargetSpeed;
	RETURN_NOERROR;
}

tFloat32 UltrasonicACC::GetLastModifiedTargetSpeed(){
	__synchronized_obj(m_oCriticalSectionLastModifiedSpeedAccess);
	return last_mod_targetSpeed;
}

tResult UltrasonicACC::LoadConfigurationData(cFilename& m_fileConfig, vector<tFloat32>& m_xValues, vector<tFloat32>& m_yValues,currentOperationalMode OpMode) {

	// check if file exits
	if (m_fileConfig.IsEmpty()) {
		LOG_ERROR("UltrasonicACC: Configuration file not found");
		RETURN_ERROR(ERR_INVALID_FILE);
	}

	// create absolute path
	ADTF_GET_CONFIG_FILENAME(m_fileConfig);
	m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

	//Load file, parse configuration, print the data

	if (cFileSystem::Exists(m_fileConfig)) {
		cDOM oDOM;
		oDOM.Load(m_fileConfig);
		/* changelog: only linear interpolation is allowed, so cubic interpolation is removed */
		//load supporting points
		cDOMElementRefList oElems;
		if (IS_OK(oDOM.FindNodes("calibration/supportingPoints/point", oElems))) {
			for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem) {

				cDOMElement* pConfigElement;
				if (IS_OK((*itElem)->FindNode("xValue", pConfigElement))) {
					m_xValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
				}
				if (IS_OK((*itElem)->FindNode("yValue", pConfigElement))) {
					m_yValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
				}
			}
		}
		if (oElems.size() > 0) {
			if (m_bPrintInitialtable) {
				if(OpMode==DRIVING_MODE_ACTIVE) LOG_WARNING(cString::Format("UltrasonicACC: printing xml data for driving mode file: "));
				if(OpMode==PARKING_MODE_ACTIVE) LOG_WARNING(cString::Format("UltrasonicACC: printing xml data for parking mode file: "));
				for (tUInt i = 0; i < m_xValues.size(); i++) {
					if (i > m_yValues.size())
						break;
					LOG_WARNING(cString::Format("UltrasonicACC: supportingPoint #%d: (%lf/%lf)", i, m_xValues[i], m_yValues[i]));
				}
			}
		} else {
			RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"UltrasonicACC: no supporting points in given file found!");
		}
		//checks if data are valid
		RETURN_IF_FAILED(CheckConfigurationData(m_fileConfig,m_xValues));

	} else {
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"UltrasonicACC: Configured configuration file not found");
	}

	RETURN_NOERROR;
}

tResult UltrasonicACC::CheckConfigurationData(cFilename m_fileConfig, vector<tFloat32> m_xValues) {
	//checks if the xValues of the calibration table are increasing
	for (vector<tFloat32>::iterator it = m_xValues.begin(); it != m_xValues.end(); it++) {
		vector<tFloat32>::iterator it2 = it;
		it2++;
		if (it2 != m_xValues.end()) {
			// next values is smaller than current value
			if ((tFloat32(*it) > tFloat32(*it2))) {
				RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("UltrasonicACC: The xValues in the file %s are not in increasing order. Please reorder the points!", m_fileConfig.GetPtr()));
			}
		}
	}

	RETURN_NOERROR;
}


tFloat32 UltrasonicACC::GetLinearInterpolatedValue(tFloat32 fl32InputValue, vector<tFloat32> m_xValues, vector<tFloat32> m_yValues) {

	// requested value is smaller than smallest value in table
	if(fl32InputValue < m_xValues.front()) {
		if(m_bBorderWarningModeEnabled){
			LOG_WARNING(cString::Format("UltrasonicACC: requested x-value %f is lower than smallest x-value in calibration table", fl32InputValue));
		}
		return m_yValues.front();
	}
	// requested value is bigger than biggest value in table
	else if(fl32InputValue > m_xValues.back()) {
		if(m_bBorderWarningModeEnabled){
			LOG_WARNING(cString::Format("UltrasonicACC: requested x-value %f is higher than highes x-value in calibration table", fl32InputValue));
		}
		return m_yValues.back();
	}
	// search in vector for corresponding index (smallest neighbor)
	tUInt iIndex;
	if(m_xValues.size() > 2) {
		for(iIndex = 0; iIndex < m_xValues.size(); iIndex++) {
			if(m_xValues[iIndex] >= fl32InputValue){
				break;
			}
		}
		// get smaller neighbor
		if(iIndex != 0){
			iIndex = iIndex - 1;
		}
	}
	else{
		iIndex = 0;
	}
	if((m_xValues[iIndex + 1] - m_xValues[iIndex]) != 0) {
		// doing the linear interpolation
		tFloat32 f32Value = (fl32InputValue - m_xValues[iIndex]) * (m_yValues[iIndex + 1] - m_yValues[iIndex])
				/ (m_xValues[iIndex + 1] - m_xValues[iIndex]) + m_yValues[iIndex];

		//tFloat32 security check to send only minimum or maximum value of table
		if(f32Value > *max_element(m_yValues.begin(), m_yValues.end())){
			return *max_element(m_yValues.begin(), m_yValues.end());
		}
		else if(f32Value < *min_element(m_yValues.begin(), m_yValues.end())){
			return *min_element(m_yValues.begin(), m_yValues.end());
		}
		else{
			return f32Value;
		}
	}
	else{
		LOG_ERROR("UltrasonicACC: invalid table in xml, multiple x_data_points with same value! Please check.");
		return 0;
	}
}


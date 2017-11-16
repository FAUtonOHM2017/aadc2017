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



#ifndef _ULTRASONIC_ACC_H_
#define _ULTRASONIC_ACC_H_

#define OID_ADTF_ULTRASONIC_ACC "adtf.user.ultrasonic_acc"


#include "stdafx.h"


//*************************************************************************************************
class UltrasonicACC: public adtf::cFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ULTRASONIC_ACC, "Ultrasonic ACC", OBJCAT_DataFilter, "Ultrasonic ACC", 1, 0, 0, "FAUtonOHM");

protected:
	cInputPin actionInput;
	cInputPin ultrasonicStruct_input;
	cInputPin targetSpeed_input;
	cInputPin originalTargetSpeed_input;
	cInputPin steeringAngle_input;
	cOutputPin targetSpeed_output;
	cOutputPin feedbackOutput;

	TUltrasonicStruct tUltrasonicStructInput_object;
	TSignalValue tSignalValueSpeedInput_object;
	TSignalValue tSignalValueOriginalSpeedInput_object;
	TSignalValue tSignalValueSteeringInput_object;
	TSignalValue tSignalValueOutput_object;

	TActionStruct	tActionStruct_object;
	TFeedbackStruct	tFeedbackStruct_object;

	TFeedbackStruct::Data feedback_atRest;
	TFeedbackStruct::Data feedback_movingAgain;


public:
	UltrasonicACC(const tChar* __info);
	virtual ~UltrasonicACC();

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
private:


	enum currentOperationalMode{
		DRIVING_MODE_ACTIVE = 1, // default mode
		PARKING_MODE_ACTIVE = 2,
	};

	/* structs to manage weights of ultrasonic sensors */
	struct USweightsFront{
		tFloat32 usFrontLeft;
		tFloat32 usFrontCenterLeft;
		tFloat32 usFrontCenter;
		tFloat32 usFrontCenterRight;
		tFloat32 usFrontRight;

		USweightsFront(){
			usFrontLeft = 0.0f;
			usFrontCenterLeft = 0.0f;
			usFrontCenter = 0.0f;
			usFrontCenterRight = 0.0f;
			usFrontRight = 0.0f;
		}
	};

	struct RedPercentFront{
			tFloat32 RedPercFrontLeft;
			tFloat32 RedPercFrontCenterLeft;
			tFloat32 RedPercFrontCenter;
			tFloat32 RedPercFrontCenterRight;
			tFloat32 RedPercFrontRight;

			RedPercentFront(){
				RedPercFrontLeft = 0.0f;
				RedPercFrontCenterLeft = 0.0f;
				RedPercFrontCenter = 0.0f;
				RedPercFrontCenterRight = 0.0f;
				RedPercFrontRight = 0.0f;
			}
		};

	struct USweightsRear{
		tFloat32 usRearLeft;
		tFloat32 usRearCenter;
		tFloat32 usRearRight;

		USweightsRear(){
			usRearLeft = 0.0f;
			usRearCenter = 0.0f;
			usRearRight = 0.0f;
		}
	};

	struct RedPercentRear{
		tFloat32 RedPercRearLeft;
		tFloat32 RedPercRearCenter;
		tFloat32 RedPercRearRight;

		RedPercentRear(){
			RedPercRearLeft = 0.0f;
			RedPercRearCenter = 0.0f;
			RedPercRearRight = 0.0f;
		}
	};

        tUInt32 debugLogCounter;

	/* struct to save temporary data of type UltrasonicStruct */
	TUltrasonicStruct::Data tmp_US_struct;

	/* tSignalValue to save temporary values of the steering angle */
	TSignalValue::Data tmp_steeringAngle;

	/*! creates all the input Pins */
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins */
	tResult CreateOutputPins(__exception = NULL);

	/* functions for processing and transmitting */
	tResult ProcessUSInput(TUltrasonicStruct::Data tmp_USdata);
	TUltrasonicStruct::Data GetUSInput();
	tResult ProcessSteeringAngleInput(TSignalValue::Data tmp_steeringData);
	TSignalValue::Data GetSteeringAngleInput();
        tFloat32 CalcTmpWeight(tInt32 sensorAngleSetting, tFloat32 tmp_steeringAngle, tUInt32 scalingFactor);
	USweightsFront GetSensorWeightsFront(TSignalValue::Data steeringAngleData);
	USweightsRear GetSensorWeightsRear(TSignalValue::Data steeringAngleData);
	tFloat32 GetReductionPercentage(tFloat32 distance, currentOperationalMode tmp_operationalMode);
	RedPercentFront GetRedPerscentFront(TUltrasonicStruct::Data USStuct,currentOperationalMode tmp_operationalMode);
	RedPercentRear GetRedPerscentRear(TUltrasonicStruct::Data USStuct,currentOperationalMode tmp_operationalMode);
	tResult TransmitTargetSpeed(TSignalValue::Data mod_targetSpeed);
	tResult ProcessAction(TActionStruct::ActionSub actionSub);

	tResult TransmitFeedback(TFeedbackStruct::Data feedback);

	currentOperationalMode GetOperationalMode();
	tResult  SetOperationalMode(currentOperationalMode  new_mode);

	currentOperationalMode operationalMode;

	/** Necessary code for loading an using an xml-data file **/

	/*! reads the xml file which is set in the filter properties */
	tResult LoadConfigurationData(cFilename& m_fileConfig, vector<tFloat32>& m_xValues, vector<tFloat32>& m_yValues,currentOperationalMode OpMode);
	/*! checks the loaded configuration data; checks if the xvalues are in increasing order*/
	tResult CheckConfigurationData(cFilename m_fileConfig, vector<tFloat32> m_xValues);

	/*! doing the linear interpolation
	 @param fl32InputValue the value which should be interpolated
	 */
	tFloat32 GetLinearInterpolatedValue(tFloat32 fl32InputValue, vector<tFloat32> m_xValues, vector<tFloat32> m_yValues);

        tResult PropertyChanged(const tChar* );
        TUltrasonicStruct::Data ComputeWeightedDistanceFront(TUltrasonicStruct::Data , USweightsFront);
        TUltrasonicStruct::Data ComputeWeightedDistanceRear(TUltrasonicStruct::Data , USweightsRear );
        TSignalValue::Data ChooseOutputSpeedFront(RedPercentFront , TSignalValue::Data );
        TSignalValue::Data ChooseOutputSpeedRear(RedPercentRear , TSignalValue::Data );

        tUInt32 timeoutCounter;
	tResult IncreaseTimeoutCounter();
	tResult ResetTimeoutCounter();
	tUInt32 GetTimeoutCounter();

        tBool deactivateACCFilter;

	tFloat32 modifiedSpeedLowerBound;
	tUInt32 ObstacleNoMovementCounter;
	tUInt32 noMovementThreshold;
	tUInt32 ObstacleMovementAgainCounter;
	tUInt32 MovementAgainThreshold;
	tResult IncreaseObstacleNoMovementCounter();
	tResult ResetObstacleNoMovementCounter();
	tUInt32 GetObstacleNoMovementCounter();
	tResult IncreaseObstacleMovementAgainCounter();
	tResult ResetObstacleMovementAgainCounter();
	tUInt32 GetObstacleMovementAgainCounter();

	tFloat32 last_input_targetSpeed;
	tFloat32 last_input_originalTargetSpeed;
	tFloat32 last_mod_targetSpeed;
	tResult SetLastInputTargetSpeed(tFloat32 inputTargetSpeed);
	tFloat32 GetLastInputTargetSpeed();
	tResult SetLastModifiedTargetSpeed(tFloat32 modifiedTargetSpeed);
	tFloat32 GetLastModifiedTargetSpeed();
	tResult SetLastOriginalTargetSpeed(tFloat32);
	tFloat32 GetLastOriginalTargetSpeed();

	tBool GetRunningState();
	tResult SetRunningState(tBool new_state);
	tBool runningstate;
	tBool GetRunningStateMoveAgain();
	tResult SetRunningStateMoveAgain(tBool new_state);
	tBool runningstate_moveagain;

	/*! holds the xml file for the supporting points*/
	/* name of xml-file for driving mode of ACC */
	cFilename m_fileDrivingModeACC;
	/* name of xml-file for parking mode of ACC */
	cFilename m_fileParkingModeACC;

	/*! holds the yValues for the supporting points*/
	vector<tFloat32> m_yValues_driving;
	/*! holds the xValues for the supporting points*/
	vector<tFloat32> m_xValues_driving;

	/*! holds the yValues for the supporting points*/
	vector<tFloat32> m_yValues_parking;
	/*! holds the xValues for the supporting points*/
	vector<tFloat32> m_xValues_parking;


	/*! enable/disable warning at reached borders of the xml-file */
	tBool m_bBorderWarningModeEnabled;
	tBool m_bPrintInitialtable;

	/* scaling factor for adapting the weighting function for the US-sensors depending on steering angle */
	tUInt32 scalingFactorUSFront;
	tUInt32 scalingFactorUSRear;

	/* US values greater this property are ignored, area should then be checked with 3D camera (values in meter)	 */
	tFloat32 frontSensorsCheckLimit;

	/* enable/disable debug messages to console */
	tBool m_bdebugModeEnabled;
	tBool m_bextendedDebugModeEnabled;

	/* changelog-info : using ONLY linear interpolation now, due to license-uncertainties*/

	/** Critical sections, mutex **/

	/*! the critical section of accessing the US data */
	cCriticalSection m_oCriticalSectionUSdataAccess;

	/*! the critical section of accessing the SteeringAngle data */
	cCriticalSection m_oCriticalSectionStrgAngledataAccess;

	/*! the critical section of accessing the US data */
	cCriticalSection m_oCriticalSectionLastInputSpeedAccess;

	/*! the critical section of accessing the SteeringAngle data */
	cCriticalSection m_oCriticalSectionLastModifiedSpeedAccess;

	/*! the critical section of accessing the runningstate data */
	cCriticalSection m_oCriticalSectionRunningstateAccess;

	/*! the critical section of accessing the runningstate data */
	cCriticalSection m_oCriticalSectionRunningstateAccessMove;

	/*! the critical section of accessing the timeoutCounter data */
	cCriticalSection m_oCriticalSectionTimeoutCounterAccess;

	/*! the critical section of accessing the timeoutCounter data */
	cCriticalSection m_oCriticalSectionNoMovementCounterAccess;

	/*! the critical section of accessing the timeoutCounter data */
	cCriticalSection m_oCriticalSectionMovementAgainCounterAccess;

    /*! the critical section of the transmit of speed signal */
    cCriticalSection m_oCriticalSectionTransmitTargetSpeed;

    /*! the critical section of accessing the operationalMode*/
    cCriticalSection m_oCriticalSectionOperationalMode;

    /*! the critical section of accessing the operationalMode*/
    cCriticalSection m_oCriticalSectionTransmitFeedback;

    /*! the critical section of accessing the last received original target speed*/
    cCriticalSection m_oCriticalSectionOriginalTargetSpeedAccess;
};


//*************************************************************************************************
#endif // _ULTRASONIC_ACC_H_

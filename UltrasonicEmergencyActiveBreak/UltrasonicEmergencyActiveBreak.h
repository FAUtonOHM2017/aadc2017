/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for Emergency Stopping Functionality based on Ultrasonic Sensors
**********************************************************************
* $Author::  $ fink, hiller  $Date:: 2016-01-05 00:00:00#$ $Rev:: 1.2 (hiller)   $
**********************************************************************/

#ifndef _ULTRASONIC_EMERGENCY_STOP_ACTIVE_H_
#define _ULTRASONIC_EMERGENCY_STOP_ACTIVE_H_

#define OID_ADTF_EMERGENCY_STOP_ACTIVE "adtf.user.ultrasonic_emergency_stop_active"

//*************************************************************************************************
class UltrasonicEmergencyActiveBreak: public adtf::cFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_EMERGENCY_STOP_ACTIVE, "Ultrasonic Emergency Stop Active", OBJCAT_DataFilter, "Ultrasonic Emergency Stop Active", 1, 0, 0, "FAUtonOHM");

protected:
	cInputPin ultrasonicStruct_input;
	cInputPin speedController_input;
	cInputPin actualSpeed_input;
	cOutputPin speedControllerEmergency_output;
	cOutputPin emergency_output;
	cOutputPin hazardLights;		// tBoolSignalValue; True: On; False: Off

public:
	UltrasonicEmergencyActiveBreak(const tChar* __info);
	virtual ~UltrasonicEmergencyActiveBreak();

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
private:

	tFloat32 m_f32ThresValueFront;
	tFloat32 m_f32ThresValueRear;
	tFloat32 m_f32ThresValueSideLeft;
	tFloat32 m_f32ThresValueSideRight;
	tFloat32 m_f32PercentageActiveBreaking;
	tBool m_boolThresFrontEnable;
	tBool m_boolThresRearEnable;
	tBool m_boolThresSideLeftEnable;
	tBool m_boolThresSideRightEnable;
	tBool m_boolEmergency_output_enable;
	tBool m_boolDoNotRestartAfterObstacle;
	tBool m_boolHazardLightsEnabled;

	// flag is set when obstacle detected
	tBool m_boolObstacleFlag;

	// memory sizes of media samples to create
	tInt nSize_emergency;	// size of JuryEmergencyStop
	tInt nSize_tSignalValue;		// size of tSignalValue
	tInt nSize_tBoolSignalValue; // size of tBoolSignalValue

	typedef struct tmptSignalVal {
		tUInt32 arduino_timestamp;
		tFloat32 data_value;

		tmptSignalVal() :
				arduino_timestamp(0), data_value(0.0) {
		}
	} tmp_tSignalValue;

	typedef struct {
		tmp_tSignalValue FrontLeft;
		tmp_tSignalValue FrontCenterLeft;
		tmp_tSignalValue FrontCenter;
		tmp_tSignalValue FrontCenterRight;
		tmp_tSignalValue FrontRight;
		tmp_tSignalValue SideLeft;
		tmp_tSignalValue SideRight;
		tmp_tSignalValue RearLeft;
		tmp_tSignalValue RearCenter;
		tmp_tSignalValue RearRight;
	} tmp_UltrasonicStruct;

	// struct to save temporary sub-structs of mediatype "UltrasonicStruct"
	tmp_UltrasonicStruct tmp_US_struct;

	// temporary tSignalValue to save current measured speed
	tmp_tSignalValue tmp_actualSpeed;

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);


	// functions for processing and transmitting
	tResult ProcessUSInput(IMediaSample*);
	tResult ProcessActSpeedInput(IMediaSample*);
	tBool CheckUSValuesForStopSignal();
	tFloat32 CalcBreakSpeed();
	tResult TransmitEmergencyOutput(IMediaSample* , tBool );
	tResult TransmitHazardLights(IMediaSample*);
	tResult TransmitSpeedControllerEmergencyOut(IMediaSample*, tmp_tSignalValue);

	/** Necessary for inputs**/

	/*! descriptor for ultrasonic sensor struct data */
	cObjectPtr<IMediaTypeDescription> m_pDescriptionUltrasonicStruct;
	/*! the id for the struct-part 'tFrontLeft' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tFrontLeft;
	/*! the id for the struct-part 'tFrontCenterLeft' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tFrontCenterLeft;
	/*! the id for the struct-part 'tFrontCenter' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tFrontCenter;
	/*! the id for the struct-part 'tFrontCenterRight' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tFrontCenterRight;
	/*! the id for the struct-part 'tFrontRight' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tFrontRight;
	/*! the id for the struct-part 'tSideLeft' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tSideLeft;
	/*! the id for the struct-part 'tSideRight' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tSideRight;
	/*! the id for the struct-part 'tRearLeft' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tRearLeft;
	/*! the id for the struct-part 'tRearCenter' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tRearCenter;
	/*! the id for the struct-part 'tRearRight' of the media description for input pin of ultrasoncic struct data */
	tBufferID tbufID_tRearRight;
	/*! indicates if bufferIDs were set */
	tBool tboolIDs_UltrasonicStructSet;

	/*! descriptor for tSignalValue input data */
	cObjectPtr<IMediaTypeDescription> m_pDescription_tSignalValue_IN;
	/*! the id for the f32value of the media description for input pin of type tSignalValue */
	tBufferID tbufID_tSignalValue_IN_F32Value;
	/*! the id for the arduino time stamp of the media description for input pin of type tSignalValue */
	tBufferID tbufID_tSignalValue_IN_ArduinoTimestamp;
	/*! indicates if bufferIDs were set */
	tBool tboolIDs_tSignalValue_IN_Set;

	/** Necessary for outputs**/

	/** Output tSignalValue **/

	/*! descriptor for tSignalValue output data */
	cObjectPtr<IMediaTypeDescription> m_pDescription_tSignalValue_OUT;
	/*! the id for the f32value of the media description for input pin of the ultrasoncic data */
	tBufferID tbufID_tSignalValue_OUT_F32Value;
	/*! the id for the arduino time stamp of the media description for input pin of the ultrasoncic data */
	tBufferID tbufID_tSignalValue_OUT_ArduinoTimestamp;
	/*! indicates if bufferIDs were set */
	tBool tboolIDs_tSignalValue_OUT_Set;

	/** Output Emergency Stop **/

	/*! descriptor for Emergency output data */
	cObjectPtr<IMediaTypeDescription> m_pDescription_JuryEmergencyStop;
	/*! the id for the bEmergencyStop output */
	tBufferID tbufID_bEmergencyStop;
	/*! indicates if bufferIDs were set */
	tBool tboolIDs_EmergencyOutput_Set;


	/** Output Emergency Stop **/

	/*! descriptor for hazard lights output data */
	cObjectPtr<IMediaTypeDescription> m_pDescription_Bool_Signal_Value;
	/*! the ids for the hazard lights arduino timestamp output */
	tBufferID tbufID_tBoolSignalValue_ArduinoTimestamp;
	/*! the ids for the hazard lights bool value output */
	tBufferID tbufID_tBoolSignalValue_Bool;
	/*! indicates if bufferIDs were set */
	tBool tboolIDs_tBoolSignalValue_Set;

	/** From cSEnsorAnalyzer.h :
	 for functionality: "__synchronized_obj(m_oProcessUsDataCritSection)";
	 critical section for the processing of the ultrasonic data samples because function can be called from different onPinEvents
	 --> In Header definiert : cCriticalSection    m_oProcessUsDataCritSection; **/

	/*! the critical section of accessing the US data */
	cCriticalSection m_oCriticalSectionUSdataAccess;
	/*! the critical section of accessing the actual speed data */
	cCriticalSection m_oCriticalSectionActSpeeddataAccess;

	/*! the critical section of transmitting the emergency stop activation*/
	cCriticalSection m_oCriticalSectionTransmitEmergencyOut;
	/*! the critical section of accessing the actual speed data */
	cCriticalSection m_oCriticalSectionTransmitHazardLights;
	/*! the critical section of accessing the actual speed data */
	cCriticalSection m_oCriticalSectionTransmitSpeedControllerEmergency;

};

//*************************************************************************************************
#endif // _ULTRASONIC_EMERGENCY_STOP_ACTIVE_H_

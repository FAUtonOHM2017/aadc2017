/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for splitting up the defined 'Ultrasonic Struct' into its different components
**********************************************************************
* $Author::  $ hiller  $Date:: 2015-12-13 00:00:00#$ $Rev:: 1.0   $
**********************************************************************/

#ifndef _ULTRASONIC_STRUCT_SPLIT_H_
#define _ULTRASONIC_STRUCT_SPLIT_H_

#define OID_ADTF_US_STRUCT_SPLIT "adtf.user.ultrasonic_struct_split"

//*************************************************************************************************
class UltrasonicStructSplit: public adtf::cFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_US_STRUCT_SPLIT, "Ultrasonic Struct Split", OBJCAT_DataFilter, "Ultrasonic Struct Split", 1, 0, 0, "FAUtonOHM");

protected:
	cInputPin ultrasonicStruct_input;
	cOutputPin US_FrontLeft_out;
	cOutputPin US_FrontCenterLeft_out;
	cOutputPin US_FrontCenter_out;
	cOutputPin US_FrontCenterRight_out;
	cOutputPin US_FrontRight_out;
	cOutputPin US_SideLeft_out;
	cOutputPin US_SideRight_out;
	cOutputPin US_RearLeft_out;
	cOutputPin US_RearCenter_out;
	cOutputPin US_RearRight_out;

public:
	UltrasonicStructSplit(const tChar* __info);
	virtual ~UltrasonicStructSplit();

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
private:

	tInt nSize_tVal;

	typedef struct temptSignalVal {
		tUInt32 arduino_timestep;
		tFloat32 data_value;

		temptSignalVal() :
				arduino_timestep(0), data_value(0.0) {
		}
	} temptSignalValue;

	typedef struct {
		temptSignalValue FrontLeft;
		temptSignalValue FrontCenterLeft;
		temptSignalValue FrontCenter;
		temptSignalValue FrontCenterRight;
		temptSignalValue FrontRight;
		temptSignalValue SideLeft;
		temptSignalValue SideRight;
		temptSignalValue RearLeft;
		temptSignalValue RearCenter;
		temptSignalValue RearRight;
	} tempUltrasonicStruct;

	// struct to save temporary sub-structs of mediatype "UltrasonicStruct"
	tempUltrasonicStruct temp_US_struct;

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);

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

	/** Necessary for outputs**/

	/*! descriptor for tSignalValue output data */
	cObjectPtr<IMediaTypeDescription> m_pDescription_tSignalValue;
	/*! the id for the f32value of the media description for input pin of the ultrasoncic data */
	tBufferID tbufID_tSignalValue_F32Value;
	/*! the id for the arduino time stamp of the media description for input pin of the ultrasoncic data */
	tBufferID tbufID_tSignalValue_ArduinoTimestamp;
	/*! indicates if bufferIDs were set */
	tBool tboolIDs_outputsSet;

	/** From cSEnsorAnalyzer.h :
	 for functionality: "__synchronized_obj(m_oProcessUsDataCritSection)";
	 critical section for the processing of the ultrasonic data samples because function can be called from different onPinEvents
	 --> In Header definiert : cCriticalSection    m_oProcessUsDataCritSection; **/

};

//*************************************************************************************************
#endif // _ULTRASONIC_STRUCT_SPLIT_H_

/**
 * Copyright (c)
 * Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
 * 4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************
 * $Author:: schoen $  #$
 **********************************************************************/

#ifndef T_POSESTRUCT_H_
#define T_POSESTRUCT_H_

#include <adtf_ucom.h>
#include <adtf_plugin_sdk.h>

using namespace adtf;

class TPoseStruct {

	tInt size;

	struct IDs {
		tBool set;

    	tBufferID ui32_arduinoTimestamp;
    	tBufferID f32_x;
    	tBufferID f32_y;
		tBufferID f32_yaw;

		IDs() {
			set = tFalse;

	    	ui32_arduinoTimestamp = 0;
	    	f32_x = 0;
	    	f32_y = 0;
			f32_yaw = 0;
		}
	} ids;

	ucom::cObjectPtr<IMediaTypeDescription> description;
	ucom::cObjectPtr<IMediaType> type;

	tBool stageFirstCalled;

public:

	struct Data {
		tUInt32 ui32_arduinoTimestamp;
		tFloat32 f32_x;
		tFloat32 f32_y;
		tFloat32 f32_yaw;

		Data() {
			ui32_arduinoTimestamp = 0;
			f32_x = 0;
			f32_y = 0;
			f32_yaw = 0;
		}

		cString ToString() {
			return cString::Format("x %f, y %f, yaw %f, a %f", f32_x, f32_y, f32_yaw, ui32_arduinoTimestamp);
		}
	};

	TPoseStruct() {
		size = 0;
		stageFirstCalled = tFalse;
	}

	tResult StageFirst( __exception) {
		ucom::cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		//get description for tSignalValue
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tPoseStruct");
		RETURN_IF_POINTER_NULL(strDescSignalValue);

		type = new cMediaType(0, 0, 0, "tPoseStruct", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_POINTER_NULL(type);

		//get mediatype description for tSignalValue data type
		RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&description));

		stageFirstCalled = tTrue;

		RETURN_NOERROR;
	}

	tResult StageGraphReady() {
		ids.set = tFalse;

		//allocate memory with the size given by the descriptor tPoseStruct
		cObjectPtr<IMediaSerializer> serializer;
		description->GetMediaSampleSerializer(&serializer);
		size = serializer->GetDeserializedSize();

		RETURN_NOERROR;
	}

	ucom::cObjectPtr<IMediaType> GetMediaType() {
		return type;
	}

	tVoid Read(IMediaSample* mediaSample, Data *data) {
		__adtf_sample_read_lock_mediadescription(description, mediaSample, pCoderInput);

		// get the IDs for the items in the media sample
		if (!ids.set) {
			pCoderInput->GetID("ui32ArduinoTimestamp", ids.ui32_arduinoTimestamp);
			pCoderInput->GetID("f32_x", ids.f32_x);
			pCoderInput->GetID("f32_y", ids.f32_y);
			pCoderInput->GetID("f32_yaw", ids.f32_yaw);
			ids.set = tTrue;
		}

		//get values from media sample
		pCoderInput->Get(ids.ui32_arduinoTimestamp, (tVoid*) &(data->ui32_arduinoTimestamp));
		pCoderInput->Get(ids.f32_x,(tVoid*) &(data->f32_x));
		pCoderInput->Get(ids.f32_y, (tVoid*) &(data->f32_y));
		pCoderInput->Get(ids.f32_yaw,(tVoid*) &(data->f32_yaw));
	}

	/**
	 * @param outputPin
	 * @param data
	 * @param outputTime
	 * @return
	 */
	tResult Transmit(cOutputPin *outputPin, Data data, tTimeStamp outputTime) {
		if(size == 0 || !stageFirstCalled) {
			RETURN_AND_LOG_ERROR_STR(ERR_NOT_INITIALISED, cString::Format("TPoseStruct: Transmit failed on pin %s due to: size not set or not initialized", outputPin->GetName()));
		}

		ucom::cObjectPtr<IMediaSample> newMediaSample;
		if (IS_OK(cMediaAllocHelper::AllocMediaSample((tVoid** )&newMediaSample))) {
			newMediaSample->AllocBuffer(size);

			{
				__adtf_sample_write_lock_mediadescription(description, newMediaSample, pCoderOutput);

				// get the IDs for the items in the media sample
				if (!ids.set) {
					pCoderOutput->GetID("ui32ArduinoTimestamp", ids.ui32_arduinoTimestamp);
					pCoderOutput->GetID("f32_x", ids.f32_x);
					pCoderOutput->GetID("f32_y", ids.f32_y);
					pCoderOutput->GetID("f32_yaw", ids.f32_yaw);
					ids.set = tTrue;
				}

				// set values in new media sample
				pCoderOutput->Set(ids.ui32_arduinoTimestamp, (tVoid*) &data.ui32_arduinoTimestamp);
				pCoderOutput->Set(ids.f32_x, (tVoid*) &data.f32_x);
				pCoderOutput->Set(ids.f32_y, (tVoid*) &data.f32_y);
				pCoderOutput->Set(ids.f32_yaw, (tVoid*) &data.f32_yaw);
			}

			//transmit media sample over output pin
			newMediaSample->SetTime(outputTime);
			outputPin->Transmit(newMediaSample);
		}

		RETURN_NOERROR;
	}

	/**
	 * simple helper
	 * @param outputPin
	 * @param ui32_arduinoTimestamp
	 * @param f32_x
	 * @param f32_y
	 * @param f32_yaw
	 * @param outputTime
	 * @return
	 */
	inline tResult Transmit(cOutputPin *outputPin, tUInt32 ui32_arduinoTimestamp, tFloat32 f32_x, tFloat32 f32_y, tFloat32 f32_yaw, tTimeStamp outputTime) {
		Data data;
		data.ui32_arduinoTimestamp = ui32_arduinoTimestamp;
		data.f32_x = f32_x;
		data.f32_y = f32_y;
		data.f32_yaw = f32_yaw;

		return Transmit(outputPin, data, outputTime);
	}
};

#endif // T_POSESTRUCT_H_

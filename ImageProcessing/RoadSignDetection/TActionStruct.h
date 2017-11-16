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
 * $Author:: schoen $   $Date:: 2016-01-21 #$
 **********************************************************************/

#ifndef T_ACTIONSTRUCT_H_
#define T_ACTIONSTRUCT_H_

#include <adtf_ucom.h>
#include <adtf_plugin_sdk.h>

using namespace adtf;

class TActionStruct {

	tInt size;

	struct IDs {
		tBool set;

		tBufferID action_1;
		tBufferID action_2;
		tBufferID action_3;
		tBufferID action_4;
		tBufferID action_5;

		IDs() {
			set = tFalse;

			action_1 = 0;
			action_2 = 0;
			action_3 = 0;
			action_4 = 0;
			action_5 = 0;
		}
	} ids;

	ucom::cObjectPtr<IMediaTypeDescription> description;
	ucom::cObjectPtr<IMediaType> type;

	tBool stageFirstCalled;

public:
	struct ActionSub {
		tBool enabled;
		tBool started;
		tUInt32 command;

		ActionSub() {
			enabled = tFalse;
			started = tFalse;
			command = 0;
		}

		cString ToString() {
			return cString::Format("enabled %d, started %d, command %d,", enabled, started, command);
		}
	};

	struct Action {
		tUInt32 filterID;
		ActionSub subAction;

		Action() {
			filterID = 0;
		}
	};

	struct Data {
		Action action_1;
		Action action_2;
		Action action_3;
		Action action_4;
		Action action_5;
	};

	TActionStruct() {
		size = 0;
		stageFirstCalled = tFalse;
	}

	tResult StageFirst( __exception) {
		ucom::cObjectPtr<IMediaDescriptionManager> pDescManager;
		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

		//get description for tSignalValue
		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tActionStruct");
		RETURN_IF_POINTER_NULL(strDescSignalValue);

		type = new cMediaType(0, 0, 0, "tActionStruct", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
		RETURN_IF_POINTER_NULL(type);

		//get mediatype description for tSignalValue data type
		RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&description));

		stageFirstCalled = tTrue;

		RETURN_NOERROR;
	}

	tResult StageGraphReady() {
		ids.set = tFalse;

		//allocate memory with the size given by the descriptor
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
			pCoderInput->GetID("action_1", ids.action_1);
			pCoderInput->GetID("action_2", ids.action_2);
			pCoderInput->GetID("action_3", ids.action_3);
			pCoderInput->GetID("action_4", ids.action_4);
			pCoderInput->GetID("action_5", ids.action_5);
			ids.set = tTrue;
		}

		//get values from media sample
		pCoderInput->Get(ids.action_1, (tVoid*) &(data->action_1));
		pCoderInput->Get(ids.action_2,(tVoid*) &(data->action_2));
		pCoderInput->Get(ids.action_3, (tVoid*) &(data->action_3));
		pCoderInput->Get(ids.action_4,(tVoid*) &(data->action_4));
		pCoderInput->Get(ids.action_5,(tVoid*) &(data->action_5));
	}

	ActionSub Read_Action(IMediaSample* mediaSample, tUInt32 filterID) {
		Data data;
		Read(mediaSample, &data);

		ActionSub resultActionSub;

		if(data.action_1.filterID == filterID) {
			resultActionSub = data.action_1.subAction;
		} else if(data.action_2.filterID == filterID) {
			resultActionSub = data.action_2.subAction;
		} else if(data.action_3.filterID == filterID) {
			resultActionSub = data.action_3.subAction;
		} else if(data.action_4.filterID == filterID) {
			resultActionSub = data.action_4.subAction;
		} else if(data.action_5.filterID == filterID) {
			resultActionSub = data.action_5.subAction;
		} else {
			// return empty zero result;
			return (ActionSub());
		}

		tUInt32 filterIDFlag = filterID * 1000;
		if(resultActionSub.command < filterIDFlag || resultActionSub.command >= filterIDFlag + 1000) {
			LOG_WARNING(cString::Format ("TActionStruct.h::Read_Action(): ERROR filter ID %d does not match to command %d", filterID, resultActionSub.command));
			// return empty zero result;
			return (ActionSub());
		}

		return resultActionSub;
	}

	tResult Transmit(cOutputPin *outputPin, Data data, tTimeStamp outputTime) {
		if(size == 0 || !stageFirstCalled) {
			RETURN_AND_LOG_ERROR_STR(ERR_NOT_INITIALISED, cString::Format("TActionStruct: Transmit failed on pin %s due to: size not set or not initialized", outputPin->GetName()));
		}

		ucom::cObjectPtr<IMediaSample> newMediaSample;
		if (IS_OK(cMediaAllocHelper::AllocMediaSample((tVoid** )&newMediaSample))) {
			newMediaSample->AllocBuffer(size);

			{
				__adtf_sample_write_lock_mediadescription(description, newMediaSample, pCoderOutput);

				// get the IDs for the items in the media sample
				if (!ids.set) {
					pCoderOutput->GetID("action_1", ids.action_1);
					pCoderOutput->GetID("action_2", ids.action_2);
					pCoderOutput->GetID("action_3", ids.action_3);
					pCoderOutput->GetID("action_4", ids.action_4);
					pCoderOutput->GetID("action_5", ids.action_5);
					ids.set = tTrue;
				}

				// set values in new media sample
				pCoderOutput->Set(ids.action_1, (tVoid*) &(data.action_1));
				pCoderOutput->Set(ids.action_2,(tVoid*) &(data.action_2));
				pCoderOutput->Set(ids.action_3, (tVoid*) &(data.action_3));
				pCoderOutput->Set(ids.action_4,(tVoid*) &(data.action_4));
				pCoderOutput->Set(ids.action_5,(tVoid*) &(data.action_5));
			}

			//transmit media sample over output pin
			newMediaSample->SetTime(outputTime);
			outputPin->Transmit(newMediaSample);
		}

		RETURN_NOERROR;
	}
};

#endif // T_ACTIONSTRUCT_H_

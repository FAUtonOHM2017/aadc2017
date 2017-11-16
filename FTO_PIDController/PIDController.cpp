/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************
* $Author:: fink/hiller $  $Date:: 2016-02-10 08:29:07#$ $Rev:: 0   $
**********************************************************************
Based on PIDController by Audi Autonomous Driving Cup:
**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/

#include "stdafx.h"
#include "PIDController.h"

#define WSC_PROP_PT1_TIMECONSTANT "PT1::TimeConstant"
#define WSC_PROP_PT1_GAIN "PT1::Gain"
#define WSC_PROP_PT1_OUTPUT_FACTOR "PT1::OutputFactor"
#define WSC_PROP_PT1_CORRECTION_FACTOR "PT1::Correction Factor"

#define WSC_PROP_PID_KP "PID::Kp_value"
#define WSC_PROP_PID_KI "PID::Ki_value"
#define WSC_PROP_PID_KD "PID::Kd_value"
#define WSC_PROP_PID_WINDUP "PID::Anti Windup Method"
#define WSC_PROP_PID_SETZERO "PID::Setpoint Zero - Output Zero"
#define WSC_PROP_PID_USEDIRINPUT "PID::Use direction info from set speed"
#define WSC_PROP_PID_SAMPLE_TIME "PID::Sample_Interval_[msec]"

#define WSC_PROP_PID_MAXOUTPUT "PID::Maxiumum output"
#define WSC_PROP_PID_MINOUTPUT "PID::Minimum output"
#define WSC_PROP_DEBUG_MODE "Debug Mode"

ADTF_FILTER_PLUGIN("AADC Wheel Speed Controller", OID_ADTF_PID_CONTROLLER, PIDController)

PIDController::PIDController(const tChar* __info) : cFilter(__info), m_f64LastMeasuredError(0), m_f64SetPoint(0), m_lastSampleTime(0),m_f64LastSpeedValue(0)
{

    SetPropertyFloat(WSC_PROP_PID_KP,16);

    SetPropertyBool(WSC_PROP_PID_KP NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KP NSSUBPROP_DESCRIPTION, "The proportional factor Kp for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_KI,8);
    SetPropertyBool(WSC_PROP_PID_KI NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KI NSSUBPROP_DESCRIPTION, "The integral factor Ki for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_KD, 4);
    SetPropertyBool(WSC_PROP_PID_KD NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_KD NSSUBPROP_DESCRIPTION, "The differential factor Kd for the PID Controller");

    SetPropertyFloat(WSC_PROP_PID_SAMPLE_TIME,0.025);
    SetPropertyBool(WSC_PROP_PID_SAMPLE_TIME NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_SAMPLE_TIME NSSUBPROP_DESCRIPTION, "The sample interval in msec used by the PID controller");


    SetPropertyFloat(WSC_PROP_PID_MAXOUTPUT,16);
    SetPropertyBool(WSC_PROP_PID_MAXOUTPUT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_MAXOUTPUT NSSUBPROP_DESCRIPTION, "The maximum allowed output for the wheel speed controller (speed in m/sec^2)");

    SetPropertyFloat(WSC_PROP_PID_MINOUTPUT,-16);

    SetPropertyBool(WSC_PROP_PID_MINOUTPUT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_MINOUTPUT NSSUBPROP_DESCRIPTION, "The minimum allowed output for the wheel speed controller (speed in m/sec^2)");

    SetPropertyInt(WSC_PROP_PID_WINDUP, 3);
    SetPropertyStr(WSC_PROP_PID_WINDUP NSSUBPROP_DESCRIPTION, "Method for Anti Windup (1: Off, 2: Reset integral term, 3: Stabilize integral term)");
    SetPropertyStr(WSC_PROP_PID_WINDUP NSSUBPROP_VALUELISTNOEDIT, "1@No|2@Reset|3@Stabilize");

    SetPropertyBool(WSC_PROP_PID_SETZERO, tTrue);
    SetPropertyBool(WSC_PROP_PID_SETZERO NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_SETZERO NSSUBPROP_DESCRIPTION, "True: When Setpoint is zero, Output is set to zero (AADC default True)");

    SetPropertyBool(WSC_PROP_PID_USEDIRINPUT, tTrue);
    SetPropertyBool(WSC_PROP_PID_USEDIRINPUT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PID_USEDIRINPUT NSSUBPROP_DESCRIPTION, "True: Use car speed direction info based on set speed input, False: use measured input for direction info");

    SetPropertyBool(WSC_PROP_DEBUG_MODE, tFalse);
    SetPropertyStr(WSC_PROP_DEBUG_MODE NSSUBPROP_DESCRIPTION, "If true debug infos are plotted to registry");

    SetPropertyFloat(WSC_PROP_PT1_OUTPUT_FACTOR, 1);
    SetPropertyBool(WSC_PROP_PT1_OUTPUT_FACTOR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PT1_OUTPUT_FACTOR NSSUBPROP_DESCRIPTION, "The factor to normalize the output value");

    SetPropertyFloat(WSC_PROP_PT1_TIMECONSTANT, 1.5);
    SetPropertyBool(WSC_PROP_PT1_TIMECONSTANT NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PT1_TIMECONSTANT NSSUBPROP_DESCRIPTION, "Time Constant for PT1 Controller");

    SetPropertyFloat(WSC_PROP_PT1_CORRECTION_FACTOR, 1.15);
    SetPropertyBool(WSC_PROP_PT1_CORRECTION_FACTOR NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PT1_CORRECTION_FACTOR NSSUBPROP_DESCRIPTION, "Correction factor for input set point");

    SetPropertyFloat(WSC_PROP_PT1_GAIN, 6);
    SetPropertyBool(WSC_PROP_PT1_GAIN NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(WSC_PROP_PT1_GAIN NSSUBPROP_DESCRIPTION, "Gain for PT1 Controller");

    SetPropertyInt("Controller Typ", 2);
    SetPropertyStr("Controller Typ" NSSUBPROP_VALUELISTNOEDIT, "1@P|2@PI|3@PID|4@PT1");

    //m_pISignalRegistry = NULL;
}

PIDController::~PIDController()
{
}


tResult PIDController::PropertyChanged(const char* strProperty)
{
    ReadProperties(strProperty);

    RETURN_NOERROR;
}

tResult PIDController::ReadProperties(const tChar* strPropertyName)
{

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_KP))
    {
        m_f64PIDKp = GetPropertyFloat(WSC_PROP_PID_KP);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_KD))
    {
        m_f64PIDKd = GetPropertyFloat(WSC_PROP_PID_KD);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_KI))
    {
        m_f64PIDKi = GetPropertyFloat(WSC_PROP_PID_KI);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_SAMPLE_TIME))
    {
        m_f64PIDSampleTime = GetPropertyFloat(WSC_PROP_PID_SAMPLE_TIME);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PT1_OUTPUT_FACTOR))
    {
        m_f64PT1OutputFactor = GetPropertyFloat(WSC_PROP_PT1_OUTPUT_FACTOR);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PT1_GAIN))
    {
        m_f64PT1Gain = GetPropertyFloat(WSC_PROP_PT1_GAIN);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PT1_TIMECONSTANT))
    {
        m_f64PT1TimeConstant = GetPropertyFloat(WSC_PROP_PT1_TIMECONSTANT);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, "Controller Typ"))
    {
        m_i32ControllerMode = GetPropertyInt("Controller Typ");
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_DEBUG_MODE))
    {
        m_bShowDebug = static_cast<tBool>(GetPropertyBool(WSC_PROP_DEBUG_MODE));
        if(m_bShowDebug) {
                //////////LOG_INFO("PIDController: Debug mode is enabled");
        }
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PT1_CORRECTION_FACTOR))
    {
        m_f64PT1CorrectionFactor = GetPropertyFloat(WSC_PROP_PT1_CORRECTION_FACTOR);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_MINOUTPUT))
    {
        m_f64PIDMinimumOutput = GetPropertyFloat(WSC_PROP_PID_MINOUTPUT);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_MAXOUTPUT))
    {
        m_f64PIDMaximumOutput = GetPropertyFloat(WSC_PROP_PID_MAXOUTPUT);
    }

    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_WINDUP))
    {
        m_i32AntiWindupMethod = GetPropertyInt(WSC_PROP_PID_WINDUP);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_SETZERO))
    {
    	m_bPID_Zero_Beh = GetPropertyBool(WSC_PROP_PID_SETZERO);
    }
    if (NULL == strPropertyName || cString::IsEqual(strPropertyName, WSC_PROP_PID_USEDIRINPUT))
    {
    	m_bUseSetSpeedDirInfo = GetPropertyBool(WSC_PROP_PID_USEDIRINPUT);
    }

    RETURN_NOERROR;
}


tResult PIDController::CreateInputPins(__exception)
{

	RETURN_IF_FAILED(m_measured_speed_input.Create("meas_speed", measuredSpeedIn.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_measured_speed_input));

	RETURN_IF_FAILED(m_set_point_input.Create("set_speed", setPointIn.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_set_point_input));

    RETURN_NOERROR;
}

tResult PIDController::CreateOutputPins(__exception)
{
	RETURN_IF_FAILED(m_manipulated_speed_output.Create("actuator_output", manipulatedSpeedOut.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&m_manipulated_speed_output));

    RETURN_NOERROR;
}

tResult PIDController::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

        if (eStage == StageFirst)
        {
        	RETURN_IF_FAILED(measuredSpeedIn.StageFirst(__exception_ptr));
        	RETURN_IF_FAILED(setPointIn.StageFirst(__exception_ptr));
        	RETURN_IF_FAILED(manipulatedSpeedOut.StageFirst(__exception_ptr));

            RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
            RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
        }
        else if (eStage == StageNormal)
        {
            ReadProperties(NULL);

        }
        else if(eStage == StageGraphReady)
        {    

        	RETURN_IF_FAILED(measuredSpeedIn.StageGraphReady());
        	RETURN_IF_FAILED(setPointIn.StageGraphReady());
        	RETURN_IF_FAILED(manipulatedSpeedOut.StageGraphReady());

            m_f64LastOutput = 0.0f;
            direction_forwards = tTrue;

			if (m_bShowDebug) {
				LOG_WARNING(cString::Format("PIDController: Initialized in windup mode: %d, controller mode: %d", m_i32AntiWindupMethod, m_i32ControllerMode));
			}
        }

        RETURN_NOERROR;
}

tResult PIDController::Start(__exception)
{
    m_f64LastOutput = 0;
    m_f64LastMeasuredError = 0;
    m_f64SetPoint = 0;
    m_lastSampleTime = 0;
    m_f64LastSpeedValue = 0;
    m_f64accumulatedVariable = 0;
    m_f64MeasuredVariable = 0;
    direction_forwards = tTrue;

    return cFilter::Start(__exception_ptr);
}

tResult PIDController::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult PIDController::Shutdown(tInitStage eStage, __exception)
{
    if (eStage == StageNormal)
    {

    }
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tFloat32 PIDController::ProcessWheelSpeedInput(IMediaSample* pMediaSample){

	// save last car speed
	TSignalValue::Data wheel_speed;
	measuredSpeedIn.Read(pMediaSample, &wheel_speed);

	return wheel_speed.f32_value;
        //////////LOG_INFO(cString::Format("WheelSpeed: %f", wheel_speed.f32_value));
}

tFloat32 PIDController::ProcessSetPointInput(IMediaSample* pMediaSample){

	// save last car speed
	TSignalValue::Data set_point;
	setPointIn.Read(pMediaSample, &set_point);

	// set car direction based on set speed input
	if(set_point.f32_value > 0) {
		direction_forwards = tTrue;
	} else if(set_point.f32_value < 0) {
		direction_forwards = tFalse;
	}

	return set_point.f32_value;
}

tResult PIDController::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    __synchronized_obj(m_critSecOnPinEvent);
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL)
    {
        RETURN_IF_POINTER_NULL( pMediaSample);
        if (pSource == &m_measured_speed_input)
        {
                m_f64MeasuredVariable = static_cast<tFloat64>(ProcessWheelSpeedInput(pMediaSample));
                ////////LOG_INFO(cString::Format("PIDController: org. speed: %lf", m_f64MeasuredVariable));

        	// if property True: set car direction based on set speed input
        	if(m_bUseSetSpeedDirInfo) {
        		if(direction_forwards) {
        			m_f64MeasuredVariable = fabsf(m_f64MeasuredVariable);
        		} else {
        			m_f64MeasuredVariable = fabsf(m_f64MeasuredVariable) * -1.0;
        		}
        	}

            //calculation
            // if speed = 0 is requested output is immediately set to zero (when enabled)
        	// TODO using set speed direction is a big problem, check this
            //if (m_f64SetPoint==0 && m_bPID_Zero_Beh) {
        	if (fabsf(m_f64SetPoint)<0.02) {
                m_f64LastOutput = 0;
                m_f64accumulatedVariable = 0;
                m_f64LastMeasuredError = 0;
            //} else if(m_f64SetPoint==0 && m_f64MeasuredVariable < 0.05) {
            //	m_f64LastOutput = 0;
            //	m_f64accumulatedVariable = 0;
            //	 m_f64LastMeasuredError = 0;
            } else {
                m_f64LastOutput = getControllerValue(m_f64MeasuredVariable)*m_f64PT1OutputFactor;
            }

            TSignalValue::Data manipuated_speed; // controller output
            manipuated_speed.ui32_arduinoTimestamp = 0; // TODO timestamp
            manipuated_speed.f32_value = static_cast<tFloat32>(m_f64LastOutput);
            manipulatedSpeedOut.Transmit(&m_manipulated_speed_output, manipuated_speed, _clock->GetStreamTime());

            ////////LOG_INFO(cString::Format("PIDController: out. speed: %f", manipuated_speed.f32_value));
        }
        else if (pSource == &m_set_point_input)
        {                
        	m_f64SetPoint = static_cast<tFloat64>(ProcessSetPointInput(pMediaSample));

        	if (m_i32ControllerMode==4) {
        		m_f64SetPoint = m_f64SetPoint* m_f64PT1CorrectionFactor;
        	}
        }
    }
    RETURN_NOERROR;
}

tFloat64 PIDController::getControllerValue(tFloat64 i_f64MeasuredValue)
{

    //i_f64MeasuredValue = (i_f64MeasuredValue +  m_f64LastSpeedValue) /2.0;

    //m_f64LastSpeedValue = i_f64MeasuredValue;

    tFloat f64Result = 0;

    //the three controller algorithms
    if (m_i32ControllerMode==1)
    {
        //m_lastSampleTime = GetTime();

        //algorithm:
        //y = Kp * e
        //error:
        tFloat64 f64Error = (m_f64SetPoint-i_f64MeasuredValue);

        f64Result = m_f64PIDKp * f64Error;
    }
    else if(m_i32ControllerMode==2) //PI- Regler
    {
        //m_lastSampleTime = GetTime();

        //algorithm:
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum
        //error:
        tFloat64 f64Error = (m_f64SetPoint-i_f64MeasuredValue);
        // accumulated error:

        m_f64accumulatedVariable +=(f64Error*m_f64PIDSampleTime);

        tFloat64 tmp_result = m_f64PIDKp*f64Error
                +(m_f64PIDKi*m_f64accumulatedVariable);

        // Anti Windup
        if(m_i32AntiWindupMethod == 2) {
			if (tmp_result >= m_f64PIDMaximumOutput) {
				m_f64accumulatedVariable = 0;
				if (m_bShowDebug) {
					LOG_WARNING(cString::Format("PIDController: Anti windup, Reset mode (max), resetted integral term, tmp_res: %f, max: %f", tmp_result, m_f64PIDMaximumOutput));
				}
			} else if (tmp_result <= m_f64PIDMinimumOutput) {
				m_f64accumulatedVariable = 0;
				if (m_bShowDebug) {
					LOG_WARNING(cString::Format("PIDController: Anti windup, Reset mode (min), resetted integral term, tmp_res: %f, min: %f", tmp_result, m_f64PIDMinimumOutput));
				}
		    }
        } else if(m_i32AntiWindupMethod == 3) {
			if (tmp_result >= m_f64PIDMaximumOutput) {
				m_f64accumulatedVariable += m_f64PIDMaximumOutput - tmp_result;
				if(m_f64accumulatedVariable < 0) m_f64accumulatedVariable = 0;
				if (m_bShowDebug) {
					LOG_WARNING(cString::Format("PIDController: Anti windup, Stabil mode (max), stabilized integral term, tmp_res: %f, new integral term: %f, max: %f", tmp_result, m_f64accumulatedVariable, m_f64PIDMaximumOutput));
				}
			} else if (tmp_result <= m_f64PIDMinimumOutput) {
				m_f64accumulatedVariable += m_f64PIDMinimumOutput - tmp_result;
				if(m_f64accumulatedVariable > 0) m_f64accumulatedVariable = 0;
				if (m_bShowDebug) {
					LOG_WARNING(cString::Format("PIDController: Anti windup, Stabil mode (min), stabilized integral term, tmp_res: %f, new integral term: %f, min: %f", tmp_result, m_f64accumulatedVariable, m_f64PIDMinimumOutput));
				}
		    }
        }

        f64Result = m_f64PIDKp*f64Error
            +(m_f64PIDKi*m_f64accumulatedVariable);

    }
    else if(m_i32ControllerMode==3) // PID
    {
        m_lastSampleTime = GetTime();
        tFloat64 f64SampleTime = m_f64PIDSampleTime;

        //algorithm:
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum + Kd * (e � ealt)/Ta
        //ealt = e

        //error:
        tFloat64 f64Error = (m_f64SetPoint-i_f64MeasuredValue);
        // accumulated error:
        m_f64accumulatedVariable +=f64Error*m_f64PIDSampleTime;

        f64Result =  m_f64PIDKp*f64Error
            +(m_f64PIDKi*m_f64accumulatedVariable)
            +m_f64PIDKd*(f64Error-m_f64LastMeasuredError)/f64SampleTime;

        m_f64LastMeasuredError = f64Error;
    }    
    else if (m_i32ControllerMode==4) // PT1
    {
        /*********************************
        * PT 1 discrete algorithm
        *
        *               Tau
        *       In +    ---  * LastOut
        *             Tsample
        * Out = ---------------------
        *               Tau
        *       1 +     ---
        *             Tsample
        *
        *                           Tau
        * here with     PT1Gain =   ---
        *                         Tsample
        *
        *                  T
        * y(k) = y(k-1) + --- ( v * e(k)  - y(k-1))
        *                  T1
        *
        * e(k):  input
        * y(k-1) : last output
        * v : gain
        * T/T1: time constant
        ********************************************/
         f64Result = m_f64LastOutput + m_f64PT1TimeConstant * (m_f64PT1Gain *
             (m_f64SetPoint-i_f64MeasuredValue) - m_f64LastOutput);

    }
    // checking for minimum and maximum limits
    if (f64Result>m_f64PIDMaximumOutput)
    {
        f64Result = m_f64PIDMaximumOutput;
    }
    else if (f64Result<m_f64PIDMinimumOutput)
    {
        f64Result = m_f64PIDMinimumOutput;
    }

    m_f64LastOutput = f64Result;

    return f64Result;
}

tTimeStamp PIDController::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime();
}

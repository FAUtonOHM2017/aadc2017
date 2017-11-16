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
* $Author:: kuckal  $  $Date:: 2017-05-22 09:58:28#$ $Rev:: 63664    $
**********************************************************************/

#include "stdafx.h"
#include "cMarkerPos.h"
#include "TTrafficSignMap.h"

ADTF_FILTER_PLUGIN(ADTF_FILTER_DESC, OID_ADTF_FILTER_DEF, cMarkerPos);


/*! support function for getting time */
tTimeStamp cMarkerPos::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}

tResult cMarkerPos::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cMarkerPos::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cMarkerPos::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

cMarkerPos::cMarkerPos(const tChar* __info) : cFilter(__info)
{
    SetPropertyStr("Configuration","roadSign.xml");
    SetPropertyBool("Configuration" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configuration" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configuration" NSSUBPROP_DESCRIPTION, "Configuration file for the roadsign coordinates");

    getposition = tFalse;
    getroodsign = tFalse;
}

cMarkerPos::~cMarkerPos()
{
}

tResult cMarkerPos::CreateInputPins(__exception)
{
    // create the description manager
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

    // create the description for the position pin
    tChar const * strDescPosition = pDescManager->GetMediaDescription("tPosition");
    RETURN_IF_POINTER_NULL(strDescPosition);
    cObjectPtr<IMediaType> pTypePosition = new cMediaType(0, 0, 0, "tPosition", strDescPosition, IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    // create the position OutputPin
    RETURN_IF_FAILED(m_InputPosition.Create("Position", pTypePosition, this));
    RETURN_IF_FAILED(RegisterPin(&m_InputPosition));
    // set the description for the extended marker pin
    RETURN_IF_FAILED(pTypePosition->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescPosition));


    // create the description for the road sign pin
    tChar const * strDescIn = pDescManager->GetMediaDescription("tRoadSignExt");
    RETURN_IF_POINTER_NULL(strDescIn);
    cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSignExt", strDescIn, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    // create the road sign OutputPin
    RETURN_IF_FAILED(m_inputRoadSign.Create("RoadSignInput", pType, this));
    RETURN_IF_FAILED(RegisterPin(&m_inputRoadSign));
    // set the description for the road sign pin
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionRoadSignExt));

    RETURN_NOERROR;
}

tResult cMarkerPos::CreateOutputPins(__exception)
{
    RETURN_IF_FAILED(m_mapOutputPin.Create("RoadSign_MAP", tTrafficSignMap.GetMediaType(), NULL));
    RETURN_IF_FAILED(RegisterPin(&m_mapOutputPin));
    RETURN_NOERROR;
}

tResult cMarkerPos::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        RETURN_IF_FAILED(tTrafficSignMap.StageFirst(__exception_ptr));

        m_bIDsRoadSignExtSet = tFalse;
        m_bIDsPositionSet = tFalse;
        m_bIDsRoadSignExtGet = tFalse;
        // create the input and output pins
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));
    }
    else if (eStage == StageNormal)
    {
        if (m_bDebugModeEnabled)
        {
            m_log = fopen("markerLog.txt","w");
        }

        // load roadsign configuration
        LoadConfiguration();
    }
    else if (eStage == StageGraphReady)
    {
        RETURN_IF_FAILED(tTrafficSignMap.StageGraphReady());
    }

    RETURN_NOERROR;
}

tResult cMarkerPos::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    __synchronized_obj(m_critSecOnPinEvent);

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if(getroodsign == tTrue && getposition == tTrue)
    {
        //LOG_ERROR("ProcessDistance");
        ProcessDistance();
    }

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // process RoadSignExt sample
        if (pSource == &m_inputRoadSign)
        {
            getroodsign = tTrue;
            RETURN_IF_FAILED(ProcessRoadSignStructExt(pMediaSample));

        }
        else if (pSource == &m_InputPosition)
        {
            getposition = tTrue;
            RETURN_IF_FAILED(ProcessPosition(pMediaSample));

        }
    }

    RETURN_NOERROR;
}


/*!
 * loads road-sign configuration from a file, and stores into memory
 * */
tResult cMarkerPos::LoadConfiguration()
{
    cFilename fileConfig = GetPropertyStr("Configuration");

    // create absolute path for marker configuration file
    ADTF_GET_CONFIG_FILENAME(fileConfig);
    fileConfig = fileConfig.CreateAbsolutePath(".");

    if (fileConfig.IsEmpty())
    {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    tInt i = 0;
    if (cFileSystem::Exists(fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(fileConfig);
        cDOMElementRefList oElems;

        if(IS_OK(oDOM.FindNodes("configuration/roadSign", oElems)))
        {
            for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
            {
                roadSign item;

                item.u16Id = tUInt16((*itElem)->GetAttribute("id","0").AsInt32());
                item.f32X = tFloat32((*itElem)->GetAttribute("x","0").AsFloat64());
                item.f32Y = tFloat32((*itElem)->GetAttribute("y","0").AsFloat64());
                item.f32Radius = tFloat32((*itElem)->GetAttribute("radius","0").AsFloat64());
                item.f32Direction = tFloat32((*itElem)->GetAttribute("direction","0").AsFloat64());

                item.u16Cnt = 0;
                item.u32ticks = GetTime();

                item.f32Direction *= DEG2RAD; // convert to radians

//                if (!m_bDebugModeEnabled)
//                {
//                    LOG_INFO(cString::Format("LoadConfiguration ID %d XY %f %f Radius %f Direction %f",
//                                             item.u16Id, item.f32X, item.f32Y, item.f32Radius, item.f32Direction));
//                }

                m_roadSigns.push_back(item);
                i++;
            }
        }
    }
    else
    {
        LOG_ERROR("Configuration file does not exist");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

tResult cMarkerPos::ProcessRoadSignStructExt(IMediaSample* pMediaSampleIn)
{
    {
        // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescriptionRoadSignExt,pMediaSampleIn,pCoderInput);

        // get IDs
        if(!m_bIDsRoadSignExtGet)
        {
            pCoderInput->GetID("i16Identifier",ids.i16Identifier);
            pCoderInput->GetID("f32Imagesize", ids.f32Imagesize);
            pCoderInput->GetID("af32RVec[0]", ids.af32RVec[0]);
            pCoderInput->GetID("af32RVec[1]", ids.af32RVec[1]);
            pCoderInput->GetID("af32RVec[2]", ids.af32RVec[2]);

            pCoderInput->GetID("af32TVec[0]", ids.af32TVec[0]);
            pCoderInput->GetID("af32TVec[1]", ids.af32TVec[1]);
            pCoderInput->GetID("af32TVec[2]", ids.af32TVec[2]);
            m_bIDsRoadSignExtGet = tTrue;
        }

        pCoderInput->Get(ids.i16Identifier, (tVoid*)&m_i16ID);
        pCoderInput->Get(ids.f32Imagesize, (tVoid*)&m_f32MarkerSize);


        for (int i = 0; i < 3; ++i) {
            pCoderInput->Get(ids.af32RVec[i], (tVoid*) &data.af32RVec[i]);
            pCoderInput->Get(ids.af32TVec[i], (tVoid*) &data.af32TVec[i]);
        }
    }

    //LOG_ERROR(cString::Format("ProcessRoadSignStructExt"));
    RETURN_NOERROR;
}


tResult cMarkerPos::ProcessPosition(IMediaSample* pMediaSampleIn)
{
    {
        // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pDescPosition,pMediaSampleIn,pCoderInput);

        // get IDs
        if(!m_bIDsPositionSet)
        {
            pCoderInput->GetID("f32x", m_szIDPositionF32X);
            pCoderInput->GetID("f32y", m_szIDPositionF32Y);
            pCoderInput->GetID("f32radius", m_szIDPositionF32Radius);
            pCoderInput->GetID("f32speed", m_szIDPositionF32Speed);
            pCoderInput->GetID("f32heading", m_szIDPositionF32Heading);

            m_bIDsPositionSet = tTrue;
        }

        pCoderInput->Get(m_szIDPositionF32X, (tVoid*)&f32X);
        pCoderInput->Get(m_szIDPositionF32Y, (tVoid*)&f32Y);
        pCoderInput->Get(m_szIDPositionF32Radius, (tVoid*)&f32Radius);
        pCoderInput->Get(m_szIDPositionF32Speed, (tVoid*)&f32Speed);
        pCoderInput->Get(m_szIDPositionF32Heading, (tVoid*)&f32Heading);
    }

    RETURN_NOERROR;
}


tResult cMarkerPos::ProcessDistance()
{
    tFloat32 pi = 3.14159;
    _frameCount++;

    //Only process every 10 frame
    if(_frameCount < 30){
        RETURN_NOERROR;
    }
    else{
        _frameCount = 0;
    }

    tFloat32 foundradius = 0.45;
    //tInt32 notfoundradius = 0.3;

    for(uint i=0;i<m_roadSigns.size();i++)
    {
        if(f32Heading <= (pi/4)  && f32Heading >=  (-pi/4))
        {
            if (m_i16ID >= 0 && m_i16ID <= 14)
            {
                rosi.i16Identifier = m_i16ID;
                //                LOG_ERROR(cString::Format("X : plus"));
                //                LOG_ERROR(cString::Format("ID : %d", rosi.i16Identifier));
                if(m_roadSigns[i].f32X + foundradius >= data.af32TVec[0] + f32X && m_roadSigns[i].f32Y + foundradius >= data.af32TVec[1] + f32Y &&
                        m_roadSigns[i].f32X - foundradius  <= data.af32TVec[0] + f32X && m_roadSigns[i].f32Y - foundradius <= data.af32TVec[1] + f32Y)
                {
                    rosi.f32x = m_roadSigns[i].f32X;
                    rosi.f32y = m_roadSigns[i].f32Y;
                    rosi.f32angle = ( (m_roadSigns[i].f32Direction * 2 * pi) / 360 );
                    LOG_ERROR("111111111111");
                }
            }
        }

        else if(f32Heading <= (-pi/4)  && f32Heading >=  (-(3*pi)/4))
        {
            if (m_i16ID >= 0 && m_i16ID <= 14)
            {
                rosi.i16Identifier = m_i16ID;
                //                LOG_ERROR(cString::Format("Y : minus"));
                //                LOG_ERROR(cString::Format("ID : %d", rosi.i16Identifier));
                if(m_roadSigns[i].f32X + foundradius >= data.af32TVec[0] + f32X && m_roadSigns[i].f32Y + foundradius >= data.af32TVec[1] + f32Y &&
                        m_roadSigns[i].f32X - foundradius  <= data.af32TVec[0] + f32X && m_roadSigns[i].f32Y - foundradius <= data.af32TVec[1] + f32Y)
                {
                    rosi.f32x = m_roadSigns[i].f32X;
                    rosi.f32y = m_roadSigns[i].f32Y;
                    rosi.f32angle = (( m_roadSigns[i].f32Direction * 2 * pi) / 360 );
                    LOG_ERROR("2222222222222222");
                }
            }
        }

        else if(f32Heading >= (pi/4)  && f32Heading <=  ((3*pi)/4))
        {
            if (m_i16ID >= 0 && m_i16ID <= 14)
            {
                rosi.i16Identifier = m_i16ID;
                //                LOG_ERROR(cString::Format("Y : plus"));
                //                LOG_ERROR(cString::Format("ID : %d", rosi.i16Identifier));
                if(m_roadSigns[i].f32X + foundradius >= data.af32TVec[0] + f32X && m_roadSigns[i].f32Y + foundradius >= data.af32TVec[1] + f32Y &&
                        m_roadSigns[i].f32X - foundradius  <= data.af32TVec[0] + f32X && m_roadSigns[i].f32Y - foundradius <= data.af32TVec[1] + f32Y)
                {
                    rosi.f32x = m_roadSigns[i].f32X;
                    rosi.f32y = m_roadSigns[i].f32Y;
                    rosi.f32angle = (( m_roadSigns[i].f32Direction * 2 * pi) / 360 );
                    LOG_ERROR("3333333333333333");
                }
            }
        }

        else if(f32Heading >= ((3*pi)/4)  || f32Heading <=  (-(3*pi)/4))
        {
            if (m_i16ID >= 0 && m_i16ID <= 14)
            {
                rosi.i16Identifier = m_i16ID;
                //                LOG_ERROR(cString::Format("X : minus"));
                //                LOG_ERROR(cString::Format("ID : %d", rosi.i16Identifier));
                if(m_roadSigns[i].f32X + foundradius >= data.af32TVec[0] + f32X && m_roadSigns[i].f32Y + foundradius >= data.af32TVec[1] + f32Y &&
                        m_roadSigns[i].f32X - foundradius  <= data.af32TVec[0] + f32X && m_roadSigns[i].f32Y - foundradius <= data.af32TVec[1] + f32Y)
                {
                    rosi.f32x = m_roadSigns[i].f32X;
                    rosi.f32y = m_roadSigns[i].f32Y;
                    rosi.f32angle = (( m_roadSigns[i].f32Direction * 2 * pi) / 360 );
                    LOG_ERROR("44444444444444");
                }
            }
        }



        if(rosi.f32x != 0 || rosi.f32y != 0)
        {
            LOG_ERROR(cString::Format("rosi.f32x = %f ---- rosi.f32y = %f", rosi.f32x , rosi.f32y));
            //LOG_ERROR(cString::Format("TVECY = %f", data.af32TVec[1]));
            tTrafficSignMap.Transmit(&m_mapOutputPin, rosi,_clock->GetStreamTime());
        }
        getroodsign = tFalse;
        getposition = tFalse;
    }
    RETURN_NOERROR;
}



//                LOG_ERROR(cString::Format("Y : minus"));
//                LOG_ERROR(cString::Format("ID : %d", rosi.i16Identifier));
//                if(m_roadSigns[i].f32X + foundradius >= data.af32TVec[0] + f32X)
//                {
//                    LOG_ERROR("11111111111111111111111111111111111111111111111111111111111111111");
//                    LOG_ERROR("______________________________________________________________________________________________________________________________");
//                    LOG_ERROR(cString::Format(" X-Pos Schild = %f ------ %d", m_roadSigns[i].f32X, i));
// //                    LOG_ERROR(cString::Format(" Y-Pos Schild = %f ", m_roadSigns[i].f32Y));
// //                    LOG_ERROR(cString::Format(" X-Pos Car = %f", f32X));
// //                    LOG_ERROR(cString::Format(" Y-Pos Car = %f", f32Y));
// //                    LOG_ERROR(cString::Format(" X-Distance - Schild = %f ", data.af32TVec[0]));
// //                    LOG_ERROR(cString::Format(" Y-Distance - Schild = %f ", data.af32TVec[1]));
// //                    LOG_ERROR("______________________________________________________________________________________________________________________________");
// //                    LOG_ERROR("KEINE POSE");
//                }

//                if(m_roadSigns[i].f32Y + foundradius >= data.af32TVec[1] + f32Y)
//                {
//                    LOG_ERROR("2222222222222222222222222222222222222222222222222222222222222222");
//                     LOG_ERROR("______________________________________________________________________________________________________________________________");
//                     LOG_ERROR(cString::Format(" X-Pos Schild = %f ------ %d", m_roadSigns[i].f32X, i));
// //                    LOG_ERROR(cString::Format(" Y-Pos Schild = %f ", m_roadSigns[i].f32Y));
// //                    LOG_ERROR(cString::Format(" X-Pos Car = %f", f32X));
// //                    LOG_ERROR(cString::Format(" Y-Pos Car = %f", f32Y));
// //                    LOG_ERROR(cString::Format(" X-Distance - Schild = %f ", data.af32TVec[0]));
// //                    LOG_ERROR(cString::Format(" Y-Distance - Schild = %f ", data.af32TVec[1]));
// //                    LOG_ERROR("______________________________________________________________________________________________________________________________");
// //                    LOG_ERROR("KEINE POSE");
//                }

//                if(m_roadSigns[i].f32X - foundradius  <= data.af32TVec[0] + f32X)
//                {
//                    LOG_ERROR("3333333333333333333333333333333333333333333333333333333333333333");
//                    LOG_ERROR("______________________________________________________________________________________________________________________________");
//                    LOG_ERROR(cString::Format(" X-Pos Schild = %f ------ %d", m_roadSigns[i].f32X, i));
// //                    LOG_ERROR(cString::Format(" Y-Pos Schild = %f ", m_roadSigns[i].f32Y));
// //                    LOG_ERROR(cString::Format(" X-Pos Car = %f", f32X));
// //                    LOG_ERROR(cString::Format(" Y-Pos Car = %f", f32Y));
// //                    LOG_ERROR(cString::Format(" X-Distance - Schild = %f ", data.af32TVec[0]));
// //                    LOG_ERROR(cString::Format(" Y-Distance - Schild = %f ", data.af32TVec[1]));
// //                    LOG_ERROR("______________________________________________________________________________________________________________________________");
// //                    LOG_ERROR("KEINE POSE");
//                }

//                if(m_roadSigns[i].f32Y - foundradius <= data.af32TVec[1] + f32Y)
//                {
//                    LOG_ERROR("4444444444444444444444444444444444444444444444444444444444444444");
//                    LOG_ERROR("______________________________________________________________________________________________________________________________");
//                    LOG_ERROR(cString::Format(" X-Pos Schild = %f ------ %d", m_roadSigns[i].f32X, i));
// //                    LOG_ERROR(cString::Format(" Y-Pos Schild = %f ", m_roadSigns[i].f32Y));
// //                    LOG_ERROR(cString::Format(" X-Pos Car = %f", f32X));
// //                    LOG_ERROR(cString::Format(" Y-Pos Car = %f", f32Y));
// //                    LOG_ERROR(cString::Format(" X-Distance - Schild = %f ", data.af32TVec[0]));
// //                    LOG_ERROR(cString::Format(" Y-Distance - Schild = %f ", data.af32TVec[1]));
// //                    LOG_ERROR("______________________________________________________________________________________________________________________________");
// //                    LOG_ERROR("KEINE POSE");
//                }


//                else
//                {
// //                    LOG_ERROR("______________________________________________________________________________________________________________________________");
// //                    LOG_ERROR(cString::Format(" X-Pos Schild = %f ------ %d", m_roadSigns[i].f32X, i));
// //                    LOG_ERROR(cString::Format(" Y-Pos Schild = %f ", m_roadSigns[i].f32Y));
// //                    LOG_ERROR(cString::Format(" X-Pos Car = %f", f32X));
// //                    LOG_ERROR(cString::Format(" Y-Pos Car = %f", f32Y));
// //                    LOG_ERROR(cString::Format(" X-Distance - Schild = %f ", data.af32TVec[0]));
// //                    LOG_ERROR(cString::Format(" Y-Distance - Schild = %f ", data.af32TVec[1]));
// //                    LOG_ERROR("______________________________________________________________________________________________________________________________");
//                    LOG_ERROR("KEINE POSE");
//                }


//LOG_ERROR("**********************************************************************************");
//LOG_ERROR("**********************************************************************************");
//LOG_ERROR("**********************************************************************************");
//LOG_ERROR("**********************************************************************************");
//LOG_ERROR("**********************************************************************************");


//        else
//        {
//            LOG_ERROR("______________________________________________________________________________________________________________________________");
//            LOG_ERROR(cString::Format("Kein OutPut für die Map, da ID = %d , f32x = %f , f32y = %f , f32angle = %f", rosi.i16Identifier, rosi.f32x , rosi.f32y , rosi.f32angle));
//            LOG_ERROR(cString::Format(" X-Pos Schild = %f ", m_roadSigns[i].f32X));
//            LOG_ERROR(cString::Format(" Y-Pos Schild = %f ", m_roadSigns[i].f32Y));
//            LOG_ERROR(cString::Format(" X-Pos Car = %f", f32X));
//            LOG_ERROR(cString::Format(" Y-Pos Car = %f", f32Y));
//            LOG_ERROR(cString::Format(" X-Distance - Schild  ", data.af32TVec[0]));
//            LOG_ERROR(cString::Format(" Y-Distance - Schild  ", data.af32TVec[1]));
//            LOG_ERROR("______________________________________________________________________________________________________________________________");
//        }




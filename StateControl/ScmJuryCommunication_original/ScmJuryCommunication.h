/**
 * THIS FILTER IS BASED ON A VERSION FROM:
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved. Team FAUtonOHM.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************

New Filter communication between the car of team FAUtonOHM and jury
**********************************************************************
* $Author:: mahill $    $Date:: 2016-03-01 22:34:07#$ $Rev:: 1.0.1   $
**********************************************************************/


#ifndef _SCM_JURY_COMMUNICATION_HEADER
#define _SCM_JURY_COMMUNICATION_HEADER

#define OID_ADTF_SCM_JURYCOMMUNICATION "adtf.user.scmJuryCommunication"

#include "stdafx.h"


class ScmJuryCommunication: public adtf::cAsyncDataTriggeredFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SCM_JURYCOMMUNICATION, "Scm Jury Communication", OBJCAT_Tool, "Scm Jury Communication", 1, 0, 3, "FAUtonOHM");


public: // construction
    ScmJuryCommunication(const tChar *);
    virtual ~ScmJuryCommunication();

protected:
    /* Object of type TActionStruct */
    TActionStruct TActionStruct_object;

    /* Object of type TFeedbackStruct */
    TFeedbackStruct TFeedbackStruct_object;

    /** INPUTS **/
    /*! input pin for the run commands submitted by the jury */
    cInputPin m_JuryStructInputPin;
    /*! input pin for the maneuver list*/
    cInputPin m_ManeuverListInputPin;
    /*! input pin for receiving signals from StateControllManagement*/
    cInputPin m_ActionStructInputPin;
    /*! dedicated input pin for */


    /** OUTPUT **/
    /*! output pin for state from driver, used for transmitting status to jury */
    cOutputPin m_DriverStructOutputPin;
    /*! output pin for FeedbackStruct, used for communication to SCM */
    cOutputPin m_FeedbackStructOutputPin;

    /* size to allocate for creating mediasample of type DriverStruct*/
    tInt nSize_DriverStruct;


    /*! overrides cFilter */
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    /*! overrides cFilter */
    virtual tResult Start(__exception = NULL);
    /*! overrides cFilter */
    virtual tResult Stop(__exception = NULL);
    /*! overrides cFilter */
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);
    
    /*! overrides cFilter, implements IPinEventSink */
    tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! overrides cFilter */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);



    /*! signal for sending the state
    @param i8StateID state to be sent; -2: Startup, -1: Error, 0: Ready, 1: Running, 2: Complete
    @param i16ManeuverEntry current entry to be sent
    */
    tResult SendState(stateCar state, tInt16 i16ManeuverEntry);
    
    /*! creates the timer for the cyclic transmits*/
    tResult createTimer();

   /*! destroys the timer for the cyclic transmits*/
    tResult destroyTimer(__exception = NULL);

    /* increments the id of the maneuver id by one and updates the list indexes;
     *  returns retval < 0 in case of error, retval = 0 for normal behaviour, retval > 0 for end of list reached */
    tInt32 incrementManeuverID();

    /*! resets the counters to the start of the current section*/
    tResult resetSection();

   /*! changes the state of the car
    @param newState the new state of the car
    */
    tResult changeState(stateCar newState);
    /* returns the current state of the car */
    stateCar getCarState();

    /*! set the maneuver id and find the correct indexes
    @param maneuverId the id of the maneuver which has to be set*/
    tResult setManeuverID(tInt maneuverId);

    /* returns the current maneuverID index*/
    tInt16 getCurManeuverID();

    /*! this functions loads the maneuver list given in the properties*/
    tResult loadManeuverList();

    /* returns the current maneuver from the maneuverList sent from jury, as uint32 (refer to enumeration in StateControl.h) */
    tUInt32 getCurrentManeuverFromJuryList();

    /* returns the finishing maneuver from the maneuverList sent from jury, that means the maneuver that was executed
     * as the last maneuver on the list, as uint32 (refer to enumeration in StateControl.h) */
    tUInt32 getFinishingManeuverFromJuryList();

    /* decodes the jury-maneuver in 'cString' format and returns the corresponding predefined tUInt32 (refer to enumeration in StateControl.h)*/
    tUInt32 decodeManeuverFromJuryList(cString tmp_currJuryManeuver);

    /* Processes the received ActionStruct from SCM */
    tResult ProcessActionData(TActionStruct::ActionSub actionSub);

    /* Function to transmit the Feedback to SCM */
    tResult TransmitFeedbackStruct(TFeedbackStruct::Data feedbackStruct);

    /* Function to get and set flag whether maneuver list was already loaded */
    tBool getFlagManeuverListLoaded();
    tResult setFlagManeuverListLoaded(tBool value);
    /* Function to get and set flag whether first transmit of maneuver was requested */
    tBool getFlagFirstTransmitRequested();
    tResult setFlagFirstTransmitRequested(tBool value);

    /* Function to get and set flag whether start signal from jury was requested */
    tBool getFlagStartSignalRequested();
    tResult setFlagStartSignalRequested(tBool value);
    /* Function to get and set flag whether start signal from jury was received */
    tBool getFlagStartSignalReceived();
    tResult setFlagStartSignalReceived(tBool value);
    /* Function to get and set flag whether jury list was completed */
    tBool getFlagCompletedJuryList();
    tResult setFlagCompletedJuryList(tBool value);

    /* Function to change or access information regarding the previously executed maneuver */
     tUInt32 getPreviousManeuver();
     tResult setPreviousManeuver(tUInt32 maneuver);
     tResult resetPreviousManeuver();

    /* Variables and descriptions necessary for input/outputs */
    /* Coder description for maneuver list */
    cObjectPtr<IMediaTypeDescription> m_pDescManeuverList;

    /*! Coder Descriptor for input jury struct */
    cObjectPtr<IMediaTypeDescription> m_pDescriptionJuryStruct;
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDJuryStructI8ActionID; 
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDJuryStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsJuryStructSet;
            
    /* not necessary for ActionStruct since information is provided by class TActionStruct */

    /*! Coder Descriptor for output driver struct*/
    cObjectPtr<IMediaTypeDescription> m_pDescriptionDriverStruct;    
    /*! the id for the i8StateID of the media description */
    tBufferID m_szIDDriverStructI8StateID; 
    /*! the id for the i16ManeuverEntry of the media description data */
    tBufferID m_szIDDriverStructI16ManeuverEntry;
    /*! indicates if bufferIDs were set */
    tBool m_bIDsDriverStructSet;

    /* not necessary for FeedbackStruct since information is provided by class TFeedbackStruct */



/** Variables, parameters and other necessary declarations **/
    
    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;
    /*! whether output of structure to console is enabled or not*/
    tBool m_bPrintStrucModeEnabled;
    /* whether output of method 'sendState' is printed to console (cyclic!) */
    tBool m_bDebugSendStateModeEnabled;

    /* bool showing whether all maneuvers have been successfully completed */
    tBool m_bCompletedJuryList;

    /* flag expressing if jury-START-signal was already sent */
    tBool m_startSignalreceived;
    /* flag expressing if jury-START-signal was already requested */
    tBool m_startSignalrequested;
    /* flag expressing if maneuver list was already sent */
    tBool m_maneuverListLoaded;
    /* flag expressing if first maneuver was already sent */
    tBool m_maneuverFirstTransmitrequested;

    /** LOADING and saving maneuver list **/

    /*! this is the filename of the maneuver list*/
    cString m_strManeuverFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

    /** Managing the states of the Car via List **/
    /*! holds the current state of the car */
    stateCar m_CarState;

    /*! holds the (global) current maneuver id of the car*/
    tInt16 m_i16CurrentManeuverID;

    /*! holds the current index of the maneuvers in the list in the section */
    tInt16 m_i16ManeuverListIndex;

    /*! holds the current index in the lists of sections */
    tInt16 m_i16SectionListIndex;

    /*! handle for the timer */
    tHandle m_hTimer;

    /*! variable to temporarily save the previous, already successfully executed maneuver */
    tUInt32 tmpPrevManeuverInt_fb;

    /** Declaration of Critical Section Parameters **/
    /* Critical Section for Output Pins */
    /*! the critical section of the transmit of carState*/
    cCriticalSection m_oCriticalSectionTransmitCarState;
    /*! the critical section of the transmit of ActionStruct */
    cCriticalSection m_oCriticalSectionTransmitFeedbackStruct;
    /* Critical Section for Timer Setup */
    cCriticalSection m_oCriticalSectionTimerSetup;
    /* Critical Section for CarState changes*/
    cCriticalSection m_oCriticalSectionCarStatus;
    /* Critical Section for ManeuverList */
    cCriticalSection m_oCriticalSectionManeuverList;
    /* Critical Section for BoolValue expressing whether maneuver list was already Loaded */
    cCriticalSection m_oCriticalSectionManListLoaded;
    /* Critical Section for BoolValue expressing whether first transmit was requested */
    cCriticalSection m_oCriticalSectionFirstTransmitReq;
    /* Critical Section for BoolValue expressing whether first transmit was requested */
    cCriticalSection m_oCriticalSectionStartSignalReq;
    /* Critical Section for BoolValue expressing whether first transmit was requested */
    cCriticalSection m_oCriticalSectionStartSignalReceived;
    /* Critical Section for BoolValue expressing whether jury maneuver list was completed */
    cCriticalSection m_oCriticalSectionCompletedJuryList;
    /* Critical Section for accessing info regarding the previously executed maneuver */
    cCriticalSection m_oCriticalSectionPreviousManeuverAccess;

};

#endif // _SCM_JURY_COMMUNICATION_HEADER

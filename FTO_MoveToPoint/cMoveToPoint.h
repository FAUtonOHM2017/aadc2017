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
#ifndef _MOVE_TO_POINT_H_
#define _MOVE_TO_POINT_H_

#define OID_ADTF_MOTE_TO_POINT_FILTER "adtf.aadc.FTO_Move_To_Point"

#include "TPoseStruct.h"
#include "TSignalValue.h"
#include "TBoolSignalValue.h"
#include "TActionStruct.h"
#include "TFeedbackStruct.h"
#include "TFollowLaneStruct.h"

#include "tinyxml2.h"



#define XMLCheckResult(a_eResult) if (a_eResult != XML_SUCCESS) { LOG_WARNING(cString::Format("Error while accessing the XML: %i\n", a_eResult)); RETURN_ERROR(-1); }
#define DEBUG


class cMoveToPoint : public adtf::cFilter
{
    /*! set the filter ID and the version */
    ADTF_FILTER(OID_ADTF_MOTE_TO_POINT_FILTER, "FTO_Move_To_Point_Filter", adtf::OBJCAT_DataFilter);



protected:

    /*! the input pins */
    //Pose input from CarPose filter can be replaced with Audi filter for absolute Pose
    cInputPin car_pose_input;
    TPoseStruct carPoseIn;

    //GoalPose with Position the car is supposed to drive to
    cInputPin goal_pose_input;
    TPoseStruct goalPoseInput;

    //ActionStruct encoding the action the filter is supposed to do
    cInputPin sm_action_input;
    TActionStruct smActionInput;

    //enabling/disabling the filter results in no output values and no reaction to inputs
    cInputPin enable_input;
    TBoolSignalValue enableInput;

    //Struct used to keep car on a straigt/rounded trajectory
    cInputPin follow_lane_input;
    TFollowLaneStruct followLaneInput; //use this as spacer for a net struct containing: distance from car center to path, roadstatus, distance to obstacle


    /*! the output pins */
    //Output to the PID_Controller with the desired speed in m/s
    cOutputPin speed_controller_output;
    TSignalValue speedControllerOut;

    //output to the steeringController with the desired steeringAngle in deg (negative levt, positive right)
    cOutputPin steering_controller_output;
    TSignalValue steeringControllerOut;

    //output to the state controller returning the momentary state of the filter / requested information by the state machine
    cOutputPin feedback_struct_output;
    TFeedbackStruct feedbackStructOut;


    /*! Data Stucts to hold last values */
    TPoseStruct::Data   latestCarPose;
    TPoseStruct::Data   carPoseOffset;
    TPoseStruct::Data   goalPoseIn;
    TBoolSignalValue::Data enableFilter;
    TFollowLaneStruct::Data followLaneStruct;

    TActionStruct::ActionSub actionStruct;

    TSignalValue::Data  nextSpeedControllerOut;
    TSignalValue::Data  nextSteeringControllerOut;



private:

    //DEBUG parameter
    #ifdef DEBUG
    tBool debugToFile, debugToConsole, debugToConsoleCounter;
    cFilename debugFileDirectory;
    #endif

    //MoveToPoint parameter
    TPoseStruct::Data mtp_desiredCarPose;
    tBool    mtp_enable, mtp_startFilter;
    tFloat32 reduceYValue;
    tFloat32 movementSpeed; //if no other value specified this one is used for setting the car speed

    //General parameter
    tFloat32 max_speed;
    tFloat32 krho_forw; // Car control factor krho forwards
    tFloat32 krho_back; // Car control factor krho backwards
    tFloat32 reduction_offset; // Speed Reduction Offset in m/s
    tFloat32 goalDistanceShutOff; //Shutoff when rho< goalDistanceShutoff
    tFloat32 kalpha;
    tFloat32 kbeta;
    tFloat32 propotional_param, integral_param, derivative_param;

    //Log Trajectory To XML
    tBool lt_enable, lt_erase;
    cFilename loggedXMLDirectory;

    //Drive Trajectory
    cFilename followPathPointsXMLPath;
    tBool dt_enable, dt_startFilter;
    tUInt32 dt_path;

    //MoveWithoutSteer


    //Action
    tUInt32 command;
    tUInt32 last_completed_path;



    /*! variables */
    //state machine
    tBool isRunning, transmitZeros, newActionCommand;
    tUInt32 transmitZerosCounter;
    tUInt32 commandBuffer;


    //move to point
    tUInt32 counter_movetopoint, counter_posesamples, counter_followLane;
    tFloat32 direction, mtp_lastRho;


    //variables for first call
    tBool firstCallMTPA, firstCallStateMachine, firstCallLogTrajectory, firstCallLastRho;

    //variables for Drive Trajectory
    TFollowLaneStruct::Data lastFollowLaneStruct;
    tUInt32 pathToGoalIndex;
    tFloat32 fl_integralValue;

    //variables to LogTrajectoryToXML
    TPoseStruct::Data   lt_lastCarPose, lt_CarPoseOffset;
    int lt_counter, lt_devider;
    tinyxml2::XMLNode * pRoot;
    tinyxml2::XMLDocument xmlDoc;


    /*! critical sections */
    cCriticalSection criticalSectionLatestCarPoseAccess, criticalSectionActionAccess, criticalSectionPoseOffsetAccess;
    cCriticalSection criticalSectionGoalPoseAccess, criticalSectionFollowLaneAccess, criticalSectionEnableAccess;
    cCriticalSection criticalSectionTransmit, m_oCriticalSectionPathAccess, criticalSectionActionCommandAccess;
    cCriticalSection criticalSectionAccessRunningState;
    typedef struct fp_points_tmp {
            tFloat32 x;
            tFloat32 y;
            tFloat32 yaw;
            tFloat32 car_speed;

            fp_points_tmp() :
                    x(0), y(0.0), yaw(0.0), car_speed(0.0){}
    } fp_points;

    typedef struct fp_paths_tmp {
            tUInt32 path_id;
            const tChar * name;
            std::vector<fp_points> points;

        fp_paths_tmp() :
            path_id(0){}
    } fp_paths;

    std::vector<fp_paths> xml_paths; // read points from xml file
    std::vector<fp_points> path_to_goal; // set path points
    std::vector<fp_points> smooth_path_to_goal; //this vector holds the points a smoothing algorithm has outputtet


    /*! function declaration*/
    //init stuff
    tResult Init(tInitStage eStage, ucom::IException** __exception_ptr);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);
    tResult Start(ucom::IException** __exception_ptr=NULL);
    tResult CreateOutputPins(__exception);
    tResult CreateInputPins(__exception);
    tResult PropertyChanged(const tChar* strName);
    //input/output handler
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult ProcessCarPoseInput(IMediaSample *);
    tResult ProcessGoalPointInput(IMediaSample*);
    tResult ProcessActionInput(IMediaSample*);
    tResult ProcessEnableInput(IMediaSample*);
    tResult ProcessFollowLaneInput(IMediaSample*);
    tResult TransmitSpeedController(tTimeStamp);
    tResult TransmitSteeringController(tTimeStamp);
    //getter/setter
    tUInt32 GetCommand();
    tResult SetCommand(tUInt32);
    tBool   GetRunningState();
    tResult SetRunningState(tBool);
    TPoseStruct::Data GetLatestCarPose();
    tResult SetLatestCarPose(TPoseStruct::Data );
    TPoseStruct::Data GetCarPoseOffset();
    tResult SetCarPoseOffset(TPoseStruct::Data);
    TPoseStruct::Data GetGoalPose();
    tResult SetGoalPose(TPoseStruct::Data);
    TFollowLaneStruct::Data GetFollowLaneStuct();
    tResult LoadPathPointData();
    tResult SetPath();


    //all the rest
    tResult InternalStateMachine(IMediaSample*);
    tResult LogTrajectoryToXML();
    tResult DriveTrajectory();
    tResult MoveWithoutSteer();
    tResult MoveToPoint();
    tResult InitNewCommand();

    //bezier function / smoothing algorithms
    tResult CreateSmoothTrajectory(tUInt32 );
    fp_points_tmp ComputeCurve(std::vector<fp_points_tmp> &, tFloat32 );


public:

    cMoveToPoint(const tChar* __info);

    virtual ~cMoveToPoint();

};

//*************************************************************************************************
#endif // _MOVE_TO_POINT_H_

/*!
*@}
*/

/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:



**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>


#include "stdafx.h"
#include "cMoveToPoint.h"
#include <unistd.h>


#include "ScmCommunication.h"




/// Create filter shell
ADTF_FILTER_PLUGIN("FTO_Move_To_Point", OID_ADTF_MOTE_TO_POINT_FILTER, cMoveToPoint);


cMoveToPoint::cMoveToPoint(const tChar* __info):cFilter(__info)
{
    //move to point
    counter_movetopoint = 0, counter_posesamples = 0, counter_followLane = 0;
    direction = 0;
    goalDistanceShutOff = 0.1;

    dt_enable = false;
    mtp_enable = false;
    lt_enable = false;
    fl_integralValue = 0;
    firstCallLogTrajectory = true;
    firstCallLastRho = true;
    isRunning = false;
    transmitZeros = true;


#ifdef DEBUG
    SetPropertyBool("DEBUG::Debug_To_File", debugToFile = false);
    SetPropertyBool("DEBUG::Debug_To_File" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool("DEBUG::Debug_To_File" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr("DEBUG::Debug_To_File" NSSUBPROP_DESCRIPTION, "enable this property to log information concerning FollowPath to file");

    SetPropertyBool("DEBUG::Debug_To_Console", debugToConsole = false);
    SetPropertyBool("DEBUG::Debug_To_Console" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool("DEBUG::Debug_To_Console" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr("DEBUG::Debug_To_Console" NSSUBPROP_DESCRIPTION, "enable this property to show debug information concerning FollowPath on console");

    SetPropertyStr("DEBUG::Debug_File_Path", debugFileDirectory = "../../../utilities/move_to_point.dat");
    SetPropertyBool("DEBUG::Debug_File_Path" NSSUBPROP_FILENAME, tTrue);
    SetPropertyBool("DEBUG::Debug_File_Path" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr("DEBUG::Debug_File_Path" NSSUBPROP_DESCRIPTION, "The .dat file containing the logged information. The Folder in which the debugging file will be placed has to exist before any logging files can be created");
    ADTF_GET_CONFIG_FILENAME(debugFileDirectory);
#endif

    /*! General Parameters*/
    SetPropertyFloat("GeneralParameter::max_speed", max_speed = 1.0);
    SetPropertyBool ("GeneralParameter::max_speed" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyInt  ("GeneralParameter::max_speed" NSSUBPROP_MAX, 2);
    SetPropertyInt  ("GeneralParameter::max_speed" NSSUBPROP_MIN, 0);
    SetPropertyStr  ("GeneralParameter::max_speed" NSSUBPROP_DESCRIPTION, "Car_Speed wont get any bigger than specified value");

    SetPropertyFloat("GeneralParameter::krho_forwards", krho_forw =  10);
    SetPropertyFloat("GeneralParameter::krho_forwards" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool ("GeneralParameter::krho_forwards" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("GeneralParameter::krho_forwards" NSSUBPROP_DESCRIPTION,"Car control factor krho forwards (speed reduction) (krho > 0)");

    SetPropertyFloat("GeneralParameter::krho_backwards", krho_back = 10);
    SetPropertyFloat("GeneralParameter::krho_backwards" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool ("GeneralParameter::krho_backwards" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("GeneralParameter::krho_backwards" NSSUBPROP_DESCRIPTION, "Car control factor krho backwards (speed reduction) (krho > 0)");

    SetPropertyFloat("GeneralParameter::Speed_Reduction_Offset", reduction_offset = 0);
    SetPropertyFloat("GeneralParameter::Speed_Reduction_Offset" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool ("GeneralParameter::Speed_Reduction_Offset" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("GeneralParameter::Speed_Reduction_Offset" NSSUBPROP_DESCRIPTION, "Speed Reduction Offset in m/s (increases absolute value of car speed when near next path point)");

    SetPropertyFloat("GeneralParameter::Goal_Distance_Shutoff", goalDistanceShutOff = 0.1);
    SetPropertyFloat("GeneralParameter::Goal_Distance_Shutoff" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool ("GeneralParameter::Goal_Distance_Shutoff" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("GeneralParameter::Goal_Distance_Shutoff" NSSUBPROP_DESCRIPTION, "as soon as p < goalDistanceShutOff the speedControllerOutput is set to 0");

    SetPropertyFloat("GeneralParameter::kBeta", kbeta = 7);
    SetPropertyFloat("GeneralParameter::kBeta" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyBool ("GeneralParameter::kBeta" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("GeneralParameter::kBeta" NSSUBPROP_DESCRIPTION, "beta value for steering calculation");

    SetPropertyFloat("GeneralParameter::kAlpha", kalpha = 8);
    SetPropertyBool ("GeneralParameter::kAlpha" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("GeneralParameter::kAlpha" NSSUBPROP_DESCRIPTION, "Car control factor kalpha (car to goal) (kalpha - krho > 0)");

    SetPropertyFloat("GeneralParameter::Proportional_Term", propotional_param = 1);
    SetPropertyBool ("GeneralParameter::Proportional_Term" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("GeneralParameter::Proportional_Term" NSSUBPROP_DESCRIPTION, "Proportional_Term used to control the steering angle");

    SetPropertyFloat("GeneralParameter::Derivative_Term", derivative_param = 1);
    SetPropertyBool ("GeneralParameter::Derivative_Term" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("GeneralParameter::Derivative_Term" NSSUBPROP_DESCRIPTION, "Derivative_Term used to control the steering angle");

    SetPropertyFloat("GeneralParameter::Integral_Term", integral_param = 1);
    SetPropertyBool ("GeneralParameter::Integral_Term" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("GeneralParameter::Integral_Term" NSSUBPROP_DESCRIPTION, "Integral_Term used to control the steering angle");


    /*! MoveToPoint Parameters*/
    SetPropertyBool ("MoveToPoint::Enable_Filter", mtp_enable = false);
    SetPropertyBool ("MoveToPoint::Enable_Filter" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool ("MoveToPoint::Enable_Filter" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("MoveToPoint::Enable_Filter"NSSUBPROP_DESCRIPTION, "As soon as this function is activated it overrides the other functions (other driving options are not possible)");

    SetPropertyBool ("MoveToPoint::Start_Filter", mtp_startFilter = false);
    SetPropertyBool ("MoveToPoint::Start_Filter" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool ("MoveToPoint::Start_Filter" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("MoveToPoint::Start_Filter"NSSUBPROP_DESCRIPTION, "Starts the Filter to drive to the specified pose");

    SetPropertyFloat("MoveToPoint::Car_Pose_X", mtp_desiredCarPose.f32_x = 0);
    SetPropertyBool ("MoveToPoint::Car_Pose_X" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool ("MoveToPoint::Car_Pose_X" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("MoveToPoint::Car_Pose_X" NSSUBPROP_DESCRIPTION, "For testing purpose set the desired X-Position right here");

    SetPropertyFloat("MoveToPoint::Car_Pose_Y", mtp_desiredCarPose.f32_y = 0);
    SetPropertyBool ("MoveToPoint::Car_Pose_Y" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool ("MoveToPoint::Car_Pose_Y" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("MoveToPoint::Car_Pose_Y" NSSUBPROP_DESCRIPTION, "For testing purpose set the desired Y-Position right here");

    SetPropertyFloat("MoveToPoint::Car_Pose_Yaw", mtp_desiredCarPose.f32_yaw = 0);
    SetPropertyBool ("MoveToPoint::Car_Pose_Yaw" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool ("MoveToPoint::Car_Pose_Yaw" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("MoveToPoint::Car_Pose_Yaw" NSSUBPROP_DESCRIPTION, "For testing purpose set the desired Yaw-Position in Degree right here");

    SetPropertyFloat("MoveToPoint::Movement_Speed", movementSpeed = 0.5);
    SetPropertyBool ("MoveToPoint::Movement_Speed" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("MoveToPoint::Movement_Speed" NSSUBPROP_DESCRIPTION, "Change this Parameter to increase the speed the car will drive");
    SetPropertyInt  ("MoveToPoint::Movement_Speed" NSSUBPROP_MAX, 4);
    SetPropertyInt  ("MoveToPoint::Movement_Speed" NSSUBPROP_MIN, 0);


    SetPropertyFloat("MoveToPoint::Set_Y_Parameter", reduceYValue = 0.03);
    SetPropertyBool ("MoveToPoint::Set_Y_Parameter" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("MoveToPoint::Set_Y_Parameter" NSSUBPROP_DESCRIPTION, "Change this parameter to reduce y value of goto_xy position");
    SetPropertyInt  ("MoveToPoint::Set_Y_Parameter" NSSUBPROP_MAX, 1.0);
    SetPropertyInt  ("MoveToPoint::Set_Y_Parameter" NSSUBPROP_MIN, 0);




    /*! properties for logging driven trajectory to xml*/
    SetPropertyBool("LogTrajectory::Enable_Filter", lt_enable = false);
    SetPropertyBool("LogTrajectory::Enable_Filter" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool("LogTrajectory::Enable_Filter" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr ("LogTrajectory::Enable_Filter"NSSUBPROP_DESCRIPTION, "As soon as this function is activated it overrides the other functions (other driving options are not possible)");

    SetPropertyInt ("LogTrajectory::Log_Counter", lt_devider = 30);
    SetPropertyBool("LogTrajectory::Log_Counter" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool("LogTrajectory::Log_Counter" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr ("LogTrajectory::Log_Counter"NSSUBPROP_DESCRIPTION, cString::Format("every %d. sample will be logged to file", lt_counter));

    SetPropertyBool("LogTrajectory::Erase_XML", lt_erase = false);
    SetPropertyBool("LogTrajectory::Erase_XML" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool("LogTrajectory::Erase_XML" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr ("LogTrajectory::Erase_XML"NSSUBPROP_DESCRIPTION, "activate to erase the whole file (has to be unchecked before the new trajectory will be logged)");

    SetPropertyStr ("LogTrajectory::Path_To_XML", loggedXMLDirectory = "../../../utilities/TrajectoryToXML.xml");
    SetPropertyBool("LogTrajectory::Path_To_XML" NSSUBPROP_FILENAME, tTrue);
    SetPropertyBool("LogTrajectory::Path_To_XML" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr ("LogTrajectory::Path_To_XML" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr ("LogTrajectory::Path_To_XML" NSSUBPROP_DESCRIPTION, "The XML file to which the path will be saved");


    /*! Move Without Steer Parameters*/


    /*! Drive Trajectory */

    SetPropertyBool ("DriveTrajectory::Enable_Filter", dt_enable = false);
    SetPropertyBool ("DriveTrajectory::Enable_Filter" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool ("DriveTrajectory::Enable_Filter" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("DriveTrajectory::Enable_Filter"NSSUBPROP_DESCRIPTION, "As soon as this function is activated it overrides the other functions (other driving options are not possible)");

    SetPropertyBool ("DriveTrajectory::Start_Filter", dt_startFilter = false);
    SetPropertyBool ("DriveTrajectory::Start_Filter" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool ("DriveTrajectory::Start_Filter" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr  ("DriveTrajectory::Start_Filter"NSSUBPROP_DESCRIPTION, "Starts the Filter to drive to the specified pose");

    SetPropertyInt ("DriveTrajectory::Command", dt_path = 2020);
    SetPropertyBool("DriveTrajectory::Command" NSSUBPROP_DONTEXPORT, tTrue);
    SetPropertyBool("DriveTrajectory::Command" NSSUBPROP_ISCHANGEABLE , true);
    SetPropertyStr ("DriveTrajectory::Command"NSSUBPROP_DESCRIPTION, cString::Format("with start of this Filter the path: %d will be executed ", lt_counter));

    SetPropertyStr  ("DriveTrajectory::Path_Points_XML_Directory", followPathPointsXMLPath = "../../../utilities/FollowPathPoints_16.xml");
    SetPropertyBool ("DriveTrajectory::Path_Points_XML_Directory" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr  ("DriveTrajectory::Path_Points_XML_Directory" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyBool ("DriveTrajectory::Path_Points_XML_Directory" NSSUBPROP_ISCHANGEABLE, true);
    SetPropertyStr  ("DriveTrajectory::Path_Points_XML_Directory" NSSUBPROP_DESCRIPTION, "The XML file containing path points has to be set here");


    /*! properties for follow lane */


    /*! properties used by different functionalities */
}

cMoveToPoint::~cMoveToPoint()
{
}


tResult cMoveToPoint::Start(ucom::IException** __exception_ptr){
	LoadPathPointData();
	RETURN_NOERROR;
}

tResult cMoveToPoint::Init(tInitStage eStage, __exception)
{

    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        //input
        RETURN_IF_FAILED(carPoseIn.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(goalPoseInput.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(smActionInput.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(enableInput.StageFirst(__exception_ptr));
        //RETURN_IF_FAILED(followLaneInput.StageFirst(__exception_ptr));

        //output
        RETURN_IF_FAILED(speedControllerOut.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(steeringControllerOut.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(feedbackStructOut.StageFirst(__exception_ptr));

        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

#ifdef DEBUG
        LOG_INFO(cString::Format("MoveToPoint::Creation of Pins complete"));
#endif

    }
    else if (eStage == StageNormal)
    {
    }
    else if (eStage == StageGraphReady)
    {
        //input
        RETURN_IF_FAILED(carPoseIn.StageGraphReady());
        RETURN_IF_FAILED(goalPoseInput.StageGraphReady());
        RETURN_IF_FAILED(smActionInput.StageGraphReady());
        RETURN_IF_FAILED(enableInput.StageGraphReady());
        //RETURN_IF_FAILED(followLaneInput.StageGraphReady());

        //output
        RETURN_IF_FAILED(speedControllerOut.StageGraphReady());
        RETURN_IF_FAILED(steeringControllerOut.StageGraphReady());
        RETURN_IF_FAILED(feedbackStructOut.StageGraphReady());

    }
#ifdef DEBUG
    LOG_INFO(cString::Format("MoveToPoint::Init() successfull"));
#endif

    RETURN_NOERROR;
}

tResult cMoveToPoint::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception:
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    if (eStage == StageGraphReady)
    {
        firstCallStateMachine = true;
        firstCallMTPA = false;
        mtp_enable = false;
        //change speed controller output to work with the car (forward is -, backwards +)
        if(nextSpeedControllerOut.f32_value > 0.5){
            nextSpeedControllerOut.f32_value = 0.5;
        } else if(nextSpeedControllerOut.f32_value < -0.5){
                nextSpeedControllerOut.f32_value = -0.5;
        }
        mtp_startFilter = false;
        counter_posesamples = 0, counter_movetopoint = 0;
    }
    else if (eStage == StageNormal)
    {


    }
    else if (eStage == StageFirst)
    {
    }

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}


tResult cMoveToPoint::PropertyChanged(const tChar* strName)
{

    #ifdef DEBUG
    //DEBUG
    if(      cString::IsEqual(strName, "DEBUG::Debug_To_Console")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(debugToConsole = cFilter::GetPropertyBool("DEBUG::Debug_To_Console", false), "Error while fetching Debug_To_Console");
    }else if(cString::IsEqual(strName, "DEBUG::Debug_To_File")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(debugToFile = cFilter::GetPropertyBool("DEBUG::Debug_To_File", false), "Error while fetching Debug_To_File");
    }else if(cString::IsEqual(strName, "DEBUG::Debug_File_Path")){
        debugFileDirectory = cFilter::GetPropertyStr("DEBUG::Debug_File_Path", NULL, 0, "../../../utilities/move_to_point.dat");
        ADTF_GET_CONFIG_FILENAME(debugFileDirectory);
        if(debugFileDirectory == NULL){
            RETURN_ERROR(-1);
        }
    }
    #endif

    //Move to Point
    if(      cString::IsEqual(strName, "MoveToPoint::Enable_Filter")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(mtp_enable = cFilter::GetPropertyBool("MoveToPoint::Enable_Filter", false), "Error while fetching Enable Filterproperty");
    }else if(cString::IsEqual(strName, "MoveToPoint::Start_Filter")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(mtp_startFilter = cFilter::GetPropertyBool("MoveToPoint::Start_Filter", false), "Error while start Filter property");
    }else if(cString::IsEqual(strName, "MoveToPoint::Car_Pose_X")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(mtp_desiredCarPose.f32_x = cFilter::GetPropertyFloat("MoveToPoint::Car_Pose_X", 0), "Error while fetching CarPoseX property");
    }else if(cString::IsEqual(strName, "MoveToPoint::Car_Pose_Y")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(mtp_desiredCarPose.f32_y = cFilter::GetPropertyFloat("MoveToPoint::Car_Pose_Y", 0), "Error while fetching CarPoseY property");
    }else if(cString::IsEqual(strName, "MoveToPoint::Car_Pose_Yaw")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(mtp_desiredCarPose.f32_yaw = cFilter::GetPropertyFloat("MoveToPoint::Car_Pose_Yaw", 0), "Error while fetching CarPoseYaw property");
    }else if(cString::IsEqual(strName, "MoveToPoint::Movement_Speed")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(movementSpeed = cFilter::GetPropertyFloat("MoveToPoint::Movement_Speed", 0.5), "Error while fetching Movement Speed property");
    }else if(cString::IsEqual(strName, "MoveToPoint::Set_Y_Parameter")){
            RETURN_IF_FAILED_AND_LOG_ERROR_STR(reduceYValue = cFilter::GetPropertyFloat("MoveToPoint::Set_Y_Parameter", 0.5), "Error while fetching Set_Y_Parameter property");
        }

    //General parameter
    else if(cString::IsEqual(strName, "GeneralParameter::max_speed")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(max_speed = cFilter::GetPropertyFloat("GeneralParameter::max_speed", 1.0), "Error while fetching GeneralParameter::max_speed");
    }else if(cString::IsEqual(strName, "GeneralParameter::krho_forwards")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(krho_forw = cFilter::GetPropertyFloat("GeneralParameter::krho_forwards", 0), "Error while fetching krho_forw property");
    }else if(cString::IsEqual(strName, "GeneralParameter::krho_backwards")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(krho_back = cFilter::GetPropertyFloat("GeneralParameter::krho_backwards", 0), "Error while fetching krho_back property");
    }else if(cString::IsEqual(strName, "GeneralParameter::Speed_Reduction_Offset")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(reduction_offset = cFilter::GetPropertyFloat("GeneralParameter::Speed_Reduction_Offset", 0), "Error while fetching reduction_offset property");
    }else if(cString::IsEqual(strName, "GeneralParameter::Goal_Distance_Shutoff")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(goalDistanceShutOff = cFilter::GetPropertyFloat("GeneralParameter::Goal_Distance_Shutoff", 0), "Error while fetching goalDistanceShutOff property");
    }else if(cString::IsEqual(strName, "GeneralParameter::kBeta")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(kbeta = cFilter::GetPropertyFloat("GeneralParameter::kBeta", 5), "Error while fetching KBeta property Put here a positive Value as ADTF is not capable of reading negative values");
    }else if(cString::IsEqual(strName, "GeneralParameter::kAlpha")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(kalpha = cFilter::GetPropertyFloat("GeneralParameter::kAlpha", 7), "Error while fetching MoveToPoint::kalpha");
    }else if(cString::IsEqual(strName, "GeneralParameter::Proportional_Term")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(propotional_param = cFilter::GetPropertyFloat("GeneralParameter::Proportional_Term", 0), "Error while fetching FollowLane::Proportional_Term");
    }else if(cString::IsEqual(strName, "GeneralParameter::Integral_Term")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(integral_param = cFilter::GetPropertyFloat("GeneralParameter::Integral_Term", 0), "Error while fetching FollowLane::Integral_Term");
    }else if(cString::IsEqual(strName, "GeneralParameter::Derivative_Term")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(derivative_param = cFilter::GetPropertyFloat("GeneralParameter::Derivative_Term", 0), "Error while fetching FollowLane::Derivative_Term");
    }

    //Logging Trajectory
    else if(cString::IsEqual(strName, "LogTrajectory::Enable_Filter")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(lt_enable = cFilter::GetPropertyBool("LogTrajectory::Enable_Filter", false), "Error while fetching LogTrajectory::Enable_Filter");
    }else if(cString::IsEqual(strName, "LogTrajectory::Erase_XML")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(lt_erase = cFilter::GetPropertyBool("LogTrajectory::Erase_XML", false), "Error while fetching LogTrajectory::Erase_XML");
    }else if(cString::IsEqual(strName, "LogTrajectory::Path_To_XML")){
        loggedXMLDirectory = cFilter::GetPropertyStr("LogTrajectory::Path_To_XML", NULL, 0, "../../../utilities/TrajectoryToXML.xml");
        if(loggedXMLDirectory == NULL){
            RETURN_ERROR(-1);
        }
    }else if(cString::IsEqual(strName, "LogTrajectory::Log_Counter")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(lt_devider = cFilter::GetPropertyInt("LogTrajectory::Log_Counter", 30), "Error while fetching LogTrajectory::Log_Counter");
    }

    //Driving Trajectory
    else if(cString::IsEqual(strName, "DriveTrajectory::Path_Points_XML_Directory")){
        followPathPointsXMLPath = cFilter::GetPropertyStr("DriveTrajectory::Path_Points_XML_Directory", NULL, 0, "../../../utilities/FollowPathPoints_16.xml");
        RETURN_IF_FAILED(LoadPathPointData());
        //LOG_INFO(cString::Format("cMoveToPoint:: followpathpoints has changed to  %s", followPathPointsXMLPath.GetName()));
        if(followPathPointsXMLPath == NULL){
            RETURN_ERROR(-1);
        }
    }else if(cString::IsEqual(strName, "DriveTrajectory::Enable_Filter")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(dt_enable = cFilter::GetPropertyBool("DriveTrajectory::Enable_Filter", false), "Error while fetching DriveTrajectory::Enable_Filter");
    }else if(cString::IsEqual(strName, "DriveTrajectory::Start_Filter")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(dt_startFilter = cFilter::GetPropertyBool("DriveTrajectory::Start_Filter", false), "Error while fetching DriveTrajectory::Start_Filter");
    }else if(cString::IsEqual(strName, "DriveTrajectory::Command")){
        RETURN_IF_FAILED_AND_LOG_ERROR_STR(dt_path = cFilter::GetPropertyInt("DriveTrajectory::Command", 2020), "Error while fetching DriveTrajectory::Command");
    }
    RETURN_NOERROR;
}


tResult cMoveToPoint::CreateOutputPins(__exception){

    RETURN_IF_FAILED(speed_controller_output.Create("speed_controller_output", speedControllerOut.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&speed_controller_output));

    RETURN_IF_FAILED(steering_controller_output.Create("steering_controller_output", steeringControllerOut.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&steering_controller_output));

    RETURN_IF_FAILED(feedback_struct_output.Create("feedback_struct_output", feedbackStructOut.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&feedback_struct_output));

#ifdef DEBUG
    LOG_INFO(cString::Format("MoveToPoint::Call to CreateOutputPins() successfull"));
#endif

    RETURN_NOERROR;
}


tResult cMoveToPoint::CreateInputPins(__exception){

    RETURN_IF_FAILED(car_pose_input.Create("car_pose_input", carPoseIn.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&car_pose_input));

    RETURN_IF_FAILED(goal_pose_input.Create("goal_pose_input", goalPoseInput.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&goal_pose_input));

    RETURN_IF_FAILED(sm_action_input.Create("sm_action_input", smActionInput.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&sm_action_input));

    //RETURN_IF_FAILED(follow_lane_input.Create("follow_lane_input", followLaneInput.GetMediaType(), static_cast<IPinEventSink*> (this)));
    //RETURN_IF_FAILED(RegisterPin(&follow_lane_input));

    RETURN_IF_FAILED(enable_input.Create("enable_input", enableInput.GetMediaType(), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&enable_input));

#ifdef DEBUG
    LOG_INFO(cString::Format("MoveToPoint::Call to CreateInputPins() successfull"));
#endif

    RETURN_NOERROR;
}



/*****************************************************************/
/* processing input and output pins                              */
/*****************************************************************/


tResult cMoveToPoint::OnPinEvent(IPin* pSource,
                                    tInt nEventCode,
                                    tInt nParam1,
                                    tInt nParam2,
                                    IMediaSample* pMediaSample)
{
    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        // so we received a media sample, so this pointer better be valid.
        RETURN_IF_POINTER_NULL(pMediaSample);

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &car_pose_input)
        {
            ProcessCarPoseInput(pMediaSample);
            InternalStateMachine(pMediaSample);
        }else if(pSource == &goal_pose_input){
            ProcessGoalPointInput(pMediaSample);
        }else if(pSource == &sm_action_input){
            ProcessActionInput(pMediaSample);
        }else if(pSource == &enable_input){
            ProcessEnableInput(pMediaSample);
        }else if(pSource == &follow_lane_input){
            ProcessFollowLaneInput(pMediaSample);
        }
    }

    RETURN_NOERROR;
}
//inputs

tResult cMoveToPoint::ProcessCarPoseInput(IMediaSample* pMediaSample){
    __synchronized_obj(criticalSectionLatestCarPoseAccess);

    TPoseStruct::Data tmpData;
    carPoseIn.Read(pMediaSample, &tmpData);

    // log pose to file
    #ifdef DEBUG
    if ( counter_posesamples % 5 == 0) { // 1 out of 5
        if(debugToFile){
            fstream file_pose;
            file_pose.open(debugFileDirectory.CreateAbsolutePath("."), ios::out | ios::app);
            file_pose <<"f32_x: " << tmpData.f32_x << "  f32_y: " << tmpData.f32_y << "  f32_yaw(in Deg): " << tmpData.f32_yaw * 180/M_PI << "\n";
            file_pose.close();
        }
        counter_posesamples = 0;
    }
    counter_posesamples++;
    #endif

    latestCarPose = tmpData;

    RETURN_NOERROR;
}

tResult cMoveToPoint::ProcessGoalPointInput(IMediaSample* pMediaSample){
    __synchronized_obj(criticalSectionGoalPoseAccess);

    // save external goal pose sample
    goalPoseInput.Read(pMediaSample, &goalPoseIn);

    RETURN_NOERROR;
}

tResult cMoveToPoint::ProcessActionInput(IMediaSample* pMediaSample){
    __synchronized_obj(criticalSectionActionAccess);

    actionStruct = smActionInput.Read_Action(pMediaSample, F_FOLLOW_PATH);
    if(actionStruct.command == AC_FP_STOP){
        SetRunningState(tFalse);
    }
    if(actionStruct.command == AC_FP_RESET_PARKING_STATUS || actionStruct.command == AC_FP_GET_PARKING_STATUS){
        //do stuff in case it is needed
    //load new trajectory / point to drive to
    }else if(actionStruct.command >=  2010 && actionStruct.command <=  2500){
        LOG_WARNING(cString::Format("cMoveToPoint::New command received with id : %d", actionStruct.command));
        if(isRunning){
            isRunning = false;
            commandBuffer = actionStruct.command;
            newActionCommand = true;
            //send Stop Feedback
            // transmit feedback struct with filter ID and FB_FP_STOPPED
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = FB_FP_STOPPED; // TODO check feedback
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
        }
        commandBuffer = actionStruct.command;
        newActionCommand = true;
    }
    RETURN_NOERROR;
}

tResult cMoveToPoint::ProcessEnableInput(IMediaSample* pMediaSample){
    __synchronized_obj(criticalSectionEnableAccess);

    //save filter state set by state machine
    enableInput.Read(pMediaSample, &enableFilter);

    RETURN_NOERROR;
}

tResult cMoveToPoint::ProcessFollowLaneInput(IMediaSample * pMediaSample){
    __synchronized_obj(criticalSectionFollowLaneAccess);

    followLaneInput.Read(pMediaSample, &followLaneStruct);
    RETURN_NOERROR;
}


//outputs

tResult cMoveToPoint::TransmitSpeedController(tTimeStamp inputTime){
     __synchronized_obj(criticalSectionTransmit);

     TSignalValue::Data tmp_speed;
     tmp_speed.f32_value = nextSpeedControllerOut.f32_value;

     if( tmp_speed.f32_value > max_speed){
         tmp_speed.f32_value = max_speed;
     }
     if( tmp_speed.f32_value < -max_speed){
         tmp_speed.f32_value = -max_speed;
     }

     speedControllerOut.Transmit(&speed_controller_output, tmp_speed, inputTime);
     RETURN_NOERROR;
}

tResult cMoveToPoint::TransmitSteeringController(tTimeStamp inputTime){
     __synchronized_obj(criticalSectionTransmit);

     TSignalValue::Data tmp_steer;
     tmp_steer.f32_value = nextSteeringControllerOut.f32_value;

     steeringControllerOut.Transmit(&steering_controller_output, tmp_steer, inputTime);
     RETURN_NOERROR;

}


/*****************************************************************/
/* Getter/Setter                                                 */
/*****************************************************************/

tResult cMoveToPoint::SetRunningState(tBool running){
    __synchronized_obj(criticalSectionAccessRunningState);
    isRunning = running;
    RETURN_NOERROR;
}

tBool cMoveToPoint::GetRunningState(){
    __synchronized_obj(criticalSectionAccessRunningState);
    return isRunning;
}

tUInt32 cMoveToPoint::GetCommand(){
    __synchronized_obj(criticalSectionActionCommandAccess);
    return command;
}

tResult cMoveToPoint::SetCommand(tUInt32 command_){
    __synchronized_obj(criticalSectionActionCommandAccess);
    command = command_;
    RETURN_NOERROR;
}

TPoseStruct::Data cMoveToPoint::GetLatestCarPose(){
    __synchronized_obj(criticalSectionLatestCarPoseAccess);
    return latestCarPose;
}

tResult cMoveToPoint::SetLatestCarPose(TPoseStruct::Data pose){
    __synchronized_obj(criticalSectionLatestCarPoseAccess);
    latestCarPose = pose;
    RETURN_NOERROR;
}

TPoseStruct::Data cMoveToPoint::GetCarPoseOffset(){
    __synchronized_obj(criticalSectionPoseOffsetAccess);
    return carPoseOffset;
}

tResult cMoveToPoint::SetCarPoseOffset(TPoseStruct::Data offset){
    __synchronized_obj(criticalSectionPoseOffsetAccess);
    carPoseOffset = offset;
    RETURN_NOERROR;
}

TPoseStruct::Data cMoveToPoint::GetGoalPose(){
    __synchronized_obj(criticalSectionGoalPoseAccess);
    return goalPoseIn;
}

tResult cMoveToPoint::SetGoalPose(TPoseStruct::Data pose){
    __synchronized_obj(criticalSectionGoalPoseAccess);
    goalPoseIn = pose;
    RETURN_NOERROR;
}

TFollowLaneStruct::Data cMoveToPoint::GetFollowLaneStuct(){
    __synchronized_obj(criticalSectionFollowLaneAccess);
    return followLaneStruct;
}

tResult cMoveToPoint::SetPath(){
    tUInt32 tmp_command = GetCommand();
    path_to_goal.clear();

    if(tmp_command == AC_FP_GOTO_XY || tmp_command == AC_FP_GOTO_XY_NOSTOP){
       //do nothing beside setting y = 0 as linespecifier returns shitty values for y
       TPoseStruct::Data tmp_external_goal = GetGoalPose();
       //if(tmp_external_goal.f32_y > reduceYValue){
        //   tmp_external_goal.f32_y = reduceYValue;
       //}
       //tmp_external_goal.f32_y = 0;
       if(tmp_external_goal.f32_car_speed == 0){
           tmp_external_goal.f32_car_speed = 0.5;
       }
       SetGoalPose(tmp_external_goal);
    }else if(tmp_command == AC_FP_GOTO_XY_NOSTEERING || tmp_command == AC_FP_GOTO_XY_NOSTEERING_NOSTOP){
        TPoseStruct::Data tmp_external_goal = GetGoalPose();
        if(tmp_external_goal.f32_car_speed == 0){
            tmp_external_goal.f32_car_speed = 0.5;
        }
        SetGoalPose(tmp_external_goal);
    }else if(tmp_command == AC_FP_NOSTEERING_PARKSPACE_TWO || tmp_command == AC_FP_NOSTEERING_PARKSPACE_THREE || tmp_command == AC_FP_NOSTEERING_PARKSPACE_FOUR || (tmp_command >= 2370 && tmp_command <= 2490)){
        // check if path for command id exists
        std::vector<fp_paths>::iterator it = xml_paths.begin();
        while(it != xml_paths.end()) {
            if(it->path_id == tmp_command) {
                break;
            }
            it++;
        }
        if(it != xml_paths.end() && it->points.size() > 0){
            TPoseStruct::Data tmp_external_goal;
            tmp_external_goal.f32_x = it->points[0].x;
            tmp_external_goal.f32_y = it->points[0].y;
            tmp_external_goal.f32_yaw = it->points[0].yaw;
            tmp_external_goal.f32_car_speed = it->points[0].car_speed;
            SetGoalPose(tmp_external_goal);
            RETURN_NOERROR;
        }
        LOG_ERROR(cString::Format("FollowPath: No points in path for command %d in tmp_command == AC_FP_NOSTEER", tmp_command));
        RETURN_ERROR(ERR_INVALID_STATE);
    }else{
        std::vector<fp_paths>::iterator it = xml_paths.begin();
        // check if path for command id exists
        while(it != xml_paths.end()) {
            if(it->path_id == tmp_command) {
                break;
            }
            it++;
        }
        //add path to path_to_goal variable if path was found
        if (it != xml_paths.end() && it->points.size() > 0) {
                path_to_goal.resize(it->points.size());
                path_to_goal = it->points; // get points
        } else {
                LOG_ERROR(cString::Format("FollowPath: No points in path for command %d", tmp_command));
                RETURN_ERROR(ERR_INVALID_STATE);
        }

    }
    RETURN_NOERROR;
}

tResult cMoveToPoint::LoadPathPointData(){
    __synchronized_obj(m_oCriticalSectionPathAccess);
    using namespace tinyxml2;
    XMLDocument xmlDoc;

    //LOG_WARNING(cString::Format("cMoveToPoint:: followpathpoints before calling adtf_get.. %c", followPathPointsXMLPath.GetName ()));
    ADTF_GET_CONFIG_FILENAME(followPathPointsXMLPath);
    //LOG_WARNING(cString::Format("cMoveToPoint:: followpathpoints after calling adtf_get.. %c", followPathPointsXMLPath.GetName ()));
    cFilename absFilename = followPathPointsXMLPath.CreateAbsolutePath(".");

    if (cFileSystem::Exists(followPathPointsXMLPath) == tFalse ) {
            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,"cMoveToPoint:: File FollowPathPoints.xml does not exist.");
    }


    XMLError eResult = xmlDoc.LoadFile(absFilename);
    if(eResult != XML_SUCCESS) {
        LOG_WARNING(cString::Format("Error while loding the FollowPathPoints: %i\n", eResult));
        RETURN_ERROR(-1);
    }

    XMLNode * pRoot = xmlDoc.FirstChildElement("calibration");

    if(pRoot == NULL){
        LOG_WARNING(cString::Format("cMoveToPoint::No Root in FollowPathPoints XML"));
        RETURN_ERROR(-1);
    }
    xml_paths.clear();

    XMLElement * pListElementPath = pRoot->FirstChildElement("Path");
    while(pListElementPath != NULL){
        fp_paths new_path;
        pListElementPath->QueryUnsignedAttribute("id", &new_path.path_id);
        new_path.name = pListElementPath->Attribute("name");
        XMLElement * pListElementPoint = pListElementPath->FirstChildElement("Point");

        while(pListElementPoint != NULL){
            fp_points tmpPoint;
            pListElementPoint->QueryFloatAttribute("x", &tmpPoint.x);
            pListElementPoint->QueryFloatAttribute("y", &tmpPoint.y);
            pListElementPoint->QueryFloatAttribute("yaw", &tmpPoint.yaw);
            pListElementPoint->QueryFloatAttribute("speed", &tmpPoint.car_speed);
            //pListElementPoint->QueryFloatAttribute("radius", tmpPoint.radius);

            new_path.points.push_back(tmpPoint);
            pListElementPoint = pListElementPoint->NextSiblingElement("Point");
        }

        xml_paths.push_back(new_path);
        pListElementPath = pListElementPath->NextSiblingElement("Path");
    }

#ifdef DEBUG
        for(tUInt i = 0; i < xml_paths.size(); i++){
            fp_paths tmp = xml_paths[i];
            for(tUInt x = 0; x < tmp.points.size(); x++){
                fp_points tmpPoint = tmp.points[x];
                LOG_WARNING(cString::Format("cMoveToPoint::path: %d, point %d, f32_x: %f, f32_y: %f, f32_yaw: %f", tmp.path_id, i, tmpPoint.x, tmpPoint.y, tmpPoint.yaw));
            }
        }
#endif
    RETURN_NOERROR;
}


/*****************************************************************/
/* Implementation of real intelligence of filter                 */
/*****************************************************************/


//filter intern state machine, decides what action filter will exectue
tResult cMoveToPoint::InternalStateMachine(IMediaSample* pMediaSample){
    //make sure 0 is transmitted if no function is called
    nextSpeedControllerOut.f32_value = 0;
    nextSteeringControllerOut.f32_value = 0;

    //transmit zeros
    if(transmitZeros){
        if(transmitZerosCounter > 10){  //about 1,5s
            transmitZerosCounter = 0;
            transmitZeros = false;
        }else{
            if(counter_movetopoint%1 == 0 && debugToConsole) LOG_INFO("cMoveToPoint::transmit zero samples");
            nextSpeedControllerOut.f32_value = 0;
            nextSteeringControllerOut.f32_value = 0;
            TransmitSpeedController(pMediaSample->GetTime());
            TransmitSteeringController(pMediaSample->GetTime());
            transmitZerosCounter++;
        }
    //drive to a manually selected point / trajectory
    }else if(dt_enable){
            if(!dt_startFilter){
                SetCarPoseOffset(GetLatestCarPose());
                direction = 0;
                pathToGoalIndex = 0;
                SetCommand(dt_path);
                SetPath();
                SetRunningState(true);
                firstCallLastRho = true;
                //CreateSmoothTrajectory(dt_path);
            }else{
                DriveTrajectory();
                //FollowTrajectory();
            }
            TransmitSpeedController(pMediaSample->GetTime());
            TransmitSteeringController(pMediaSample->GetTime());
    }else if(mtp_enable){
        if(!mtp_startFilter){
            SetCarPoseOffset(GetLatestCarPose());
            isRunning =true;
            direction = 0;
            TPoseStruct::Data goal;
            goal.f32_x = mtp_desiredCarPose.f32_x;
            goal.f32_y = mtp_desiredCarPose.f32_y;
            goal.f32_yaw = mtp_desiredCarPose.f32_yaw * M_PI/180;
            goal.f32_car_speed = movementSpeed;
            //SetCommand(AC_FP_GOTO_XY_NOSTEERING);
            SetGoalPose(goal);
            firstCallLastRho = true;
        }else{
            //MoveWithoutSteer();
            MoveToPoint();
            TransmitSpeedController(pMediaSample->GetTime());
            TransmitSteeringController(pMediaSample->GetTime());
        }


    //logTrajectory to xml file
    }
    else if(lt_enable){
            LogTrajectoryToXML();
    //this section is for the automatic functions
    }else if(isRunning){
        if(actionStruct.command == AC_FP_GOTO_XY || actionStruct.command == AC_FP_GOTO_XY_NOSTOP){
            MoveToPoint();
            TransmitSpeedController(pMediaSample->GetTime());
            TransmitSteeringController(pMediaSample->GetTime());
        }else if(actionStruct.command >= 2020 && actionStruct.command <= 2240){
            DriveTrajectory();
            TransmitSpeedController(pMediaSample->GetTime());
            TransmitSteeringController(pMediaSample->GetTime());
        }else if(actionStruct.command == AC_FP_GOTO_XY_NOSTEERING || actionStruct.command == AC_FP_GOTO_XY_NOSTEERING_NOSTOP || AC_FP_NOSTEERING_PARKSPACE_TWO || AC_FP_NOSTEERING_PARKSPACE_THREE || AC_FP_NOSTEERING_PARKSPACE_FOUR){
            if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_WARNING("cMoveToPoint::internal state machine went into moveWithoutSteer");
            MoveWithoutSteer();
            TransmitSpeedController(pMediaSample->GetTime());
        }else{
            if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_WARNING("cMoveToPoint::no matching Command found in InternalStateMachine");
            nextSpeedControllerOut.f32_value = 0;
            nextSteeringControllerOut.f32_value = 0;
            TransmitSpeedController(pMediaSample->GetTime());
            TransmitSteeringController(pMediaSample->GetTime());
        }
        if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_WARNING("cMoveToPoint::noutput Speed and steer values");

    //check if a new action is in buffer
    }else if(newActionCommand){
        InitNewCommand();
    }else{
        //do nothing

    }
#ifdef DEBUG
    debugToConsoleCounter++;
#endif
    RETURN_NOERROR;
}

tResult cMoveToPoint::InitNewCommand(){
    command = commandBuffer; //take most recent command as new Command
    newActionCommand = false;
    pathToGoalIndex = 0; // reset counter
    transmitZeros = false; // reset wait flag
    direction = 0; // reset direction
    firstCallLastRho = true; //reset last Rho
    isRunning = true;  //activate computation

    SetCarPoseOffset(GetLatestCarPose());
    if(!IS_OK(SetPath())){
        LOG_WARNING(cString::Format("cMoveToPoint::could not set new path set filter inactive"));
        isRunning = false;
        RETURN_ERROR(-1);
    }

     #ifdef DEBUG
       if (debugToConsole) {
           LOG_WARNING(cString::Format("cMoveToPoint::New ActionCommand initialised" ));
       }
     #endif
     RETURN_NOERROR;
}

tResult cMoveToPoint::LogTrajectoryToXML(){
    using namespace tinyxml2;

    ADTF_GET_CONFIG_FILENAME(loggedXMLDirectory);
    cFilename absFilename = loggedXMLDirectory.CreateAbsolutePath(".");

    if(lt_erase){
        if( remove(absFilename) != 0 ){
    #ifdef DEBUG
           if(debugToConsole){
                //LOG_INFO(cString::Format("MoveToPoint::Error deleting xml file in LogTrajectoryToXML()"));
           }
    #endif
        }
        lt_CarPoseOffset = GetLatestCarPose();
        xmlDoc.Clear();
        pRoot = xmlDoc.NewElement("Path");
        xmlDoc.InsertFirstChild(pRoot);
        RETURN_NOERROR;
    }

    if(firstCallLogTrajectory){
        pRoot = xmlDoc.NewElement("Path");
        xmlDoc.InsertFirstChild(pRoot);
        firstCallLogTrajectory = false;
        lt_CarPoseOffset = GetLatestCarPose();
        lt_counter = 0;
    }

    if(lt_counter % lt_devider == 0){

        XMLElement * pElementPose = xmlDoc.NewElement(cString::Format("Point"));

        // get car pose and car pose offset
        TPoseStruct::Data car_pose_tmp = GetLatestCarPose();
        TPoseStruct::Data car_pose_offset_tmp = lt_CarPoseOffset;

        // calculate car offset
        car_pose_tmp.f32_x -= car_pose_offset_tmp.f32_x;
        car_pose_tmp.f32_y -= car_pose_offset_tmp.f32_y;
        car_pose_tmp.f32_yaw -= car_pose_offset_tmp.f32_yaw;

        // car pose yaw: convert yaw angle to be between -180 and 180 degrees
        if(car_pose_offset_tmp.f32_yaw < -M_PI) {
                car_pose_offset_tmp.f32_yaw += 2*M_PI;
        } else if(car_pose_offset_tmp.f32_yaw > M_PI) {
                car_pose_offset_tmp.f32_yaw -= 2*M_PI;
        }

        // rotate coordinates for local pose
        tFloat32 y_sin = car_pose_tmp.f32_y * sin(car_pose_offset_tmp.f32_yaw);
        tFloat32 y_cos = car_pose_tmp.f32_y * cos(car_pose_offset_tmp.f32_yaw);
        tFloat32 x_cos = car_pose_tmp.f32_x * cos(car_pose_offset_tmp.f32_yaw);
        tFloat32 x_sin = -car_pose_tmp.f32_x * sin(car_pose_offset_tmp.f32_yaw);

        // car_pose_tmp contains now local pose starting at x 0, y 0, yaw 0
        car_pose_tmp.f32_x = y_sin + x_cos;
        car_pose_tmp.f32_y = x_sin + y_cos;


        pElementPose->SetAttribute("x", car_pose_tmp.f32_x);
        pElementPose->SetAttribute("y", car_pose_tmp.f32_y);
        pElementPose->SetAttribute("yaw", car_pose_tmp.f32_yaw);
        pElementPose->SetAttribute("speed", 0.5);
        pElementPose->SetAttribute("radius", 0);


        pRoot->InsertEndChild(pElementPose);
        LOG_WARNING(absFilename);
        XMLError eResult = xmlDoc.SaveFile(absFilename);
        if (eResult != XML_SUCCESS) {
            LOG_WARNING(cString::Format("cMoveToPoint::Error: %i\n", eResult));

        }
    }
    lt_counter++;

    RETURN_NOERROR;
}


tResult cMoveToPoint::MoveToPoint(){

    // get car pose and car pose offset
    TPoseStruct::Data car_pose_tmp = GetLatestCarPose();
    TPoseStruct::Data car_pose_offset_tmp = GetCarPoseOffset();
    TPoseStruct::Data car_pose_goal = GetGoalPose();

    // calculate car offset
    car_pose_tmp.f32_x -= car_pose_offset_tmp.f32_x;
    car_pose_tmp.f32_y -= car_pose_offset_tmp.f32_y;
    car_pose_tmp.f32_yaw -= car_pose_offset_tmp.f32_yaw;

    //limit to -180 <> 180 Deg
    if(car_pose_offset_tmp.f32_yaw < -M_PI) {
            car_pose_offset_tmp.f32_yaw += 2*M_PI;
    } else if(car_pose_offset_tmp.f32_yaw > M_PI) {
            car_pose_offset_tmp.f32_yaw -= 2*M_PI;
    }

    // rotate coordinates for local pose
    tFloat32 y_sin = car_pose_tmp.f32_y * sin(car_pose_offset_tmp.f32_yaw);
    tFloat32 y_cos = car_pose_tmp.f32_y * cos(car_pose_offset_tmp.f32_yaw);
    tFloat32 x_cos = car_pose_tmp.f32_x * cos(car_pose_offset_tmp.f32_yaw);
    tFloat32 x_sin = -car_pose_tmp.f32_x * sin(car_pose_offset_tmp.f32_yaw);
    tFloat32 temp_1 = y_sin + x_cos;
    tFloat32 temp_2 = x_sin + y_cos;

    // car_pose_tmp contains now local pose starting at x 0, y 0, yaw 0
    car_pose_tmp.f32_x = temp_1;
    car_pose_tmp.f32_y = temp_2;

    // calculate delta x and delta y (car pose - goal point)
    tFloat32 delta_x, delta_y;
    delta_x = car_pose_tmp.f32_x - car_pose_goal.f32_x;
    delta_y =  car_pose_tmp.f32_y - car_pose_goal.f32_y;

    // distance to goal
    tFloat32 rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    if(firstCallLastRho){
        mtp_lastRho = rho;
        firstCallLastRho = false;
    }
    //set lastRho only if distance decreased
    if(rho < mtp_lastRho) mtp_lastRho = rho;

    //calculate speed
    TSignalValue::Data car_speed;
    car_speed.f32_value = car_pose_goal.f32_car_speed;

    //if near next goal (range, point with speed 0.0)
    if(rho < goalDistanceShutOff || car_speed.f32_value == 0.0 || (rho - mtp_lastRho) > 0.05 ){
        counter_movetopoint = 0; //reset value so all next logs are printed
        if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_INFO("cMoveToPoint: stepped into rho < dt_DistanceToNextPathPoint || car_speed == 0 || rho > mtp_lastRho");
        //hit point return feedbackStruct for achieved goal
        if(rho < goalDistanceShutOff){
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = GetCommand()+1;
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
            SetRunningState(tFalse);
            LOG_WARNING("cMoveToPoint:: Hit Target, feedback sent, runningState = false also transmit some zero samples");
        }else if((rho - mtp_lastRho) > 0.05){
            LOG_WARNING(cString::Format("cMoveToPoint::increasing distance as rho:%f lastRho: %f diff %f", rho, mtp_lastRho, rho- mtp_lastRho));
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = GetCommand()+1;
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
            SetRunningState(tFalse);
            LOG_WARNING("cMoveToPoint:: Missed Target as distance to goal is increasing! feedback sent, runningState = false");
        }else if(car_speed.f32_value == 0.0 && rho > goalDistanceShutOff) {
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = GetCommand()+1;
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
            SetRunningState(tFalse);
            LOG_WARNING("cMoveToPoint: Missed Point as speed = 0.0 (and out of range)! feedback sent, runningState = false");
        }
        car_speed.f32_value = 0;
    }
    tFloat32 reduced_speed = car_speed.f32_value;

    #ifdef DEBUG
      if(counter_movetopoint%10 == 0 && debugToConsole == true){
          LOG_INFO(cString::Format("MoveToPoint::TransformedX: %f, TransformedY: %f, TransformedYaw %f", car_pose_tmp.f32_x, car_pose_tmp.f32_y, car_pose_tmp.f32_yaw * 180/M_PI));
          LOG_INFO(cString::Format("MoveToPoint::DesiredCarPoseX: %f, DesiredCarPoseY: %f, DesiredCarPoseYaw %f", car_pose_goal.f32_x, car_pose_goal.f32_y, car_pose_goal.f32_yaw * 180/M_PI));
          LOG_INFO(cString::Format("MoveToPoint::Rho: %f", rho));
          LOG_INFO(cString::Format("MoveToPoint::car_speed: %f, reduced_speed: %f",  car_speed.f32_value , reduced_speed));
      }
      if(counter_movetopoint%10 == 0 && debugToFile == true){
          fstream file_pose;
          file_pose.open( debugFileDirectory.CreateAbsolutePath("."), ios::out | ios::app);
          file_pose << cString::Format("MoveToPoint::TransformedX: %f, TransformedY: %f, TransformedYaw %f", car_pose_tmp.f32_x, car_pose_tmp.f32_y, car_pose_tmp.f32_yaw * 180/M_PI);
          file_pose << cString::Format("MoveToPoint::DesiredCarPoseX: %f, DesiredCarPoseY: %f, DesiredCarPoseYaw %f", car_pose_goal.f32_x, car_pose_goal.f32_y, car_pose_goal.f32_yaw * 180/M_PI);
          file_pose << cString::Format("MoveToPoint::Rho: %f", rho);
          file_pose << cString::Format("MoveToPoint::car_speed: %f, reduced_speed: %f",  car_speed.f32_value , reduced_speed);
          file_pose.close();
      }
    #endif
    //when near final point speed has to go down, to avoid never reaching the final point as to low speed (P-controller) an reduction_offsett can be added
    if(GetCommand() == AC_FP_GOTO_XY){
        if(car_speed.f32_value > 0) car_speed.f32_value = krho_forw * rho * car_speed.f32_value + reduction_offset;
        if(car_speed.f32_value < 0) car_speed.f32_value = krho_back * rho * car_speed.f32_value - reduction_offset;
    }else if(GetCommand() == AC_FP_GOTO_XY_NOSTOP){
        //do nothing as car should not slow done when reaching point
    }
    // calculation based on 'polar_sfunc.m' (p. corke robotics toolbox)
    tFloat32 alpha, beta, gamma, beta_;
    if(direction == 0) { // first run
        beta = -atan2(-delta_y, -delta_x);
        alpha = -car_pose_tmp.f32_yaw - beta;

        // first run: direction of travel
        if(alpha > M_PI_2 || alpha < -M_PI_2) {
            direction = -1;
        }else{
              direction = 1;
        }
    } else if(direction == -1) { // backwards
        beta = -atan2(delta_y, delta_x);
        alpha = -car_pose_tmp.f32_yaw - beta;
    } else{ // forwards
        beta = -atan2(-delta_y, -delta_x);
        alpha = -car_pose_tmp.f32_yaw - beta;
    }

    if(alpha > M_PI_2){
        alpha = M_PI_2;
    }
    if(alpha < -M_PI_2){
        alpha = -M_PI_2;
    }

    beta_ = beta + car_pose_goal.f32_yaw;

    gamma = direction * (-kbeta * beta_ + kalpha * alpha); // gamma
    nextSteeringControllerOut.f32_value = gamma * 180/M_PI;


    //limit steering to +-100 degrees
    nextSteeringControllerOut.f32_value = - nextSteeringControllerOut.f32_value;
    if(nextSteeringControllerOut.f32_value > 100){
       nextSteeringControllerOut.f32_value = 100;
    }else if(nextSteeringControllerOut.f32_value < -100){
        nextSteeringControllerOut.f32_value = -100;
    }

    nextSpeedControllerOut.f32_value = car_speed.f32_value * direction;
    if(nextSpeedControllerOut.f32_value > car_pose_goal.f32_car_speed){
       nextSpeedControllerOut.f32_value = car_pose_goal.f32_car_speed;
    }
    if(nextSpeedControllerOut.f32_value < -car_pose_goal.f32_car_speed){
       nextSpeedControllerOut.f32_value = -car_pose_goal.f32_car_speed;
    }

    //increase speed if car is steering
    nextSpeedControllerOut.f32_value *= (1.0 + (0.2  /*[m/s]*/  * fabs(nextSteeringControllerOut.f32_value) /100.0));

    // log MoveToPoint to file
    #ifdef DEBUG
    if ( counter_movetopoint % 10 == 0) { // 1 out of 5
        if(debugToFile){
            fstream file_pose;
            file_pose.open( debugFileDirectory.CreateAbsolutePath("."), ios::out | ios::app);
            file_pose <<"steering_output: " << nextSteeringControllerOut.f32_value << "  speed_out: " << nextSpeedControllerOut.f32_value << "\n\n";
            file_pose.close();
        }
        if(debugToConsole){
            LOG_INFO(cString::Format("MoveToPoint::SpeedControllerOut: %f, SteeringControllerOut: %f", nextSpeedControllerOut.f32_value, nextSteeringControllerOut.f32_value));
            LOG_INFO(cString::Format(""));
        }
    }
    counter_movetopoint++;
    #endif

    //if running state changed while function is still active transmit 0s as next time there wont be any values committed
    if(!GetRunningState()){
        nextSteeringControllerOut.f32_value = 0;
        nextSpeedControllerOut.f32_value = 0;
    }

    RETURN_NOERROR;
}

tResult cMoveToPoint::MoveWithoutSteer(){

    // get car pose and car pose offset
    TPoseStruct::Data car_pose_tmp = GetLatestCarPose();
    TPoseStruct::Data car_pose_offset_tmp = GetCarPoseOffset();
    TPoseStruct::Data car_pose_goal = GetGoalPose();
    tFloat32 deltaX, deltaY, distanceToDrive, distanceSinceStart, rho;
    deltaX = car_pose_tmp.f32_x - car_pose_offset_tmp.f32_x;
    deltaY = car_pose_tmp.f32_y - car_pose_offset_tmp.f32_y;
    distanceSinceStart = sqrt(pow(deltaX,2) + pow(deltaY,2));
    distanceToDrive = sqrt(pow(car_pose_goal.f32_x, 2) + pow(car_pose_goal.f32_y,2));
    rho = distanceToDrive - distanceSinceStart;

    if(firstCallLastRho){
        mtp_lastRho = rho;
        firstCallLastRho = false;
    }
    //set lastRho only if distance decreased
    if(rho < mtp_lastRho) mtp_lastRho = rho;

    //calculate speed
    TSignalValue::Data car_speed;
    if(car_pose_goal.f32_x > 0){
        car_speed.f32_value = car_pose_goal.f32_car_speed;
    }else{
        car_speed.f32_value = - car_pose_goal.f32_car_speed;
    }


    //if near next goal (range, point with speed 0.0)
    if(rho < goalDistanceShutOff || car_speed.f32_value == 0.0 || (rho - mtp_lastRho) > 0.05 ){
        counter_movetopoint = 0; //reset value so all next logs are printed
        if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_INFO("cMoveToPoint: stepped into rho < dt_DistanceToNextPathPoint || car_speed == 0 || rho > mtp_lastRho");
        //hit point return feedbackStruct for achieved goal
        if(rho < goalDistanceShutOff){
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = GetCommand()+1;
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
            SetRunningState(tFalse);
            LOG_WARNING("cMoveToPoint:: PENIS Hit Target, feedback sent, runningState = false also transmit some zero samples");
        }else if((rho - mtp_lastRho) > 0.05){
            LOG_WARNING(cString::Format("cMoveToPoint::increasing distance as rho:%f lastRho: %f diff %f", rho, mtp_lastRho, rho- mtp_lastRho));
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = GetCommand()+1;
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
            SetRunningState(tFalse);
            LOG_WARNING("cMoveToPoint:: Missed Target as distance to goal is increasing! feedback sent, runningState = false");
        }else if(car_speed.f32_value == 0.0 && rho > goalDistanceShutOff) {
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = GetCommand()+1;
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
            SetRunningState(tFalse);
            LOG_WARNING("cMoveToPoint: Missed Point as speed = 0.0 (and out of range)! feedback sent, runningState = false");
        }
        car_speed.f32_value = 0;
    }
    tFloat32 reduced_speed = car_speed.f32_value;

    #ifdef DEBUG
      if(counter_movetopoint%10 == 0 && debugToConsole == true){
          LOG_INFO(cString::Format("MoveToPoint::Rho: %f", rho));
          LOG_INFO(cString::Format("MoveToPoint::car_speed: %f, reduced_speed: %f",  car_speed.f32_value , reduced_speed));
      }
      if(counter_movetopoint%10 == 0 && debugToFile == true){
          fstream file_pose;
          file_pose.open( debugFileDirectory.CreateAbsolutePath("."), ios::out | ios::app);
          file_pose << cString::Format("MoveToPoint::Rho: %f", rho);
          file_pose << cString::Format("MoveToPoint::car_speed: %f, reduced_speed: %f",  car_speed.f32_value , reduced_speed);
          file_pose.close();
      }
    #endif
    //when near final point speed has to go down, to avoid never reaching the final point as to low speed (P-controller) an reduction_offsett can be added
    if(GetCommand() == AC_FP_GOTO_XY_NOSTEERING){
        if(car_speed.f32_value > 0) reduced_speed = krho_forw * rho * car_speed.f32_value + reduction_offset;
        if(car_speed.f32_value < 0) reduced_speed = krho_back * rho * car_speed.f32_value - reduction_offset;
        if ( counter_movetopoint % 10 == 0) LOG_INFO(cString::Format("MoveToPoint::AC_FP_GOTO_XY_NOSTEERING"));

    }else if(GetCommand() == AC_FP_GOTO_XY_NOSTEERING_NOSTOP){
       if ( counter_movetopoint % 10 == 0) LOG_INFO(cString::Format("MoveToPoint::AC_FP_GOTO_XY_NOSTEERING_NOSTOP"));
        //do nothing as car should not slow done when reaching point
    }
    if ( counter_movetopoint % 10 == 0) LOG_INFO(cString::Format("MoveToPoint::SpeedControllerOut after reduced speed: %f", reduced_speed ));

    nextSpeedControllerOut.f32_value = reduced_speed;
    if(nextSpeedControllerOut.f32_value > car_speed.f32_value){
       nextSpeedControllerOut.f32_value = car_speed.f32_value;
    }
    if(nextSpeedControllerOut.f32_value < -car_speed.f32_value){
       nextSpeedControllerOut.f32_value = -car_speed.f32_value;
    }

    if ( counter_movetopoint % 10 == 0) LOG_INFO(cString::Format("MoveToPoint::SpeedControllerOut after direction: %f", nextSpeedControllerOut.f32_value));

    // log MoveToPoint to file
    #ifdef DEBUG
    if ( counter_movetopoint % 10 == 0) { // 1 out of 5
        if(debugToFile){
            fstream file_pose;
            file_pose.open( debugFileDirectory.CreateAbsolutePath("."), ios::out | ios::app);
            file_pose << "speed_out: " << nextSpeedControllerOut.f32_value << "\n\n";
            file_pose.close();
        }
        if(debugToConsole){
            LOG_INFO(cString::Format("MoveToPoint::SpeedControllerOut: %f", nextSpeedControllerOut.f32_value));
            LOG_INFO(cString::Format(""));
        }
    }

    counter_movetopoint++;
    #endif

    //if running state changed while function is still active transmit 0s as next time there wont be any values committed
    if(!GetRunningState()){
       // nextSpeedControllerOut.f32_value = 0;
    }


    RETURN_NOERROR;
}



tResult cMoveToPoint::DriveTrajectory(){

    // get car pose and car pose offset
    TPoseStruct::Data car_pose_tmp = GetLatestCarPose();
    TPoseStruct::Data car_pose_offset_tmp = GetCarPoseOffset();

    // calculate car offset
    car_pose_tmp.f32_x -= car_pose_offset_tmp.f32_x;
    car_pose_tmp.f32_y -= car_pose_offset_tmp.f32_y;
    car_pose_tmp.f32_yaw -= car_pose_offset_tmp.f32_yaw;

    if(car_pose_offset_tmp.f32_yaw < -M_PI) {
            car_pose_offset_tmp.f32_yaw += 2*M_PI;
    } else if(car_pose_offset_tmp.f32_yaw > M_PI) {
            car_pose_offset_tmp.f32_yaw -= 2*M_PI;
    }

    // rotate coordinates for local pose
    tFloat32 y_sin = car_pose_tmp.f32_y * sin(car_pose_offset_tmp.f32_yaw);
    tFloat32 y_cos = car_pose_tmp.f32_y * cos(car_pose_offset_tmp.f32_yaw);
    tFloat32 x_cos = car_pose_tmp.f32_x * cos(car_pose_offset_tmp.f32_yaw);
    tFloat32 x_sin = -car_pose_tmp.f32_x * sin(car_pose_offset_tmp.f32_yaw);
    tFloat32 temp_1 = y_sin + x_cos;
    tFloat32 temp_2 = x_sin + y_cos;

    // car_pose_tmp contains now local pose starting at x 0, y 0, yaw 0
    car_pose_tmp.f32_x = temp_1;
    car_pose_tmp.f32_y = temp_2;

    // calculate delta x and delta y (car pose - goal point)
    tFloat32 delta_x;
    delta_x = car_pose_tmp.f32_x - path_to_goal[pathToGoalIndex].x;
    tFloat32 delta_y;
    delta_y =  car_pose_tmp.f32_y - path_to_goal[pathToGoalIndex].y;

    // distance to goal
    tFloat32 rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    if(firstCallLastRho || direction == 0){
        mtp_lastRho = rho;
        firstCallLastRho = false;
    }
    //set lastRho only if distance decreased
    if(rho < mtp_lastRho) mtp_lastRho = rho;

    //calculate speed
    TSignalValue::Data car_speed;
    car_speed.f32_value = path_to_goal[pathToGoalIndex].car_speed;

    //if near next goal (range, point with speed 0.0)
    if(rho < goalDistanceShutOff || car_speed.f32_value == 0.0 || (rho - mtp_lastRho) > 0.1){
        counter_movetopoint = 0; //reset value so all next logs are printed
        if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_INFO("cMoveToPoint: stepped into rho < dt_DistanceToNextPathPoint || car_speed == 0 || rho > mtp_lastRho");

        //hit point return feedbackStruct for achieved goal
        if(rho < goalDistanceShutOff && pathToGoalIndex >= path_to_goal.size()-1){
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = GetCommand()+1;
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
            SetRunningState(tFalse);
            LOG_WARNING("cMoveToPoint:: Hit Target and no more points in path, feedback sent, runningState = false");
        }else if((rho - mtp_lastRho) > 0.1){
            LOG_WARNING(cString::Format("cMoveToPoint::increasing distance as rho:%f lastRho: %f diff %f", rho, mtp_lastRho, rho- mtp_lastRho));
            if(pathToGoalIndex < path_to_goal.size() -1){
                LOG_WARNING("cMoveToPoint:: Missed point in path, load next point");
            }else{
                TFeedbackStruct::Data feedback;
                feedback.ui32_filterID = F_FOLLOW_PATH;
                feedback.ui32_status = GetCommand()+1;
                feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
                SetRunningState(tFalse);
                LOG_WARNING("cMoveToPoint:: Missed Target as distance to goal is increasing! feedback sent, runningState = false");
            }
        }else if(car_speed.f32_value == 0.0 && rho > goalDistanceShutOff) {
            TFeedbackStruct::Data feedback;
            feedback.ui32_filterID = F_FOLLOW_PATH;
            feedback.ui32_status = GetCommand()+1;
            feedbackStructOut.Transmit(&feedback_struct_output, feedback, _clock->GetStreamTime());
            SetRunningState(tFalse);
            LOG_WARNING("cMoveToPoint: Missed Point as speed = 0.0 (and out of range)! feedback sent, runningState = false");
        }

        //set next goal pose if still goals in vector
        if(pathToGoalIndex < path_to_goal.size() -1){
            if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_WARNING("cMoveToPoint:: adding the next point in the path");
            //if direction changes or car has to stop send a few 0 samples to prevent any damage to drive train of car
            // next point with new direction: set flag and wait a few samples before transmitting new car speed

            tFloat32 alpha, beta;
            tInt32 direction_tmp;
            // => new delta x and delta y (car pose - goal point)
            delta_x = car_pose_tmp.f32_x - path_to_goal[pathToGoalIndex +1].x;
            delta_y = car_pose_tmp.f32_y - path_to_goal[pathToGoalIndex +1].y;
            beta = -atan2(-delta_y, -delta_x);
            alpha = -car_pose_tmp.f32_yaw - beta;
            if(alpha > M_PI_2 || alpha < -M_PI_2) {
                direction_tmp = -1;
            }else{
                direction_tmp = 1;
            }

            if (path_to_goal[pathToGoalIndex+1].car_speed == 0.0 || direction_tmp != direction) {
                    transmitZeros = tTrue;
                    transmitZerosCounter = 0;
                    // debug message
                    if(debugToConsole == true) LOG_WARNING(cString::Format("cMoveToPoint:: change in car direction or car speed 0 at next path point, waiting few samples"));

            }

            // increase path point counter
            pathToGoalIndex++;
            direction = 0; // reset direction

            // => new distance to goal
            rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
            mtp_lastRho = rho;
            // => new car speed from xml
            car_speed.f32_value = path_to_goal[pathToGoalIndex].car_speed;

        }else{
            if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_WARNING("cMoveToPoint:: no more points in path!");
            car_speed.f32_value = 0.0;
            isRunning = false;
        }
    }
    tFloat32 reduced_speed = car_speed.f32_value;

    #ifdef DEBUG
      if(counter_movetopoint%10 == 0 && debugToConsole == true){
          LOG_INFO(cString::Format("MoveToPoint::TransformedX: %f, TransformedY: %f, TransformedYaw %f", car_pose_tmp.f32_x, car_pose_tmp.f32_y, car_pose_tmp.f32_yaw * 180/M_PI));
          LOG_INFO(cString::Format("MoveToPoint::DesiredCarPoseX: %f, DesiredCarPoseY: %f, DesiredCarPoseYaw %f", path_to_goal[pathToGoalIndex].x, path_to_goal[pathToGoalIndex].y, path_to_goal[pathToGoalIndex].yaw * 180/M_PI));
          LOG_INFO(cString::Format("MoveToPoint::Rho: %f", rho));
      }
      if(counter_movetopoint%10 == 0 && debugToFile == true){
          fstream file_pose;
          file_pose.open( debugFileDirectory.CreateAbsolutePath("."), ios::out | ios::app);
          file_pose << cString::Format("MoveToPoint::DesiredCarPoseX: %f, DesiredCarPoseY: %f, DesiredCarPoseYaw %f", path_to_goal[pathToGoalIndex].x, path_to_goal[pathToGoalIndex].y, path_to_goal[pathToGoalIndex].yaw * 180/M_PI);
          file_pose << cString::Format("MoveToPoint::TransformedX: %f, TransformedY: %f, TransformedYaw %f", car_pose_tmp.f32_x, car_pose_tmp.f32_y, car_pose_tmp.f32_yaw * 180/M_PI);
          file_pose << cString::Format("MoveToPoint::Delta_x: %f, Delta_y: %f, Rho: %f", delta_x, delta_y, rho);
          file_pose.close();
      }
    #endif

    if(pathToGoalIndex == path_to_goal.size() -1){
        if(car_speed.f32_value > 0) reduced_speed = krho_forw * rho * reduced_speed + reduction_offset;
        if(car_speed.f32_value < 0) reduced_speed = krho_back * rho * reduced_speed - reduction_offset;

    // additional points are in the trajectory but a direction change / break (speed =0) happens
    } else if( pathToGoalIndex +1 <  path_to_goal.size()) {
        if(path_to_goal[pathToGoalIndex+1].car_speed == 0.0 ||  ( path_to_goal[pathToGoalIndex].x > 0 && (path_to_goal[pathToGoalIndex+1].x <  path_to_goal[pathToGoalIndex].x )) || ( path_to_goal[pathToGoalIndex].x  < 0 && (path_to_goal[pathToGoalIndex+1].x > path_to_goal[pathToGoalIndex].x )) ) {
            if(car_speed.f32_value > 0) reduced_speed = krho_forw * rho * reduced_speed + reduction_offset;
            if(car_speed.f32_value < 0) reduced_speed = krho_back * rho * reduced_speed - reduction_offset;

        }
    }
    car_speed.f32_value = reduced_speed;

    // calculation based on 'polar_sfunc.m' (p. corke robotics toolbox)
    tFloat32 alpha, beta, gamma, beta_;
    if(direction == 0) { // first run
        beta = -atan2(-delta_y, -delta_x);
        alpha = -car_pose_tmp.f32_yaw - beta;

        // first run: direction of travel
        if(alpha > M_PI_2 || alpha < -M_PI_2) {
            direction = -1;
        }else{
              direction = 1;
        }
    } else if(direction == -1) { // backwards
        beta = -atan2(delta_y, delta_x);
        alpha = -car_pose_tmp.f32_yaw - beta;
    } else{ // forwards
        beta = -atan2(-delta_y, -delta_x);
        alpha = -car_pose_tmp.f32_yaw - beta;
    }

    if(alpha > M_PI_2){
        alpha = M_PI_2;
    }
    if(alpha < -M_PI_2){
        alpha = -M_PI_2;
    }


    beta_ = beta + path_to_goal[pathToGoalIndex].yaw;

    gamma = direction * (-kbeta * beta_ + kalpha * alpha); // gamma
    nextSteeringControllerOut.f32_value = gamma * 180/M_PI;


    //limit steering to +-80 degrees
    nextSteeringControllerOut.f32_value = - nextSteeringControllerOut.f32_value;
    if(nextSteeringControllerOut.f32_value > 100){
       nextSteeringControllerOut.f32_value = 100;
    }else if(nextSteeringControllerOut.f32_value < -100){
        nextSteeringControllerOut.f32_value = -100;
    }

    nextSpeedControllerOut.f32_value = car_speed.f32_value * direction;
    if(nextSpeedControllerOut.f32_value > path_to_goal[pathToGoalIndex].car_speed){
       nextSpeedControllerOut.f32_value = path_to_goal[pathToGoalIndex].car_speed;
    }
    if(nextSpeedControllerOut.f32_value < -path_to_goal[pathToGoalIndex].car_speed){
       nextSpeedControllerOut.f32_value = -path_to_goal[pathToGoalIndex].car_speed;
    }

    //increase speed if car is steering
    nextSpeedControllerOut.f32_value *= (1.0 + (0.5  /*[m/s]*/  * fabs(nextSteeringControllerOut.f32_value) /100.0));


    // log MoveToPoint to file
    #ifdef DEBUG
      if ( counter_movetopoint % 10 == 0) { // 1 out of 5
          if(debugToFile){
              fstream file_pose;
              file_pose.open( debugFileDirectory.CreateAbsolutePath("."), ios::out | ios::app);
              file_pose <<"distance to goal: " << rho << "  beta: "<< beta << "  alpha: "<< alpha << "  direction: " << direction <<"\n";
              file_pose <<"steering_output: " << nextSteeringControllerOut.f32_value << "  speed_out: " << nextSpeedControllerOut.f32_value << "\n\n";
              file_pose.close();
          }
          if(debugToConsole){
              LOG_INFO(cString::Format("MoveToPoint::SpeedControllerOut: %f, SteeringControllerOut: %f", nextSpeedControllerOut.f32_value, nextSteeringControllerOut.f32_value));
              LOG_INFO(cString::Format(""));

          }
      }
      counter_movetopoint++;
    #endif

      //if running state changed while function is still active transmit 0s as next time there wont be any values committed
    if(!GetRunningState()){
        nextSteeringControllerOut.f32_value = 0;
        nextSpeedControllerOut.f32_value = 0;
    }
    RETURN_NOERROR;
}



tResult cMoveToPoint::CreateSmoothTrajectory(tUInt32 path){
    std::vector<fp_paths>::iterator it = xml_paths.begin();
    std::vector<fp_points> points;
    // check if path for command id exists
    while(it != xml_paths.end()) {
        if(it->path_id == path) {
            break;
        }
        it++;
    }
    //add path to path_to_goal variable if path was found
    if (it != xml_paths.end() && it->points.size() > 0) {
        points.resize(it->points.size());
        points = it->points;
    }else{
        RETURN_ERROR(-1);
    }


    smooth_path_to_goal.resize(4 * points.size());
    for(tUInt i = 1; i < smooth_path_to_goal.size() +1; i++){
        smooth_path_to_goal[i-1] = ComputeCurve(points, ((tFloat32)i)/smooth_path_to_goal.size());
    }
    RETURN_NOERROR;
}

cMoveToPoint::fp_points cMoveToPoint::ComputeCurve(std::vector<fp_points> & points, tFloat32 t){
    if(points.size() == 1){
        return points[0];
    }else{
        std::vector<fp_points> newPoints;
        newPoints.resize(points.size() -1);
        for(tUInt i = 0; i < points.size()-1; i++){
            fp_points tmp;
            tmp.x = points[i].x *(1.0-t) + points[i+1].x *t;
            tmp.y = points[i].y *(1.0-t) + points[i+1].y *t;
            newPoints[i] = tmp;
        }
        return ComputeCurve(newPoints, t);
    }
}

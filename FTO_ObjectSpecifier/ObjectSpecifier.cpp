#include "ObjectSpecifier.h"

#include "ScmCommunication.h"

// properties
#define DEBUG_IMG "Debug image"
#define DEBUG_IMG_CREATE "Create debug image"
#define DEBUG_IMG_TEXT_COLOR "Classification text color"
#define DEBUG_IMG_TEXT_FONTSCALE "Font scale"
#define DEBUG_IMG_TEXT_THICKNESS "Font thickness"
#define DEBUG_IMG_TEXT_LINE_HEIGHT "Line Height"

#define DETECTION "Detection parameters"
#define CNN_DETECTION_MODE "Detection mode"
#define DETECTION_THRESHOLD "Detection thresholds"
#define DETECTION_ADULT "Adult"
#define DETECTION_CHILD "Child"
#define DETECTION_CAR "Car"

#define DISTANCE_TO_DRIVE "Distance to drive past detection"

#define ROI_CROSSING "ROIs at crossings"
#define ROI_NORMAL_DRIVE "ROIs for normal drive operation"
#define ROI_ZEBRA_CROSSING "ROIs at zebra crossings"

#define THRIFT "Thrift (extern python server)"
#define THRIFT_IP "IP address"
#define THRIFT_PORT "Port"

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using boost::shared_ptr;

using namespace ::ext_iface;

/// Create filter shell
ADTF_FILTER_PLUGIN("FTO_ObjectSpecifier", OID_ADTF_OBJECT_SPECIFIER, ObjectSpecifier);

// constructor & destructor
ObjectSpecifier::ObjectSpecifier(const tChar* __info) : cFilter(__info)
{
    /* debug image properties */
    bool defaultCreateDebugImage = true;
    SetPropertyBool(DEBUG_IMG "::" DEBUG_IMG_CREATE, defaultCreateDebugImage);
    SetPropertyStr(DEBUG_IMG "::" DEBUG_IMG_CREATE NSSUBPROP_DESCRIPTION, "Create a debug image where the areas the filter searches for objects is shown.");

    SetPropertyStr(DEBUG_IMG "::" DEBUG_IMG_TEXT_COLOR, "Black");
    SetPropertyStr(DEBUG_IMG "::" DEBUG_IMG_TEXT_COLOR NSSUBPROP_DESCRIPTION, "Defines the color in which the classification(s) are printed on the debug image.");
    SetPropertyStr(DEBUG_IMG "::" DEBUG_IMG_TEXT_COLOR NSSUBPROP_VALUELIST, "Black@Black|White@White");
    SetPropertyBool(DEBUG_IMG "::" DEBUG_IMG_TEXT_COLOR NSSUBPROP_ISCHANGEABLE, tTrue);

    double defaultClassificationTextFontScale = 0.8;
    SetPropertyFloat(DEBUG_IMG "::" DEBUG_IMG_TEXT_FONTSCALE, defaultClassificationTextFontScale);
    SetPropertyStr(DEBUG_IMG "::" DEBUG_IMG_TEXT_FONTSCALE NSSUBPROP_DESCRIPTION, "Scale value for font and line height.");
    SetPropertyBool(DEBUG_IMG "::" DEBUG_IMG_TEXT_FONTSCALE NSSUBPROP_ISCHANGEABLE, tTrue);

    int defaultClassificationTextFontThickness = 2;
    SetPropertyFloat(DEBUG_IMG "::" DEBUG_IMG_TEXT_THICKNESS, defaultClassificationTextFontThickness);
    SetPropertyStr(DEBUG_IMG "::" DEBUG_IMG_TEXT_THICKNESS NSSUBPROP_DESCRIPTION, "Thickness of font.");
    SetPropertyBool(DEBUG_IMG "::" DEBUG_IMG_TEXT_THICKNESS NSSUBPROP_ISCHANGEABLE, tTrue);

    int defaultClassificationTextLineHeight = 30;
    SetPropertyInt(DEBUG_IMG "::" DEBUG_IMG_TEXT_LINE_HEIGHT, defaultClassificationTextLineHeight);
    SetPropertyStr(DEBUG_IMG "::" DEBUG_IMG_TEXT_LINE_HEIGHT NSSUBPROP_DESCRIPTION, "Line height for labels.");
    SetPropertyBool(DEBUG_IMG "::" DEBUG_IMG_TEXT_LINE_HEIGHT NSSUBPROP_ISCHANGEABLE, tTrue);


    /* detection mode properties */
    SetPropertyStr(DETECTION "::" CNN_DETECTION_MODE, "Normal");
    SetPropertyStr(DETECTION "::" CNN_DETECTION_MODE NSSUBPROP_DESCRIPTION, "Defines the ROIs where objects can be detected.");
    SetPropertyStr(DETECTION "::" CNN_DETECTION_MODE NSSUBPROP_VALUELIST, "Normal drive@Normal drive|Crossing@Crossing|Zebra crossing@Zebra crossing|None@None");
    SetPropertyBool(DETECTION "::" CNN_DETECTION_MODE NSSUBPROP_ISCHANGEABLE, tTrue);

    // detection thresholds
    SetPropertyFloat(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_ADULT, 0.8);
    SetPropertyStr(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_ADULT NSSUBPROP_DESCRIPTION, "Threshold above which an object is seen as an 'adult'");
    SetPropertyBool(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_ADULT NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_CHILD, 0.8);
    SetPropertyStr(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_CHILD NSSUBPROP_DESCRIPTION, "Threshold above which an object is seen as a 'child'");
    SetPropertyBool(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_CHILD NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyFloat(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_CAR, 0.8);
    SetPropertyStr(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_CAR NSSUBPROP_DESCRIPTION, "Threshold above which an object is seen as a 'car'");
    SetPropertyBool(DETECTION "::" DETECTION_THRESHOLD "::" DETECTION_CAR NSSUBPROP_ISCHANGEABLE, tTrue);

    /* crossing: pedestrians */
    // near right
    SetPropertyInt(ROI_CROSSING "::Pedestrians::nearRight::x", 960);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::nearRight::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI on the near right");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::nearRight::x" NSSUBPROP_ISCHANGEABLE, tTrue);
    
    SetPropertyInt(ROI_CROSSING "::Pedestrians::nearRight::y", 490);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::nearRight::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI on the near right");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::nearRight::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Pedestrians::nearRight::width", 300);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::nearRight::width" NSSUBPROP_DESCRIPTION, "width of ROI on the near right");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::nearRight::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Pedestrians::nearRight::height", 300);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::nearRight::height" NSSUBPROP_DESCRIPTION, "height of ROI on the near right");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::nearRight::height" NSSUBPROP_ISCHANGEABLE, tTrue);

    // far right
    SetPropertyInt(ROI_CROSSING "::Pedestrians::farRight::x", 675);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::farRight::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI on the far right");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::farRight::x" NSSUBPROP_ISCHANGEABLE, tTrue);
    
    SetPropertyInt(ROI_CROSSING "::Pedestrians::farRight::y", 490);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::farRight::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI on the far right");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::farRight::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Pedestrians::farRight::width", 100);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::farRight::width" NSSUBPROP_DESCRIPTION, "width of ROI on the far right");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::farRight::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Pedestrians::farRight::height", 100);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::farRight::height" NSSUBPROP_DESCRIPTION, "height of ROI on the far right");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::farRight::height" NSSUBPROP_ISCHANGEABLE, tTrue);

    // far left
    SetPropertyInt(ROI_CROSSING "::Pedestrians::farLeft::x", 390);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::farLeft::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI on the far left");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::farLeft::x" NSSUBPROP_ISCHANGEABLE, tTrue);
    
    SetPropertyInt(ROI_CROSSING "::Pedestrians::farLeft::y", 495);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::farLeft::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI on the far left");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::farLeft::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Pedestrians::farLeft::width", 100);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::farLeft::width" NSSUBPROP_DESCRIPTION, "width of ROI on the far left");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::farLeft::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Pedestrians::farLeft::height", 100);
    SetPropertyStr(ROI_CROSSING "::Pedestrians::farLeft::height" NSSUBPROP_DESCRIPTION, "height of ROI on the far left");
    SetPropertyBool(ROI_CROSSING "::Pedestrians::farLeft::height" NSSUBPROP_ISCHANGEABLE, tTrue);


    /* crossing: cars */
    // right
    SetPropertyInt(ROI_CROSSING "::Cars::right::x", 715);
    SetPropertyStr(ROI_CROSSING "::Cars::right::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI on the right");
    SetPropertyBool(ROI_CROSSING "::Cars::right::x" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::right::y", 520);
    SetPropertyStr(ROI_CROSSING "::Cars::right::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI on the right");
    SetPropertyBool(ROI_CROSSING "::Cars::right::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::right::width", 200);
    SetPropertyStr(ROI_CROSSING "::Cars::right::width" NSSUBPROP_DESCRIPTION, "width of ROI on the right");
    SetPropertyBool(ROI_CROSSING "::Cars::right::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::right::height", 100);
    SetPropertyStr(ROI_CROSSING "::Cars::right::height" NSSUBPROP_DESCRIPTION, "height of ROI on the right");
    SetPropertyBool(ROI_CROSSING "::Cars::right::height" NSSUBPROP_ISCHANGEABLE, tTrue);

    // straight ahead
    SetPropertyInt(ROI_CROSSING "::Cars::straight::x", 490);
    SetPropertyStr(ROI_CROSSING "::Cars::straight::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI straight ahead");
    SetPropertyBool(ROI_CROSSING "::Cars::straight::x" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::straight::y", 495);
    SetPropertyStr(ROI_CROSSING "::Cars::straight::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI straight ahead");
    SetPropertyBool(ROI_CROSSING "::Cars::straight::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::straight::width", 100);
    SetPropertyStr(ROI_CROSSING "::Cars::straight::width" NSSUBPROP_DESCRIPTION, "width of ROI straight ahead");
    SetPropertyBool(ROI_CROSSING "::Cars::straight::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::straight::height", 100);
    SetPropertyStr(ROI_CROSSING "::Cars::straight::height" NSSUBPROP_DESCRIPTION, "height of ROI straight ahead");
    SetPropertyBool(ROI_CROSSING "::Cars::straight::height" NSSUBPROP_ISCHANGEABLE, tTrue);

    // left
    SetPropertyInt(ROI_CROSSING "::Cars::left::x", 0);
    SetPropertyStr(ROI_CROSSING "::Cars::left::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI on the left");
    SetPropertyBool(ROI_CROSSING "::Cars::left::x" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::left::y", 560);
    SetPropertyStr(ROI_CROSSING "::Cars::left::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI on the left");
    SetPropertyBool(ROI_CROSSING "::Cars::left::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::left::width", 280);
    SetPropertyStr(ROI_CROSSING "::Cars::left::width" NSSUBPROP_DESCRIPTION, "width of ROI on the left");
    SetPropertyBool(ROI_CROSSING "::Cars::left::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_CROSSING "::Cars::left::height", 200);
    SetPropertyStr(ROI_CROSSING "::Cars::left::height" NSSUBPROP_DESCRIPTION, "height of ROI on the left");
    SetPropertyBool(ROI_CROSSING "::Cars::left::height" NSSUBPROP_ISCHANGEABLE, tTrue);


    /* normal: pedestrians (foremost children) */
    // right
    SetPropertyInt(ROI_NORMAL_DRIVE "::right::x", 720);
    SetPropertyStr(ROI_NORMAL_DRIVE "::right::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI on the right");
    SetPropertyBool(ROI_NORMAL_DRIVE "::right::x" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_NORMAL_DRIVE "::right::y", 550);
    SetPropertyStr(ROI_NORMAL_DRIVE "::right::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI on the right");
    SetPropertyBool(ROI_NORMAL_DRIVE "::right::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_NORMAL_DRIVE "::right::width", 100);
    SetPropertyStr(ROI_NORMAL_DRIVE "::right::width" NSSUBPROP_DESCRIPTION, "width of ROI on the right");
    SetPropertyBool(ROI_NORMAL_DRIVE "::right::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_NORMAL_DRIVE "::right::height", 100);
    SetPropertyStr(ROI_NORMAL_DRIVE "::right::height" NSSUBPROP_DESCRIPTION, "height of ROI on the right");
    SetPropertyBool(ROI_NORMAL_DRIVE "::right::height" NSSUBPROP_ISCHANGEABLE, tTrue);

    // left
    SetPropertyInt(ROI_NORMAL_DRIVE "::left::x", 220);
    SetPropertyStr(ROI_NORMAL_DRIVE "::left::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI on the left");
    SetPropertyBool(ROI_NORMAL_DRIVE "::left::x" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_NORMAL_DRIVE "::left::y", 550);
    SetPropertyStr(ROI_NORMAL_DRIVE "::left::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI on the left");
    SetPropertyBool(ROI_NORMAL_DRIVE "::left::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_NORMAL_DRIVE "::left::width", 100);
    SetPropertyStr(ROI_NORMAL_DRIVE "::left::width" NSSUBPROP_DESCRIPTION, "width of ROI on the left");
    SetPropertyBool(ROI_NORMAL_DRIVE "::left::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_NORMAL_DRIVE "::left::height", 100);
    SetPropertyStr(ROI_NORMAL_DRIVE "::left::height" NSSUBPROP_DESCRIPTION, "height of ROI on the left");
    SetPropertyBool(ROI_NORMAL_DRIVE "::left::height" NSSUBPROP_ISCHANGEABLE, tTrue);

    /* zebra crossing: pedestrians */
    SetPropertyInt(ROI_ZEBRA_CROSSING "::center::x", 150);
    SetPropertyStr(ROI_ZEBRA_CROSSING "::center::x" NSSUBPROP_DESCRIPTION, "x coordinate of top left corner of ROI at a zebra crossing");
    SetPropertyBool(ROI_ZEBRA_CROSSING "::center::x" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_ZEBRA_CROSSING "::center::y", 575);
    SetPropertyStr(ROI_ZEBRA_CROSSING "::center::y" NSSUBPROP_DESCRIPTION, "y coordinate of top left corner of ROI at a zebra crossing");
    SetPropertyBool(ROI_ZEBRA_CROSSING "::center::y" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_ZEBRA_CROSSING "::center::width", 700);
    SetPropertyStr(ROI_ZEBRA_CROSSING "::center::width" NSSUBPROP_DESCRIPTION, "width of ROI at a zebra crossing");
    SetPropertyBool(ROI_ZEBRA_CROSSING "::center::width" NSSUBPROP_ISCHANGEABLE, tTrue);

    SetPropertyInt(ROI_ZEBRA_CROSSING "::center::height", 75);
    SetPropertyStr(ROI_ZEBRA_CROSSING "::center::height" NSSUBPROP_DESCRIPTION, "height of ROI at a zebra crossing");
    SetPropertyBool(ROI_ZEBRA_CROSSING "::center::height" NSSUBPROP_ISCHANGEABLE, tTrue);


    /* distance to drive past detection */
    double defaultDistanceToDrivePastDetection = 1.0;
    SetPropertyFloat(DISTANCE_TO_DRIVE, defaultDistanceToDrivePastDetection);
    SetPropertyStr(DISTANCE_TO_DRIVE NSSUBPROP_DESCRIPTION, "The distance the car has to drive to be allowed to accelerate again.");
    SetPropertyBool(DISTANCE_TO_DRIVE NSSUBPROP_ISCHANGEABLE, tTrue);


    /* communication with python server */
    cString defaultThriftIP = "localhost";
    SetPropertyStr(THRIFT "::" THRIFT_IP, defaultThriftIP);
    SetPropertyStr(THRIFT "::" THRIFT_IP NSSUBPROP_DESCRIPTION, "The IP V4 adress of the thrift RPC server.");

    tInt defaultThriftPort = 1833;
    SetPropertyInt(THRIFT "::" THRIFT_PORT, defaultThriftPort);
    SetPropertyStr(THRIFT "::" THRIFT_PORT NSSUBPROP_DESCRIPTION, "Port number for thrift server.");


    // initialize properties
    m_frameCount = -1;
    m_detectionMode = DETECT_NOTHING;
    m_sendClassificationResults = false;

    m_detectionStartingPoint = 0.0;
    m_distanceToDrivePastDetection = defaultDistanceToDrivePastDetection;

    m_createDebugImage = defaultCreateDebugImage;
    m_debugClassificationTextColor = cv::Scalar(0, 0, 0);
    m_debugClassificationTextFontScale = defaultClassificationTextFontScale;
    m_debugClassificationTextThickness = defaultClassificationTextFontThickness;
    m_debugClassificationTextLineHeight = defaultClassificationTextLineHeight;

    m_thriftIP = defaultThriftIP;
    m_thriftPort = defaultThriftPort;
}

ObjectSpecifier::~ObjectSpecifier()
{
}



// adtf functions
tResult ObjectSpecifier::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**) &pDescManager, __exception_ptr));

        // action input
        RETURN_IF_FAILED(m_actionPort.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(m_actionInput.Create("action", m_actionPort.GetMediaType(), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_actionInput));

        // video input
        RETURN_IF_FAILED(m_videoInput.Create("videoInput", IPin::PD_Input, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_videoInput));

        // car speed input
        RETURN_IF_FAILED(m_carSpeedInputPort.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(m_carSpeedInput.Create("carSpeedInput", m_carSpeedInputPort.GetMediaType(), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_carSpeedInput));

        // distance driven input
        RETURN_IF_FAILED(m_distanceDrivenPort.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(m_distanceDrivenInput.Create("distanceDriven", m_distanceDrivenPort.GetMediaType(), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_distanceDrivenInput));

        // feedback output
        RETURN_IF_FAILED(m_feedback.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(m_feedbackOutput.Create("feedback", m_feedback.GetMediaType(), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_feedbackOutput));

        // debug video output
        RETURN_IF_FAILED(m_videoOutput.Create("debugVideoOutput", IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_videoOutput));

        // car speed output
        RETURN_IF_FAILED(m_carSpeedOutputPort.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(m_carSpeedOutput.Create("carSpeedOutput", m_carSpeedOutputPort.GetMediaType(), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_carSpeedOutput));
    }
    else if (eStage == StageNormal)
    {
        // In this stage you would do further initialisation and/or create your dynamic pins.
        // Please take a look at the demo_dynamicpin example for further reference.
    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.

        // init the socket, transport and the protocol from thrift
        boost::shared_ptr<TTransport> socket(new TSocket(m_thriftIP.GetPtr(), m_thriftPort));
        boost::shared_ptr<TTransport> transport(new TBufferedTransport(socket));
        boost::shared_ptr<TProtocol> protocol(new TBinaryProtocol(transport));
        // make the client
        m_thriftClient = boost::make_shared<ext_iface::ExtServiceClient>(protocol);
        // create the client thread
        tResult nResult = m_thriftClientThread.Create(cKernelThread::TF_Suspended, static_cast<adtf::IKernelThreadFunc*>(this));

        if (IS_FAILED(nResult))
            THROW_ERROR_DESC(nResult, "Failed to create threads");

        // get the image format of the input video pin
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_videoInput.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**) &pTypeVideo));

        // set the image format of the input video pin
        if (IS_FAILED(UpdateInputImageFormat(pTypeVideo->GetFormat())))
            LOG_ERROR("Invalid input format for this filter.");

        // call StageGraphReady for outsourced MediaTypes
        RETURN_IF_FAILED(m_actionPort.StageGraphReady());
        RETURN_IF_FAILED(m_carSpeedInputPort.StageGraphReady());
        RETURN_IF_FAILED(m_distanceDrivenPort.StageGraphReady());

        RETURN_IF_FAILED(m_feedback.StageGraphReady());
        RETURN_IF_FAILED(m_carSpeedOutputPort.StageGraphReady());
    }

    RETURN_NOERROR;
}

tResult ObjectSpecifier::Start(__exception)
{
    // start the thread if not running by now
    if (m_thriftClientThread.GetState() != cKernelThread::TS_Running)
        m_thriftClientThread.Run();

    return cFilter::Start(__exception_ptr);
}

tResult ObjectSpecifier::Stop(__exception)
{
    // suspend the thread
    if (m_thriftClientThread.GetState() == cKernelThread::TS_Running)
        m_thriftClientThread.Suspend();

    // clean out data
    ClearClassificationValues(m_ROIsNormalDrive);
    ClearClassificationValues(m_ROIsCrossing);
    ClearClassificationValues(m_ROIsZebraCrossing);

    return cFilter::Stop(__exception_ptr);
}

tResult ObjectSpecifier::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception: 
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.

    // release the thread
    m_thriftClientThread.Terminate(tTrue);
    m_thriftClientThread.Release();
    
    if (eStage == StageGraphReady)
    {
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




tResult ObjectSpecifier::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    __synchronized_obj(m_critSectionSpeed);

    RETURN_IF_POINTER_NULL(pSource);

    // first check what kind of event it is
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

        // process action
        if (pSource == &m_actionInput)
        {
            ProcessAction(pMediaSample);
        }

        // process video frame
        else if (pSource == &m_videoInput)
        {
            if (m_detectionMode == DETECT_NOTHING)
                RETURN_NOERROR;

            // check if video format is still unkown
            if (m_videoInputFormat.nPixelFormat == IImage::PF_UNKNOWN)
                RETURN_IF_FAILED(UpdateInputImageFormat(m_videoInput.GetFormat()));

            ProcessVideoInput(pMediaSample);

            if (m_sendClassificationResults)
                TransmitClassificationResults();
        }

        // process car speed
        else if (pSource == &m_carSpeedInput)
        {
            ProcessCarSpeed(pMediaSample);
        }

        // process distance driven
        else if (pSource == &m_distanceDrivenInput)
        {
            ProcessDistanceDriven(pMediaSample);
        }
    }

    // the input format was changed, so the image format has to changed in this filter, too
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged)
    {
        if (pSource == &m_videoInput)
        {
            RETURN_IF_FAILED(UpdateInputImageFormat(m_videoInput.GetFormat()));
        }
    }

    RETURN_NOERROR;
}

tResult ObjectSpecifier::PropertyChanged(const tChar* pPropertyName)
{
    RETURN_IF_FAILED(cFilter::PropertyChanged(pPropertyName));

    cString propName(pPropertyName);
    std::map<cString, ROI>* usedMap;
    std::vector<cString> locations;

    // debug image properties
    if (propName.StartsWith(DEBUG_IMG))
    {
        if (propName.EndsWith(DEBUG_IMG_CREATE))
            m_createDebugImage = GetPropertyBool(pPropertyName);

        else if (propName.EndsWith(DEBUG_IMG_TEXT_COLOR))
        {
            std::string color = GetPropertyStr(pPropertyName);

            if (color == "Black")
                m_debugClassificationTextColor = cv::Scalar(0, 0, 0);
            else
                m_debugClassificationTextColor = cv::Scalar(255, 255, 255);
        }

        else if (propName.EndsWith(DEBUG_IMG_TEXT_FONTSCALE))
            m_debugClassificationTextFontScale = GetPropertyFloat(pPropertyName);

        else if (propName.EndsWith(DEBUG_IMG_TEXT_THICKNESS))
            m_debugClassificationTextThickness = GetPropertyInt(pPropertyName);

        else if (propName.EndsWith(DEBUG_IMG_TEXT_LINE_HEIGHT))
            m_debugClassificationTextLineHeight = GetPropertyInt(pPropertyName);
        
        RETURN_NOERROR;
    }

    // distance to drive past detection
    else if (propName.IsEqual(DISTANCE_TO_DRIVE))
    {
        m_distanceToDrivePastDetection = GetPropertyFloat(pPropertyName);
    }

    // thrift properties
    else if (propName.StartsWith(THRIFT))
    {
        if (propName.EndsWith(THRIFT_IP))
            m_thriftIP = GetPropertyStr(pPropertyName);

        else if (propName.EndsWith(THRIFT_PORT))
            m_thriftPort = GetPropertyInt(pPropertyName);

        RETURN_NOERROR;
    }

    // detection properties
    else if (propName.StartsWith(DETECTION))
    {
        if (propName.EndsWith(CNN_DETECTION_MODE))
        {
            cString value = GetPropertyStr(pPropertyName);

            if (value == "Normal drive")
                m_detectionMode = DETECT_NORMAL_DRIVE;
            else if (value == "Crossing")
                m_detectionMode = DETECT_CROSSING;
            else if (value == "Zebra crossing")
                m_detectionMode = DETECT_ZEBRA_CROSSING;
            else
                m_detectionMode = DETECT_NOTHING;

            LOG_INFO(cString::Format("changed value of '%s' to %s", pPropertyName, value.GetPtr()));
        }

        else if (propName.EndsWith(DETECTION_ADULT))
            m_detectionThresholds["adult"] = GetPropertyFloat(pPropertyName);

        else if (propName.EndsWith(DETECTION_CHILD))
            m_detectionThresholds["child"] = GetPropertyFloat(pPropertyName);

        else if (propName.EndsWith(DETECTION_CAR))
            m_detectionThresholds["car"] = GetPropertyFloat(pPropertyName);

        RETURN_NOERROR;
    }

    // ROI properties; check which map should be used
    else if (propName.StartsWith(ROI_CROSSING))
    {
        usedMap = &m_ROIsCrossing;
        locations.push_back("nearRight");
        locations.push_back("farRight");
        locations.push_back("farLeft");
        locations.push_back("right");
        locations.push_back("straight");
        locations.push_back("left");

        m_ROIsToCareAbout.insert("nearRight");
        m_ROIsToCareAbout.insert("farRight");
        m_ROIsToCareAbout.insert("farLeft");
        m_ROIsToCareAbout.insert("right");
        m_ROIsToCareAbout.insert("straight");
        m_ROIsToCareAbout.insert("left");
    }

    else if (propName.StartsWith(ROI_NORMAL_DRIVE))
    {
        usedMap = &m_ROIsNormalDrive;
        locations.push_back("right");
        locations.push_back("left");

        m_ROIsToCareAbout.insert("right");
        m_ROIsToCareAbout.insert("left");
    }

    else if (propName.StartsWith(ROI_ZEBRA_CROSSING))
    {
        usedMap = &m_ROIsZebraCrossing;
        locations.push_back("center");

        m_ROIsToCareAbout.insert("center");
    }

    // cycle through possible values
    for(std::vector<cString>::const_iterator it = locations.begin(); it != locations.end(); ++it)
    {
        cString location = it->GetPtr();
        tBool containsString = propName.Find(location) > 0;

        // search for other value if the current isn't found
        if (!containsString)
            continue;

        // cycle through possible values
        cString values[4] = { "x", "y", "width", "height" };

        for (int k = 0; k < 4; k++)
        {
            if (propName.EndsWith(values[k]))
            {
                // property found, change value in respective map entry
                ROI roi = (*usedMap)[location];
                int value = GetPropertyInt(pPropertyName);

                if (values[k] == "x")
                    roi.x = value;

                else if (values[k] == "y")
                    roi.y = value;

                else if (values[k] == "width")
                    roi.width = value;

                else if (values[k] == "height")
                    roi.height = value;

                (*usedMap)[location] = roi;
                //LOG_INFO(cString::Format("changed value of '%s' to %d", pPropertyName, value));

                // break out of this loop, we have found our value
                break;
            }
        }

        // break here to save some time
        break;
    }

    RETURN_NOERROR;
}

tResult ObjectSpecifier::ThreadFunc(cKernelThread* pThread, tVoid* pvUserData, tSize szUserData)
{
    if (pThread == &m_thriftClientThread)
    {
        tBool clientIsOpen = tFalse;
        try
        {
            // try to open if not opened
            if (!m_thriftClient->getOutputProtocol()->getOutputTransport()->isOpen())
                m_thriftClient->getOutputProtocol()->getOutputTransport()->open();

            // verify
            if (m_thriftClient->getOutputProtocol()->getOutputTransport()->isOpen())
                clientIsOpen = tTrue;

        }
        catch (apache::thrift::TException except)
        {
            LOG_ERROR("Thrift client could not connect to server");
        }

        if (clientIsOpen)
        {
            // enter mutex
            m_critSectionSendToPython.Enter();
            if (m_thriftRawMessageBuffer.raw_data.size() != 0)
            {
                try
                {
                    // send raw data to python server and receive response
                    TDataResultList response;
                    m_thriftClient->rawData(response, TransportDef::IMAGEDATA, m_thriftRawMessageBuffer, m_thriftImageParamsBuffer);
                    
                    // process response from server
                    ProcessClassificationResults(response);

                    // clear the buffer
                    m_thriftRawMessageBuffer.raw_data.clear();
                    m_critSectionSendToPython.Leave();

                }
                catch (apache::thrift::TException except)
                {
                    //send method quit with exception so write it here to console
                    LOG_ERROR("Thrift client could not send to server");
                    LOG_ERROR(cString::Format("EXCEPTION: %s", except.what()));
                    m_critSectionSendToPython.Leave();
                    cSystem::Sleep(200000);
                }
            }
            else
            {
                // nothing to transmit
                m_critSectionSendToPython.Leave();
                cSystem::Sleep(50000);
            }
        }
        else
        {
            //no client open
            LOG_ERROR("Thrift client could not connect to server");
            cSystem::Sleep(200000);
        }
    }

    RETURN_NOERROR;
}



// own functions
tResult ObjectSpecifier::ProcessVideoInput(IMediaSample* pMediaSample)
{
    ++m_frameCount;

    // only process every fifth frame
    if (m_frameCount%5 != 0)
        RETURN_NOERROR;

    // we have stuffed bytes here so we cannot process it correctly
    if (m_videoInputFormat.nWidth * m_videoInputFormat.nBitsPerPixel / 8 != m_videoInputFormat.nBytesPerLine)
        RETURN_ERROR(ERR_NOT_SUPPORTED);

    // reading data from input
    const tVoid* l_pSrcBuffer;

    if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer)))
    {
        // read data; preserve original in inputFrame ...
        memcpy(m_inputFrame.data, l_pSrcBuffer, size_t(m_videoInputFormat.nSize));

        // enter mutex and set data from media to buffer
        // if still blocked we return and drop the data
        if (m_critSectionSendToPython.TryEnter())
        {
            // convert to bitmap
            std::vector<unsigned char> outFileData;
            if (cv::imencode("*.bmp", m_inputFrame, outFileData))
            {
                // clear ROIs buffer & fill with appropriate info
                m_thriftRawMessageBuffer.rois.clear();

                std::map<cString, ROI> iterMap;
                bool runThroughMap = false;

                switch (m_detectionMode)
                {
                    case DETECT_NORMAL_DRIVE:
                        iterMap = m_ROIsNormalDrive;
                        runThroughMap = true;
                        break;

                    case DETECT_CROSSING:
                        iterMap = m_ROIsCrossing;
                        runThroughMap = true;
                        break;

                    case DETECT_ZEBRA_CROSSING:
                        iterMap = m_ROIsZebraCrossing;
                        runThroughMap = true;
                        break;

                    default:
                    case DETECT_NOTHING:
                        // nothing to do
                        runThroughMap = false;
                        break;
                }

                if (runThroughMap)
                {
                    for (std::map<cString, ROI>::iterator it = iterMap.begin(); it != iterMap.end(); ++it)
                    {
                        ROI roi = it->second;
                        
                        TROI troi;
                        troi.x = roi.x;
                        troi.y = roi.y;
                        troi.width = roi.width;
                        troi.height = roi.height;
                        troi.name = std::string(it->first.GetPtr());

                        m_thriftRawMessageBuffer.rois.push_back(troi);
                    }     
                }


                // clear image buffer & fill with new frame
                m_thriftRawMessageBuffer.raw_data.clear();
                m_thriftRawMessageBuffer.raw_data.assign((const char*)(outFileData.data()), outFileData.size());
            }

            m_critSectionSendToPython.Leave();
        }

        RETURN_IF_FAILED(pMediaSample->Unlock(l_pSrcBuffer));
    }

    RETURN_NOERROR;
}

tResult ObjectSpecifier::ProcessCarSpeed(IMediaSample* pMediaSample)
{
    // read the sample
    TSignalValue::Data speedSample;
    m_carSpeedInputPort.Read(pMediaSample, &speedSample);

    //LOG_INFO(cString::Format("input speed: %.2f", speedSample.f32_value));

    // control output speed
    double outputSpeed = static_cast<double>(speedSample.f32_value);
    bool childDetected = false;

    switch (m_detectionMode)
    {
        case DETECT_NORMAL_DRIVE:
            childDetected = HasDetectedChild(m_ROIsNormalDrive);
            break;

        case DETECT_CROSSING:
            childDetected = HasDetectedChild(m_ROIsCrossing);
            break;

        case DETECT_ZEBRA_CROSSING:
            childDetected = HasDetectedChild(m_ROIsZebraCrossing);
            break;

        default:
        case DETECT_NOTHING:
            // nothing to do
            outputSpeed = static_cast<double>(speedSample.f32_value);
            childDetected = false;
            break;
    }

    // track distance to accelerate again after some time
    if (childDetected)
        m_detectionStartingPoint = static_cast<double>(m_distanceDrivenValue.f32_value);

    // if the car still has to drive some distance to exit the area were the
    // object was detected, then cap the speed no matter if there was an object
    // detected in the last few frames (decided within the switch-case above)
    double endPoint = m_detectionStartingPoint + m_distanceToDrivePastDetection;
    if (endPoint >= static_cast<double>(m_distanceDrivenValue.f32_value))
    {
        if (outputSpeed > MAX_SPEED)
            outputSpeed = MAX_SPEED;
    }

    /*LOG_INFO(cString::Format("starting point: %.2f", m_detectionStartingPoint));
    LOG_INFO(cString::Format("distance driven: %.2f", static_cast<double>(m_distanceDrivenValue.f32_value)));
    LOG_INFO(cString::Format("output speed: %.2f", outputSpeed));*/

    // transmit new speed sample
    m_carSpeedInputValue.f32_value = static_cast<tFloat32>(outputSpeed);
    m_carSpeedOutputPort.Transmit(&m_carSpeedOutput, m_carSpeedInputValue, _clock->GetStreamTime());

    RETURN_NOERROR;
}

tResult ObjectSpecifier::ProcessDistanceDriven(IMediaSample* pMediaSample)
{
    // read the sample
    m_distanceDrivenPort.Read(pMediaSample, &m_distanceDrivenValue);

    RETURN_NOERROR;
}

tResult ObjectSpecifier::ProcessAction(IMediaSample* pMediaSample)
{
    // read action
    m_actionStruct = m_actionPort.Read_Action(pMediaSample, F_OBJECT_SPECIFIER);

    // send feedback back to the StateMachine about what was detected
    if (m_actionStruct.command == AC_OS_REQUEST_FEEDBACK)
        m_sendClassificationResults = true;

    // detect nothing
    else if (m_actionStruct.command == AC_OS_DETECT_NOTHING)
    {
        m_detectionMode = DETECT_NOTHING;
        m_ROIsToCareAbout.clear();
        m_sendClassificationResults = false;
    }

    // detect chil(dren) at both sides of the road
    else if (m_actionStruct.command == AC_OS_DETECT_NORMAL_DRIVE)
    {
        m_detectionMode = DETECT_NORMAL_DRIVE;
        m_ROIsToCareAbout.clear();
        m_sendClassificationResults = false;

        m_ROIsToCareAbout.insert("left");
        m_ROIsToCareAbout.insert("right");
    }

    // detect cars & pedestrians everywhere at crossings
    else if (m_actionStruct.command == AC_OS_DETECT_CROSSING)
    {
        m_detectionMode = DETECT_CROSSING;
        m_ROIsToCareAbout.clear();
        m_sendClassificationResults = false;

        // cars
        m_ROIsToCareAbout.insert("left");
        m_ROIsToCareAbout.insert("right");
        m_ROIsToCareAbout.insert("straight");

        // pedestrians
        /*m_ROIsToCareAbout.insert("nearRight");
        m_ROIsToCareAbout.insert("farRight");
        m_ROIsToCareAbout.insert("farLeft");*/
    }

    // detect cars & pedestrians only for a right turn
    else if (m_actionStruct.command == AC_OS_DETECT_CROSSING_RIGHT)
    {
        m_detectionMode = DETECT_CROSSING;
        m_ROIsToCareAbout.clear();
        m_sendClassificationResults = false;

        // cars
        m_ROIsToCareAbout.insert("right");

        // pedestrians
        /*m_ROIsToCareAbout.insert("nearRight");
        m_ROIsToCareAbout.insert("farRight");*/
    }

    // detect cars & pedestrians only for a left turn
    else if (m_actionStruct.command == AC_OS_DETECT_CROSSING_LEFT)
    {
        m_detectionMode = DETECT_CROSSING;
        m_ROIsToCareAbout.clear();
        m_sendClassificationResults = false;

        // cars
        m_ROIsToCareAbout.insert("left");
        m_ROIsToCareAbout.insert("straight");

        // pedestrians
        //m_ROIsToCareAbout.insert("farLeft");
    }

    // detect cars & pedestrians only for driving straight ahead
    else if (m_actionStruct.command == AC_OS_DETECT_CROSSING_STRAIGHT_AHEAD)
    {
        m_detectionMode = DETECT_CROSSING;
        m_ROIsToCareAbout.clear();
        m_sendClassificationResults = false;

        // cars
        m_ROIsToCareAbout.insert("straight");

        // pedestrians
        /*m_ROIsToCareAbout.insert("farRight");
        m_ROIsToCareAbout.insert("farLeft");*/
    }

    // detect pedestrians at a zebra crossing
    else if (m_actionStruct.command == AC_OS_DETECT_ZEBRA_CROSSING)
    {
        m_detectionMode = DETECT_ZEBRA_CROSSING;
        m_ROIsToCareAbout.clear();
        m_sendClassificationResults = false;

        m_ROIsToCareAbout.insert("center");
    }

    RETURN_NOERROR;
}




tResult ObjectSpecifier::ProcessClassificationResults(TDataResultList serverResponse)
{
    // create image with ROIs for debug purposes
    if (m_createDebugImage)
    {
        // save working copy in outputFrame
        m_outputFrame = m_inputFrame.clone();

        // paint ROIs onto debug image
        PaintROIsOnDebugFrame();
    }


    // process response from server
    if (serverResponse.size() == 0)
        RETURN_NOERROR;

    for (std::vector<TDataResult>::iterator rit = serverResponse.begin(); rit != serverResponse.end(); ++rit)
    {
        // container for needed labels
        std::map<cString, double> labels;

        TROI troi = rit->roi;
        std::vector<TDataClassification> results = rit->results;

        for (std::vector<class TDataClassification>::iterator it = results.begin(); it != results.end(); ++it)
        {
            cString classification = cString(it->classification.c_str());
            double probability = it->probability;

            //LOG_INFO(cString::Format("%s: %.3f", classification.GetPtr(), probability));

            // check if classification is above threshold
            double threshold = m_detectionThresholds[classification];
            if (probability >= threshold)
                labels[classification] = probability;

            // save values in corresponding map
            switch (m_detectionMode)
            {
                case DETECT_NORMAL_DRIVE:
                    StoreClassificationValue(m_ROIsNormalDrive[cString(troi.name.c_str())].probabilities[classification], probability);
                    break;

                case DETECT_CROSSING:
                    StoreClassificationValue(m_ROIsCrossing[cString(troi.name.c_str())].probabilities[classification], probability);
                    break;

                case DETECT_ZEBRA_CROSSING:
                    StoreClassificationValue(m_ROIsZebraCrossing[cString(troi.name.c_str())].probabilities[classification], probability);
                    break;

                default:
                case DETECT_NOTHING:
                    // nothing to do
                    break;
            } 
        }

        // debug
        /*LOG_INFO(cString::Format("here"));
        std::map<cString, std::deque<double> > values = m_ROIsZebraCrossing["center"].probabilities;
        for (std::map<cString, std::deque<double> >::iterator it = values.begin(); it != values.end(); ++it)
        {
            LOG_INFO(cString::Format("class %s:", it->first.GetPtr()));

            for (int i = 0; i < it->second.size(); ++i)
                LOG_INFO(cString::Format("%d: %.3f", i, it->second.at(i)));
        }*/

        // paint labels
        PaintLabelsOnDebugFrame(troi, labels);
    }

    // sending back data
    if (m_createDebugImage && !m_outputFrame.empty())
    {
        UpdateOutputImageFormat(m_outputFrame);

        // create image
        cImage img;
        img.Create(
            m_videoOutputFormat.nWidth,
            m_videoOutputFormat.nHeight,
            m_videoOutputFormat.nBitsPerPixel,
            m_videoOutputFormat.nBytesPerLine,
            m_outputFrame.data
        );

        // create new media sample ...
        cObjectPtr<IMediaSample> sample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**) &sample));

        // ... update it ...
        RETURN_IF_FAILED(
            sample->Update(
                _clock->GetStreamTime(),
                img.GetBitmap(),
                img.GetSize(),
                IMediaSample::MSF_None
            )
        );

        // ... and transmit it
        RETURN_IF_FAILED(m_videoOutput.Transmit(sample));

        m_outputFrame.release();
    }

    RETURN_NOERROR;
}

tResult ObjectSpecifier::StoreClassificationValue(std::deque<double>& list, double probability)
{   
    int numValues = list.size();

    if (numValues > (ROI::MAX_NUM_PROBABILITIES - 1))
    {
        // remove the first element from the list until there is space for a new one
        do
            list.pop_front();
        while (list.size() > (ROI::MAX_NUM_PROBABILITIES - 1));
    }

    // append the new value
    list.push_back(probability);

    RETURN_NOERROR;
}

tResult ObjectSpecifier::ClearClassificationValues(std::map<cString, ROI>& usedMap)
{
    for (std::map<cString, ROI>::iterator it = usedMap.begin(); it != usedMap.end(); ++it)
    {
        for (std::map<cString, std::deque<double> >::iterator iit = it->second.probabilities.begin(); iit != it->second.probabilities.end(); ++iit)
            iit->second.clear();
    }

    RETURN_NOERROR;
}

tResult ObjectSpecifier::TransmitClassificationResults()
{
    // create feedback & set filter ID
    TFeedbackStruct::Data feedback;
    feedback.ui32_filterID = F_OBJECT_SPECIFIER;

    // check which ROIs should be considered
    switch (m_detectionMode)
    {
        case DETECT_NORMAL_DRIVE:
            // normally no feedback for normal drive needed, filter caps speed on its own
            if (HasDetectedChild(m_ROIsNormalDrive))
                feedback.ui32_status = FB_OS_DETECTED_CHILD;
            else
                feedback.ui32_status = FB_OS_DETECTED_NOTHING;

            break;

        case DETECT_CROSSING:
            // for now only detect cars, pedestrians are ignored (!!!)
            if (HasDetectedCar(m_ROIsCrossing))
                feedback.ui32_status = FB_OS_DETECTED_CAR;
            else
                feedback.ui32_status = FB_OS_DETECTED_NOTHING;

            break;

        case DETECT_ZEBRA_CROSSING:

            if (HasDetectedAdult(m_ROIsZebraCrossing))
                feedback.ui32_status = FB_OS_DETECTED_ADULT;
            else if (HasDetectedChild(m_ROIsZebraCrossing))
                feedback.ui32_status = FB_OS_DETECTED_CHILD;
            else
                feedback.ui32_status = FB_OS_DETECTED_NOTHING;

            break;

        default:
        case DETECT_NOTHING:
            // nothing to do, send feedback AC_OS_DETECT_NOTHING
            feedback.ui32_status = FB_OS_DETECTED_NOTHING;
            break;
    }

    LOG_INFO(cString::Format("feedback: %d", feedback.ui32_status));

    // transmit feedback
    m_feedback.Transmit(&m_feedbackOutput, feedback, _clock->GetStreamTime());

    RETURN_NOERROR;
}



bool ObjectSpecifier::HasDetectedObject(std::map<cString, ROI>& usedMap, cString objectName)
{
    __synchronized_obj(m_critSectionSpeed);

    for (std::map<cString, ROI>::iterator it = usedMap.begin(); it != usedMap.end(); ++it)
    {
        cString roiName = it->first;

        if (m_ROIsToCareAbout.find(roiName) != m_ROIsToCareAbout.end())
        {
            std::deque<double> probs = it->second.probabilities[objectName];
            double threshold = m_detectionThresholds[objectName];

            int numDetectionsNeededToAct = 1;
            int numDetections = 0;

            for (std::deque<double>::iterator iit = probs.begin(); iit != probs.end(); ++iit)
            {
                if (*iit > threshold)
                    numDetections++;
            }

            // when one ROI has detected an object, cap speed & return
            if (numDetections >= numDetectionsNeededToAct)
                return true;
        }
    }

    return false;
}

bool ObjectSpecifier::HasDetectedAdult(std::map<cString, ROI>& usedMap)
{
    return ObjectSpecifier::HasDetectedObject(usedMap, "adult");
}

bool ObjectSpecifier::HasDetectedChild(std::map<cString, ROI>& usedMap)
{
    return ObjectSpecifier::HasDetectedObject(usedMap, "child");
}

bool ObjectSpecifier::HasDetectedCar(std::map<cString, ROI>& usedMap)
{
    return ObjectSpecifier::HasDetectedObject(usedMap, "car");
}



void ObjectSpecifier::PaintROIsOnDebugFrame()
{
    if (!m_createDebugImage)
        return;

    switch (m_detectionMode)
    {
        // look for children on the side of the road
        case DETECT_NORMAL_DRIVE:

            for (std::map<cString, ROI>::iterator it = m_ROIsNormalDrive.begin(); it != m_ROIsNormalDrive.end(); ++it)
            {
                ROI roi = it->second;
                cString roiName = it->first;

                if (m_ROIsToCareAbout.find(roiName) != m_ROIsToCareAbout.end())
                    cv::rectangle(m_outputFrame, cv::Rect(roi.x, roi.y, roi.width, roi.height), CV_RGB(0, 200, 0), 3, 8, 0);
            }

            break;

        // look for pedestrians & cars
        case DETECT_CROSSING:

            for (std::map<cString, ROI>::iterator it = m_ROIsCrossing.begin(); it != m_ROIsCrossing.end(); ++it)
            {
                ROI roi = it->second;
                cString roiName = it->first;

                if (m_ROIsToCareAbout.find(roiName) != m_ROIsToCareAbout.end())
                    cv::rectangle(m_outputFrame, cv::Rect(roi.x, roi.y, roi.width, roi.height), CV_RGB(200, 0, 0), 3, 8, 0);
            }

            break;

        // look for pedestrians at a zebra crossing
        case DETECT_ZEBRA_CROSSING:

            for (std::map<cString, ROI>::iterator it = m_ROIsZebraCrossing.begin(); it != m_ROIsZebraCrossing.end(); ++it)
            {
                ROI roi = it->second;
                cString roiName = it->first;

                if (m_ROIsToCareAbout.find(roiName) != m_ROIsToCareAbout.end())
                    cv::rectangle(m_outputFrame, cv::Rect(roi.x, roi.y, roi.width, roi.height), CV_RGB(0, 0, 200), 3, 8, 0);
            }

            break;

        // detect nothing
        case DETECT_NOTHING:
        default:
            // nothing to do
            break;
    }
}

void ObjectSpecifier::PaintLabelsOnDebugFrame(TROI roi, std::map<cString, double> labels)
{
    if (!m_createDebugImage)
        return;

    // print all labels in the list
    int numLine = 0;
    double offset = m_debugClassificationTextLineHeight * m_debugClassificationTextFontScale;
    
    for (std::map<cString, double>::iterator it = labels.begin(); it != labels.end(); ++it)
    {
        std::string text = std::string(it->first.GetPtr()) + ": " + std::to_string(it->second);

        // paint labels above each other at the upper left corner of the ROI
        cv::putText(
            m_outputFrame,
            text,
            cv::Point(roi.x, (roi.y - 10) - (numLine * offset)),
            cv::FONT_HERSHEY_SIMPLEX,
            m_debugClassificationTextFontScale,
            m_debugClassificationTextColor,
            m_debugClassificationTextThickness
        );

        numLine++;
    }
}



tResult ObjectSpecifier::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        // update member variable
        m_videoInputFormat = (*pFormat);
        cMemoryBlock::MemCopy(&m_videoInputFormat, pFormat, sizeof(tBitmapFormat));
        LOG_INFO(
                adtf_util::cString::Format(
                        "UpdateInput: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
                        m_videoInputFormat.nWidth, m_videoInputFormat.nHeight,
                        m_videoInputFormat.nBytesPerLine, m_videoInputFormat.nSize,
                        m_videoInputFormat.nPixelFormat));

        // write the struct to be send to thrift
        m_thriftImageParamsBuffer.__set_bytesPerPixel(m_videoInputFormat.nBitsPerPixel);
        m_thriftImageParamsBuffer.__set_height(int16_t(m_videoInputFormat.nHeight));
        m_thriftImageParamsBuffer.__set_width(int16_t(m_videoInputFormat.nWidth));

        // create the input matrix
        RETURN_IF_FAILED(BmpFormat2Mat(m_videoInputFormat, m_inputFrame));
    }

    RETURN_NOERROR;
}

tResult ObjectSpecifier::UpdateOutputImageFormat(const cv::Mat& outputImage)
{
    // check if pixelformat or size has changed
    if (tInt32(outputImage.total() * outputImage.elemSize()) != m_videoOutputFormat.nSize) {
        Mat2BmpFormat(outputImage, m_videoOutputFormat);

        LOG_INFO(
                adtf_util::cString::Format(
                        "UpdateOutput: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",
                        m_videoOutputFormat.nWidth, m_videoOutputFormat.nHeight,
                        m_videoOutputFormat.nBytesPerLine, m_videoOutputFormat.nSize,
                        m_videoOutputFormat.nPixelFormat));

        // set output format for output pin
        m_videoOutput.SetFormat(&m_videoOutputFormat, NULL);
    }

    RETURN_NOERROR;
}
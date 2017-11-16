#ifndef __STD_INCLUDES_HEADER
#define __STD_INCLUDES_HEADER

#ifdef WIN32
#include <windows.h>
#endif

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <additional/adtf_signal_registry_support.h>
using namespace adtf;

#include <adtf_graphics.h>
using namespace adtf_graphics;

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>

#include "TSignalValue.h"
#include "TActionStruct.h"
#include "TFeedbackStruct.h"
#include "ScmCommunication.h"

#include <stdlib.h>   
#include <cmath>

#include <fstream>
#include <iostream>
#include <time.h>

using namespace std;
using namespace cv;


#endif // __STD_INCLUDES_HEADER

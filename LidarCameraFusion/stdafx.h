// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: reference additional headers your program requires here
#ifdef _DEBUG 
#pragma comment (lib, "opencv_world320d.lib") 
#else 
#pragma comment (lib, "opencv_world320.lib")
#endif


#include <iostream>
#include <opencv2/opencv.hpp> //opencv3.2.0
using namespace std;
using namespace cv;


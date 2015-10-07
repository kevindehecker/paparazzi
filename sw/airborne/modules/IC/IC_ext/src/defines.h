#ifndef DEFINES_H
#define DEFINES_H

#define VIZ_right_input_image 1
#define VIZ_texton_intensity_color_encoding 2
#define VIZ_texton_intensity_texton_encoding 3
#define VIZ_texton_gradient_color_encoding 4
#define VIZ_texton_gradient_texton_encoding 5
#define VIZ_histogram 6
#define VIZ_ROC 7
#define VIZ_MEGA 8

#ifdef _PC /*************************     PC     **************************************************************************/
#define SSE2

//#ifndef DEBUG_FLAG
//#define RUNMODE learn_stereo_textons // start with this mode (enum defined in IC.cpp)
//#else
//#define RUNMODE learn_stereo_textons // start with this mode (enum defined in IC.cpp)
//#endif
#define DELFLY //use the Delfly stereo cam over usb2serial
//#define DUOWEBCAM // use the double webcam stereo set up

//#define DELFLY_WIFI // use the delfly stereo cam, while it is connected to the ardrone2, streaming over wifi
#define FILECAM // use a video as source instead of the camera. The file name is defined in filecam.cpp

//#define FILESTEREO
//#define GEIGER // use libelas Geiger stereo algorithm
//#define SGM  //use OpenCV Semi Global Matching stereo algorithm
//#define BM currently not completely implemented
#define LONGSEC // use Sjoerds improved sparse stereo algorithm

//#ifdef DEBUG_FLAG
#define VIDEORESULTS // show the main results window
#define DRAWVIZS    //show secundairy results (histograms and texton visulaisations
//#endif
//#define EXPORT //create export.txt and seperate stereo pair png images

#ifdef DEBUG_FLAG
#define HASSCREEN // dont disable in qt debugger!
#endif
#ifndef FILECAM
#define VIDEORAW // write the raw video footage from the camera to a video file
#endif

#define VIDEOFPS 10.0f // the estimated frame rate of the video used for creating output videos

//#define USE_TERMINAL_INPUT // using this will conflict with qt debugging
//#define USE_SOCKET // communication to the pprz IC module

#else /*************************    DRONE     *****************************************************************************/
#define RUNMODE stereo_textons
#define DELFLY

#define NEON

#define VIDEORAW
//#define RAWVIDEOWIFISTREAM // for wifi streaming

//#define VIDEORESULTS
//#define EXPORT

//#define SGM
#define LONGSEC // use Sjoerds improved sparse stereo algorithm

#define VIDEOFPS 10.0f // setting this lower than 5 will crash the drone...

//#define USE_TERMINAL_INPUT // using this disables running in background
#define USE_SOCKET
#endif // pc/drone

#endif //DEFINES_H

#include "filecam.h"
#ifdef FILECAM
#include <iostream>
#include <unistd.h> // sleep

#include <opencv2/contrib/contrib.hpp>



bool FileCam::init () {
	scaleFactor = 1;
//	video = cv::VideoCapture("/home/goggles/Desktop/cubicle_walk/LeftRight.avi");
	//video = cv::VideoCapture("/home/goggles/Desktop/LeftRight_glasstablemanualhfps.avi");
    //video = cv::VideoCapture("/home/houjebek/AfstudeerData/DroneCam/AutonomousFlightGroundtruth1/video_dsp.avi");
    //video = cv::VideoCapture("/home/houjebek/Desktop/video_walk_house.avi");
    //video = cv::VideoCapture("/home/houjebek/paparazzi/sw/airborne/modules/IC/IC_ext/drone/build/video_dsp.avi");
    //video = cv::VideoCapture("/home/houjebek/Desktop/turnlogs/15_07_24__15_04_00_foonfilmpie_10fps.avi");
    video = cv::VideoCapture("/home/houjebek/Desktop/turnlogs/15_07_25__14_24_30_halfwaycrash.avi");


    if (!video.isOpened()) {
        std::cerr << "Error opening video file!\n";
        return false;
    } else {
        im_width = (int) video.get(CV_CAP_PROP_FRAME_WIDTH)/2;
        im_height = (int)video.get(CV_CAP_PROP_FRAME_HEIGHT);
        nFrames = (int) video.get(CV_CAP_PROP_FRAME_COUNT);
        std::cout << "Opened filecam, nFrames: " << nFrames << std::endl;
        CurrentFrame=0;
        return true;
    }
}

void FileCam::start () {
    cams_are_running=true;
    thread_cam = std::thread(&FileCam::workerThread,this);    
    waitForImage();
    waitForImage();
    std::cout << "File opened!\n";
}

void FileCam::waitForImage() {

    g_lockWaitForImage1.unlock();
	g_lockWaitForImage2.lock();

}

void FileCam::close () {
    cams_are_running = false;
    g_lockWaitForImage1.unlock();
    g_lockWaitForImage1.unlock();
	thread_cam.join();    
    video.release();
}


void FileCam::splitIm(cv::Mat frameC, cv::Mat * frameL,cv::Mat * frameR ) {
    *frameL = cv::Mat(frameC,cv::Rect(0,0,frameC.cols/2,frameC.rows));
    *frameR = cv::Mat(frameC,cv::Rect(frameC.cols/2,0,frameC.cols/2,frameC.rows));
}

void FileCam::workerThread() {

    cv::Mat frameC = cv::Mat::zeros(im_height,im_width*2, CV_8UC1);
    cv::Mat frameL = cv::Mat::zeros(im_height,im_width, CV_8UC1);
    cv::Mat frameR = cv::Mat::zeros(im_height,im_width, CV_8UC1);
    stopWatch.Start();

	int skipstart = SKIPFRAMES_START;
    //skip start
    for (int i =0; i < skipstart;i++)
        video >> frameC;

    while (cams_are_running)  {

#ifdef HASSCREEN
        //arrange speed to be ~10fps
        float time = stopWatch.Read();
		time = 1000/(float)VIDEOFPS - time/1000;
        if (fastforward==0) {
            if (time > 0)  {usleep((int)time*1000);}
        }
        stopWatch.Restart();
#endif
        g_lockWaitForImage1.lock();


        if (rewind==1) {
         int id = video.get(CV_CAP_PROP_POS_FRAMES);
         if (id>0) {
             id-=2;
         }
         video.set(CV_CAP_PROP_POS_FRAMES,id);
        }

        video >> frameC;

        if (frameC.empty())
        {
            cams_are_running=false;
            g_lockWaitForImage2.unlock();
            break;
        }

        cvtColor(frameC,frameC,CV_RGB2GRAY,CV_8UC1);
        splitIm(frameC,&frameL,&frameR);

		cv::Point size(frameL.cols/scaleFactor,frameL.rows/scaleFactor);
		cv::resize(frameL,frameL_mat,size);


        frameL.copyTo(frameL_mat);
        frameR.copyTo(frameR_mat);


        g_lockWaitForImage2.unlock();

        CurrentFrame++;

    } // while loop


}

int FileCam::getImHeight() {
	return im_height/scaleFactor;
}

int FileCam::getImWidth() {
	return im_width/scaleFactor;
}
#endif

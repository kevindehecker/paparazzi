#include <string.h> 		/* memset */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "defines.h"
#include "exporter.h"
#include <sstream>

#ifdef EXPORT

void Exporter::write(int frameID, int avgdisp_gt, float avgdisp_est, int thresh_est, cv::Mat frame_currentHist) {
    std::stringstream s;
    s <<frameID << ", "<<  avgdisp_gt << ", " << avgdisp_est << ", " << thresh_est << ", " << frame_currentHist << std::endl;
    myfile << s.str();	

   // std::stringstream s2;
   // s2 << "/left" << saveid << ".png " << avgdisp_gt << std::endl;
  //  s2 << "/right" << saveid << ".png " << avgdisp_gt << std::endl;
  //  caffefile << s2.str();

}

void Exporter::init() {
    myfile.open ("offline_export.txt");
  //  caffefile.open("data.txt");
}

void Exporter::close() {
    myfile.close();
   // caffefile.close();
}


/*
 * Saves the current stereo image to png files
 */
void Exporter::saveStereoPair(cv::Mat frameL_mat,cv::Mat frameR_mat,cv::Mat DisparityMat) {
    char str[64];

    sprintf(str,"left%d.png", saveid);
    cv::imwrite( str, frameL_mat );
    sprintf(str,"right%d.png", saveid);
    cv::imwrite( str, frameR_mat );
    sprintf(str,"disp%d.png", saveid);
    cv::imwrite( str, DisparityMat);
    saveid++;
}

#endif

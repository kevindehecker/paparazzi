#ifndef Exporter_H
#define Exporter_H
#include "defines.h"
#ifdef EXPORT

#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>


/*
 * This class exports data to csv txt file, which may be imported in Matlab etc
 *
 */
class Exporter{
private:
    /*  Global variables  */
    std::ofstream myfile;
    std::ofstream caffefile;
    int saveid=1;

    /*  Function declarations  */


public:
    /*  Global variables  */


    /*  Function declarations  */
    void close();
    void init();
    void write(int frameID, int avgdisp_gt, float avgdisp_est, int thresh_est, cv::Mat frame_currentHist);
    void saveStereoPair(cv::Mat frameL_mat,cv::Mat frameR_mat,cv::Mat DisparityMat);
};
#endif // EXPORT
#endif  /*  Exporter_H  */


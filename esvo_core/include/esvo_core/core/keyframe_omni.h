// #ifndef INTENSITYFRAME_H_INCLUDED
// #define INTENSITYFRAME_H_INCLUDED

// #include <Eigen/Geometry>

// #include <ros/time.h>

// #include <opencv2/opencv.hpp>

// #include "utils.h"
// #include "point.h"

#include <visp/vpImage.h>
// #include <visp/vpImageIo.h>
#include <visp/vpImageFilter.h>
#include <visp/vpColVector.h>
// #include <visp/vpDisplayX.h>
////////////////////////////////////////////////////////////////////////////////

class KeyframeOmni
{
public:
    KeyframeOmni() {};
    KeyframeOmni(vpImage<unsigned char> I, vpImage<float> Idepth);
    void kfresize(double width, double height);
    //omni part
    // void displayInit(double width);
    // void displayImage(double width);

    vpImage<unsigned char> I;
    vpImage<float> Idepth;
    // vpDisplayX dI;
};

////////////////////////////////////////////////////////////////////////////////

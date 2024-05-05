#include "esvo_core/core/keyframe_omni.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

KeyframeOmni::KeyframeOmni(vpImage<unsigned char> I, vpImage<float> Idepth)
    : I(I), Idepth(Idepth)
{
}

void KeyframeOmni::kfresize(double width, double height)
{
    I.resize(height, width, false);
    Idepth.resize(height, width, false);
}


// void KeyframeOmni::displayInit(double width)
// {
//     // to display the current intensity image
//     dI.init(I, width, 50, "Photometric omnidirectional 3D tracking: s") ;

//       vpDisplay::display ( I ) ;
//       vpDisplay::flush ( I ) ;

      
// }
// void KeyframeOmni::displayImage(double width)
// {

//       vpDisplay::display ( I ) ;
//       vpDisplay::flush ( I ) ;

// }
#include "esvo_core/core/camera_omni.h"
#include <iostream>

using namespace std;

///////////////////////////////////////////////////////////////////////////////

CameraOmni::CameraOmni()
  : px(0), py(0),
  u0(0), v0(0),
  xi(0),
  height(0), width(0),
  pose({0,0,0,0,0,0}),
  clip_near(0), clip_far(0),
  k({})
{
}

///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

CameraOmni::CameraOmni(YAML::Node config)
  : px(0), py(0),
  u0(0), v0(0),
  xi(0),
  height(0), width(0),
  pose({0,0,0,0,0,0}),
  clip_near(0), clip_far(0),
  k({})
{
    if (config["cam0"]) {
        YAML::Node c = config["cam0"];

        // check a few things
        if (c["camera_model"].as<string>() != "omni") {
            std::cout << "ERROR: only Omni camera model is supported." << std::endl;
        }

        px = c["intrinsics"][0].as<double>();
        py = c["intrinsics"][2].as<double>();
        u0 = c["intrinsics"][1].as<double>();
        v0 = c["intrinsics"][3].as<double>();
        xi = c["intrinsics"][4].as<double>();

        k[0] = c["distortion"][0].as<double>();
        k[1] = c["distortion"][1].as<double>();
        k[2] = c["distortion"][2].as<double>();
        k[3] = c["distortion"][3].as<double>();
        k[4] = c["distortion"][4].as<double>();

        height = c["resolution"][1].as<double>();
        width  = c["resolution"][0].as<double>();

        pose[0] = c["pose"][0].as<double>();
        pose[1] = c["pose"][1].as<double>();
        pose[2] = c["pose"][2].as<double>();
        pose[3] = c["pose"][3].as<double>();
        pose[4] = c["pose"][4].as<double>();
        pose[5] = c["pose"][5].as<double>();
        pose[6] = c["pose"][6].as<double>();

        clip_near = c["clip"][0].as<float>();
        clip_far  = c["clip"][1].as<float>();

    } 
    else {
        throw "ERROR: Invalid camera intrinsics file. Please use correct format.";
    }
}

///////////////////////////////////////////////////////////////////////////////

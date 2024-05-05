#include "esvo_core/core/utils.h"

using namespace cv;
using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
/**
 *
 * see also https://docs.opencv.org/3.2.0/d2/d2c/tutorial_sobel_derivatives.html
 *
 * TODO: this function could probably be optimized quite a lot by not having to
 * create all these temporary matrices.
 */
void image_gradient(const cv::Mat &src, cv::Mat &dst)
{
    assert(!src.empty());

    // convert input to grayscale if necessary
    if (src.channels() == 3)
    {
        cvtColor(src, src, COLOR_BGR2GRAY);
    }
    else
    {
        assert(src.channels() == 1);
    }

    // reduce noice a bit
    // -> Sobel operator already includes a gaussian smoothing term
    // GaussianBlur(src_gray, src_gray, Size(3,3), 0, 0, BORDER_ISOLATED);

    Mat grad_x, grad_y;

#if CV_MAJOR_VERSION >= 4
#define CV_SCHARR FILTER_SCHARR
#endif

    // calculate actual gradient (using a normalized kernel)
    // normalization factor is sum of elements for the smoothing part (16 for SCHARR)
    // times 1/2 for the finite differences due to employing central differences
    // (or for the full kernel: sum(abs(elements))
    Sobel(src, grad_x, CV_64F, 1, 0, CV_SCHARR, 1 / 32.0f, 0, BORDER_REPLICATE);
    Sobel(src, grad_y, CV_64F, 0, 1, CV_SCHARR, 1 / 32.0f, 0, BORDER_REPLICATE);

    // merge gradients into single image with two channels
    Mat mats[2] = {grad_x, grad_y};
    // original
    merge(mats, 2, dst);

    // Tsuru added
    // merge(grad_x, 1, dst);
    // merge(grad_y, dst);
}

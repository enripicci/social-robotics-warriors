#ifndef _DATA_FITTING_H_
#define _DATA_FITTING_H_

#include <vector>
using namespace std;

// OpenCV Lib
#include <cv.h>
#include <cxcore.h>

vector < double > fit_data( CvSeq * data, CvPoint top_left, CvPoint bottom_right);

void paint_fitter( IplImage * img, vector < double > params, CvPoint top_left, CvPoint bottom_right, CvScalar color);


#endif // _DATA_FITTING_H_

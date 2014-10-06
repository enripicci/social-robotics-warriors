#ifndef __COLORTABLE_H__
#define __COLORTABLE_H__

#include <map>
#include <string>
using namespace std;

// OpenCV Lib
#include <cv.h>
#include <cxcore.h>

#include "kdtree.h"

//direct pixel access
#define PIXEL(_img,_x,_y) (&((uchar*)(_img->imageData+ _img->widthStep* ( (_y < 0)? 0: ((_y >= _img->height)? _img->height -1: _y)) ))[ ( (_x < 0)? 0: ((_x >= _img->width)? _img->width -1: _x)) *_img->nChannels])


typedef int ColorClass;

class ColorTable
{
public:
	ColorTable();
	virtual ~ColorTable();

	void clearSamples();
	void clearSpecifics();

	ColorClass getColorClass( const CvScalar color);
	bool addSample( const ColorClass colorClass, const CvScalar color);

	CvScalar getRGB( const ColorClass colorClass) {
		return cvScalar( colorTable[colorClass].b, colorTable[colorClass].g, colorTable[colorClass].r);
	}

//	bool load( const string filename);

	int segment( const IplImage * in, const IplImage * out, int d_x = 1, int d_y = 1);

	bool isSeg( const ColorClass c ) {
		return colorTable[c].seg;
	}

	void setColorClassSpecifics( ColorClass id, CvScalar color, bool seg = true)
	{
		ColorClassSpecifics c;
		c.seg = seg;
		c.b = (int) color.val[0];
		c.g = (int) color.val[1];
		c.r = (int) color.val[2];

		colorTable[id] = c;
	}

private:

	typedef struct
	{
		bool seg;
		int r, g, b;
	} ColorClassSpecifics;

	map< ColorClass, ColorClassSpecifics> colorTable;

	struct kdtree * tree;
};

#endif  // __COLORTABLE_H__

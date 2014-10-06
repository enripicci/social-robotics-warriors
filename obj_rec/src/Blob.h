#ifndef __BLOB_H__
#define __BLOB_H__

#include <map>
#include <vector>
#include <list>
using namespace std;

// OpenCV Lib
#include <cv.h>
#include <cxcore.h>

#include "ColorTable.h"

class Blob
{
public:

static list < Blob > extractBlobs( IplImage * image, ColorTable & colorTable, int d_x = 3, int d_y = 3);
static void drawBlobs( ColorTable & colorTable, list< Blob > & blobs, IplImage * img, long minArea = 1000);
static void deleteBlobs( list< Blob > & blobs);

	Blob( ColorClass _color);
	virtual ~Blob();

	void clear();

	CvPoint getTopLeft() const {
		return top_left;
	}

	CvPoint getBottomRight() const {
		return bottom_right;
	}

	CvPoint getAvg() {
		CvPoint avg;
		avg.x = (int) (w.x/(float)area);
		avg.y = (int) (w.y/(float)area);
		return avg;
	}

	long getArea() const {
		return area;
	}

	bool isNear( Blob & blob, const int d_x = 2, const int d_y = 2);

	CvSeq * getConvexHull();
	CvSeq * getEdges();

	ColorClass getColorClass() {
		return color;
	}

	bool operator== ( Blob blob);

private:
	typedef struct
	{
		int x, y;
		int len;
	} Run;

	ColorClass color;

	CvPoint top_left;
	CvPoint bottom_right;

	CvPoint w;
	long area;

	Blob( Run run, ColorClass _color, int d_y);
	bool merge( Run run, int d_y);
	Blob merge( Blob & blob, int d_y);

	vector < Run > runs;
};

#endif  //__BLOB_H__ 

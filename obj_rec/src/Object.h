#ifndef __OBJECT_H__
#define __OBJECT_H__

#include <set>
#include <list>
using namespace std;

// OpenCV Lib
#include <cv.h>
#include <cxcore.h>

#include "ColorTable.h"
#include "Blob.h"

class Object
{
public:

//
// Functions for object Lists...
//
static list < Object > extractObjects( list< Blob > & blobs, long minArea = 1000, int d_x = 5, int d_y = 5);

static void drawObjects( list < Object > & objects, IplImage * img, CvScalar color = cvScalar(0,0,0));

//
// Functions for recognizing objects in a list
//
typedef map< int, set< ColorClass > > ObjectsMemory;

static map < int, list < Object > > recognizeObjects(
								list < Object > & objs,
								map< int, set< ColorClass > > & objects_memory);

static Object * extractMaxAreaObject( list < Object > & objs );


	Object();
	virtual ~Object();

	void clear();

	CvPoint getTopLeft() const {
		return top_left;
	}

	CvPoint getBottomRight() const {
		return bottom_right;
	}

	CvPoint getAvg() const {
		return avg;
	}

	long getArea() const {
		return area;
	}

	bool isNear( Object & object, const int d_x = 4, const int d_y = 4);

	Object merge( Object & object);
	Object remove( Object & object);

	list < Object > divide( int m = 1);

	vector < double > fitSuperellipse();

	void draw( IplImage * img, CvScalar color);

	set < ColorClass > getColors() {
		set< ColorClass > colors;

		list< Blob >::iterator blob_it;
		for( blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++)
			colors.insert(blob_it->getColorClass());

		return colors;
	}

	int getNumOfBlobs() {
		return blobs.size();
	}

	vector < double > getEllipseParams()
	{
		if(ellipseParams.size() == 0)
			ellipseParams = fitSuperellipse();

		return ellipseParams;
	}

private:
	CvPoint top_left;
	CvPoint bottom_right;
	CvPoint avg;
	long area;

	vector < double > ellipseParams;


	Object( Blob & blob);
	bool merge( Blob & blob);

	list < Blob > blobs;
};

#endif  //__OBJECT_H__ 

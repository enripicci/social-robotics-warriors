#ifndef __OBJECT_H__
#define __OBJECT_H__

#include <set>
#include <list>
// OpenCV Lib
#include <cv.h>
#include <cxcore.h>

#include "ColorTable.h"
#include "Blob.h"

using namespace std;

/*  An Object is a collection of Blobs
*/
class Object {
    public:

        typedef map< int, set< ColorClass > > ObjectsMemory;

        /** Functions for object Lists...
        */
        /* Create a list of objects formed by near blobs
        */
        static list < Object > extractObjects( list< Blob > & blobs, long minArea = 1000, int d_x = 5, int d_y = 5);

        /* For every object in the list, draw its super-ellipse
        */
        static void drawObjects( list < Object > & objects, IplImage * img, CvScalar color = cvScalar(0,0,0));

        //
        // Functions for recognizing objects in a list
        //

        static map < int, list < Object > > recognizeObjects(
                                        list < Object > & objs,
                                        map< int, set< ColorClass > > & objects_memory);

        /* Return the object in the supplied list with greater area.
        */
        static Object * extractMaxAreaObject( list < Object > & objs );


        Object();
        virtual ~Object();

        /* Clear all the blobs data.
        */
        void clear();

        /** Bounding box functions
        **/
        inline CvPoint getTopLeft() const {  return top_left; }
        inline CvPoint getBottomRight() const { return bottom_right; }

        /* Bbox rectangle
        */
	    inline CvRect getBoundingBox() {
	        return cvRect(top_left.x, top_left.y, bottom_right.x - top_left.x, bottom_right.y - top_left.y);
	    }

        CvPoint getAvg() const { return avg; }
        long getArea() const { return area; }

        /* Check if the current object is near to the objects in the supplied list.
        *  Return true if yes, false if not.
        */
        bool isNear( Object & object, const int d_x = 4, const int d_y = 4);

        /* Add the blob to the object, and update the bounding box
        */
        Object merge( Object & object);

        /* Return a new object that contains the blobs of this object
        *  minus the blobs of the supplied object.
        */
        Object remove( Object & object);

        /*
        */
        list < Object > divide( int m = 1);

        /** Data fitting
        **/
        /* Compute the super-ellipse parameters.
        *  NOT IMPLEMENTED NOW.
        */
        vector < double > fitSuperellipse();

        /* Draw the fitting curve.
        *  NOT IMPLEMENTED NOW.
        */
        void drawFitter( IplImage * img, CvScalar color);

        /* Return the last computed super-ellipse parameters
        *
        *  TODO: automatically reset the old parameters when the object change
        */
        vector < double > getEllipseParams();

        /**
        **/
        /* Return the set of the colors id of the blobs contained in the object.
        */
        set < ColorClass > getColors();

        /* Get the numbers of blobs contained into the object.
        */
        int getNumOfBlobs() { return blobs.size(); }

    private:
        CvPoint top_left;
        CvPoint bottom_right;
        CvPoint avg;
        long area;
        list < Blob > blobs;
        vector < double > ellipseParams;    // Last computed super-ellipse parameters

        Object( Blob & blob);

        /* Add the blob to the object, and update the bounding box
        */
        bool merge( Blob & blob);

};

#endif  //__OBJECT_H__

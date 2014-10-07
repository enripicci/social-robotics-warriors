#ifndef __BLOB_H__
#define __BLOB_H__

#include <map>
#include <vector>
#include <list>
// OpenCV Lib
#include <cv.h>
#include <cxcore.h>

#include "ColorTable.h"

using namespace std;

/*
*/
class Blob {
    public:
        /* Create a blob that track pixels with color of class colorId
        */
        Blob( ColorClass colorId);
	    virtual ~Blob();
        
        /* Extract the blobs from the image image, according to the colorId rules
        *  defined in colorTable.
        *  The image's pixels are evaluated every d_x pixel along x and every d_y along y.
        */
        static list < Blob > extractBlobs( IplImage * image, ColorTable & colorTable, int d_x = 3, int d_y = 3);
        
        /* Draw the ConvexHull of every blobs in the list and their center into the image img.
        *  Blobs with area lesser than minArea are excluded.
        */
        static void drawBlobs( ColorTable & colorTable, list< Blob > & blobs, IplImage * img, long minArea = 1000);
        
        /* Call clear() on every blob in the supplied list.
        *  The blob list is freed after use.
        */
        static void deleteBlobs( list< Blob > & blobs);
        
        /* Clear all the runs in the blob.
        */
	    void clear();
        
        /**    Bounding box
        **/
        /* Top-left point of bbox
        */
	    inline CvPoint getTopLeft() const { return top_left; }
	    
	    /* Bottom-right point of bbox
        */
	    inline CvPoint getBottomRight() const { return bottom_right; }
	    
	    /* Bbox rectangle
        */
	    inline CvRect getBoundingBox() {
	        return cvRect(top_left.x, top_left.y, bottom_right.x - top_left.x, bottom_right.y - top_left.y);
	    }

	    CvPoint getAvg();

	    long getArea() const { return area; }
        
        /* Return true if blob has some (1 or more) runs that share a border or
        *  overlap.
        *  d_x and d_y are the pixel of tolerance along X and Y.
        */
	    bool isNear( Blob & blob, const int d_x = 2, const int d_y = 2);
        
        /* Convex hull
        */
	    CvSeq * getConvexHull();
	    
	    /* Create a sequence of extremity points
	    */
	    CvSeq * getEdges();
        
        /* ColorId tracked in the blob
        */
	    inline ColorClass getColorClass() const { return color; }
        
        /* Two blobs are equal if they track the same colorId
        */
	    bool operator== ( const Blob &blob);

    private:
        
        /* A Run is an horizontal line of pixels with same attributes, such as
        *  color, like in this case.
        */
	    typedef struct {
		    int x, y;
		    int len;
	    } Run;

	    ColorClass color;
	    CvPoint top_left;
	    CvPoint bottom_right;
	    CvPoint w;
	    long area;
        
        /* Create a Blob and initialize it with the supplied run
        */
	    Blob( Run run, ColorClass _color, int d_y);
	    
	    /* Add a new run.
	    *  This call can change the bounding box.
	    *  Return true on success.
	    */
	    bool merge( Run run, int d_y);
	    
	    /* Merge all the Runs of blob with the current one.
	    */
	    Blob merge( Blob & blob, int d_y);

	    vector < Run > runs;
};

#endif  //__BLOB_H__ 

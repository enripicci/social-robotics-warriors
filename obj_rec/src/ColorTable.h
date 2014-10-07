#ifndef __COLORTABLE_H__
#define __COLORTABLE_H__

#include <map>
#include <string>
// OpenCV Lib
#include <cv.h>
#include <cxcore.h>

#include "kdtree.h"

using namespace std;

// id associated to a sample
typedef int ColorClass;

/* This class assolve to two works:
*    1) Store the color samples, indexing them with an id.
*       The value zero is reserved, and the uni of the ids is left to the user.
*    2) Store the color map used to map the samples ids into quantized colors.
*/
class ColorTable {
    public:
	    ColorTable();
	    virtual ~ColorTable();
	    
	    /**   Samples manager
	    **/
        /* Return the ColorClass that suite better the supplied color.
        *  radius is the treshold inside which the research is done.
        *  Return 0 on error, the color class otherwise
        */
        /// TODO: check if the return value for error is usable for that scope
	    ColorClass getColorClass( const CvScalar color, double radius = 10.0);
	    
	    /* Add a new sample with id colorClass and color color, RGB coded.
	    *  Return false on error, true otherwise.
	    */
	    bool addSample( const ColorClass colorClass, const CvScalar color);
	    
	    /* Clear all the samples, and delete them from memory
	    */
	    void clearSamples();
	    
	    /**   Color map manager
	    **/
        /* Map the sample with specified id to the specified color.
        */
	    void setColorClassSpecifics( ColorClass id, CvScalar color, bool seg = true);
	    
	    /* Clear the color map
	    */
	    void clearSpecifics();
	    
	    /* Take the pixel of the input in image (RGB encoded), apply the color map
        *  and write the same pixels in the output image out.
        *  The output image must be of the same size of the input image.
        *  The parameters d_x and d_y permit to evaluate the input image pixels every
        *  d_x pixels along the x dimension and d_y pixels along y direction.
        *  Return 0 if it fails, non-zero otherwise.
        */
	    int segment( const IplImage * in, const IplImage * out, int d_x = 1, int d_y = 1);
	    
	    /* 
	    */
	    inline CvScalar getRGB( const ColorClass c) { 
	        return cvScalar(colorTable[c].b, colorTable[c].g, colorTable[c].r);
	    }
	    
	    /*
	    */
	    inline bool isSeg( const ColorClass c ) { return colorTable[c].seg; }
	    
	    /* Load a color map from a file.
	    */
	//  bool readColorClassSpecifics( const string filename, unsigned int mergemod);
    //  bool readColorClassSpecifics( const char * filename, unsigned int mergemod);
        /* Save the color map into the specified file
        */
    //  int writeColorClassSpecifics( const string filename);
    //  int writeColorClassSpecifics( const char * filename);

    private:

	    typedef struct {
		    bool seg;
		    int r, g, b;    // R,G,B color values
	    } ColorClassSpecifics;
        
        /* Map the ColorClass ids to the ColorClassSpecifics
        */
	    map< ColorClass, ColorClassSpecifics> colorTable;
        /* Tree that keep the sequence of samples.
        */
	    struct kdtree * tree;
};

#endif  // __COLORTABLE_H__

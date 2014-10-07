#include <cstdlib>  // ensure we have malloc
#include "ColorTable.h"

using namespace std;

// direct access to pixel (_x,_y) of image _img
#define PIXEL(_img,_x,_y)   (                                                       \
    &((uchar*)(_img->imageData + _img->widthStep *                                  \
            ( (_y < 0) ? (0) : ( (_y >= _img->height) ? (_img->height-1) : (_y) ) ) \
    )) [ ( (_x < 0)? (0) : ( (_x >= _img->width) ?( _img->width -1) : (_x))) * _img->nChannels ]    \
  )


/* Given two vectors a1 and a2, each of dimension dims,
*  return the 
*/
static double dist_sq( double *a1, double *a2, int dims ) {
  double dist = 0, diff;
  for(--dims; dims>=0; dims--) {
    diff = a1[dims] - a2[dims];
    dist += diff*diff;
  }
  return dist;
}


ColorTable::ColorTable() {
	// Let's create the tree
	tree = kd_create(3);
	kd_data_destructor( tree, free);
}


ColorTable::~ColorTable() {
	colorTable.clear();

	kd_clear(tree);
	kd_free(tree);
	tree = 0;
}

ColorClass ColorTable::getColorClass( CvScalar color, double radius) {

	ColorClass retVal = 0;
	ColorClass *c;
	double dist;
	double pos[3];

	/* find points closest to the origin and within distance radius */
	struct kdres * presults = kd_nearest_range( tree, color.val, radius);
	
	if(NULL==presults) {
        // something went wrong
        /// TODO: return 0 is a good error value?
        return 0;
    }
    
    
    // iterate all the results
	while( NULL != kd_res_end( presults ) ) {
		// get the data and position of the current result item. c should never be NULL
		c = (ColorClass *) kd_res_item( presults, pos);
		
		// compute the distance of the current result from the pt
		dist = sqrt( dist_sq( color.val, pos, 3 ) );

		if( dist <= radius) {
			retVal = *c;
			radius = dist;
		}
		
		// go to the next entry
		kd_res_next( presults );
	}

	kd_res_free(presults);

	return retVal;
}

ColorClass ColorTable::getColorClass( const IplImage * in, int x, int y, double radius = 10.0 ) {
    uchar *ptr = PIXEL(in,x,y);
    return getColorClass( cvScalar( ptr[0], ptr[1], ptr[2]), radius );
}


bool ColorTable::addSample( ColorClass colorClass, CvScalar color) {
    int res;
    // malloc is required because the deallocator of the tree is free()
	ColorClass *c = static_cast<ColorClass*>( malloc(sizeof(ColorClass)) );
	
	if(NULL==c)
	    return false;
	    
	*c = colorClass;
	
	// Insert the color class, ordered by color value
	res = kd_insert( tree, color.val, c);
	return (0==res)?(true):(false);
}

void ColorTable::clearSamples() {
	kd_clear(tree);
}


void ColorTable::setColorClassSpecifics( ColorClass id, CvScalar color, bool seg) {
    ColorClassSpecifics c;
    c.seg = seg;
    c.b = (int) color.val[0];
    c.g = (int) color.val[1];
    c.r = (int) color.val[2];

    colorTable[id] = c;
}

void ColorTable::clearSpecifics() {
	colorTable.clear();
}

/*
bool ColorTable::load( string filename, unsigned int mergemod)
{
	//colorTable.clear();

	FILE * fp = fopen(filename.c_str(), "r");
	if(!fp) return false;

	while(!feof(fp))
	{
		char * str;
		size_t size = 0;
		getline(&str, &size, fp);
		if(str[0] == '#')
			continue;

		ColorClass id;
		ColorClassSpecifics specs;

		char seg = 'y';

		sscanf(str, "%d %d,%d,%d %d,%d %d,%d %d,%d %c",
			&id, &(specs.r), &(specs.g), &(specs.b),
			&(specs.hl), &(specs.hr),
			&(specs.sl), &(specs.sr),
			&(specs.vl), &(specs.vr), &(seg) );

		specs.seg = (seg=='y')?true:false;

		printf(str);

		colorTable[id] = specs;
	}
	fclose(fp);

	printf("\nReaded ColorTable");
	printf("\n");

	return true;
}

bool loadTable(const char* filename, unsigned int mergemod) {

}
*/

int ColorTable::segment( const IplImage * in, const IplImage * out, int d_x, int d_y) {
    register int i;
    register int j;
    uchar *pIn, *pOut;
    ColorClass c;
    ColorClassSpecifics cSpec;
//	IplImage * hsv = cvCreateImage( cvSize(in->width, in->height), in->depth, in->nChannels);
//	cvCvtColor( in, hsv, CV_BGR2HSV);

	for(i=0; i < in->width; i+=d_x) {
	    for(j=0; j < in->height; j+=d_y) {
		    pIn = PIXEL( in, i, j);
		    pOut = PIXEL( out, i, j);

		    c = getColorClass(cvScalar(pIn[0], pIn[1], pIn[2]));
		    
		    /// TODO: check the return value of c
		    cSpec = colorTable[c];

		    pOut[0] = cSpec.b;
		    pOut[1] = cSpec.g;
		    pOut[2] = cSpec.r;
		}
    }

//	cvReleaseImage(&hsv);

	return 1;
}

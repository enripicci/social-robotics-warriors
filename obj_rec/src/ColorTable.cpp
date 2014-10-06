#include "ColorTable.h"

double dist_sq( double *a1, double *a2, int dims ) {
  double dist_sq = 0, diff;
  while( --dims >= 0 ) {
    diff = (a1[dims] - a2[dims]);
    dist_sq += diff*diff;
  }
  return dist_sq;
}


ColorTable::ColorTable()
{
	// Let's create the tree
	tree = kd_create(3);

	kd_data_destructor( tree, free);
}


ColorTable::~ColorTable()
{
	colorTable.clear();

	kd_clear(tree);
	kd_free(tree);
	tree = 0;
}

void ColorTable::clearSamples()
{
	kd_clear(tree);
}

void ColorTable::clearSpecifics()
{
	colorTable.clear();
}


ColorClass ColorTable::getColorClass( CvScalar color)
{
	double radius = 10.0;

	ColorClass retVal = 0;

	/* find points closest to the origin and within distance radius */
	struct kdres * presults = kd_nearest_range( tree, color.val, radius);

	while( !kd_res_end( presults ) )
	{
		double pos[3];

		// get the data and position of the current result item
		ColorClass * c = (ColorClass *) kd_res_item( presults, pos);
		
		// compute the distance of the current result from the pt
		double dist = sqrt( dist_sq( color.val, pos, 3 ) );

		if( dist <= radius)
		{
			retVal = *c;
			radius = dist;
		}
		
		// go to the next entry
		kd_res_next( presults );
	}

	kd_res_free(presults);

	return retVal;
}


bool ColorTable::addSample( ColorClass colorClass, CvScalar color)
{
	ColorClass *c = new ColorClass; *c = colorClass;
	kd_insert( tree, color.val, c);
	return true;
}


/*
bool ColorTable::load( string filename)
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
*/

int ColorTable::segment( const IplImage * in, const IplImage * out, int d_x, int d_y)
{
//	IplImage * hsv = cvCreateImage( cvSize(in->width, in->height), in->depth, in->nChannels);
//	cvCvtColor( in, hsv, CV_BGR2HSV);

	for( int i = 0; i < in->width; i += d_x)
	for( int j = 0; j < in->height; j += d_y)
	{
		uchar * pIn = PIXEL( in, i, j);
		uchar * pOut = PIXEL( out, i, j);

		ColorClass c = getColorClass(cvScalar(pIn[0], pIn[1], pIn[2]));

		pOut[0] = colorTable[c].b;
		pOut[1] = colorTable[c].g;
		pOut[2] = colorTable[c].r;
	}

//	cvReleaseImage(&hsv);

	return 1;
}

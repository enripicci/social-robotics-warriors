#include "Blob.h"
#include <cmath>

Blob::Blob( ColorClass _color ) {
    color = _color;
	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	w = cvPoint( 0, 0);
	area = 0;
}

Blob::Blob( Run run, ColorClass _color, int d_y) {
    color = _color;
	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	w = cvPoint( 0, 0);
	area = 0;

	this->merge( run, d_y);
}


Blob::~Blob() {
	clear();
}


void Blob::clear() {
	runs.clear();

	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	w = cvPoint( 0, 0);
	area = 0;
}


bool Blob::isNear( Blob & blob, const int d_x, const int d_y) {
    register int i;
    register int j;
    int diff_left, diff_right;
    
	for( i = 0; i < (int)runs.size(); i++) {
	    for( j = 0; j < (int) blob.runs.size(); j++) {
		    if( fabs(runs[i].y - blob.runs[j].y) < d_y) {
		        
		        diff_left = (runs[i].x - d_x) - (blob.runs[j].x + blob.runs[j].len);
		        diff_right = (runs[i].x + runs[i].len + d_x) - (blob.runs[j].x);
		        
		        if((diff_left * diff_right) < 0)
		            return true;
		        return false;
		        
		    /*
		        if(( runs[i].x -d_x <= blob.runs[j].x) &&
				    ( runs[i].x + runs[i].len + d_x >= blob.runs[j].x) ) return true;

			    if(( this->runs[i].x -d_x <= blob.runs[j].x + blob.runs[j].len ) &&
				    ( this->runs[i].x + this->runs[i].len + d_x >= blob.runs[j].x + blob.runs[j].len) ) return true;

			    if(( blob.runs[j].x -d_x <= this->runs[i].x) &&
				    ( blob.runs[j].x + blob.runs[j].len + d_x >= this->runs[i].x) ) return true;

			    if(( blob.runs[j].x -d_x <= this->runs[i].x + this->runs[i].len ) &&
				    ( blob.runs[j].x + blob.runs[j].len + d_x >= this->runs[i].x + this->runs[i].len) ) return true;
            */
		    }
	    }
	}

	return false;
}


Blob Blob::merge( Blob & blob, int d_y) {
	vector <Run>::iterator it;
	for( it = blob.runs.begin(); it != blob.runs.end(); it++)
		this->merge(*it, d_y);

	return *this;
}


/// FIXME: what to do if two runs lay on the same line (same y coordinate) ?
///        They should be merged into an unique run?
bool Blob::merge( Run run, int d_y) {
    register int i;

	runs.push_back(run);
	
    if( false == runs.empty() ) {
        // update the bbox
	    if( run.x < top_left.x)
		    top_left.x = run.x;

	    if( run.y < top_left.y)
		    top_left.y = run.y;

	    if( (run.x + run.len) > bottom_right.x )
		    bottom_right.x = run.x + run.len;

	    if( run.y > bottom_right.y)
		    bottom_right.y = run.y;
    } else {
        // just set the bbox
        top_left.x = run.x;
        top_left.y = bottom_right.y = run.y;
        bottom_right.x = run.x + run.len;
    }

	for( i = 0; i < run.len; i++)
		w.x += (run.x + i) *d_y;

	w.y += run.y * run.len * d_y;

	area += run.len * d_y;

	return true;
}


CvPoint Blob::getAvg() {
    CvPoint avg;
    avg.x = (int) (w.x/(float)area);
    avg.y = (int) (w.y/(float)area);
    return avg;
}


CvSeq * Blob::getEdges() {
	CvSeq* ptseq = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, 
	                            sizeof(CvContour),
							    sizeof(CvPoint),
							    cvCreateMemStorage() ); 
	CvPoint pt;
	
	for( vector <Run>::iterator it = runs.begin(); it != runs.end(); it++) {
	    // left extreme
		pt.x = it->x;
		pt.y = it->y;
		cvSeqPush( ptseq, &pt );
		
		// right extreme
		pt.x += it->len;
		cvSeqPush( ptseq, &pt );
	}

	return ptseq;
}


CvSeq * Blob::getConvexHull() {
	CvSeq* ptseq = getEdges();

	CvSeq * hull = cvConvexHull2( ptseq, cvCreateMemStorage(), CV_CLOCKWISE, 0 );

	cvReleaseMemStorage( &(ptseq->storage) );

	return hull;
}


bool Blob::operator== ( const Blob &blob) {
	if( (blob.color != this->color) )
        return false;

/*
	vector < Run  >::iterator it1;
	vector < Run  >::iterator it2;
	for( it1 = runs.begin(); it1 != runs.end(); it1++)
	for( it2 = blob.runs.begin(); it2 != blob.runs.end(); it2++)
		if( (it1->y != it2->y) || (it1->x != it2->x) || (it1->len != it2->len) )
			return false;
*/
	return true;
}


list < Blob > Blob::extractBlobs( IplImage * image, ColorTable & colorTable, int d_x, int d_y) {
	list < Blob > blobs;
	list < Blob >::reverse_iterator it; // use reverse iterator for improve efficiency
	Run run;
	register int y;
	register int x;
	ColorClass c;

	for( y = 0; y < image->height; y += d_y) {
	    for( x = 0; x < image->width; x += d_x) {
	    
		    c = colorTable.getColorClass(image,x,y);

		    if( true == colorTable.isSeg(c) ) {
			    // Create the run
			    run.x = x;
			    run.y = y;
			    run.len = 0;
			    
			    for( x+=d_x ; (x < image->width) && (c == colorTable.getColorClass(image,x,y)) ; x+=d_x, run.len+=d_x) {
			        // nothing to do here.
			    }

			    // Create a blob and insert it into the list. It can be accessed with blobs.back() method.
			    Blob tmp( run, c, d_y + d_y /2);
			    blobs.push_back(tmp);

			    for( it = (blobs.rbegin() + 1); it != blobs.rend(); it++) {
				    // merge near blobs
				    if( (it->getColorClass() == c) && ( it->isNear(blobs.back(),d_x + d_x /2,d_y + d_y /2))) {
					    it->merge(blobs.back(), d_y + d_y /2);
					    blobs.pop_back();
					    break;
				    }
			    }
		    }
	    }
	}

	return blobs;
}


void Blob::drawBlobs( ColorTable & colorTable, list< Blob > & blobs, IplImage * img, long minArea) {
    CvSeq * hull
    CvPoint pt0, pt1;
	list< Blob >::iterator b_it;
	
	for( b_it = blobs.begin(); b_it != blobs.end(); b_it++) {

		if(b_it->getArea() > minArea) {
			hull = b_it->getConvexHull();

			for( int i = 0; i < hull->total; i++ ) {
				pt0 = **CV_GET_SEQ_ELEM( CvPoint *, hull, i );
				pt1 = **CV_GET_SEQ_ELEM( CvPoint *, hull, (i+1)%(hull->total) );
				
				// paint something
				cvCircle( img, pt0, 2, colorTable.getRGB(b.color));
				cvLine( img, pt0, pt1, colorTable.getRGB(b.color), 1);
			}
			cvReleaseMemStorage( &(hull->storage) );

			CvPoint avg = b.getAvg();
			cvCircle( img, avg, 2, cvScalar(0xff,0xff,0xff), 0, 0 );
		}
	}
}


void Blob::deleteBlobs( list< Blob > & blobs) {
	list< Blob >::iterator b_it;
	for( b_it = blobs.begin(); b_it != blobs.end(); b_it++) {
		b_it->clear();
	}

	blobs.clear();  // free the list
}

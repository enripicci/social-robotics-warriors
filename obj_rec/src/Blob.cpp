#include "Blob.h"

//direct pixel access
#define PIXEL(_img,_x,_y) (&((uchar*)(_img->imageData+ _img->widthStep* ( (_y < 0)? 0: ((_y >= _img->height)? _img->height -1: _y)) ))[ ( (_x < 0)? 0: ((_x >= _img->width)? _img->width -1: _x)) *_img->nChannels])


Blob::Blob( ColorClass _color )
{
	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	w = cvPoint( 0, 0);
	area = 0;
	color = _color;
}


Blob::Blob( Run run, ColorClass _color, int d_y)
{
	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	w = cvPoint( 0, 0);
	area = 0;
	color = _color;

	this->merge( run, d_y);
}



Blob::~Blob()
{
	clear();
}


void Blob::clear()
{
	runs.clear();

	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	w = cvPoint( 0, 0);
	area = 0;
}


bool Blob::isNear( Blob & blob, const int d_x, const int d_y)
{
	for( int i = 0; i < (int) this->runs.size(); i++)
	for( int j = 0; j < (int) blob.runs.size(); j++)
	{
		if(fabs(this->runs[i].y - blob.runs[j].y) < d_y)
		{
			if(( this->runs[i].x -d_x <= blob.runs[j].x) &&
				( this->runs[i].x + this->runs[i].len + d_x >= blob.runs[j].x) ) return true;

			if(( this->runs[i].x -d_x <= blob.runs[j].x + blob.runs[j].len ) &&
				( this->runs[i].x + this->runs[i].len + d_x >= blob.runs[j].x + blob.runs[j].len) ) return true;

			if(( blob.runs[j].x -d_x <= this->runs[i].x) &&
				( blob.runs[j].x + blob.runs[j].len + d_x >= this->runs[i].x) ) return true;

			if(( blob.runs[j].x -d_x <= this->runs[i].x + this->runs[i].len ) &&
				( blob.runs[j].x + blob.runs[j].len + d_x >= this->runs[i].x + this->runs[i].len) ) return true;
		}
	}

	return false;
}


Blob Blob::merge( Blob & blob, int d_y)
{

	vector < Run >::iterator it;
	for( it = blob.runs.begin(); it != blob.runs.end(); it++)
		this->merge(*it, d_y);

	return *this;
}


bool Blob::merge( Run run, int d_y)
{
	runs.push_back(run);

	if( run.x < top_left.x)
		top_left.x = run.x;

	if( run.y < top_left.y)
		top_left.y = run.y;

	if( (run.x + run.len) > bottom_right.x )
		bottom_right.x = run.x + run.len;

	if( run.y > bottom_right.y)
		bottom_right.y = run.y;



	for( int i = 0; i < run.len; i++)
		w.x += (run.x + i) *d_y;

	w.y += run.y * run.len * d_y;

	area += run.len * d_y;

	return true;
}


CvSeq * Blob::getEdges()
{
	CvSeq* ptseq = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour),
							sizeof(CvPoint), cvCreateMemStorage() ); 

	vector < Run >::iterator it;
	for( it = runs.begin(); it != runs.end(); it++)
	{
		CvPoint pt0;
		pt0.x = (*it).x;
		pt0.y = (*it).y;
		cvSeqPush( ptseq, &pt0 );

		CvPoint pt1;
		pt1.x = (*it).x + (*it).len;
		pt1.y = (*it).y;
		cvSeqPush( ptseq, &pt1 );
	}

	return ptseq;
}


CvSeq * Blob::getConvexHull()
{
	CvSeq* ptseq = getEdges();

	CvSeq * hull = cvConvexHull2( ptseq, cvCreateMemStorage(), CV_CLOCKWISE, 0 );

	cvReleaseMemStorage( &(ptseq->storage) );

	return hull;
}


bool Blob::operator== ( Blob blob)
{
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


list < Blob > Blob::extractBlobs( IplImage * image, ColorTable & colorTable, int d_x, int d_y)
{
	list < Blob > blobs;

	for( int y = 0; y < image->height; y += d_y)
	for( int x = 0; x < image->width; x += d_x)
	{
		ColorClass c = colorTable.getColorClass( cvScalar( PIXEL(image,x,y)[0], PIXEL(image,x,y)[1], PIXEL(image,x,y)[2]) );

		if(colorTable.isSeg(c))
		{
			//Creiamo la run
			Run run;
			run.x = x;
			run.y = y;
			run.len = 0;
	
			x+=d_x;
			while( (c == colorTable.getColorClass( cvScalar( PIXEL(image,x,y)[0], PIXEL(image,x,y)[1], PIXEL(image,x,y)[2]) ) ) )
			{
				run.len += d_x;
				if(x+d_x < image->width) x+=d_x;
				else break;
			}

			//facciamo un Blob con quella run
			Blob tmp( run, c, d_y + d_y /2);
			blobs.push_front( tmp);
			list < Blob >::iterator p_it = blobs.begin();

			//vicinanza con gli altri blob dello stesso colore
			list < Blob >::iterator it;
			for( it = blobs.begin(); it != blobs.end(); it++)
			{
				//se c'è un blob vicino..
				if( ((*it).getColorClass() == c) && ( (*it).isNear( *p_it, d_x + d_x /2, d_y + d_y /2)))
				{
					if(it != p_it)
					{
						(*it).merge(*p_it, d_y + d_y /2);
						blobs.erase(p_it);
						p_it = it;
					}
				}
			}
		}
	}

	return blobs;
}


void Blob::drawBlobs( ColorTable & colorTable, list< Blob > & blobs, IplImage * img, long minArea)
{
	list< Blob >::iterator b_it;
	for( b_it = blobs.begin(); b_it != blobs.end(); b_it++)
	{
		Blob b = *b_it;

		if(b.getArea() > minArea)
		{
			CvSeq * hull = b.getConvexHull();

			for( int i = 0; i < hull->total; i++ )
			{
				CvPoint pt0, pt1;
				pt0 = **CV_GET_SEQ_ELEM( CvPoint *, hull, i );
				pt1 = **CV_GET_SEQ_ELEM( CvPoint *, hull, (i+1)%(hull->total) );
				cvCircle( img, pt0, 2, colorTable.getRGB(b.color));
				cvLine( img, pt0, pt1, colorTable.getRGB(b.color), 1);
			}
			cvReleaseMemStorage( &(hull->storage) );

			CvPoint avg = b.getAvg();
			cvCircle( img, avg, 2, cvScalar(0xff,0xff,0xff), 0, 0 );
		}
	}
}


void Blob::deleteBlobs( list< Blob > & blobs)
{
	list< Blob >::iterator b_it;
	for( b_it = blobs.begin(); b_it != blobs.end(); b_it++)
	{
		Blob b = *b_it;

		b.clear();
	}

	blobs.clear();
}

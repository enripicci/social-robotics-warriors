#include "Object.h"

#include "DataFitting.h"


Object::Object()
{
	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	avg = cvPoint( 0, 0);
	area = 0;
}


Object::Object( Blob & blob)
{
	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	avg = cvPoint( 0, 0);
	area = 0;

	this->merge( blob);
}



Object::~Object()
{
	list < Blob >::iterator i_it;
	for( i_it = this->blobs.begin(); i_it != this->blobs.end(); i_it++)
		i_it->clear();
}


void Object::clear()
{
	blobs.clear();

	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	avg = cvPoint( 0, 0);
	area = 0;
}


bool Object::isNear( Object & object, const int d_x, const int d_y)
{
	list < Blob >::iterator i_it;
	for( i_it = this->blobs.begin(); i_it != this->blobs.end(); i_it++)
	{
		list < Blob >::iterator j_it;
		for( j_it = object.blobs.begin(); j_it != object.blobs.end(); j_it++)
			if( (*i_it).isNear( (*j_it), d_x, d_y ) ) return true;
	}
	return false;
}


Object Object::merge( Object & object)
{
	list < Blob >::iterator it;
	for( it = object.blobs.begin(); it != object.blobs.end(); it++)
		this->merge(*it);

	return *this;
}


bool Object::merge( Blob & blob)
{
	blobs.push_back(blob);

	if( blob.getTopLeft().x < top_left.x)
		top_left.x = blob.getTopLeft().x;

	if( blob.getTopLeft().y < top_left.y)
		top_left.y = blob.getTopLeft().y;

	if( blob.getBottomRight().x > bottom_right.x )
		bottom_right.x = blob.getBottomRight().x;

	if( blob.getBottomRight().y > bottom_right.y)
		bottom_right.y = blob.getBottomRight().y;

	area += blob.getArea();

	return true;
}


vector< double > Object::fitSuperellipse()
{
	vector < double > params;

	CvSize axes = cvSize( fabs(top_left.x - bottom_right.x)/2.0,  fabs(top_left.y - bottom_right.y)/2.0);

//	cvRectangle( img, top_left, bottom_right, cvScalar(0xff,0xff,0xff), 2);

//	CvPoint avg;
	avg.x = fabs(bottom_right.x + top_left.x) /2.0;
	avg.y = fabs(bottom_right.y + top_left.y) /2.0;


	CvSeq * edges = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour),
							sizeof(CvPoint), cvCreateMemStorage() ); 

	list< Blob >::iterator blob_it;
	for( blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++)
	{
		CvSeq * blob_hull = blob_it->getEdges();

		for( int i = 0; i < blob_hull->total; i++ )
		{
			CvPoint pt0;
			pt0 = *CV_GET_SEQ_ELEM( CvPoint, blob_hull, i );
			cvSeqPush( edges, &pt0 );
		}

		cvReleaseMemStorage( &(blob_hull->storage) );
	}

	CvSeq * hull = cvConvexHull2( edges, cvCreateMemStorage(), CV_CLOCKWISE, 0 );


	params = fit_data( hull, top_left, bottom_right);

	cvReleaseMemStorage( &(hull->storage) );
	cvReleaseMemStorage( &(edges->storage) );

	return params;
}


void Object::draw( IplImage * img, CvScalar color)
{
	vector < double > params = getEllipseParams();
	paint_fitter( img, params, top_left, bottom_right, color);
}


list < Object > Object::divide( int m)
{
	list <Object > retVal;

	/*if( N >= object.colors.size())
		retVal.push_back(object);
	*/

	int N = pow(2, blobs.size());

	for(unsigned i = 1; i < N-1; i++)
	{
		unsigned byte = 0x01;
		unsigned bit = 0;
		Object obj;

		int n = 0;

		list< Blob >::iterator blob_it;
		for( blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++)
		{
			if(byte & i)
			{
				obj.merge(*blob_it);
				n++;
			}

			byte <<= 1;
			bit++;
		}

		if( n == blobs.size() -m)
			retVal.push_back(obj);
	}


	return retVal;
}

Object Object::remove( Object & object)
{
	Object o;
	list < Blob >::iterator it;
	for( it = blobs.begin(); it != blobs.end(); it++)
	{
		bool find = false;
		list < Blob >::iterator b_it;
		for( b_it = object.blobs.begin(); b_it != object.blobs.end(); b_it++)
		{
			if((*b_it) == (*it))
			{
				find = true;
				break;
			}
		}

		if(!find) o.merge(*it);
	}

	return o;
}


map < int, list < Object > > Object::recognizeObjects( list < Object > & objs, map < int, set < ColorClass > > & objects_memory)
{
	map < int, list< Object > > retVal;

	list < Object >::iterator obj_it;
	for( obj_it = objs.begin(); obj_it != objs.end(); obj_it++)
	{
		bool rec = false;

		map< int, set< ColorClass > >::iterator feat_it;
		for( feat_it = objects_memory.begin(); feat_it != objects_memory.end(); feat_it++)
			if(feat_it->second == obj_it->getColors())
			{
				retVal[feat_it->first].push_back(*obj_it);
				rec = true;
				break;
			}

		if((!rec) && (obj_it->getColors().size() >1))
		{
			list < Object > sub_objs = (*obj_it).divide();

			list < Object >::iterator sub_obj_it;
			for( sub_obj_it = sub_objs.begin(); (sub_obj_it != sub_objs.end()); sub_obj_it++)
			{
				map< int, set< ColorClass > >::iterator feat_it;
				for( feat_it = objects_memory.begin(); feat_it != objects_memory.end(); feat_it++)
					if(feat_it->second == sub_obj_it->getColors())
					{
						retVal[feat_it->first].push_front(*sub_obj_it); // + piccolo

						objs.push_back((*obj_it).remove(*sub_obj_it));

						rec = true;
						break;
					}
				if(!rec)
				{
					objs.push_back(*sub_obj_it);
				}
			}

		}
	}

	return retVal;
}


Object * Object::extractMaxAreaObject( list < Object > & objs )
{
	long max_area = -1;
	list < Object >::iterator obj_it, max_area_obj_it;
	for( obj_it = objs.begin(); obj_it != objs.end(); obj_it++)
		if(obj_it->getArea() > max_area)
		{
			max_area = obj_it->getArea();
			max_area_obj_it = obj_it;
		}

	if(max_area != -1)
	{
		return &(*max_area_obj_it);
	}
	else return 0;
}


list < Object > Object::extractObjects( list< Blob > & blobs, long minArea, int d_x, int d_y)
{
	list < Object > objects;

	list< Blob >::iterator blob_it;
	for( blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++)
	{
		if( (*blob_it).getArea() > minArea)
		{
			//facciamo un Object con quel blob
			Object tmp( (*blob_it));
			objects.push_front( tmp);
			list < Object >::iterator p_it = objects.begin();

			//vicinanza con gli altri Object dello stesso colore
			list < Object >::iterator it;
			for( it = objects.begin(); it != objects.end(); it++)
			{
				//se c'è un Object vicino..
				if( (*it).isNear( *p_it, d_x + d_x /2, d_y + d_y /2))
				{
					if(it != p_it)
					{
						(*it).merge(*p_it);
						objects.erase(p_it);
						p_it = it;
					}
				}
			}

		}
	}

	return objects;
}


void Object::drawObjects( list < Object > & objects, IplImage * img, CvScalar color)
{
	list < Object >::iterator it;
	for( it = objects.begin(); it != objects.end(); it++)
	{
		it->draw(img, color);
	}
}

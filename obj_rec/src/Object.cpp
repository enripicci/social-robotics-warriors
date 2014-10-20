#include "Object.h"
#include "DataFitting.h"    // obsolete


Object::Object() {
	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	avg = cvPoint( 0, 0);
	area = 0;
}

Object::Object( Blob & blob) {
	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	avg = cvPoint( 0, 0);
	area = 0;

	this->merge( blob);
}

Object::~Object() {
	list < Blob >::iterator i_it;
	for( i_it = this->blobs.begin(); i_it != this->blobs.end(); i_it++)
		i_it->clear();
}


void Object::clear() {
	blobs.clear();

	top_left = cvPoint( 1000000, 1000000);
	bottom_right = cvPoint( -1, -1);
	avg = cvPoint( 0, 0);
	area = 0;
}


bool Object::isNear( Object & object, const int d_x, const int d_y) {

	for( list < Blob >::iterator i_it = this->blobs.begin(); i_it != this->blobs.end(); i_it++) {
		for( list < Blob >::iterator j_it = object.blobs.begin(); j_it != object.blobs.end(); j_it++) {
			if( i_it->isNear( *j_it, d_x, d_y ) )
                return true;
        }
	}

	return false;
}


Object Object::merge( Object & object) {
	for( list < Blob >::iterator it = object.blobs.begin(); it != object.blobs.end(); it++)
		this->merge(*it);

	return *this;
}

bool Object::merge( Blob & blob) {
	blobs.push_back(blob);

	if( false == blobs.empty() ) {
        // update the bbox
	    if( blob.getTopLeft().x < top_left.x)
            top_left.x = blob.getTopLeft().x;

        if( blob.getTopLeft().y < top_left.y)
            top_left.y = blob.getTopLeft().y;

        if( blob.getBottomRight().x > bottom_right.x )
            bottom_right.x = blob.getBottomRight().x;

        if( blob.getBottomRight().y > bottom_right.y)
            bottom_right.y = blob.getBottomRight().y;

        area += blob.getArea();
    } else {
        // just set the bbox
        top_left.x = run.x;
        top_left.y = bottom_right.y = run.y;
        bottom_right.x = run.x + run.len;
        area = blob.getArea();
    }

	return true;
}


vector< double > Object::fitSuperellipse() {
    vector < double > params;
#if 0
	CvPoint pt0;

	CvSize axes = cvSize( fabs(top_left.x - bottom_right.x)/2.0,  fabs(top_left.y - bottom_right.y)/2.0);

//	cvRectangle( img, top_left, bottom_right, cvScalar(0xff,0xff,0xff), 2);

//	CvPoint avg;
	avg.x = fabs(bottom_right.x + top_left.x) /2.0;
	avg.y = fabs(bottom_right.y + top_left.y) /2.0;


	CvSeq * edges = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2,
                                 sizeof(CvContour),
                                 sizeof(CvPoint),
                                 cvCreateMemStorage() );

	for( list< Blob >::iterator blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++) {
		CvSeq * blob_hull = blob_it->getEdges();

		for( int i = 0; i < blob_hull->total; i++ ) {
			pt0 = *CV_GET_SEQ_ELEM( CvPoint, blob_hull, i );
			cvSeqPush( edges, &pt0 );
		}

		cvReleaseMemStorage( &(blob_hull->storage) );
	}

	CvSeq * hull = cvConvexHull2( edges, cvCreateMemStorage(), CV_CLOCKWISE, 0 );

	params = fit_data( hull, top_left, bottom_right);

	cvReleaseMemStorage( &(hull->storage) );
	cvReleaseMemStorage( &(edges->storage) );
#endif


	return params;
}

void Object::drawFitter( IplImage * img, CvScalar color) {
	//vector < double > params = getEllipseParams();
	//paint_fitter( img, params, top_left, bottom_right, color);
}

vector < double > Object::getEllipseParams() {
    if(0 == ellipseParams.size())
        ellipseParams = fitSuperellipse();

    return ellipseParams;
}


list < Object > Object::divide( int m ) {
    list <Object > retVal;
    unsigned int byte = 0x01;
    unsigned int bit = 0;
    Object obj;
    int n = 0;
    int N = pow(2, blobs.size()) - 1;

	/*if( N >= object.colors.size())
		retVal.push_back(object);
	*/

	for(unsigned int i = 1; i < N; i++) {
		for( list< Blob >::iterator blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++) {
			if(byte & i) {
				obj.merge(*blob_it);
				n++;
			}
			byte <<= 1;
			bit++;
		}

		if( n == blobs.size() - m)
			retVal.push_back(obj);
	}

	return retVal;
}

Object Object::remove( Object & object) {
	Object o;
	bool found = false;

	for( list <Blob>::iterator it = blobs.begin(); it != blobs.end(); it++) {
		for( list <Blob>::iterator b_it = object.blobs.begin(); b_it != object.blobs.end(); b_it++) {
			if((*b_it) == (*it)) {
				found = true;
				break;
			}
		}

		if(false == found)
            o.merge(*it);
	}

	return o;
}


map < int, list < Object > > Object::recognizeObjects( list < Object > & objs, map < int, set < ColorClass > > & objects_memory) {
	map < int, list< Object > > retVal;
	bool rec = false;

	for( list <Objec >::iterator obj_it = objs.begin(); obj_it != objs.end(); obj_it++ ) {

		map< int, set< ColorClass > >::iterator feat_it;
		for( feat_it = objects_memory.begin(); feat_it != objects_memory.end(); feat_it++) {
			if(feat_it->second == obj_it->getColors()) {
				retVal[feat_it->first].push_back(*obj_it);
				rec = true;
				break;
			}
        }

		if((false == rec) && (obj_it->getColors().size() > 1)) {
			list < Object > sub_objs = (*obj_it).divide();

			for( list <Object>::iterator sub_obj_it = sub_objs.begin(); (sub_obj_it != sub_objs.end()); sub_obj_it++) {
				map< int, set< ColorClass > >::iterator feat_it;
				for( feat_it = objects_memory.begin(); feat_it != objects_memory.end(); feat_it++) {
					if(feat_it->second == sub_obj_it->getColors()) {
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
	}

	return retVal;
}

Object * Object::extractMaxAreaObject( list < Object > & objs ) {
	long max_area = -1;
	list <Object>::iterator max_area_obj_it;

	if( true==objs.empty() )
        return 0;

	for( list <Object>::iterator obj_it = objs.begin(); obj_it != objs.end(); obj_it++) {
		if(obj_it->getArea() > max_area) {
			max_area = obj_it->getArea();
			max_area_obj_it = obj_it;
		}
    }

    return &(*max_area_obj_it);

}


list < Object > Object::extractObjects( list< Blob > & blobs, long minArea, int d_x, int d_y) {
	list <Object> objects;
	list <Object>::reverse_iterator p_it;

	for( list< Blob >::iterator blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++) {
		if( blob_it->getArea() > minArea) {
			// make an object with this blob
			Object tmp( *blob_it );

			objects.push_back( tmp );
			p_it = objects.back();

			// merge near blobs into an object
			// FIXME: The blob's color need to be the same??
			for( list < Object >::reverse iterator it = p_it-1 ; it != objects.rend(); it++) {
                if( it->isNear( *p_it, d_x + d_x /2, d_y + d_y /2)) {
                    it->merge(*p_it);
                    objects.erase(p_it);
                    p_it = it;
                }
			}
		}
	}

	return objects;
}


void Object::drawObjects( list < Object > & objects, IplImage * img, CvScalar color) {

	for( list < Object >::iterator it = objects.begin(); it != objects.end(); it++) {
		it->drawFitter(img, color);
	}
}


set < ColorClass > Object::getColors() {
    set< ColorClass > colors;

    for( list< Blob >::iterator blob_it = blobs.begin(); blob_it != blobs.end(); blob_it++) {
        colors.insert(blob_it->getColorClass());
    }

    return colors;
}

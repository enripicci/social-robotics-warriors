#include "DataFitting.h"

#include <gsl/gsl_multimin.h>

typedef struct
{
	int n;
	vector < double > x;
	vector < double > y;
} Data;

//Si ma forse la direttrice la potrei trovare in un atlro modo..
double super_ellipse( double angle, vector < double > n)
{
	//if(n.size() <2) n.push_back(0.0);
//	return pow( (pow(fabs(cos(angle + M_PI/n[1])), n[0]) + pow( fabs(sin(angle + M_PI/n[1])), n[0])), (-1/n[0]));
//	return pow( (1/(pow(fabs(cos(angle+n[1])/n[2]), n[0]) + pow( fabs(sin(angle+n[1])/n[3]), n[0])) ), (1/n[0]));
	if(!n.size()) return 0.0;
	if(n.size() == 1)
		return pow( (pow(fabs(cos(angle)), n[0]) + pow( fabs(sin(angle)), n[0])), (-1/n[0]));
	else
		return pow( (pow(fabs(cos(angle+n[1])), n[0]) + pow( fabs(sin(angle+n[1])), n[0])), (-1/n[0]));
}

double least_square_fit(const gsl_vector *v, void *params)
{
	Data *data = (Data *)params;
	vector < double > n;
	for( int i  = 0; i < data->n; i++)
		n.push_back( gsl_vector_get(v, i) );

	double retVal = 0;

	for( int i = 0; i < data->x.size(); i++)
	{
		double seRes = super_ellipse( data->x[i], n);
		retVal += (data->y[i] - seRes) * (data->y[i] - seRes);
	}

	return retVal;
}

vector < double > fit_data( CvSeq * data, CvPoint top_left, CvPoint bottom_right)
{
	vector < double > params;

	if(data->total < 4) return params;

	params.push_back(1.0);
//	params.push_back(M_PI/4.0);
// 	params.push_back(1.0);
// 	params.push_back(1.0);

	vector < double > a(data->total);
	vector < double > r(data->total);

	//normalize
	for( int i = 0; i < data->total; i++ )
	{
		CvPoint * pt0;
		pt0 = *CV_GET_SEQ_ELEM( CvPoint *, data, i );

		double x = (pt0->x - (top_left.x + fabs(bottom_right.x - top_left.x) /2.0) ) / (fabs(bottom_right.x - top_left.x) /2.0);
		double y = (pt0->y - (top_left.y + fabs(bottom_right.y - top_left.y) /2.0) ) / (fabs(bottom_right.y - top_left.y) /2.0);

		double radius = sqrt(x*x + y*y);
		double angle = atan2(y, x);

		a[i] = angle;
		r[i] = radius;
	}

//minimization
	gsl_multimin_fminimizer *s = NULL;
	gsl_vector *ss, *n;
	gsl_multimin_function minex_func;
	
	size_t iter = 0;
	int status;
	double size = 0;
	
	/* Starting point */
	n = gsl_vector_alloc (params.size());
	for(int i = 0; i < params.size(); i++)
		gsl_vector_set (n, i, params[i]);

	
	/* Set initial step sizes to 1 */
	ss = gsl_vector_alloc (params.size());
	gsl_vector_set_all (ss, .1);

	Data d;
	d.x = a; d.y = r; d.n = params.size();

	/* Initialize method and iterate */
	minex_func.n = params.size();
	minex_func.f = &least_square_fit;
	minex_func.params = (void *)&d;
	
	s = gsl_multimin_fminimizer_alloc (gsl_multimin_fminimizer_nmsimplex, params.size());
	gsl_multimin_fminimizer_set (s, &minex_func, n, ss);

	do
		{
		iter++;
		status = gsl_multimin_fminimizer_iterate(s);

// 		if (status)
// 			break;

		size = gsl_multimin_fminimizer_size (s);
		status = gsl_multimin_test_size (size, 1e-2);

//		printf("\n%f -> %f", gsl_vector_get (s->x, 0), s->fval);

		if (status == GSL_SUCCESS)
				break;
		}
	while (status == GSL_CONTINUE && iter < 100);

	for( int i  = 0; i < params.size(); i++)
		params[i] = gsl_vector_get (s->x, i);


//	printf("\n\tstatus: %d ->param 1: %f", status, params[0]);
//	printf("\n\tstatus: %d ->param 2: %f", status, params[1]/M_PI*360.0);
	gsl_vector_free(n);
	gsl_vector_free(ss);
	gsl_multimin_fminimizer_free (s);


	return params;
}


void paint_fitter( IplImage * img, vector < double > params, CvPoint top_left, CvPoint bottom_right, CvScalar color)
{
	double d_a = 0.1;

	for( double angle = -M_PI; angle < M_PI -d_a; angle += d_a)
	{
		double r = super_ellipse(angle, params);

		double x = r * cos(angle);
		double y = r * sin(angle);

		x = (x * (fabs(bottom_right.x - top_left.x) /2.0) + top_left.x + fabs(bottom_right.x - top_left.x) /2.0);
		y = (y * (fabs(bottom_right.y - top_left.y) /2.0) + top_left.y + fabs(bottom_right.y - top_left.y) /2.0);

		CvPoint p0 = cvPoint( floor(x), floor(y));

		r = super_ellipse(angle +d_a, params);

		x = r * cos(angle +d_a);
		y = r * sin(angle +d_a);
		x = (x * (fabs(bottom_right.x - top_left.x) /2.0) + top_left.x + fabs(bottom_right.x - top_left.x) /2.0);
		y = (y * (fabs(bottom_right.y - top_left.y) /2.0) + top_left.y + fabs(bottom_right.y - top_left.y) /2.0);

		CvPoint p1 = cvPoint( floor(x), floor(y));

		cvLine( img, p0, p1, color, 2);
	}
}

#include "Vision.h"

#include <qfiledialog.h>

#include <highgui.h>

#include "DataFitting.h"


// Draw the FPS figure on the image (requires at least 2 calls)
void Vision::drawFPS( IplImage *img)
{
  static clock_t t;
  char fps_text[20];
  CvFont font;
  cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1.0,1.0,0,2);

  // Add fps figure
  float fps = (1.0f/(clock()-t) * CLOCKS_PER_SEC);

  // Update variables
  t=clock(); 

  // Get the figure as a string
  sprintf(fps_text,"FPS: %.2f",fps);

  // Draw the string on the image
  cvPutText (img,fps_text,cvPoint(10,25), &font, cvScalar(255,255,0));
}


Vision::Vision( QWidget *parent, const char * name)
 : QWidget( parent, name),
	lastImageViewer( 0, this, "Image"),
	segImageViewer( 0, this, "Segmented Image"),
	ctTable( 1, 4, this, "Color Table"),
	objectsTable( 1, 3, this, "Objects Table"),
	addRowButton( "Add Row..", this, "Add Row Button"),
	colorTableUpdateButton( "Update Colors", this, "Update Colors Button"),
	segButton( "Segment", this, "Segment Button"),
	pauseButton( "Pause", this, "Pause Button"),
	undoButton( "Undo", this, "Undo Button"),
	addObjectButton( "Add Object", this, "Add Object Button"),
	loadSamplesButton( "Load ct", this, "Load Samples Button"),
	saveSamplesButton( "Save ct", this, "Save Samples Button"),
	timer( this, "Timer"),
	seg( true),
	paused(false),
	server(this, 6666)
{
	// Add labels to the color table
	ctTable.horizontalHeader()->setLabel( 0, "Red");
	ctTable.horizontalHeader()->setLabel( 1, "Green");
	ctTable.horizontalHeader()->setLabel( 2, "Blue");
	ctTable.horizontalHeader()->setLabel( 3, "Segment");

	// Add black to colortable
	ctTable.setText( 0, 0, "0");
	ctTable.setText( 0, 1, "0");
	ctTable.setText( 0, 2, "0");
	ctTable.setItem( 0, 3, new QCheckTableItem( &ctTable, ""));
	colorTable.setColorClassSpecifics( 0, cvScalar(0,0,0), false);

	// Add labels to the objects table
	objectsTable.horizontalHeader()->setLabel( 0, "Red");
	objectsTable.horizontalHeader()->setLabel( 1, "Green");
	objectsTable.horizontalHeader()->setLabel( 2, "Blue");

	// Add a black signature to the objects
	objectsTable.setText( 0, 0, "0");
	objectsTable.setText( 0, 1, "0");
	objectsTable.setText( 0, 2, "0");

	//set Undo button disabled
	undoButton.setEnabled( false);


	// Starting frame grabber
	capture = cvCaptureFromCAM(0);

//  double width = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
//  double height = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);

  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640);
  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480);


	// Reset images
	lastImage = 0;
	segImage = 0;
	storedImage = 0;

	// Connecting buttons
	connect( &colorTableUpdateButton, SIGNAL(clicked()), this, SLOT(colorTableUpdate()) );
	connect( &addRowButton, SIGNAL(clicked()), this, SLOT(addRow()) );
	connect( &segButton, SIGNAL(clicked()), this, SLOT(doSeg()) );
	connect( &pauseButton, SIGNAL(clicked()), this, SLOT(pause()) );
	connect( &undoButton, SIGNAL(clicked()), this, SLOT(undo()) );
	connect( &addObjectButton, SIGNAL(clicked()), this, SLOT(addObject()) );

	connect( &loadSamplesButton, SIGNAL(clicked()), this, SLOT(loadSamples()) );
	connect( &saveSamplesButton, SIGNAL(clicked()), this, SLOT(saveSamples()) );
	// Connecting and starting timer
	connect( &timer, SIGNAL(timeout()), this, SLOT(update()) );
	timer.start(0);
}


Vision::~Vision()
{
	// Shutdown the frame grabber
	cvReleaseCapture(&capture);
	capture = 0;
}


void Vision::update()
{
	if(!capture)
	{
		printf("\nError on connect to capture device\n");
		exit(0);
	}

	if(!cvGrabFrame(capture))	// capture a frame 
	{
		printf("\nCould not grab a frame\n");
	}


	if(segImage)
	{
		cvReleaseImage(&segImage);
		segImage = 0;
	}

	if(lastImage)
	{
		cvReleaseImage(&lastImage);
		lastImage = 0;
	}


	if(!paused)
	{
		IplImage * capturedImage = cvRetrieveFrame(capture);	// retrieve the captured frame
		
		lastImage = cvCloneImage(capturedImage);
		if(storedImage)
			cvReleaseImage(&storedImage);
		storedImage = cvCloneImage(lastImage);
	}
	else
		if(storedImage)
			lastImage = cvCloneImage(storedImage);


	if(seg)
	{
		segImage = cvCreateImage(
						cvSize(lastImage->width, lastImage->height),
						lastImage->depth,
						lastImage->nChannels);
		if(!segImage) return;
	
		colorTable.segment(lastImage, segImage, 3, 3);	// segment using the current CT

		segImageViewer.move( lastImageViewer.x() +lastImageViewer.width(), lastImageViewer.y());
		segImageViewer.update(segImage);
	}


	// Object Recognition <-
	doRecognition();

	if(!paused)
		drawFPS(lastImage);

	lastImageViewer.update(lastImage);


	ctTable.move( 0, lastImageViewer.y() +lastImageViewer.height() +5);
	ctTable.resize( 450, 250);

	addRowButton.move( 460, lastImageViewer.y() +lastImageViewer.height() +5);
	colorTableUpdateButton.move( 460, lastImageViewer.y() +lastImageViewer.height() +5 +50);
	segButton.move( 460, lastImageViewer.y() +lastImageViewer.height() +5 +100);
	pauseButton.move( 460, lastImageViewer.y() +lastImageViewer.height() +5 +150);
	undoButton.move( 460, lastImageViewer.y() +lastImageViewer.height() +5 +200);

	objectsTable.move( 600, lastImageViewer.y() +lastImageViewer.height() +5);
	objectsTable.resize( 350, 250);

	addObjectButton.move( 960, lastImageViewer.y() +lastImageViewer.height() +5);
	loadSamplesButton.move( 960, lastImageViewer.y() +lastImageViewer.height() +5 +100);
	saveSamplesButton.move( 960, lastImageViewer.y() +lastImageViewer.height() +5 +150);
}

void Vision::mouseReleaseEvent ( QMouseEvent * e)
{
	if(lastImage)
	{
		QPoint point = lastImageViewer.mapFromParent( e->pos());
		if( lastImageViewer.visibleRect().contains( point) )
		{
			uchar * color = PIXEL( lastImage, point.x(), point.y());

			colorTable.addSample( ctTable.currentRow(), cvScalar(color[0], color[1], color[2]));

			ColorSample cs = { ctTable.currentRow(), cvScalar(color[0], color[1], color[2]) };
			colorSamples.push_back(cs);

			if(!undoButton.isEnabled())
				undoButton.setEnabled( true);
		}
	}
}


void Vision::colorTableUpdate()
{
	for(int i = 0; i < ctTable.numRows(); i++)
	{
		CvScalar c = cvScalar( ctTable.text(i, 2).toDouble(), ctTable.text(i, 1).toDouble(), ctTable.text(i, 0).toDouble());
		bool seg = ((QCheckTableItem *)(ctTable.item(i,3)))->isChecked();
		colorTable.setColorClassSpecifics( i, c, seg);
	}
}


void Vision::addRow()
{
	ctTable.insertRows( ctTable.numRows());
	ctTable.setItem( ctTable.numRows()-1, 3, new QCheckTableItem( &ctTable, ""));
	ctTable.setText( ctTable.numRows()-1, 0, "0");
	ctTable.setText( ctTable.numRows()-1, 1, "0");
	ctTable.setText( ctTable.numRows()-1, 2, "0");
}


void Vision::doSeg()
{
	seg = !seg;
}


void Vision::addObject()
{
	Object * bigger = Object::extractMaxAreaObject( objs );
	if(bigger)
	{
		objects_memory[objectsTable.numRows()-1] = bigger->getColors();

		objectsTable.insertRows( objectsTable.numRows());
		objectsTable.setText( objectsTable.numRows()-1, 0, "0");
		objectsTable.setText( objectsTable.numRows()-1, 1, "0");
		objectsTable.setText( objectsTable.numRows()-1, 2, "0");
	}

}


void Vision::doRecognition()
{
	list < Blob > blobs;

	blobs = Blob::extractBlobs( lastImage, colorTable, MERGING_DISTANCE_X, MERGING_DISTANCE_Y);

	Blob::drawBlobs( colorTable, blobs, lastImage, MIN_AREA);

	objs = Object::extractObjects( blobs, MIN_AREA, MERGING_DISTANCE_X *2, MERGING_DISTANCE_Y *2);

	recognized.clear();

	recognized = Object::recognizeObjects( objs, objects_memory);

	map < int, list < Object > >::iterator feat_it;
	for( feat_it = recognized.begin(); feat_it != recognized.end(); feat_it++)
	{
		CvScalar color = cvScalar( objectsTable.text(feat_it->first, 2).toDouble(), objectsTable.text(feat_it->first, 1).toDouble(), objectsTable.text(feat_it->first, 0).toDouble());

		Object::drawObjects( feat_it->second, lastImage, color);
	}
}

void Vision::pause()
{
	paused = !paused;
}

void Vision::undo()
{
	colorSamples.pop_back();

	colorTable.clearSamples();

	vector< ColorSample >::iterator it;
	for( it = colorSamples.begin(); it != colorSamples.end(); it++)
		colorTable.addSample( it->colorClass, it->pos);

	if(colorSamples.size() == 0)
		undoButton.setEnabled( false);
}

void Vision::loadSamples()
{
	QString filename = QFileDialog::getOpenFileName();

	FILE * fp = fopen(filename.ascii(), "r");
	if(fp)
	{
		colorTable.clearSamples();

		while(!feof(fp))
		{
			ColorClass colorClass = 0;

			float a, b, c;

			fscanf( fp, "%d, %f, %f, %f\n", &colorClass, &a, &b, &c );

			CvScalar pos = cvScalar(a,b,c);			

			colorTable.addSample( colorClass, pos);
		}

		fclose(fp);
	}
}

void Vision::saveSamples()
{
	QString filename = QFileDialog::getSaveFileName();

	FILE * fp = fopen(filename.ascii(), "w");
	if(fp)
	{
		vector< ColorSample >::iterator it;
		for( it = colorSamples.begin(); it != colorSamples.end(); it++)
		{
			fprintf( fp, "%d, %f, %f, %f\n", it->colorClass, it->pos.val[0], it->pos.val[1], it->pos.val[2] );
		}

		fclose(fp);
	}
}

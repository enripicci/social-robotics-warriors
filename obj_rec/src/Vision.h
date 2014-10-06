#ifndef __VISION_H__
#define __VISION_H__

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <qwidget.h>
#include <qtable.h>
#include <qtimer.h>
#include <qpushbutton.h>

#include "IplImageViewer.h"
#include "ColorTable.h"
#include "Blob.h"
#include "Object.h"

#include "Server.h"


//DEFINES
#define MIN_AREA 2000
#define MERGING_DISTANCE_X 6
#define MERGING_DISTANCE_Y 6


class Vision
 : public QWidget
{
    Q_OBJECT

public:
	Vision( QWidget *parent = 0, const char *name = 0);
	virtual ~Vision();


	map < int, list < Object > > getRecognized()
	{
		return recognized;
	}

public slots:
	void update();
	void colorTableUpdate();
	void addRow();
	void doSeg();
	void pause();
	void undo();
	void addObject();
	void loadSamples();
	void saveSamples();

private:
	void mouseReleaseEvent ( QMouseEvent * e);

	IplImageViewer lastImageViewer;
	IplImageViewer segImageViewer;

	QTable ctTable;
	QTable objectsTable;
	QTimer timer;
	QPushButton colorTableUpdateButton;
	QPushButton addRowButton;
	QPushButton segButton;
	QPushButton pauseButton;
	QPushButton undoButton;

	QPushButton addObjectButton;

	QPushButton loadSamplesButton;
	QPushButton saveSamplesButton;


	CvCapture * capture;

	IplImage * lastImage;
	IplImage * segImage;
	IplImage * storedImage;

	bool seg;
	bool paused;

	// ColorTable
	ColorTable colorTable;

	// Features Memory & Objects
	Object::ObjectsMemory objects_memory;
	list < Object > objs;

	// The main recognition loop
	void doRecognition();
	map < int, list < Object > > recognized;

	// Undo Stack
	typedef struct
	{
		ColorClass colorClass;
		CvScalar pos;
	} ColorSample;
	vector< ColorSample > colorSamples;

	// Draw the FPS figure on the image (requires at least 2 calls)
	void drawFPS( IplImage *img);

	Server server;
};

#endif	// __VISION_H__

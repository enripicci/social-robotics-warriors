#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/mutex.hpp>

#include <map>
#include <list>

#include "ColorTable.h"
#include "Blob.h"
#include "Object.h"

#include "obj_rec/addColor.h"
#include "obj_rec/addPercept.h"
#include "obj_rec/Percepts.h"

//DEFINES
#define MIN_AREA 100
#define MERGING_DISTANCE_X 4
#define MERGING_DISTANCE_Y 4
#define SEG_X 2
#define SEG_Y 2

static const char *APP_NAME = "object_recognition";

// Data Lock Mutex
boost::mutex data_locker;

// Stream Image
cv::Mat frame;
std_msgs::Header header;

// Features Memory & Objects
Object::ObjectsMemory objects_memory;
typedef std::list < Object > Objects;
Objects objs;
typedef std::map < int, std::list < Object > > RecognizedObjects;
RecognizedObjects recognized;

// ColorTable
ColorTable colorTable;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImageConstPtr new_frame;

    try {
        boost::mutex::scoped_lock lock(data_locker);

        new_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
        new_frame->image.copyTo(frame);
        
        header = msg->header;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

cv::Scalar getColorFromId( int id) {
    cv::Scalar retVal;

    switch(id) {
        case 0:
            retVal = cv::Scalar(128,128,128);break; //
        case 1:
            retVal = cv::Scalar(255,0,0); break;    // blue
        case 2:
            retVal = cv::Scalar(0,255,0); break;    // green
        case 3:
            retVal = cv::Scalar(0,0,255); break;    // red
        case 4:
            retVal = cv::Scalar(255,255,0); break;  // cyan
        case 5:
            retVal = cv::Scalar(255,0,255); break;  // pink
        case 6:
            retVal = cv::Scalar(0,255,255); break;  // yellow
        case 7:
            retVal = cv::Scalar(255,255,255); break;    // white
        default:
            retVal = cv::Scalar(0,0,0); break;  // black
    };

    return retVal;
}


/* Add the sample specified in req to the color table
*/
bool addColor( obj_rec::addColor::Request  &req,
               obj_rec::addColor::Response &res ) {
                   
    res.ok = colorTable.addSample(req.id, cvScalar(req.bb, req.gg, req.rr));
    return true;
}

bool addPercept( obj_rec::addPercept::Request  &req,
                 obj_rec::addPercept::Response &res ) {

    Object * bigger = Object::extractMaxAreaObject( objs );
    if(bigger) {
        objects_memory[req.id] = bigger->getColors();
        res.ok = 1;
    } else {
        res.ok = 0;
    }

    return true;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, APP_NAME);
    ros::NodeHandle nh;

    cv::namedWindow(APP_NAME);
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/video", 1, imageCallback);

    ros::ServiceServer addColor_service = nh.advertiseService("addColor", addColor);
    ros::ServiceServer addPercept_service = nh.advertiseService("addPercept", addPercept);
    ros::Publisher percepts_pub = nh.advertise<obj_rec::Percepts>("/percepts", 100);


    // Color table management
    colorTable.setColorClassSpecifics( 0, cv::Scalar(0,0,0), false);
    for( int i = 1; i < 7; i++) {
      colorTable.setColorClassSpecifics( i, getColorFromId(i), true);
    }
    colorTable.setColorClassSpecifics( 7, cv::Scalar(255,255,255), false);


    // The main recognition loop
    ros::Rate loop_rate(100);
    
    while (ros::ok()) {
        if( 0 == frame.empty()) {
            boost::mutex::scoped_lock lock(data_locker);
            cv::Mat image, seg;
            frame.copyTo(image);
            frame.copyTo(seg);
            // cv::rectangle(seg,cv::Point(0,0), cv::Point(seg.cols, seg.rows), cv::Scalar(0,0,0), -1);

            IplImage _lastImage = IplImage(image);
            IplImage * lastImage = &_lastImage;

            IplImage _segImage = IplImage(seg);
            IplImage * segImage = &_segImage;

            colorTable.segment(lastImage, segImage, SEG_X, SEG_Y);	// segment using the current CT

            // Recognition
            std::list < Blob > blobs;
            blobs = Blob::extractBlobs( lastImage, colorTable, MERGING_DISTANCE_X, MERGING_DISTANCE_Y);

            Blob::drawBlobs( colorTable, blobs, segImage, MIN_AREA);

            objs = Object::extractObjects( blobs, MIN_AREA, MERGING_DISTANCE_X *2, MERGING_DISTANCE_Y *2);

            recognized.clear();
            recognized = Object::recognizeObjects( objs, objects_memory);

            obj_rec::Percepts percepts_msg;

            RecognizedObjects::iterator feat_it;
            for( feat_it = recognized.begin(); feat_it != recognized.end(); feat_it++) {
                CvScalar color = getColorFromId(feat_it->first);

                Object::drawObjects( feat_it->second, segImage, color);

                for( Objects::iterator obj_it = feat_it->second.begin();
                     obj_it != feat_it->second.end();
                     obj_it++) {
                    // compose msg
                    obj_rec::Percept percept_msg;

                    percept_msg.id = feat_it->first;
                    percept_msg.u = obj_it->getAvg().x;
                    percept_msg.v = obj_it->getAvg().y;
                    percept_msg.width = obj_it->getBottomRight().x - obj_it->getTopLeft().x;
                    percept_msg.height = obj_it->getBottomRight().y - obj_it->getTopLeft().y;
                    percept_msg.area = obj_it->getArea();

                    percepts_msg.percepts.push_back(percept_msg);
                }
            }

            percepts_msg.header = header;
            percepts_pub.publish(percepts_msg);

            cv::imshow(APP_NAME, seg);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    cv::destroyWindow(APP_NAME);
}


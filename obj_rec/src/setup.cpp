#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/mutex.hpp>
#include <sstream>

#include "obj_rec/addColor.h"
#include "obj_rec/addPercept.h"


boost::mutex data_locker;

cv::Mat frame;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr new_frame;
  try
  {
    boost::mutex::scoped_lock lock(data_locker);

    new_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
    new_frame->image.copyTo(frame);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

ros::ServiceClient *addColor_client = 0;
unsigned char color_id = 0;

ros::ServiceClient *addPercept_client = 0;
unsigned char percept_id = 0;

void mouse_callback(int event, int x, int y, int flags, void* param)
{
  cv::Mat *img = (cv::Mat *)param;

  if(event == CV_EVENT_LBUTTONDOWN)
  {
    if(flags & CV_EVENT_FLAG_CTRLKEY)
    {
      obj_rec::addPercept srv;
      srv.request.id = percept_id;
      if (addPercept_client->call(srv))
      {
        ROS_INFO("percept %d - %d", srv.request.id, (unsigned char)srv.response.ok);
      }
      else
      {
        ROS_ERROR("Failed to add percept");
        return ;
      }
    }
    else
    {
      cv::Vec<unsigned char,4> col = img->at<cv::Vec<unsigned char,4> >(y, x);

      obj_rec::addColor srv;
      srv.request.rr = col[2];
      srv.request.gg = col[1];
      srv.request.bb = col[0];
      srv.request.id = color_id;
      if (addColor_client->call(srv))
      {
        ROS_INFO("pixel (%d,%d): %d,%d,%d - %d ? %d", x, y, srv.request.rr, srv.request.gg, srv.request.bb, color_id, (unsigned char)srv.response.ok);
      }
      else
      {
        ROS_ERROR("Failed to add pixel");
        return ;
      }
    }
  }
  else if(event == CV_EVENT_RBUTTONDOWN)
  {
    if(flags & CV_EVENT_FLAG_CTRLKEY)
      percept_id = (percept_id+1)%8;
    else
      color_id = (color_id+1)%8;
  }
}

std::string idToCol( unsigned int id)
{
  std::string retVal;
  switch(id)
  {
  case 0: retVal = std::string("black*"); break;
  case 1: retVal = std::string("blue"); break;
  case 2: retVal = std::string("green"); break;
  case 3: retVal = std::string("red"); break;
  case 4: retVal = std::string("cyan"); break;
  case 5: retVal = std::string("pink"); break;
  case 6: retVal = std::string("yellow"); break;
  case 7: retVal = std::string("white*"); break;
  default: retVal = std::string("???*");
  }

  return retVal;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_recognition_setup");

  ros::NodeHandle nh;

  cv::namedWindow("object_recognition_setup");

  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/video", 1, imageCallback);

  ros::ServiceClient _addColor_client = nh.serviceClient<obj_rec::addColor>("addColor");
  addColor_client = &_addColor_client;

  ros::ServiceClient _addPercept_client = nh.serviceClient<obj_rec::addPercept>("addPercept");
  addPercept_client = &_addPercept_client;

  cv::Mat image;
  cv::setMouseCallback("object_recognition_setup", mouse_callback, &image);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    if(!frame.empty())
    {
      boost::mutex::scoped_lock lock(data_locker);

      frame.copyTo(image);

      std::string col_id_str = idToCol(color_id);
      cv::putText(image, col_id_str, cv::Point(10,25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);

      std::stringstream percept_id_str("");
      percept_id_str << (int)(percept_id);
      cv::putText(image, percept_id_str.str(), cv::Point(image.cols - 100,25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 1);

      cv::imshow("object_recognition_setup", image);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  cv::destroyWindow("object_recognition_setup");
}


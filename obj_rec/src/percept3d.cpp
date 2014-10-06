#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/mutex.hpp>


#include "obj_rec/Percepts.h"
#include "obj_rec/Percepts3d.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sstream>


// ROS Initializations
boost::mutex data_locker;
ros::Rate * loop_rate;
ros::Publisher * percepts3d_pub = 0;




// POINT CLOUD
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void pointsCallback(const sensor_msgs::PointCloud2 & msg)
{
  boost::mutex::scoped_lock lock(data_locker);
  pcl::fromROSMsg(msg, *cloud);
}


// PERCEPTIONS
void perceptsCallback(const obj_rec::Percepts & msg)
{
  boost::mutex::scoped_lock lock(data_locker);

  if(!cloud->size()) {
    std::cout << "not cloud" << std::endl;
    return;
  }


  obj_rec::Percepts3d percepts3d_msg;
  percepts3d_msg.header = msg.header;

  for( unsigned i = 0; i < msg.percepts.size(); i++)
  {
    obj_rec::Percept3d percept;
    percept.percept = msg.percepts[i];
    pcl::PointXYZ point = cloud->points[msg.percepts[i].v * cloud->width + msg.percepts[i].u];
    percept.x = point.x;
    percept.y = point.y;
    percept.z = point.z;

//    std::cout << "(" << percept.x << ", " << percept.y << ", " << percept.z << "): "
//              << "[" << percept.percept.u << ", " << percept.percept.v << ", " << percept.percept.area << "]"
//              << std::endl;


	std::stringstream obj_name;
	obj_name << "obj_" << (int)(msg.percepts[i].id);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(point.x, point.y, point.z) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "obj_frame", obj_name.str()));


    percepts3d_msg.percepts.push_back(percept);
  }

  percepts3d_pub->publish(percepts3d_msg);
}


// MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "percept3d");

  ros::NodeHandle nh;

  ros::Subscriber points_sub = nh.subscribe("/points", 1, pointsCallback);
  ros::Subscriber percepts_sub = nh.subscribe("/percepts", 100, perceptsCallback);

  ros::Publisher _percepts3d_pub = nh.advertise<obj_rec::Percepts3d>("/percepts3d", 100);
  percepts3d_pub = &_percepts3d_pub;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    boost::mutex::scoped_lock lock(data_locker);

    loop_rate.sleep();
  }

  return 1;
}



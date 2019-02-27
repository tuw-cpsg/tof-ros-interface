#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#define FRAME_ID "tof_camera"
// TODO: fix width and height
#define WIDTH 720
#define HEIGHT 340


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ti_tof");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);

  // TODO: remove loop rate
  // will be defined by frame rate -> publish and spinOnce without delay as soon as frame arrives
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now(); // should be time stamp of the frame
    msg.header.frame_id = FRAME_ID;
    msg.width = WIDTH;
    msg.height = HEIGHT;
    // TODO: fill message with fields info and frame
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

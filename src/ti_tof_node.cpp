#include "ros/ros.h"
#include "ti_tof/DepthArrayStamped.h"

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
  ros::Publisher pub = n.advertise<ti_tof::DepthArrayStamped>("frame", 1);

  // TODO: remove loop rate
  // will be defined by frame rate -> publish and spinOnce without delay as soon as frame arrives
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ti_tof::DepthArrayStamped msg;
    msg.header.stamp = ros::Time::now(); // should be time stamp of the frame
    msg.width = WIDTH;
    msg.height = HEIGHT;
    // TODO: fill message with depth array
    // msg.depth = voxel:frame
    // msg.amplitude = voxel:frame
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

/*
 * Node to transfer depth messages from ti_tof to image.
 * Run on a performance host.
 *
 * derived from http://wiki.ros.org/image_transport/Tutorials
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ti_tof/DepthArrayStamped.h>


image_transport::Publisher pubd;
image_transport::Publisher puba;


void callback(const ti_tof::DepthArrayStamped::ConstPtr& msg)
{
  // re-publish message as depth and amplitude image
  sensor_msgs::Image msgd, msga;
  msgd.header           = msga.header           = msg->header;
  msgd.width            = msga.width            = msg->width;
  msgd.height           = msga.height           = msg->height;
  msgd.encoding         = msga.encoding         = "mono8";
  msgd.is_bigendian     = msga.is_bigendian     = false;

  // put depth and amplitude into msg
  for(int i = 0; i < msg->depth.size(); i++) {
    msgd.data.push_back(std::min(255,int(msg->depth[i]/10.0*255)));
    msga.data.push_back(std::min(255,int(msg->amplitude[i]/0.03*255)));
  }

  pubd.publish(msgd);
  puba.publish(msga);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_to_image");

  ros::NodeHandle nh("~");

  ros::Subscriber sub = nh.subscribe("frame", 1, callback);
  image_transport::ImageTransport it(nh);
  pubd = it.advertise("depth", 1);
  puba = it.advertise("amplitude", 1);

  ros::spin();

  return 0;
}

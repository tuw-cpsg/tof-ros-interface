#include "ros/ros.h"
#include "ti_tof/DepthArrayStamped.h"

#define FRAME_ID "tof_camera" // TODO what is this and is it required?

#define WIDTH   320
#define HEIGHT  240

#include <CameraSystem.h>
using namespace Voxel;

ros::Publisher *pub_p;

static bool tof_connect(DepthCamera::FrameType frmType, int fps);

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ti_tof");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<ti_tof::DepthArrayStamped>("frame", 1);
  pub_p = &pub;

  if (!tof_connect(DepthCamera::FRAME_DEPTH_FRAME, 5)) {
    fprintf(stderr, "error: failed to connect to ToF camera\n");
    exit(EXIT_FAILURE);
  }

  ros::spin();

  return 0;
}

static void frameCallback(DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c)
{
    if (c != DepthCamera::FRAME_DEPTH_FRAME) {
        fprintf(stderr, "warning: invalid frame type, dropped\n");
        return;
    }

    const Voxel::DepthFrame *f = dynamic_cast<const Voxel::DepthFrame *>(&frame);

    if (f->size.width != WIDTH || f->size.height != HEIGHT) {
        fprintf(stderr, "warning: frame has wrong size, dropped\n");
        return;
    }

    ti_tof::DepthArrayStamped msg;
    msg.header.stamp = ros::Time::now(); // should be time stamp of the frame
    msg.width = WIDTH;
    msg.height = HEIGHT;
    msg.depth = f->depth;
    msg.amplitude = f->amplitude;
    pub_p->publish(msg);
}

#define DEFAULT_ILLUM_POWER 100
#define DEFAULT_EXPOSURE    20

CameraSystem _sys;
DepthCameraPtr _depthCamera;
int _loopDelay = 66;
int _illum_power = 90;
int _intg = 40;
Voxel::String _profile = "MetrilusLongRange";
DepthCamera::FrameType _frameType;

static bool setProfile(Voxel::String name);
static bool setIllumPower(int power);
static bool setExposure(int exposure);
static bool setFPS(int fps);

static bool tof_connect(DepthCamera::FrameType frmType, int fps)
{
   const std::vector<DevicePtr> &devices = _sys.scan();
   if (devices.size() > 0) {
      _depthCamera = _sys.connect(devices[0]);
      if (!_depthCamera)
         return false;
      if (!_depthCamera->isInitialized())
         return false;
   }
   else
      return false;

   // setup spatial and temportal median filters
   FilterPtr p = _sys.createFilter("Voxel::MedianFilter", DepthCamera::FRAME_RAW_FRAME_PROCESSED);
   if (!p)
   {
      //logger(LOG_ERROR) << "Failed to get MedianFilter" << std::endl;
      fprintf(stderr, "Failed to get MedianFilter\n");
      return -1;
   }

   _depthCamera->addFilter(p, DepthCamera::FRAME_RAW_FRAME_PROCESSED);

#if 1
   p = _sys.createFilter("Voxel::TemporalMedianFilter",  DepthCamera::FRAME_RAW_FRAME_PROCESSED);
   if (!p)
   {
      //logger(LOG_ERROR) << "Failed to get TemporalMedianFilter" << std::endl;
      fprintf(stderr, "Failed to get TemporalMedianFilter\n");
      return -1;
   }

   p->set("deadband", 0.01f);
   _depthCamera->addFilter(p, DepthCamera::FRAME_RAW_FRAME_PROCESSED);
#endif

   // Setup frame callback profile and settings
   _frameType = frmType;
   _depthCamera->registerCallback(_frameType, frameCallback);
   FrameSize _dimen;
   _dimen.width = WIDTH;
   _dimen.height = HEIGHT;
   _depthCamera->setFrameSize(_dimen);
   if (setProfile(_profile))
      //cout << "Profile " << _profile << " found." << endl;
      printf("profile found\n");
   else
      //cout << "Profile " << _profile << "not found." << endl;
      printf("profile not found\n");
   setIllumPower(DEFAULT_ILLUM_POWER);
   setExposure(DEFAULT_EXPOSURE);

   // set fps
   setFPS(fps);

   // Start the camera
   _depthCamera->start();
   //_isConnected = true;

   return true;
}

static bool setProfile(Voxel::String name)
{
   bool rc = false;
   const Map<int, Voxel::String> &profiles = _depthCamera->getCameraProfileNames();
   for (auto &p: profiles) {
      if (p.second == name) {
         int profile_id = p.first;
         ConfigurationFile *c = _depthCamera->configFile.getCameraProfile(p.first);
         if (c && c->getLocation() == ConfigurationFile::IN_CAMERA) {
            if (_depthCamera->setCameraProfile(profile_id)) {
              rc = true;
              break;
            }
         }
      }
   }
   return rc;
}

static bool setIllumPower(int power)
{
   uint p = (uint)power;
   return _depthCamera->set("illum_power_percentage", p);
}

static bool setExposure(int exposure)
{
   uint e = (uint)exposure;
   return _depthCamera->set("intg_duty_cycle", e);
}

static bool setFPS(int fps)
{
    float rate =(float) fps;

    FrameRate r;
    r.numerator = rate*10000;
    r.denominator = 10000;

    uint g = gcd(r.numerator, r.denominator);

    r.numerator /= g;
    r.denominator /= g;
    _depthCamera->setFrameRate(r);

    //_depthCamera->start();
    return true;
}

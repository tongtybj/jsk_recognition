#include "jsk_perception/fisheye_gimbal.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "fisheye_gimbal");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  jsk_perception::FisheyeGimbal*  fisheye_gimbal = new jsk_perception::FisheyeGimbal(nh, nh_private);
  ros::spin ();
  delete fisheye_gimbal;
  return 0;
}







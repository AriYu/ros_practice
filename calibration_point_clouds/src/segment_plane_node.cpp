#include <segment_plane.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Segment Plane Node");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  SegmentPlane detector(nh, n);
  detector.run();
  
  return 0;
}

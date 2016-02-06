#include "Tap_Decision.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  Decision obj;
  obj.run();
  return 0;
}
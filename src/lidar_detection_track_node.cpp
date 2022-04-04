#include "lidar_detection_track.h"

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"lidar_detection_track_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    lidarPerception core(nh,pnh);
    
    return 0;
}


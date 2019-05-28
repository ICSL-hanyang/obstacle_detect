
#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "obstacle_detect/Pair.h"
#include "obstacle_detect/VectorPair.h"

obstacle_detect::VectorPair vector_pair;
double rp_arr[360]={0.0,};
uint8_t degree = 5;
int num = 360/degree;

void rplidarCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    double msg_arr[num]={0.0,};
    double max_distance = 2.0;
    uint8_t n = 0;

    for(uint16_t i = 0; i<360; i++)
    {
        if(i%5==0 && i!=0)
        {
            msg_arr[n] /= degree;
            if(msg_arr[n] >= max_distance)
                msg_arr[n] = std::numeric_limits<double>::infinity();
            n++;
            if(n>=num) break;
        }
        msg_arr[n] += msg->ranges[i];
    }
    for(uint16_t i = 0; i < num; i++)
    {
        rp_arr[i] = msg_arr[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detect_node");
	ros::NodeHandle nh("~");

	ros::Rate rate(5);

    ros::Subscriber rplidar_sub = nh.subscribe("/scan", 10, &rplidarCB);
    ros::Publisher rplidar_pub = nh.advertise<obstacle_detect::VectorPair>("vector_pair",10);
    while (ros::ok())
	{
        
        for(uint16_t i=0; i<num; i++)
        {
            obstacle_detect::Pair pair;
            pair.distance = rp_arr[i];
            pair.angle = (degree/2.0)*(i+1);
            if(pair.distance != std::numeric_limits<double>::infinity()) 
            {
                vector_pair.data.push_back(pair);
            }
            if(num-i == 1)
            {
                rplidar_pub.publish(vector_pair);
                vector_pair.data.clear();
            }
        }

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
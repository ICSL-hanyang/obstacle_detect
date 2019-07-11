
#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <obstacle_detect.h>
#include "obstacle_detect/Pair.h"
#include "obstacle_detect/VectorPair.h"
#include "obstacle_detect/Orientation.h"

obstacle_detect::VectorPair vector_pair;
obstacle_detect::Pair pair;
obstacle_detect::Orientation o;
double rp_arr[360]={0.0,};
uint8_t degree = 3; // 몇도 마다 자를래
int num = 360/degree; // 원하는 각도로 잘랐을때 나오는 구역의 수
double detecting_range = 1.5;  //detecting_range 보다 작은 값만 받을거다!
double min = std::numeric_limits<double>::infinity();

void rplidarCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    double msg_arr[num]={0.0,}; //배열 초기화
    uint8_t n = 0;              //카운트용
    uint8_t inf_n = 0;          //무한대카운트용
    // printf(msg->ranges[0]);
    // printf(msg->ranges[180]);
    // printf(msg->ranges[270]);
    // printf(msg->ranges[90]);

    for(uint16_t i = 0; i<360; i++) //0부터 359도에 대한 센서값을 받아오기위해
    {                        
        if(i%degree == 0)
        {
            // ROS_INFO_STREAM("---------------------------------------------");
            min = msg->ranges[i];
        }
        // ROS_INFO_STREAM(msg->ranges[i]);
        min =  (min > msg->ranges[i]) ? msg->ranges[i] : min;

        // if(msg->ranges[i] != std::numeric_limits<double>::infinity()) 
        //     msg_arr[n] += msg->ranges[i];
        // else inf_n++;
        
        if(i%degree == degree-1)     //360개의 센서를 degree 만큼 나누고 평균낼거임 
        {
            msg_arr[n] = min;
            // ROS_INFO_STREAM("min : "<<min);
            // if(inf_n >= degree) 
            //     msg_arr[n] = std::numeric_limits<double>::infinity();
            // else 
            //     msg_arr[n] /= (degree - inf_n);   //0부터 시작하니까 나머지가 degree-1일때 평균내야됨
            // if(i==4)
            //     printf("deg : %d  inf_n : %d  msg : %lf\n", degree, inf_n, msg_arr[n]);
            n++;                    //n은 degree만큼 나눈 구역을 하나하나 검사하기위해 증가해주고
            // inf_n = 0;
            if(n>=num) break;       //0부터 시작하니까 n이 num이 되면 스탑
        }
    }
    for(uint16_t i = 0; i < num; i++)
    {
        if(msg_arr[i] >= detecting_range) //detecting_range보다 큰값들은 그냥 무한대 처리
            msg_arr[i] = std::numeric_limits<double>::infinity();
        rp_arr[i] = msg_arr[i];
    }
    // printf("------------------------------------------------------\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detect_node");
	ros::NodeHandle nh("");
	ros::Rate rate(10);
    ros::Subscriber rplidar_sub = nh.subscribe("camila1/laser/scan", 10, &rplidarCB);
    ros::Publisher rplidar_pub = nh.advertise<obstacle_detect::VectorPair>("/vector_pair",10);
    while (ros::ok())
	{
        nh.getParamCached("/swarm_node/setpoint/range_sp", detecting_range);
        for(uint16_t i=0; i<num; i++)
        {
            // printf("angle : %10lf \t distance : %10lf\n", pair.angle, pair.distance);
            if(rp_arr[i] == std::numeric_limits<double>::infinity())
                continue;
            pair.distance = rp_arr[i];
            pair.angle = degree * i + (degree - 1)/2; //5*i+2;
            // 180/degree(5) = 36 355/degree(5) = 71 
            
            vector_pair.data.push_back(pair);
        }
        // if( vector_pair.data.empty()==true ) vector_pair.isEmpty = false;
        // else vector_pair.isEmpty = true;

        rplidar_pub.publish(vector_pair);
        vector_pair.data.clear();
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "obstacle_detect/Pair.h"
#include "obstacle_detect/VectorPair.h"

obstacle_detect::VectorPair vector_pair;
double rp_arr[360]={0.0,};
uint8_t degree = 5; // 몇도 마다 자를래
int num = 360/degree; // 원하는 각도로 잘랐을때 나오는 구역의 수

void rplidarCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    double msg_arr[num]={0.0,}; //배열 초기화
    double max_distance = 2.0;  //max_distance 보다 작은 값만 받을거다!
    uint8_t n = 0;              //카운트용

    // printf("0 : %lf\n",msg->ranges[0]);
    // printf("90 : %lf\n",msg->ranges[90]);
    // printf("180 : %lf\n",msg->ranges[180]);
    // printf("270 : %lf\n",msg->ranges[270]);
    for(uint16_t i = 0; i<=360; i++) //0부터 359도에 대한 센서값을 받아오기위해
    {                                //360을 포함시킨 이유는 코드상 구현땜시 ㅋ
        if(i%degree == degree-1)     //360개의 센서를 degree 만큼 나누고 평균낼거임 
        {
            msg_arr[n] /= degree;   //0부터 시작하니까 나머지가 degree-1일때 평균내야됨
            if(msg_arr[n] >= max_distance) //max_distance보다 큰값들은 그냥 무한대 처리
                msg_arr[n] = std::numeric_limits<double>::infinity();
            n++;                    //n은 degree만큼 나눈 구역을 하나하나 검사하기위해 증가해주고
            if(n>=num) break;       //0부터 시작하니까 n이 num이 되면 스탑
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
            pair.angle = degree * i + (degree - 1)/2; //5*i+2;
            if(pair.distance != std::numeric_limits<double>::infinity()) 
            {
                vector_pair.data.push_back(pair);
                printf("angle : %10lf \t distance : %10lf\n", pair.angle, pair.distance);
            }
        }
        rplidar_pub.publish(vector_pair);
        vector_pair.data.clear();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
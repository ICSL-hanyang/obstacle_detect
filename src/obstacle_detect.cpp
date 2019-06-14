
#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "obstacle_detect/Pair.h"
#include "obstacle_detect/VectorPair.h"
#include <vector>
obstacle_detect::VectorPair vector_pair;
obstacle_detect::VectorPair vector_infinity;
//std::vector<obstacle_detect::Pair> vector_infinity;
//obstacle_detect::VectorPair vector_infinity;  // infinity들을 저장하는 벡터
obstacle_detect::Pair pair;
obstacle_detect::Pair inf_pair;
obstacle_detect::Pair long_pair;
obstacle_detect::Pair min_infinity;  //거리가 무한대인 값들중에 진행방향과 가장 가까운 각도
double rp_arr[360]={0.0,};
uint8_t degree = 5; // 몇도 마다 자를래
int num = 360/degree; // 원하는 각도로 잘랐을때 나오는 구역의 수
double detecting_range = 2.0;  //detecting_range 보다 작은 값만 받을거다!
void rplidarCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    double msg_arr[num]={0.0,}; //배열 초기화
    uint8_t n = 0;              //카운트용

    // printf("0 : %lf\n",msg->ranges[0]);
    // printf("90 : %lf\n",msg->ranges[90]);
    // printf("180 : %lf\n",msg->ranges[180]);
    // printf("270 : %lf\n",msg->ranges[270]);
    for(uint16_t i = 0; i<360; i++) //0부터 359도에 대한 센서값을 받아오기위해
    {                               
        if(i%degree == degree-1)     //360개의 센서를 degree 만큼 나누고 평균낼거임 
        {
            msg_arr[n] /= degree;   //0부터 시작하니까 나머지가 degree-1일때 평균내야됨
            n++;                    //n은 degree만큼 나눈 구역을 하나하나 검사하기위해 증가해주고
            if(n>=num) break;       //0부터 시작하니까 n이 num이 되면 스탑
        }
        msg_arr[n] += msg->ranges[i];
    }
    for(uint16_t i = 0; i < num; i++)
    {
        inf_pair.angle = degree * i + (degree - 1)/2;
        inf_pair.distance = msg_arr[i];
        if(inf_pair.distance == std::numeric_limits<double>::infinity())
            if(inf_pair.angle >= 180 && inf_pair.angle <= 360)
            {
                // printf("angle : %10lf \t distance : %10lf\n", inf_pair.angle, inf_pair.distance);
                vector_infinity.data.push_back(inf_pair);
            }

        if(msg_arr[i] >= detecting_range) //detecting_range보다 큰값들은 그냥 무한대 처리
            msg_arr[i] = std::numeric_limits<double>::infinity();

        rp_arr[i] = msg_arr[i];
    }
        // printf("------------------------------------------------------\n");

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detect_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(5);
    ros::Subscriber rplidar_sub = nh.subscribe("/scan", 10, &rplidarCB);
    ros::Publisher rplidar_pub = nh.advertise<obstacle_detect::VectorPair>("vector_pair",10);
    // ros::Publisher rplidar_pub_inf = nh.advertise<obstacle_detect::VectorPair>("vector_pair_inf",10);
    ros::Publisher rplidar_long_pub = nh.advertise<obstacle_detect::Pair>("longest",10);
    while (ros::ok())
	{
        long_pair.distance = 0;
        min_infinity.angle = 0;
        nh.getParamCached("detecting_range", detecting_range);
        for(uint16_t i=0; i<num; i++)
        {
            pair.distance = rp_arr[i];
            pair.angle = degree * i + (degree - 1)/2; //5*i+2;
            // 180/degree(5) = 36 355/degree(5) = 71 

            if(pair.angle >= 180 && pair.angle <= 360) 
            {
                if(long_pair.distance < pair.distance) 
                {
                    long_pair.distance = pair.distance; 
                    long_pair.angle = pair.angle;
                }
            }
            vector_pair.data.push_back(pair);
            // printf("angle : %10lf \t distance : %10lf\n", pair.angle, pair.distance);
        }
        for(auto &infinity : vector_infinity.data)   //진행방향과 거리가 무한대인 각도의 차이가 최소인 방향을 찾음
        {   
            if(fabs(270-min_infinity.angle) > fabs(270-infinity.angle))
            {
                min_infinity.angle = infinity.angle;
                min_infinity.distance = infinity.distance;
            }
            long_pair.distance = min_infinity.distance;
            long_pair.angle = min_infinity.angle;
            // printf("infan : %10lf \t infdista : %10lf \n", long_pair.angle, long_pair.distance);
        }
        //  printf("l_ang : %10lf \t long_dis : %10lf \n", long_pair.angle, long_pair.distance);

        rplidar_long_pub.publish(long_pair);
        rplidar_pub.publish(vector_pair);
        // rplidar_pub_inf.publish(vector_infinity);

        vector_pair.data.clear();
        vector_infinity.data.clear();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
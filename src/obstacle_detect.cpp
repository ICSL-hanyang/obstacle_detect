
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
uint8_t degree = 5; // 몇도 마다 자를래
int num = 360/degree; // 원하는 각도로 잘랐을때 나오는 구역의 수
double detecting_range = 2.0;  //detecting_range 보다 작은 값만 받을거다!
Detection detect;
double x,y,px,py;
int sec, psec;
bool sec_flag = true;
void rplidarCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    double msg_arr[num]={0.0,}; //배열 초기화
    uint8_t n = 0;              //카운트용
    uint8_t inf_n = 0;          //무한대카운트용
    detect.setLeft(msg->ranges[0]);
    detect.setRight(msg->ranges[180]);
    detect.setFront(msg->ranges[270]);
    detect.setBack(msg->ranges[90]);

    for(uint16_t i = 0; i<360; i++) //0부터 359도에 대한 센서값을 받아오기위해
    {                               
        
        if(msg->ranges[i] != std::numeric_limits<double>::infinity()) msg_arr[n] += msg->ranges[i];
        else inf_n++;
        
        if(i%degree == degree-1)     //360개의 센서를 degree 만큼 나누고 평균낼거임 
        {
            if(inf_n >= degree) msg_arr[n] = std::numeric_limits<double>::infinity();
            else msg_arr[n] /= (degree - inf_n);   //0부터 시작하니까 나머지가 degree-1일때 평균내야됨
            // if(i==4)
            //     printf("deg : %d  inf_n : %d  msg : %lf\n", degree, inf_n, msg_arr[n]);
            n++;                    //n은 degree만큼 나눈 구역을 하나하나 검사하기위해 증가해주고
            inf_n = 0;
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
void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(sec_flag)
    {
        sec = msg->header.stamp.sec;
        x = msg->pose.position.x;
        y = msg->pose.position.y;
    }
    psec = msg->header.stamp.sec;
    px = msg->pose.position.x;
    py = msg->pose.position.y;
}

obstacle_detect::Orientation Detection::orientationDetect()
{
    //left right hovering front back
    // ROS_INFO_STREAM(detect.getLeft());
    // ROS_INFO_STREAM(detect.getRight());
    // ROS_INFO_STREAM(detect.getFront());
    // ROS_INFO_STREAM(detect.getBack());
    if( detect.getLeft() <= detecting_range ) { this->left = false; hovering(); }
    if( detect.getRight() <= detecting_range ) { this->right = false; hovering(); }
    if( detect.getFront() <= detecting_range ) { this->front = false; hovering(); }
    // if( detect.getBack() <= detecting_range ) { this->back = false; hovering(); }
    if( detect.getState() == "hovering" )
    { 
        if(this->front && detect.getFront() >= detecting_range )
        {
            this->right = true;
            // this->back = true;
            this->left = true;
            this->front = true;
            sec_flag = true;
            if(this->front)
                detect.goFront();
        }
        //         if(detect.getLeft() >= detecting_range )
        if(!this->front && detect.getLeft() >= detecting_range )
        {
            // this->left = true;
            if(this->left)
                detect.goLeft();
            if( detect.getFront() >= detecting_range)
                this->front = true;
        }

        // if(!this->left && detect.getRight() >= detecting_range )
        if(!this->front && !this->left && detect.getRight() >= detecting_range )
        {
            if(this->right)
                detect.goRight();
            if( detect.getFront() >= detecting_range)
                this->front = true;
        }
        // if(!this->front && !this->left && !this->right && detect.getBack() >= detecting_range )
        // {
        //     sec_flag = false;
        //     double distance = 10;
        //     if( abs(sec-psec)>=10 )
        //     {
        //         distance = sqrt(pow(x-px, 2.0)+pow(y-py, 2.0));
        //     }
        //     if(this->back)
        //         detect.goBack();
        //     if(distance <= 2)
        //     {
        //         this->right = true;
        //         this->back = true;
        //         this->left = true;
        //         this->front = true;
        //         sec_flag = true;
        //     }
        // }
        // if( !this->left && !this->right && !this->front && !this->back )
        if( !this->left && !this->right )
        {
            detect.hovering();
            // printf("------------------------------------------------------------\n");
            this->right = true;
            // this->back = true;
            this->left = true;
            this->front = true;
            sec_flag = true;
        }
    }
    return detect.getOrient();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detect_node");
	ros::NodeHandle nh("");
	ros::Rate rate(10);
    ros::Subscriber rplidar_sub = nh.subscribe("camila1/laser/scan", 10, &rplidarCB);
    ros::Subscriber local_pose = nh.subscribe("mavros/local_position/pose", 10, &localPositionCB);
    ros::Publisher rplidar_pub = nh.advertise<obstacle_detect::VectorPair>("/vector_pair",10);
    ros::Publisher orient_pub = nh.advertise<obstacle_detect::Orientation>("/orientation",10);
    obstacle_detect::Orientation a;
    while (ros::ok())
	{
        nh.getParamCached("detecting_range", detecting_range);
        for(uint16_t i=0; i<num; i++)
        {
            a = detect.orientationDetect();
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
        orient_pub.publish(a);
        vector_pair.data.clear();
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
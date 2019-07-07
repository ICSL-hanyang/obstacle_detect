#ifndef OBSTACLE_DETECT
#define OBSTACLE_DETECT
#include <ros/ros.h>
#include <string.h>
#include "obstacle_detect/Orientation.h"

class Detection
{
 private :
    double lidar_left;
    double lidar_right;
    double lidar_front;
    double lidar_back;
    obstacle_detect::Orientation orient;
 public :

    bool left = true, right = true, front = true, back = true;
    Detection()
    {
        lidar_left   = 0;
        lidar_right  = 0;
        lidar_front  = 0;
        lidar_back   = 0;
        orient.curr = "hovering";
        orient.prev = "hovering";
    };
    ~Detection(){};
    void setLeft( double n ) { lidar_left = n; }
    void setRight( double n ) { lidar_right = n; }
    void setFront( double n ) { lidar_front = n; }
    void setBack( double n ) { lidar_back = n; }
    double getLeft() { return lidar_left; }
    double getRight() { return lidar_right; }
    double getFront() { return lidar_front; }
    double getBack() { return lidar_back; }
    std::string getState() { return orient.curr; }
    void setState( std::string s )
    {
        orient.prev = getState();
        orient.curr = s;
    }
    void goFront() { setState("front"); }
    void goBack() { setState("back"); }
    void goLeft() { setState("left"); }
    void goRight() { setState("right"); }
    void hovering() { setState("hovering"); }
    obstacle_detect::Orientation getOrient() { return orient; }
    obstacle_detect::Orientation orientationDetect();
};

#endif// !1
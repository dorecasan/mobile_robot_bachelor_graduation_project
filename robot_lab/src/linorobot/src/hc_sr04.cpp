#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Int16.h>
#include<iostream>
using namespace std;
ros::Publisher pub;
ros::Subscriber sub;
void rangeCallBack(const std_msgs::Int16 &range)
{
    //ROS_INFO_STREAM("khoang cach: "<<range.data);
    geometry_msgs::Twist vel;
    // xuat tin hieu xuayy 
    if (range.data >20 && range.data <=25)
    {
       
        vel.angular.z = 0.2;
        vel.linear.x = 0;
        pub.publish(vel);
    }
    // else if (range.data >20)
    // {   
    //     //ROS_INFO_STREAM("range.data2: "<<range.data);
    //     vel.angular.z = 1;
    //     vel.linear.x = 0;
    // }
    // xuat tin hieu lui 
    if(range.data <=20)
    {
        //ROS_INFO_STREAM("range.data3: "<<range.data);
        vel.angular.z = 0;
        vel.linear.x = -0.2;
         pub.publish(vel);
    }
   
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_ultra_sound");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel1",10000);
    sub = n.subscribe("range", 10000, &rangeCallBack);
    ros::spin();
}

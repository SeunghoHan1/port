#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void msgCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
float n = 4.7124/msg->angle_increment;
float maxmeter = msg->ranges[0];
float rad;
for(int i=0; i<n;i++){
     
      if(maxmeter<msg->ranges[i]){
          maxmeter = msg->ranges[i];
	  rad = i*msg->angle_increment;
      }
      // 최대값과 비교해 더 크면 최대값에 저장
  }
  
  ROS_INFO("max distance:%f[m]    angle:%f[rad]",maxmeter,rad);
}


 int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_lidar");
  ros::NodeHandle n;
  ros::Subscriber ros_tutorial_sub = n.subscribe("scan", 1000, msgCallback);
  ros::spin();
}


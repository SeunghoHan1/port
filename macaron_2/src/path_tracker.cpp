#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include "macaron_2/erp42_write.h"
#include "macaron_2/Vel_ld.h"

#define PI M_PI
#define WHEEL_BASE 1.04
#define MAX_STEER 2000
#define MIN_STEER -2000

static macaron_2::erp42_write control; //For use global

double getDistance(double x0, double y0, double x1, double y1) // euclidean transpose
{
    double dist;

    dist = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
    return dist;
}

void goalCallback(const geometry_msgs::Pose::ConstPtr& goal)
{
    float e_ld, ld, sine_alpha, steer;


    e_ld = goal->position.y;
    ld = getDistance(goal->position.x, goal->position.y, 0, 0);
    printf("steer: %f/n",goal->position.x);
    sine_alpha = e_ld/ld;
    steer = atan2(2*WHEEL_BASE*sine_alpha, ld) * 180/PI * 71;
    if (steer > MAX_STEER) {

        steer = MAX_STEER;
    }
    else if (steer < MIN_STEER) {
        steer = MIN_STEER;
    }

    // control.write_E_stop = 0;
    // control.write_gear = 0;
    // control.write_brake = 0;
    control.write_steer = steer;
    printf("steer : %d\n", control.write_steer);

}

void setup_vel(const macaron_2::Vel_ld::ConstPtr& vel_plan)
{   
    control.write_E_stop = 0;
    control.write_gear = vel_plan -> gear;
    control.write_brake = vel_plan -> brake;
    control.write_speed = vel_plan -> speed;
    // printf("speed : %d\n", control.write_speed);
    
    //printf("speed: %f/n",control.write_speed);

    // printf("control");
    // control.write_speed= vel_plan -> speed;
    // control.write_steer = vel_plan -> steer;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_tracker");
    ros::NodeHandle nh;
    ros::Publisher  control_pub = nh.advertise<macaron_2::erp42_write>("erp42_write", 1);
    ros::Subscriber goal_sub = nh.subscribe("next_goal", 1, goalCallback);
    ros::Subscriber vel_planner_sub = nh.subscribe("vel_ld", 1, setup_vel);
    ros::Rate loop_rate(25); // 25Hz
    printf("hey");

    while(ros::ok())
    {
        ros::spinOnce();
        control_pub.publish(control);
        loop_rate.sleep();
    }

    return 0;
}


#include <ros/ros.h>
#include <cmath> //  sin, cos, tan, pi, isnan
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include "macaron_2/Base_path.h"
#include "macaron_2/Test.h"
#include "baseFrameReader.cpp"

using namespace std;

#define MAPLINE_TOTAL 115

class pathPlanner {
    baseFrameReader bfr;
    int si, qi;
    double current_pose[3];
    int path_index, path_length;
    int interval;
    double* qf;
    double** candidate_path;
public:
    pathPlanner(baseFrameReader bfr, int path_index, int path_lenght);
    double** GeneratePath(double* pose_and_heading); //const geometry_msgs::Pose::ConstPtr& pose
private:
    int* Tm_xy2Sl(int ss, double x, double y);
    double* Sl2Tm_xy(int s, double q);
    ~pathPlanner();
};

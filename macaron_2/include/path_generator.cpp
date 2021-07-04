#include "path_planner.h"
#define M_PI 3.14159265358979323846

pathPlanner::pathPlanner(baseFrameReader bfr, int path_index, int path_length)
{
    this->bfr = bfr;
    this->path_index = path_index;
    this->path_length = path_length;
    si = 0;
    qi = 0;
    interval = 3;
    qf = new double[path_index]();
    for (int i = 0; i < path_index; i++)
    {
        qf[i] = interval * (i - (path_index - 1) / 2);
    }
    candidate_path = new double* [path_index];
    for (int i = 0; i < path_index; i++)
    {
        candidate_path[i] = new double[path_index];
    }
}
pathPlanner::~pathPlanner()
{
    for (int i = 0; i < path_index; i++)
        delete[] candidate_path[i];
    delete[] qf, candidate_path;

}
double** pathPlanner::GeneratePath(double* pose_and_heading) //const geometry_msgs::Pose::ConstPtr& pose && heading
{
    double* a1 = new double[path_index](); // Coefficient 1 for candidate path function
    double* a2 = new double[path_index](); // Coefficient 2 for candidate path function
    int* si_qi_info = Tm_xy2Sl(si, pose_and_heading[0], pose_and_heading[1]);
    si = si_qi_info[0];
    qi = si_qi_info[1];
    int* s = new int[path_length]();
    for (int i = 0; i < path_length; i++)
    {
        s[i] = i;
    }
    //s = range(0, _path_length)
    int ds = path_length;
    double theta = pose_and_heading[2] - bfr.BASE_THETA[si];

    // Generate Path
    for (int i = 0; i < path_index; i++) {
        for (int j = 0; j < path_index; j++)
        {
            a1[i] = (ds * tan(theta) - 2 * (qf[i] - qi)) / pow(ds, 3);
            a2[i] = (qf[i] - qi) / pow(ds, 2);
            candidate_path[i][j] = pow(s[j] - ds, 2) * (a1[i] * s[j] - a2[i]) + qf[i];
            _candidate_path_xy[i, j] = self.localize2xy(_si + s[j], _candidate_path[i, j]) // [x,y]
        }
    }
    delete[] a1, a2, si_qi_info, s;
    return candidate_path;
}
int* pathPlanner::Tm_xy2Sl(int ss, double x, double y)
{
    int search = 0;
    int baselength = bfr.length;
    int diff = 100;
    return new int[2]{ 0,0 };
}
double* pathPlanner::Sl2Tm_xy(int s, double q)
{
    double xy_cordinate[2];
    xy_cordinate[0] = bfr.BASE_XY[s, 0] + q * cos(bfr.BASE_THETA[s] + M_PI / 2);
    xy_cordinate[1] = bfr.BASE_XY[s, 1] + q * sin(bfr.BASE_THETA[s] + M_PI / 2);
    return xy_cordinate;
}

/*

    ros::init(argc, argv, "path_tracker");
    ros::NodeHandle nh;
    ros::Publisher  control_pub = nh.advertise<macaron_2::erp42_write>("erp42_write", 1);
    ros::Subscriber goal_sub = nh.subscribe("next_goal", 1, goalCallback);
    ros::Subscriber vel_planner_sub = nh.subscribe("vel_ld", 1, setup_vel);
    ros::Rate loop_rate(25); // 25Hz

    while(ros::ok())
    {
        ros::spinOnce();
        control_pub.publish(control);
        loop_rate.sleep();
    }

    return 0;
}*/
#include <path_planning/obstacles.h>
#include <geometry_msgs/Point.h>
#include <cstdlib>
#include <string>
#include <fstream>

using namespace std;

vector< vector<geometry_msgs::Point> > obstacles::getObstacleArray(geometry_msgs::Point start_point)
{
    vector<geometry_msgs::Point> obstaclePoint;
    geometry_msgs::Point point;

    //first point
    point = start_point;

    obstaclePoint.push_back(point);

    //second point
    point.x = start_point.x + 20;
    point.y = start_point.y;
    point.z = start_point.z;

    obstaclePoint.push_back(point);

    //third point
    point.x = start_point.x + 20;
    point.y = start_point.y + 20;
    point.z = start_point.z;

    obstaclePoint.push_back(point);

    //fourth point
    point.x = start_point.x;
    point.y = start_point.y + 20;
    point.z = start_point.z;
    obstaclePoint.push_back(point);

    //first point again to complete the box
    point = start_point;
    obstaclePoint.push_back(point);

    obstacleArray.push_back(obstaclePoint);

    return obstacleArray;

}

#include <fstream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "group_04_a2/obstacle_finder.h"


/*
    Class for describing a point in 2D
*/
class Point2D
{
    private:
        double x;
        double y;
    public:
        Point2D(double xx,double yy):
            x{xx}, y{yy} {}
        double getX() const { return x;}
        double getY() const { return y;}
};


/*
    Class for describing an obstacle pattern, indexes associated to two gap points and a minima points in their respective vectors
*/
class ObstacleTriangle
{
    private:
        int g1_index; //gap point 1 index
        int lm_index; //local minima point index
        int g2_index; //gap point 2 index

    public:
        ObstacleTriangle(int g1, int lm, int g2):
            g1_index{g1}, lm_index{lm}, g2_index{g2} {}
        
        int getG1_index() const { return g1_index; }
        int getLM_index() const { return lm_index; }
        int getG2_index() const { return g2_index; }
};


/**
 * Computes euclidean distance between two (x,y) points
 * 
 * @param a first point to compute distance
 * @param b second point to compute distance
 * @return distance between those two points
 */
double getEuclideanDistance(Point2D a, Point2D b)
{
    return sqrt( pow(a.getX()-b.getX(),2) + pow(a.getY()-b.getY(),2));
}


/**
 * Transforms a scan measurement into a point
 * 
 * @param rho scan range value
 * @param range index of range vector, for having theta
 * @return the corresponding Point in (x,y)
 */
Point2D polar_to_cartesian(double rho, int range)
{
    const double angle_min = -1.9198600053787231;
    const double  angle_increment = 0.005774015095084906;
    double theta = range * angle_increment;

    double x = rho * cos(theta+angle_min);
    double y = rho * sin(theta+angle_min);    

    return Point2D(x,y);
}


/**
 * Returns all Gap Points
 * 
 * @param ranges vector of Laser_scan measurements in float
 * @param angles_gaps vector of angles associated to ranges angles (to fill) / for gaps
 * @return vector of points, for which the laser_scan recognizes a "gap"
 */
std::vector<Point2D> findAllGapPoints(const std::vector<float> &ranges, std::vector<double>& angles_gaps)
{
    //HYPERPARAMETER: Difference value, in meters, for which two adjacent measurments form a gap
    const double toConsiderItGap = 0.20;

    std::vector<Point2D> gapPoints;
    for(int i=20; i<ranges.size()-21; i++)
    {
        if(abs(ranges[i]-ranges[i+1]) > toConsiderItGap)
        {
            Point2D i_  = polar_to_cartesian(ranges[i], i);
            Point2D i1_ = polar_to_cartesian(ranges[i+1], i+1);
            
            if(ranges[i] > ranges[i+1]) { gapPoints.push_back(i1_);  angles_gaps.push_back( std::atan2(i1_.getY(), i1_.getX()) );}
            else { gapPoints.push_back(i_); angles_gaps.push_back( std::atan2(i_.getY(), i_.getX()) ); }
        }
    }
    return gapPoints;
}


/**
 * Returns all Minima Points
 * 
 * @param ranges vector of Laser_scan measurements in float
 * @param angles_minima vector of angles associated to ranges angles (to fill) / for minima
 * @return vector of points, for which the laser_scan recognizes a "minima" (candidate of obsatcle recognition)
 */
std::vector<Point2D> findAllMinimaPoints(const std::vector<float> &ranges, std::vector<double>& angles_minima)
{
    //HYPERPARAMETER: Interval of ranges values in which we look for decreasing/increasing patterns "U" of measuremnets.
    const int checkWindow = 13;

    std::vector<Point2D> minimaPoints;
    for(int i=20 + checkWindow/2; i<ranges.size()-21 - checkWindow/2; i++)
    {
        bool ok = true;

        float min = 10000.0; float max = -10000.0;
        for(int j=i-checkWindow/2; j<=i; j++)
        {
            if(min>=ranges[j]) { min = ranges[j];}
            else { ok = false; }
        }
        for(int j=i; j<=i+checkWindow/2; j++)
        {
            if(max<ranges[j]) { max = ranges[j];}
            else { ok = false; }
        }
    
        if(ok) {
            Point2D point = polar_to_cartesian(ranges[i+checkWindow/2], i+checkWindow/2 );
            minimaPoints.push_back( point ); i+=checkWindow/2; 
            angles_minima.push_back( std::atan2( point.getY(), point.getX() ));
        }
    }
    return minimaPoints;
}


/**
 * Is an angle between other two angles ?
 * 
 * @param theta1 First angle
 * @param theta2 Second angle
 * @param candidate Angle for which we make the question
 * @return true or false
 */
bool isInTheMiddle( double theta1, double theta2, double candidate )
{
    //Typically data in input are theta1 < theta2
    if(theta1 > 270 && theta2 < 90)
    {
        theta1 = int(theta1+90) % 360;
        theta2 = int(theta2+90) % 360;
        candidate = int(candidate+90) % 360;
    }
    return ( candidate > theta1  && candidate < theta2 ); 

}


/**
 * We find all obstacles of our interest. An obstacle pattern is made of two gap points with a minima point between them
 * 
 * @param minimaPoints vector of minima points
 * @param gapPoints vector of gap points
 * @param angles_gaps vector of angles associated to all gap points
 * @param angles_minima vector of angles associated to all minima points
 * @return all obstacles found
 */
std::vector<ObstacleTriangle> findTriangles(const std::vector<Point2D>& minimaPoints , const std::vector<Point2D>& gapPoints, 
                                                const std::vector<double>& angles_gaps, const std::vector<double>& angles_minima )
{
    //HYPERPARAMETER: Max distance value fot which two gap points can be related to a minima point (typical obstcle pattern for this tasl)
    double maxDistanceInTriangle = 0.5;

    std::vector<ObstacleTriangle> triangles;

    for(int i=0; i<angles_minima.size(); i++)
    {
        double angle_minimum = angles_minima[i];

        for(int j=0; j<angles_gaps.size(); j++)
        {
            double angle_gap_1 = angles_gaps[j];
            double angle_gap_2 = angles_gaps[j+1];

            if(isInTheMiddle(angle_gap_1,angle_gap_2, angle_minimum) && getEuclideanDistance(gapPoints[j],gapPoints[j+1]) < maxDistanceInTriangle)
            {
                ObstacleTriangle ot = ObstacleTriangle(j,i,j+1);
                triangles.push_back(ot);
            }

        }
    }

    return triangles;
}


/**
 * We find all obstacles of our interest. An obstacle pattern is made of two gap points with a minima point between them
 * COUT ALL OBSTACLE COORDINATES IN ROBOT REFERENCE FRAME COORDINATES
 * 
 * @param scan_msg LaserScan message with vector of laser measurements within
 */
std::vector<geometry_msgs::PoseStamped> obstacle_finder_function(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    std::vector<double> angles_gaps;
    std::vector<double> angles_minima;

    std::vector<Point2D> minimaPoints = findAllMinimaPoints(scan_msg->ranges, angles_minima);
    std::vector<Point2D> gapPoints = findAllGapPoints(scan_msg->ranges, angles_gaps);

    std::vector<ObstacleTriangle> triangles = findTriangles(minimaPoints,gapPoints,angles_gaps,angles_minima);

    std::vector<geometry_msgs::PoseStamped> obstacle_poses;

    for(int i=0; i< triangles.size(); i++)
    {
        Point2D gp1 = gapPoints[triangles[i].getG1_index()];
        Point2D gp2 = gapPoints[triangles[i].getG2_index()];

        Point2D obstacle_coords = Point2D( (gp1.getX()+gp2.getX())/2, (gp1.getY()+gp2.getY())/2 );

        geometry_msgs::PoseStamped obstacle_pose;
        obstacle_pose.pose.position.x = obstacle_coords.getX();
        obstacle_pose.pose.position.y = obstacle_coords.getY();

        obstacle_poses.push_back(obstacle_pose);
    }

    return obstacle_poses;
}
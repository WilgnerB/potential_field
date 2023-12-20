#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include <iostream>
#include <vector>

#define PI 3.14159265
double D_MAX = 2.0;  // Maximum distance for obstacle sensing

ros::Publisher pub;
double roll, pitch, yaw;
geometry_msgs::Pose robotPose;

std::vector<Eigen::Vector3d> obstacles;  // Updated obstacle positions

// Define global variables
double a; // half-width of lemniscate
double t; // angle parameter of lemniscate
double Ts; // angle parameter of lemniscate
double x_d, y_d, theta_d; // desired position and orientation of robot on lemniscate
double x, y, theta; // current position and orientation of robot
double x_e, y_e, theta_e; // errors between desired and current position and orientation
double linear_vel, angular_vel; // linear and angular velocity commands for robot
double Vmax = 100; //Max velocity for the robot
double Vtot; //Max velocity for the robot
double d;
double u1, u2;
Eigen::Vector2d goal(8.0, 2.0); // Adjust goal based on your requirements
// Control parameters
double kp = 11.0;
double kp1 = 1.0;
double kp2 = 1.0;
double k_att = 0.05;
double k_rep = 0.1;
double attractive_potential(const Eigen::Vector2d& q, const Eigen::Vector2d& goal) {
    return k_att * (goal - q).norm();
}

double repulsive_potential(const Eigen::Vector2d& q, const Eigen::Vector3d& obs, double R = D_MAX) {
    Eigen::Vector2d v = q - obs.head<2>();
    double d = v.norm();

    if (d <= R) {
        return 0.5 * k_rep * std::pow((1.0 / d - 1.0 / R), 2);
    } else {
        return 0.0;
    }
}

double total_potential(const Eigen::Vector2d& q, const Eigen::Vector2d& goal, const std::vector<Eigen::Vector3d>& obstacles) {
    double U_att = attractive_potential(q, goal);
    double U_rep = 0.0;

    for (const auto& obs : obstacles) {
        U_rep += repulsive_potential(q, obs);
    }

    return U_att + U_rep;
}

Eigen::Vector2d potential_gradient(const Eigen::Vector2d& q, const Eigen::Vector2d& goal, const std::vector<Eigen::Vector3d>& obstacles) {
    const double delta = 0.01; // Small value for numerical differentiation

    double gradient_x = (total_potential(q + Eigen::Vector2d(delta, 0.0), goal, obstacles) - total_potential(q, goal, obstacles)) / delta;
    double gradient_y = (total_potential(q + Eigen::Vector2d(0.0, delta), goal, obstacles) - total_potential(q, goal, obstacles)) / delta;

    return Eigen::Vector2d(gradient_x, gradient_y);
}

void laserMessageReceived(const sensor_msgs::LaserScan& laserScan) {
    // Clear previous obstacle positions
    obstacles.clear();

    // Process laser scan data to obtain obstacle positions
    double x = robotPose.position.x;
    double y = robotPose.position.y;

    for (int i = 0; i < laserScan.ranges.size(); i++) {
        if (laserScan.ranges[i] < D_MAX) {
            double angle = laserScan.angle_min + i * laserScan.angle_increment;
            double obs_x = std::cos(angle) * laserScan.ranges[i];
            double obs_y = std::sin(angle) * laserScan.ranges[i];

            // Transform the obstacle position to the map frame
            double map_x = x + obs_x * std::cos(yaw) - obs_y * std::sin(yaw);
            double map_y = y + obs_x * std::sin(yaw) + obs_y * std::cos(yaw);

            // Store obstacle position
            obstacles.push_back(Eigen::Vector3d(map_x, map_y, 0.0));
        }
    }
}

void poseMessageReceived(const nav_msgs::Odometry::ConstPtr& msg) {
    double x, y, theta; // current position and orientation of robot
    // Get current position of robot
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    // Get current orientation of robot
    tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta); // convert quaternion to Euler angles
    robotPose.position.x = x;
    robotPose.position.y = y;
    robotPose.position.z = theta;


    Eigen::Vector2d robot_position;
    robot_position << x, y;
    goal << x_d, y_d;
    yaw = theta;

    Eigen::Vector2d control_input = -kp * potential_gradient(robot_position, goal, obstacles);

    //Feedback Linearization
    u1 = control_input[0];
    u2 = control_input[1];
    Vtot = sqrt(pow(kp1 * u1, 2) + pow(kp2 * u2, 2));


    if (Vtot >= Vmax){
        u1 = u1 * Vmax / Vtot;
        u2 = u2 * Vmax / Vtot;
    }
        
    // feedback linearization
    Eigen::Matrix2d A;
    A << cos(theta), -d*sin(theta),
            sin(theta), d*cos(theta);

    Eigen::Vector2d vw = A.inverse() * Eigen::Vector2d(u1,u2);


    ROS_INFO("x_d: %f y_d: %f", x_d, y_d);
    ROS_INFO("x_robot: %f y_robot: %f", x, y);

    // Apply inverse dynamics controllers to errors and generate velocity commands for robot
    linear_vel = kp1*vw[0];
    angular_vel = kp2*vw[1];

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;   // Linear velocity
    cmd_vel.angular.z = angular_vel;  // Angular velocity

    // Publish the control command
    pub.publish(cmd_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "potential_field_controller");
    ros::NodeHandle nh;

    // Get parameters from command line or launch file
    ros::param::get("~a", a); // half-width of lemniscate
    ros::param::get("~kp", kp); // half-width of lemniscate
    ros::param::get("~kp1", kp1); // half-width of lemniscate
    ros::param::get("~kp2", kp2); // half-width of lemniscate
    ros::param::get("~k_att", k_att); // half-width of lemniscate
    ros::param::get("~k_rep", k_rep); // half-width of lemniscate
    ros::param::get("~D_MAX", D_MAX); // half-width of lemniscate
    ros::param::get("~x_goal", x_d); // half-width of lemniscate
    ros::param::get("~y_goal", y_d); // half-width of lemniscate
    ros::param::get("~d", d); // half-width of lemniscate

    Eigen::Vector2d goal(x_d, y_d); 

    ros::Subscriber sub = nh.subscribe("base_pose_ground_truth", 1000, &poseMessageReceived);
    ros::Subscriber sub2 = nh.subscribe("base_scan", 1000, &laserMessageReceived);

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

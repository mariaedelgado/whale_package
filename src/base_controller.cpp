#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define wheel_diameter  0.356       // m
#define turning_radius  0.508       // m
#define track_width     1            // m
#define MAX_VEL         2.7778      // m/s
#define pi              3.1415926
#define two_pi          6.2831853

/* Variable definitions */

double vel_actual_right;
double vel_actual_left;
double vel_translational;
double vel_angular;

double dx = 0.0;
double dy = 0.0;
double dtheta = 0.0;
double dt;

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

ros::Time current_time;
ros::Time last_time(0.0);

char base_link[] = "/base_link";
char odom[] = "/odom";


/* Handle messages */

void vel_callback(const geometry_msgs::Vector3Stamped& msg)
{
  vel_actual_right = msg.vector.x;
  vel_actual_left = msg.vector.y;
  dt = msg.vector.z;
  //ROS_INFO("**Subscribed to /vel** Velocity right: %f, Velocity left: %f, Velocity dt: %f", vel_actual_right, vel_actual_left, vel_dt);
}

/* Main function*/

int main(int argc, char **argv)
{
  ROS_INFO("Starting base controller...");

  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("vel", 10, vel_callback);
  ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;
  ros::Rate r(10);

  ROS_INFO("Start reading incoming messages ...");

  while(ros::ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();
 
    /* Transform from velocity to position */
    vel_translational = (vel_actual_right + vel_actual_left)/2;
    vel_angular = ((vel_actual_right - vel_actual_left)*(wheel_diameter/2))/track_width;
    ROS_INFO("VEL TRANS %f, VEL ANGULAR %f", vel_translational, vel_angular);

    dtheta = vel_angular;
    dx = cos(dtheta)*vel_translational;
    dy = -sin(dtheta)*vel_translational;
    

    x_pos += (dx*cos(theta) - dy*sin(theta)) *dt;
    y_pos += (dx*sin(theta) + dy*cos(theta)) *dt;
    theta += dtheta*dt;

    //if(theta >= two_pi) theta -= two_pi;
    //if(theta <= -two_pi) theta += two_pi;

    ROS_INFO("**X POS %f, Y POS %f, THETA %f", x_pos, y_pos, theta);

    /* Publish TF */
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::TransformStamped t;
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_quat;
    t.header.stamp = current_time;

    broadcaster.sendTransform(t);

    /* Publish Odometry topic */
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    
    if( vel_actual_right==0 && vel_actual_left==0 ) {
     odom_msg.pose.covariance[0] = 1e-9;
     odom_msg.pose.covariance[7] = 1e-3;
     odom_msg.pose.covariance[8] = 1e-9;
     odom_msg.pose.covariance[14] = 1e6;
     odom_msg.pose.covariance[21] = 1e6;
     odom_msg.pose.covariance[28] = 1e6;
     odom_msg.pose.covariance[35] = 1e-9;
     odom_msg.twist.covariance[0] = 1e-9;
     odom_msg.twist.covariance[7] = 1e-3;
     odom_msg.twist.covariance[8] = 1e-9;
     odom_msg.twist.covariance[14] = 1e6;
     odom_msg.twist.covariance[21] = 1e6;
     odom_msg.twist.covariance[28] = 1e6;
     odom_msg.twist.covariance[35] = 1e-9;
    }
    else {
     odom_msg.pose.covariance[0] = 1e-3;
     odom_msg.pose.covariance[7] = 1e-3;
     odom_msg.pose.covariance[8] = 0.0;
     odom_msg.pose.covariance[14] = 1e6;
     odom_msg.pose.covariance[21] = 1e6;
     odom_msg.pose.covariance[28] = 1e6;
     odom_msg.pose.covariance[35] = 1e3;
     odom_msg.twist.covariance[0] = 1e-3;
     odom_msg.twist.covariance[7] = 1e-3;
     odom_msg.twist.covariance[8] = 0.0;
     odom_msg.twist.covariance[14] = 1e6;
     odom_msg.twist.covariance[21] = 1e6;
     odom_msg.twist.covariance[28] = 1e6;
     odom_msg.twist.covariance[35] = 1e3;
    }

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = dx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = dtheta;
    
    odometry_publisher.publish(odom_msg);
	
    //ROS_INFO("**Publishing /odom** X: %f, Y: %f, Theta: %f", x_pos, y_pos, theta);
    last_time = current_time;
    r.sleep();
  }

  printf("\nROS: Exiting...\n");
  return 0;


}

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>


void vel_callback(const geometry_msgs::Vector3Stamped& msg)
{
  double vel_actual_right = msg.vector.x;
  double vel_actual_left = msg.vector.y;
  double vel_dt = msg.vector.z;
  ROS_INFO("Velocity right: %f, Velocity left: %f, Velocity dt: %f", vel_actual_right, vel_actual_left, vel_dt);
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting subscriber script ...");

  ros::init(argc, argv, "Subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("vel", 10, vel_callback);
  ros::Rate r(10);

  ROS_INFO("Reading incoming messages ...");
  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  printf("\nROS: Exiting...\n");
  return 0;


}

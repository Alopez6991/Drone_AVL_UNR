#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <math.h>

ros::Publisher enu_odom_pub;
ros::Publisher enu_fake_gps_pub;
const Eigen::Quaternion<double> rot_q(sqrt(2)/2, 0,0,sqrt(2)/2);

void mocap_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped enu;
	enu.header = msg->header;
	enu.pose.position.x = -msg->pose.position.y;
	enu.pose.position.y = msg->pose.position.x;
	enu.pose.position.z = msg->pose.position.z;

	Eigen::Quaternion<double> ori = Eigen::Quaternion<double>(msg->pose.orientation.w,
							  msg->pose.orientation.x,
							  msg->pose.orientation.y,
							  msg->pose.orientation.z);
	
	Eigen::Quaternion<double> transform = rot_q * ori;
	enu.pose.orientation.x = transform.x();
	enu.pose.orientation.y = transform.y();
	enu.pose.orientation.z = transform.z();
	enu.pose.orientation.w = transform.w();
	
	enu_odom_pub.publish(enu);
	enu_fake_gps_pub.publish(enu);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ned2enu");
	ros::NodeHandle nh;
	ros::Subscriber ned_sub = nh.subscribe<geometry_msgs::PoseStamped>("mocap/nwu/pose_stamped", 5, mocap_callback);
	enu_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("vehicle/mocap/pose", 100);
	enu_fake_gps_pub = nh.advertise<geometry_msgs::PoseStamped>("vehicle/fake_gps/mocap/pose", 100);
	ros::Rate rate(20.0);

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
}


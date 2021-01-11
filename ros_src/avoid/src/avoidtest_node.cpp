#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <ros/ros.h>

const int ALPHA_RES = 10;
const int GRID_LENGTH_Z = 360 / ALPHA_RES;

mavros_msgs::State current_state;
sensor_msgs::Range current_range;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
	ROS_INFO("sub");
}
void tfmini_cb(const sensor_msgs::Range::ConstPtr& msg){
	current_range = *msg;
}
int main(int argc, char **argv){

	ros::init(argc, argv, "avoidtest_node");
	ros::NodeHandle nh;
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber tf_mini_sub = nh.subscribe<sensor_msgs::Range>("tfmini_i2c_node/TFmini", 10, tfmini_cb);
	ros::Publisher mavros_obstacle_distance_pub = nh.advertise<sensor_msgs::LaserScan>("mavros/obstacle/send", 10);
	ros::Publisher mavros_system_status_pub = nh.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status",1);
	ros::Rate rate(20.0);
	
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();	
		mavros_msgs::CompanionProcessStatus status_msg = {};
		status_msg.header.stamp = ros::Time::now();
		status_msg.component = 196;
		status_msg.state = 3;

		mavros_system_status_pub.publish(status_msg);
		ROS_INFO("not_connected");
		rate.sleep();
	}
	

	while(ros::ok()){
		sensor_msgs::LaserScan msg = {};
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "local_origin";
		msg.angle_increment = static_cast<double>(ALPHA_RES) * M_PI/180.0;
		msg.range_min = 0.2f;
		msg.range_max = 12.0f;
		msg.ranges.reserve(GRID_LENGTH_Z);
		
		for(int i = 0; i<GRID_LENGTH_Z; ++i){
		//	int j = (i+GRID_LENGTH_Z/2)%GRID_LENGTH_Z;
			if(i == 0 || i== 1 || i==35){
				if(current_range.range>msg.range_min) msg.ranges.push_back(current_range.range);
				else msg.ranges.push_back(msg.range_min+0.01f);
			}
			else msg.ranges.push_back(msg.range_max+0.01f);
			ROS_INFO("%f",current_range.range);
		}

		mavros_obstacle_distance_pub.publish(msg);

		mavros_msgs::CompanionProcessStatus status_msg = {};
		status_msg.header.stamp = ros::Time::now();
		status_msg.component = 196;
		status_msg.state = 3;

		mavros_system_status_pub.publish(status_msg);
		ros::spinOnce();
		rate.sleep();
	}


}

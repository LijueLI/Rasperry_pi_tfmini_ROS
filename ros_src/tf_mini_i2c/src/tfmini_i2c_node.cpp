#include <tfmini_i2c.h>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv){
	ros::init(argc,argv,"tfmini_i2c_node");
	ros::NodeHandle nh("~");
	std::string id = "TFmini";
	std::string portName;
	int address;
	benewake::TFmini_i2c *tfmini_obj;

	nh.param("file_name", portName, std::string("/dev/i2c-1"));
	nh.param("address", address, 0x10);

	tfmini_obj = new benewake::TFmini_i2c(portName, address);
	ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>(id, 1000, true);
	sensor_msgs::Range TFmini_range;
	TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;
	TFmini_range.field_of_view = 0x04;
	TFmini_range.min_range = 0.3;
	TFmini_range.max_range = 12;
	TFmini_range.header.frame_id = id;
	float dist = 0;
	ROS_INFO_STREAM("Start processing...");

	while(ros::master::check() && ros::ok()){
		dist = tfmini_obj->getDist_cm();
		if(dist > 0 && dist < TFmini_range.max_range){
			TFmini_range.range = dist;
			TFmini_range.header.stamp = ros::Time::now();
			pub_range.publish(TFmini_range);
		}
	}
	tfmini_obj->closePort();
}

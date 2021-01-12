#include <tfmini_i2c.h>
#include <vector>
#include <sensor_msgs/Range.h>

int main(int argc, char **argv){
	ros::init(argc,argv,"tfmini_i2c_node");
	ros::NodeHandle nh("~");
	std::string id[4] = {"TFminiFront","TFminiLeft","TFminiBack","TFminiRight"};
	std::string portName;
	int addressFront, addressLeft, addressBack, addressRight;
	
	std::vector<benewake::TFmini_i2c *> tfmini(4);

	nh.param("file_name", 		portName, 		std::string("/dev/i2c-1"));
	nh.param("addressFront",	addressFront,	0x12);
	nh.param("addressLeft",		addressLeft, 	0x13);
	nh.param("addressBack",		addressBack, 	0x14);
	nh.param("addressRight",	addressRight,	0x15);


	tfmini[0] = new benewake::TFmini_i2c(portName, addressFront);
	tfmini[1] = new benewake::TFmini_i2c(portName, addressLeft);
	tfmini[2] = new benewake::TFmini_i2c(portName, addressBack);
	tfmini[3] = new benewake::TFmini_i2c(portName, addressRight);

	ros::Publisher pub_range[4] = { nh.advertise<sensor_msgs::Range>(id[0], 1000, true),
									nh.advertise<sensor_msgs::Range>(id[1], 1000, true),
									nh.advertise<sensor_msgs::Range>(id[2], 1000, true),
									nh.advertise<sensor_msgs::Range>(id[3], 1000, true),};

	sensor_msgs::Range TFmini_range;
	TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;
	TFmini_range.field_of_view = 0x04;
	TFmini_range.min_range = 0.3;
	TFmini_range.max_range = 12;
	float dist = 0;
	ROS_INFO_STREAM("Start processing...");

	while(ros::master::check() && ros::ok()){
		for(int i = 0; i < 4; ++i){
			dist = tfmini[i]->getDist_cm();
			if(dist > 0 && dist < TFmini_range.max_range){
				TFmini_range.header.frame_id = id[i];
				TFmini_range.range = dist;
				TFmini_range.header.stamp = ros::Time::now();
				pub_range[i].publish(TFmini_range);
			}
		}
	}
	tfmini[0]->closePort();
	tfmini[1]->closePort();
	tfmini[2]->closePort();
	tfmini[3]->closePort();
}

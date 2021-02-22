#include <tfmini_i2c.h>
#include <ros/ros.h>

// #define DEBUG
namespace benewake{

	TFmini_i2c::TFmini_i2c(std::string _name, int _address):
		portname_(_name), address_(_address)
		{
			i2c_ = open(_name.c_str(), O_RDWR);
			if(i2c_ == -1){
				ROS_ERROR_STREAM("Failed to open i2c!");
				exit(0);
			}
			if(ioctl(i2c_, I2C_SLAVE, _address)<0){
				ROS_ERROR_STREAM("Failed to get i2c device!");
				exit(0);
			}
		}
	

	bool TFmini_i2c::checkSum(unsigned char *_buf, int size){
		int sum = 0;
		for(int i = 0; i < size-1; ++i){
			sum += (int)_buf[i];
		}
#ifdef DEBUG
		printf("%02x\n", static_cast<unsigned char>(sum));
#endif
		return static_cast<unsigned char>(sum) == _buf[size-1];
	}

	void TFmini_i2c::caculateChecksum(unsigned char *_buf, tfminiplus_packet_leagth_t size){
		int sum = 0;
		for(int i = 0; i < size-1; ++i){
			sum += (int)_buf[i];
		}
		_buf[size-1] = static_cast<unsigned char>(sum);
#ifdef DEBUG
		printf("%02x\n", sum);
		printf("%02x\n", _buf[size-1]);
#endif
	}

	bool TFmini_i2c::readData(unsigned char *_buf, tfminiplus_output_format_t format){
		
		_buf[0] = TFMINI_PLUS_FRAME_START;
		_buf[1] = TFMINI_PLUS_PACK_LENGTH_GET_DATA;
		_buf[2] = TFMINI_PLUS_GET_DATA;
		_buf[3] = format;
		caculateChecksum(_buf, TFMINI_PLUS_PACK_LENGTH_GET_DATA);

		if(write(i2c_, _buf, TFMINI_PLUS_PACK_LENGTH_GET_DATA) != TFMINI_PLUS_PACK_LENGTH_GET_DATA){
			ROS_ERROR_STREAM("Failed to write read cmd!");
		}
		if(read(i2c_, _buf, TFMINI_PLUS_PACK_LENGTH_DATA_RESPONSE) != TFMINI_PLUS_PACK_LENGTH_DATA_RESPONSE){
			ROS_ERROR_STREAM("Failed to read data!");
			return false;
		}
		else{
#ifdef DEBUG
			for(int i = 0; i < 9; i++){
				printf("%02x ", _buf[i]);
			}
			printf("\n");
#endif
			if(!checkSum(_buf, TFMINI_PLUS_PACK_LENGTH_DATA_RESPONSE)){
				ROS_ERROR_STREAM("Data CheckSum Failed!");
				return false;
			}
			return true;
		}
	}

	float TFmini_i2c::getDist_cm(){
		float dist = 0;

		if(readData(dataBuf, TFMINI_PLUS_OUTPUT_CM)){
			if(dataBuf[0] != 0x59 || dataBuf[1] != 0x59){
				ROS_ERROR_STREAM("Data Validation Failed!");
			}
			else{
				dist = static_cast<float>((dataBuf[3] << 8 | dataBuf[2])/100.0);
	#ifdef DEBUG
				printf("%f\n",dist);
	#endif
			}
		}
		return dist;
	}

	float TFmini_i2c::getDist_mm(){
		float dist = 0;

		if(readData(dataBuf, TFMINI_PLUS_OUTPUT_MM)){
			if(dataBuf[0] != 0x59 || dataBuf[1] != 0x59){
				ROS_ERROR_STREAM("Data Validation Failed!");
			}
			else{
				dist = static_cast<float>((dataBuf[3] << 8 | dataBuf[2])/100.0);
	#ifdef DEBUG
				printf("%f\n",dist);
	#endif
			}
		}
		return dist;
	}

	void TFmini_i2c::closePort(){
		close(i2c_);
	}
}

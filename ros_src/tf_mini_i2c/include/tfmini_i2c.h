#include <ros/ros.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

namespace benewake{
#define TFMINI_PLUS_FRAME_START 0x5A
	typedef enum TFMINI_PLUS_COMMANDS {
		TFMINI_PLUS_GET_DATA = 0,
		TFMINI_PLUS_GET_VERSION = 1,
		TFMINI_PLUS_SYSTEM_RESET = 2,
		TFMINI_PLUS_SET_FRAME_RATE = 3,
		TFMINI_PLUS_TRIGGER_DETECTION = 4,
		TFMINI_PLUS_SET_OUTPUT_FORMAT = 5,
		TFMINI_PLUS_SET_BAUD_RATE = 6,
		TFMINI_PLUS_ENABLE_DATA_OUTPUT = 7,
		TFMINI_PLUS_SET_COMMUNICATION_INTERFACE = 0x0A,
		TFMINI_PLUS_SET_I2C_ADDRESS = 0x0B,
		TFMINI_PLUS_SET_IO_MODE = 0x3B,
		TFMINI_PLUS_RESTORE_FACTORY_SETTINGS = 0x10,
		TFMINI_PLUS_SAVE_SETTINGS = 0x11
	} tfminiplus_command_t;

	typedef enum TFMINI_PLUS_PACKET_LENGTHS {
		TFMINI_PLUS_PACK_LENGTH_SYSTEM_RESET_RESPONSE = 5,
		TFMINI_PLUS_PACK_LENGTH_RESTORE_FACTORY_SETTINGS_RESPONSE = 5,
		TFMINI_PLUS_PACK_LENGTH_SAVE_SETTINGS_RESPONSE = 5,
		TFMINI_PLUS_PACK_LENGTH_GET_DATA = 5,
		TFMINI_PLUS_PACK_LENGTH_SET_FRAME_RATE = 6,
		TFMINI_PLUS_PACK_LENGTH_SET_OUTPUT_FORMAT = 5,
		TFMINI_PLUS_PACK_LENGTH_SET_BAUD_RATE = 8,
		TFMINI_PLUS_PACK_LENGTH_ENABLE_DATA_OUTPUT = 5,
		TFMINI_PLUS_PACK_LENGTH_SET_COMMUNICATION_INTERFACE = 5,
		TFMINI_PLUS_PACK_LENGTH_SET_I2C_ADDRESS = 5,
		TFMINI_PLUS_PACK_LENGTH_SET_IO_MODE = 9,
		TFMINI_PLUS_PACK_LENGTH_DATA_RESPONSE = 9,
		TFMINI_PLUS_PACK_LENGTH_VERSION_RESPONSE = 7
	} tfminiplus_packet_leagth_t;

	typedef enum TFMINI_PLUS_OUTPUT_FORMAT {
		TFMINI_PLUS_OUTPUT_CM = 1,
		TFMINI_PLUS_OUTPUT_PIXHAWK = 2,
		TFMINI_PLUS_OUTPUT_MM = 6
	} tfminiplus_output_format_t;

	class TFmini_i2c{
			
			public:
				TFmini_i2c(std::string _name, int _address);
				~TFmini_i2c(){};
				float getDist_cm();
				float getDist_mm();
				void closePort();
				unsigned char dataBuf[10];


			private:
				std::string portname_;
				int address_;
				int i2c_;

				bool checkSum(unsigned char *_buf, int size);
				void caculateChecksum(unsigned char *_buf, tfminiplus_packet_leagth_t size);
				bool readData(unsigned char *_buf, tfminiplus_output_format_t format);
	};
}

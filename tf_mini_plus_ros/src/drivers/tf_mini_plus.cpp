#include <tf_mini_plus_ros/drivers/tf_mini_plus.h>

TFMiniPro::TFMiniPro(uint8 address, uint8 bus)  noexcept(false) { 
	i2c_dev_=new I2C(address,bus); 
 	i2c_dev_->write(address);//check if the device exist on the Bus
	sleep(0.5f);
}

bool TFMiniPro::SendCommand( uint32 cmnd, uint32 param, uint8 *response){
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Build the command data to send to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    static uint8 cmd_length;             		// Length of command
    static uint8 reply_length;            		// Length of command reply data
    static uint8 check_sum = 0;        			// 8 byte send command array
	static uint8 cmnd_data[TFMP_MAX_CMD_LEN];	// 8 byte send command array
	
	memset( cmnd_data, 0, TFMP_MAX_CMD_LEN); 	// Clear the send command array.
	memcpy( &cmnd_data[0], &cmnd, 4);   		// Copy 4 bytes of data: reply length,command length, command number and one parameter
			
	reply_length = cmnd_data[0];            	// Save the first byte as reply length.
    cmd_length = cmnd_data[1];             		// Save the second byte as command length.
    cmnd_data[0] = TFMP_FRAME_HEADER;        	// Set the first byte to the header character.

	if( cmnd == TFMP_SET_FRAME_RATE)           	// If the command is to Set Frame Rate...
      memcpy( &cmnd_data[3], &param, 2);  		// add the 2 byte Frame Rate parameter.
    else if( cmnd == TFMP_SET_BAUD_RATE)       	// If the command is to Set Baud Rate...
      memcpy( &cmnd_data[3], &param, 4);  		// add the 3 byte Baud Rate parameter.
    else if( cmnd == TFMP_SET_I2C_ADDRESS)     	// If the command to set I2C address...
      memcpy( &cmnd_data[3], &param, 1);  		// copy the 1 byte Address parameter.

	check_sum = 0;
    // Add together all bytes but the last...
    for( uint8_t i = 0; i < ( cmd_length - 1); i++) check_sum += cmnd_data[i];
    // and save it as the last byte of command data.
    cmnd_data[cmd_length - 1] = uint8_t(check_sum);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Send the command data array to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	try{ 
		i2c_dev_->write_register_block(TFMP_REG_POINTER, cmd_length, cmnd_data);   	
	}
	catch(std::system_error& e){
		printf("error in TF Mini Pro configuration %s\n", e.what());
		return false;
	}

	// If no reply data expected, then return. Otherwise,
    // wait for device to process the command and continue.
    if( reply_length == 0) return true;
    else if(cmnd != TFMP_I2C_FORMAT_CM) sleep(0.2f);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Get command reply data back from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	try{ 
		for(uint8 i = 0; i < reply_length; ++i){
			response[i] = i2c_dev_->read(i);
		}
	}
	catch(std::system_error& e){
		printf("error reading channel%s\n", e.what());	
		return false;
	}
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 4 - Perform a checksum test.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    uint8 check_sum_response = 0;        				// 8 byte send command array
	for( uint8_t i = 0; i < ( reply_length - 1); i++) check_sum_response += response[i];
	if( response[ reply_length - 1] != (uint8_t)check_sum_response)
    {
		if(cmnd != TFMP_I2C_FORMAT_CM) return true;            // and return "false."
    }
	//TO DO: do something with response
	return true;
}

bool TFMiniPro::ChangeAddress(uint8 new_address){
	bool success = false;
	uint8 cmd_resp[5];	//array where the cmd response will be stored
	// Command to change address distance data in centimeters
	std::cout<<"Changing TF Mini plus address..."<<std::endl;
	if( SendCommand( TFMP_SET_I2C_ADDRESS, new_address, cmd_resp) != true) return success;

	//Save settings
	std::cout<<"Saving settings..."<<std::endl;
	if( SendCommand( TFMP_SAVE_SETTINGS, new_address, cmd_resp) != true) return success;
	sleep(0.5);
	//Change device address
	i2c_dev_->change_address(new_address);
	return true;
}

float TFMiniPro::ReadData () {
	float resp=-1;
	uint8 data[TFMP_DATA_LENGTH]; //array where the data response will be stored
    // Command device to ready distance data in centimeters
	if( SendCommand( TFMP_I2C_FORMAT_CM, 0, data) != true) return resp;

	uint16 dist = data[2] + ( data[3] << 8);
    uint16 flux = data[4] + ( data[5] << 8);
    uint16 temp = data[6] + ( data[7] << 8);
	// convert temp code to degrees Celsius
	temp = ( temp >> 3) - 256;
	resp = (float)dist;
	resp = resp == 0 ? -1 : resp; //avoid false measurements
	return resp/100.0f;  //Convert to meters and return
}


TFMiniPro::~TFMiniPro(){
	i2c_dev_->close_device();
}

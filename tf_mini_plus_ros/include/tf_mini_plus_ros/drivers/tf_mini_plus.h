#ifndef TF_MINI_PLUS_H_
#define TF_MINI_PLUS_H_
 /****************************************************************
 * Constants
 ****************************************************************/
#include <tf_mini_plus_ros/drivers/i2c.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <string.h>


	#define TFMP_MAX_CMD_LEN				8
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define TFMP_DEFAULT_ADDRESS        	(0x10)    // bin 1001 000 (ADDR = GND)
    #define TFMP_DEFAULT_BUS                1        // Default 0 I2C bus for Jetson TX2, J21:3 & J21:5
    #define TFMP_REG_POINTER                (0x00)        // Default 0 I2C bus for Jetson TX2, J21:3 & J21:5
/*=========================================================================*/

/*=========================================================================
    TF MINI Plus Commands (4 bytes including: reply length, command length, command number and one parameter)
	The actual command sent to the sensor will be formed in the sendCommand function so that it is compliant
	with the commands defined in the user guide
    -----------------------------------------------------------------------*/
	#define    TFMP_I2C_FORMAT_CM              	0x01000509   // returns a 9 byte data frame

	#define    TFMP_OBTAIN_FIRMWARE_VERSION    	0x00010407   // returns 3 byte firmware version
	#define    TFMP_SYSTEM_RESET               	0x00020405   // returns a 1 byte pass/fail (0/1)

	#define    TFMP_RESTORE_FACTORY_SETTINGS   	0x00100405   //           "
	#define    TFMP_SAVE_SETTINGS              	0x00110405   // Follow commands to alter parameters.
                                                   // and returns a 1 byte pass/fail (0/1)
	#define    TFMP_SET_FRAME_RATE          0x00030606   // returns an echo of the command
	#define    TFMP_SET_BAUD_RATE           0x00060808   //           "
	#define    TFMP_ENABLE_OUTPUT           0x00070505   //           "
	#define    TFMP_DISABLE_OUTPUT          0x01070505   //           "
	#define    TFMP_SET_I2C_ADDRESS         0x100B0505   // Must be followed by SAVE_SETTINGS
/*=========================================================================*/


/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define TFMP_FRAME_HEADER            (0X5A)
    #define TFMP_DATA_LENGTH             9
/*=========================================================================*/

typedef unsigned int tfMiniProChannel;

enum tfMiniProChannelNumber {
    channel0  = 0,        
	channel1 =  1,	   
	channel2 =  2,     
	channel3 =  3,    
};

class TFMiniPro {
	public:			
		TFMiniPro(uint8 address=TFMP_DEFAULT_ADDRESS, uint8 bus=TFMP_DEFAULT_BUS) noexcept(false);
    		~TFMiniPro() ;	
		float ReadData();
		bool ChangeAddress(uint8 new_address);

	private:	
		I2C * i2c_dev_;
		bool SendCommand( uint32 cmnd, uint32 param, uint8 *response);
};

#endif //ADS1115_H_

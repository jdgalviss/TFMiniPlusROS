#ifndef I2C_H_
#define I2C_H_

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <system_error>
#include <fstream>

#define I2C_DEFAULT_ADDR 0x70 
#define I2C_DEFAULT_BUS 0 

typedef unsigned char uint8;
typedef unsigned int  uint16;
typedef uint32_t  uint32;


class I2C {
	public:
		I2C(uint8 address=I2C_DEFAULT_ADDR, uint8 bus=I2C_DEFAULT_BUS) noexcept(false);
    		~I2C() ;		
		void close_device();		
		uint16 write_register16(uint8 device_register, uint16 value) noexcept(false);
		uint16 write_register_block(uint8 device_register, uint8 length, uint8 *values) noexcept(false);

		uint16 read_register16 (uint16 reg) noexcept(false);
		uint16 read_register_block (uint16 reg, uint8 *values) noexcept(false);

		int write_register8(uint8 device_register, uint8 value) noexcept(false);
		int write(uint8 writeValue) noexcept(false);
		int read( uint8 reg) noexcept(false);
		void change_address(uint8 address);
		
	private:
		void open_device() noexcept(false);
		uint8  kI2CAddress ;                // Base address of device; defaults to 0x70	
		uint8  kI2CBus ;        	    // I2C bus number
		int kI2CFileDescriptor ;            // File Descriptor to the device
	
};

#endif //I2C_H_

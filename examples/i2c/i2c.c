// from: https://raspberry-projects.com/pi/programming-in-c/i2c/using-the-i2c-interface

/*******
 * NOTES
 * I need to get this to run without using sudo and
 * I need this to spit out the data in human readable format
 */

#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdio.h>

void main(int argc, char *argv[]) {

	int file_i2c;
	int length;
	unsigned char buffer[60] = {0};

	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus\n");
		return;
	}
	
	// int addr = 0x5a;          //<<<<<The I2C address of the slave
	// I found the below address by using 'sudo i2cdetect -y 1'
	int addr = 0x68;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return;
	}
	
	
	//----- READ BYTES -----
	length = 4;			//<<< Number of bytes to read
	if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		//ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
	}
	else
	{
		printf("Data read: %s\n", buffer);
	}

	
	//----- WRITE BYTES -----
	buffer[0] = 0x01;
	buffer[1] = 0x02;
	length = 2;			//<<< Number of bytes to write
	int bytes_written = write(file_i2c, buffer, length);
	if (bytes_written != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	{
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus. (bytes written: %d)\n", bytes_written);
	}
	close(file_i2c);
}

/*
from: https://stackoverflow.com/questions/47023552/written-buffer-to-i2c-not-returned-when-read
bool i2cdev_byte_write(int file, uint8_t reg, uint8_t val)
{
    uint8_t bytes[2];

    bytes[0] = reg;
    bytes[1] = val;

    // Write the register followed by the value
    if (write(file, bytes, 2) != 2)
        return false;

    return true;
}

bool i2cdev_bytes_read(int file, uint8_t reg, unsigned int count, uint8_t *out_buf)
{
    if (!out_buf)
        return false;

    // Write the register
    if (write(file, &reg, 1) != 1)
    {
        printf("Failed to write register value\n");
        return false;
    }

    // Read the specified number of bytes
    if (read(file, out_buf, count) != count)
    {
        printf("Failed to read from the i2c bus\n");
        return false;
    }

    return true;
}


*/

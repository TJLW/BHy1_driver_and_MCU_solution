#include "bhy_support.h"
#include "bhy_uc_driver_config.h"
#include "bosch.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <time.h>

#include "i2c.h"









// Notes:
//     Manual BHI160 reset - Set bit 0 to '1' at register address 0x9B
//         sudo i2cset 0 0x29 0x9B 0x1
//
//         Chip_Control register 0x34 will be set low indicating upload disabled and CPU not running
//         Host_Status register 0x35 will have bit 0 set to '1' after manual reset or power cycle
//         Chip_Status register 0x37 will have bits 3,4 set to '1', Firmware idle and no EEPROM, respectively
//         Product_ID register 0x90 will be set to 0x83 - FUSER1_C2, BHI160 - static
//         Revision_ID register 0x91 will be set to 0x01 - FUSER1_C2, BHI160 - static?
//         Upload_CRC registers 0x97-0x9A will be set high indicating no firmware upload
//









/********************************************************************************/
/*                         EXTERN FUNCTION DECLARATIONS                         */
/********************************************************************************/

#if 0
static void hexdump(uint8_t *data, uint8_t len)
{
    for(int i=0; i<len; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");
}
#endif



// This only supports PS-GPIO as the sensor mezzanine only requres these header pins
int8_t ioport_get_pin_level(int gpio_pin)
{
    // Open GPIO file descriptor on Linux (sysfs) - Ultra96 ZynqMP PS-GPIO
    //  Base GPIO address is 338, thus targeted GPIO is 338 + gpio_pin = 374
    //  OS startup script (rc.local) exports these GPIO channels and will be
    //  available in /sys/class/gpio/gpio374
    //  DIRECTION SHOULD ALREADY BE SET TO INPUT

    int gpio = 338 + gpio_pin;

    char gpio_pin_c[4];
    snprintf(gpio_pin_c, 4, "%d" , gpio);

    char* gpio_pin_sysfs_entry;
    gpio_pin_sysfs_entry = malloc(strlen("/sys/class/gpio/gpio") + 3 + strlen("/value"));
    strcpy(gpio_pin_sysfs_entry, "/sys/class/gpio/gpio");
    strcat(gpio_pin_sysfs_entry, gpio_pin_c);
    strcat(gpio_pin_sysfs_entry, "/value");

    // printf("%s\r\n", gpio_pin_sysfs_entry);

    int fd;

    fd = open(gpio_pin_sysfs_entry, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Failed to open gpio fd!\r\n");
        exit(1);
    }

    char value_str[1];
    if (read(fd, value_str, 3) == -1) {
        fprintf(stderr, "Failed to read gpio value!\r\n");
        exit(1);
	}

    close(fd);
    return atoi(value_str);
}




// int8_t bosch_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
// {
//     uint8_t buf[size + 1];
//
//     //printf("sensor_i2c_write 0x%02x:", reg); hexdump(p_buf, size);
//     buf[0] = reg;
//     memcpy(buf + 1, p_buf, size);
//
//     int l = I2C_MasterWrite(I2C_DEVICE, addr << 1, buf, size + 1, 0);
//     //printf("wrote: %d\n", l);
//     return l == size + 1 ? 0 : -1;
//     //return l == size + 1 ? BHY_SUCCESS : BHY_ERROR;
// }
//
// int8_t bosch_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
// {
//     //printf("sensor_i2c_read 0x%02x %d\n", reg, size);
//     if(I2C_MasterWrite(I2C_DEVICE, addr << 1, &reg, 1, 1) == 1) {
//         //printf("setup read ok\n");
//         int l = I2C_MasterRead(I2C_DEVICE, addr << 1, p_buf, size, 0);
//         //printf("read:"); hexdump(p_buf, l);
//         return l == size ? 0 : -1;
//         //return l == size ? BHY_SUCCESS : BHY_ERROR;
//
//     }
//     return -1;
//     //return BHY_ERROR;
// }

/*!
* @brief        Linux write to i2c bus /dev entry
*
*/
int8_t linux_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{

    // int bus;

    // /* Open i2c bus /dev/i2c-0 */
    // if ((bus = i2c_open("/dev/i2c-2")) == -1) {
    //
    // 	/* Error process */
    // }
    //
    // I2CDevice device;
    // memset(&device, 0, sizeof(device));
    //
    // /* 24C04 */
    // device.bus = bus;	/* Bus 0 */
    // device.addr = addr;
    // device.iaddr_bytes = 1;	/* Device internal address is 0 byte */
    // device.page_bytes = 16; /* Device are capable of 16 bytes per page */
    //
    //
    // // unsigned char buffer[256];
    // // ssize_t size = (ssize_t)size;
    // // memset(buffer, 0, sizeof(buffer));
    //
    // /* From i2c 0x0 address read 256 bytes data to buffer */
    // if ((i2c_read(&device, reg, p_buf, size)) != size) {
    //
    // 	/* Error process */
    // }
    //
    // return 0;



    char *filename = "/dev/i2c-2";
    int i2c_fd = -1;

    if ((i2c_fd = open(filename, O_RDWR)) < 0) {
        char err[200];
        sprintf(err, "open('%s') in i2c_init", filename);
        perror(err);
        return -1;
    }

    // NOTE we do not call ioctl with I2C_SLAVE here because we always use the I2C_RDWR ioctl operation to do
    // writes, reads, and combined write-reads. I2C_SLAVE would be used to set the I2C slave address to communicate
    // with. With I2C_RDWR operation, you specify the slave address every time. There is no need to use normal write()
    // or read() syscalls with an I2C device which does not support SMBUS protocol. I2C_RDWR is much better especially
    // for reading device registers which requires a write first before reading the response.


    int retval;
    u8 outbuf[1 + size];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    outbuf[0] = reg;

    for(int i = 0; i < size; i = i + 1){
        outbuf[1 + i] = p_buf[i];
    }

    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].len = size + 1;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    // printf("Writing data %i bytes to register %x:\r\n\t", size, reg);
    // for(int i = 0; i < size; i = i + 1)
    // {
    //     printf("%02x", p_buf[i]);
    // }
    // printf("\r\n");

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_write");
        close(i2c_fd);
        return -1;
    }

    close(i2c_fd);
    return 0;




    // const gchar *buffer;
    //
    //
    // int file;
    // char *filename = "/dev/i2c-2";
    //
    // // Open i2c device
    // if ((file = open(filename, O_RDWR)) < 0) {
    //     /* ERROR HANDLING: you can check errno to see what went wrong */
    //     perror("Failed to open the i2c bus");
    //     ret = -1;
    // }
    //
    // // When you have opened the device, you must specify with what device address you want to communicate:
    // if (ioctl(file, I2C_SLAVE, addr) < 0) {
    //     printf("Failed to acquire bus access and/or talk to slave.\n");
    //     /* ERROR HANDLING; you can check errno to see what went wrong */
    //     ret = -1;
    // }
    //
    // // Write to i2c the register we want to write to
    // if (write(file, &reg, 1) != 1) {
    //     /* ERROR HANDLING: i2c transaction failed */
    //     printf("Failed to write register address to the i2c bus for writing.\n");
    //     buffer = g_strerror(errno);
    //     printf(buffer);
    //     printf("\n\n");
    //
    //     // printf("%d", addr);
    // }
    //
    // // Write the data
    // if (write(file, p_buf, size) != size) {
    //     /* ERROR HANDLING: i2c transaction failed */
    //     printf("Failed to write register data to the i2c bus.\n");
    //     buffer = g_strerror(errno);
    //     printf(buffer);
    //     printf("\n\n");
    //
    //     // printf("%d", addr);
    // }
    //
    // close(file);
    // return ret;
}


/*!
* @brief        Linux write to i2c bus /dev entry
*
*/
int8_t linux_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{

    // int bus;
    //
    // /* Open i2c bus /dev/i2c-0 */
    // if ((bus = i2c_open("/dev/i2c-2")) == -1) {
    //
    // 	/* Error process */
    // }
    //
    // I2CDevice device;
    // memset(&device, 0, sizeof(device));
    //
    // /* 24C04 */
    // device.bus = bus;	/* Bus 0 */
    // device.addr = addr;	/* Slave address is 0x50, 7-bit */
    // device.iaddr_bytes = 1;	/* Device internal address is 0 byte */
    // device.page_bytes = 16; /* Device are capable of 16 bytes per page */
    //
    // // char buffer[256];
    // //
    // // printf("%s", i2c_get_device_desc(&device, buffer, sizeof(buffer)));
    //
    //
    // // unsigned char buffer[256];
    // // ssize_t size = (ssize_t)size;
    // // memset(buffer, 0, sizeof(buffer));
    //
    // /* From i2c 0x0 address read 256 bytes data to buffer */
    // if ((i2c_read(&device, reg, p_buf, size)) != size) {
    //
    // 	/* Error process */
    // }
    //
    // return 0;


    char *filename = "/dev/i2c-2";
    int i2c_fd = -1;

    if ((i2c_fd = open(filename, O_RDWR)) < 0) {
        char err[200];
        sprintf(err, "open('%s') in i2c_init", filename);
        perror(err);
        return -1;
    }

    // NOTE we do not call ioctl with I2C_SLAVE here because we always use the I2C_RDWR ioctl operation to do
    // writes, reads, and combined write-reads. I2C_SLAVE would be used to set the I2C slave address to communicate
    // with. With I2C_RDWR operation, you specify the slave address every time. There is no need to use normal write()
    // or read() syscalls with an I2C device which does not support SMBUS protocol. I2C_RDWR is much better especially
    // for reading device registers which requires a write first before reading the response.


    int retval;
    u8 outbuf[1], inbuf[size];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    outbuf[0] = reg;
    memset(inbuf, 0, sizeof(inbuf));

    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = addr;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = size;
    msgs[1].buf = p_buf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;


    // printf("Reading %i bytes from register %x\r\n", size, reg);

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        close(i2c_fd);
        return -1;
    }

    // printf("P_BUF results:\r\n");
    // for(int i = 0; i < size; i++){
    //     printf("%x\r\n", p_buf[i]);
    // }


    close(i2c_fd);
    return 0;



    // const gchar *buffer;
    //
    // int file;

    // // Open i2c device
    // if ((file = open(filename, O_RDWR)) < 0) {
    //     /* ERROR HANDLING: you can check errno to see what went wrong */
    //     perror("Failed to open the i2c bus");
    //     ret = -1;
    // }
    //
    // // When you have opened the device, you must specify with what device address you want to communicate:
    // if (ioctl(file, I2C_SLAVE, addr) < 0) {
    //     printf("Failed to acquire bus access and/or talk to slave.\n");
    //     /* ERROR HANDLING; you can check errno to see what went wrong */
    //     ret = -1;
    // }
    //
    // // Write to i2c the register we want to write to
    // if (write(file, &reg, 1) != 1) {
    //     /* ERROR HANDLING: i2c transaction failed */
    //     printf("Failed to write register address to the i2c bus for reading.\n");
    //     buffer = g_strerror(errno);
    //     printf(buffer);
    //     printf("\n\n");
    //
    //     // printf("%d", addr);
    // }
    //
    // // When you have opened the device, you must specify with what device address you want to communicate:
    // if (ioctl(file, I2C_SLAVE, addr) < 0) {
    //     printf("Failed to acquire bus access and/or talk to slave.\n");
    //     /* ERROR HANDLING; you can check errno to see what went wrong */
    //     ret = -1;
    // }
    //
    // if (read(file, p_buf, size) != size) {
    //     /* ERROR HANDLING: i2c transaction failed */
    //     printf("Failed to read register data from the i2c bus.\n");
    //     buffer = g_strerror(errno);
    //     printf(buffer);
    //     printf("\n\n");
    //
    //     // printf("%d", addr);
    // }
    //
    // close(file);
    // return ret;
}



void linux_delay(uint32_t msec)
{
    struct timespec ts;
    int res;

    if (msec < 0)
    {
      errno = EINVAL;
      return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
      res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

void linux_printf(const u8 * string)
{
    printf("%s", string);
}


// int8_t bosch_i2c_write_ex(void *_, uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
// {
//     return bosch_i2c_write(addr, reg, p_buf, size);
// }
//
// int8_t bosch_i2c_read_ex(void *_, uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
// {
//     return bosch_i2c_read(addr, reg, p_buf, size);
// }

/*!
  * Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright
  * notice, this list of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright
  * notice, this list of conditions and the following disclaimer in the
  * documentation and/or other materials provided with the distribution.
  *
  * Neither the name of the copyright holder nor the names of the
  * contributors may be used to endorse or promote products derived from
  * this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
  * OR CONTRIBUTORS BE LIABLE FOR ANY
  * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
  * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  * ANY WAY OUT OF THE USE OF THIS
  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
  *
  * The information provided is believed to be accurate and reliable.
  * The copyright holder assumes no responsibility
  * for the consequences of use
  * of such information nor for any infringement of patents or
  * other rights of third parties which may result from its use.
  * No license is granted by implication or otherwise under any patent or
  * patent rights of the copyright holder.
  *
  *
  * @file              bhy_support.c
  *
  * @date              12/19/2016
  *
  * @brief             driver on MCU for bhy
  *
  *
  */


/********************************************************************************/
/*                                  HEADER FILES                                */
/********************************************************************************/
#include "bhy_support.h"
#include "bhy_uc_driver_config.h"

// Linux i2c
#include "bosch.h"

// #include "FreeRTOS.h"
// #include "task.h"


/********************************************************************************/
/*                                STATIC VARIABLES                              */
/********************************************************************************/
static struct bhy_t bhy;
static uint8_t *version = BHY_MCU_REFERENCE_VERSION;

/********************************************************************************/
/*                         EXTERN FUNCTION DECLARATIONS                         */
/********************************************************************************/
// extern int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
// extern int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
// extern void trace_log(const char *fmt, ...);

static int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{
    return linux_i2c_write(addr, reg, p_buf, size) == 0 ? BHY_SUCCESS : BHY_ERROR;
}

static int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
{
    return linux_i2c_read(addr, reg, p_buf, size) == 0 ? BHY_SUCCESS : BHY_ERROR;
}


/********************************************************************************/
/*                     LINUX PLATFORM FUNCTION DECLARATIONS                     */
// /********************************************************************************/
// /*!
// * @brief        Linux write to i2c bus /dev entry
// *
// */
// int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
// {
//
//     char *filename = "/dev/i2c-2";
//     int i2c_fd = -1;
//
//     if ((i2c_fd = open(filename, O_RDWR)) < 0) {
//         char err[200];
//         sprintf(err, "open('%s') in i2c_init", filename);
//         perror(err);
//         return -1;
//     }
//
//     // NOTE we do not call ioctl with I2C_SLAVE here because we always use the I2C_RDWR ioctl operation to do
//     // writes, reads, and combined write-reads. I2C_SLAVE would be used to set the I2C slave address to communicate
//     // with. With I2C_RDWR operation, you specify the slave address every time. There is no need to use normal write()
//     // or read() syscalls with an I2C device which does not support SMBUS protocol. I2C_RDWR is much better especially
//     // for reading device registers which requires a write first before reading the response.
//
//
//     int retval;
//     u8 outbuf[2];
//
//     struct i2c_msg msgs[1];
//     struct i2c_rdwr_ioctl_data msgset[1];
//
//     outbuf[0] = reg;
//     outbuf[1] = p_buf;
//
//     msgs[0].addr = addr;
//     msgs[0].flags = 0;
//     msgs[0].len = 2;
//     msgs[0].buf = outbuf;
//
//     msgset[0].msgs = msgs;
//     msgset[0].nmsgs = 1;
//
//     if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
//         perror("ioctl(I2C_RDWR) in i2c_write");
//         close(i2c_fd);
//         return -1;
//     }
//
//     close(i2c_fd);
//     return 0;
//
//
//
//
//     // const gchar *buffer;
//     //
//     //
//     // int file;
//     // char *filename = "/dev/i2c-2";
//     //
//     // // Open i2c device
//     // if ((file = open(filename, O_RDWR)) < 0) {
//     //     /* ERROR HANDLING: you can check errno to see what went wrong */
//     //     perror("Failed to open the i2c bus");
//     //     ret = -1;
//     // }
//     //
//     // // When you have opened the device, you must specify with what device address you want to communicate:
//     // if (ioctl(file, I2C_SLAVE, addr) < 0) {
//     //     printf("Failed to acquire bus access and/or talk to slave.\n");
//     //     /* ERROR HANDLING; you can check errno to see what went wrong */
//     //     ret = -1;
//     // }
//     //
//     // // Write to i2c the register we want to write to
//     // if (write(file, &reg, 1) != 1) {
//     //     /* ERROR HANDLING: i2c transaction failed */
//     //     printf("Failed to write register address to the i2c bus for writing.\n");
//     //     buffer = g_strerror(errno);
//     //     printf(buffer);
//     //     printf("\n\n");
//     //
//     //     // printf("%d", addr);
//     // }
//     //
//     // // Write the data
//     // if (write(file, p_buf, size) != size) {
//     //     /* ERROR HANDLING: i2c transaction failed */
//     //     printf("Failed to write register data to the i2c bus.\n");
//     //     buffer = g_strerror(errno);
//     //     printf(buffer);
//     //     printf("\n\n");
//     //
//     //     // printf("%d", addr);
//     // }
//     //
//     // close(file);
//     // return ret;
// }
//
//
// /*!
// * @brief        Linux write to i2c bus /dev entry
// *
// */
// int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size)
// {
//     char *filename = "/dev/i2c-2";
//     int i2c_fd = -1;
//
//     if ((i2c_fd = open(filename, O_RDWR)) < 0) {
//         char err[200];
//         sprintf(err, "open('%s') in i2c_init", filename);
//         perror(err);
//         return -1;
//     }
//
//     // NOTE we do not call ioctl with I2C_SLAVE here because we always use the I2C_RDWR ioctl operation to do
//     // writes, reads, and combined write-reads. I2C_SLAVE would be used to set the I2C slave address to communicate
//     // with. With I2C_RDWR operation, you specify the slave address every time. There is no need to use normal write()
//     // or read() syscalls with an I2C device which does not support SMBUS protocol. I2C_RDWR is much better especially
//     // for reading device registers which requires a write first before reading the response.
//
//
//     int retval;
//     u8 outbuf[1], inbuf[1];
//     struct i2c_msg msgs[2];
//     struct i2c_rdwr_ioctl_data msgset[1];
//
//     msgs[0].addr = addr;
//     msgs[0].flags = 0;
//     msgs[0].len = 1;
//     msgs[0].buf = outbuf;
//
//     msgs[1].addr = addr;
//     msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
//     msgs[1].len = size;
//     msgs[1].buf = inbuf;
//
//     msgset[0].msgs = msgs;
//     msgset[0].nmsgs = 2;
//
//     outbuf[0] = reg;
//
//     inbuf[0] = 0;
//
//     *p_buf = 0;
//     if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
//         perror("ioctl(I2C_RDWR) in i2c_read");
//         close(i2c_fd);
//         return -1;
//     }
//
//     *p_buf = inbuf[0];
//     close(i2c_fd);
//     return 0;
//
//
//
//     // const gchar *buffer;
//     //
//     // int file;
//
//     // // Open i2c device
//     // if ((file = open(filename, O_RDWR)) < 0) {
//     //     /* ERROR HANDLING: you can check errno to see what went wrong */
//     //     perror("Failed to open the i2c bus");
//     //     ret = -1;
//     // }
//     //
//     // // When you have opened the device, you must specify with what device address you want to communicate:
//     // if (ioctl(file, I2C_SLAVE, addr) < 0) {
//     //     printf("Failed to acquire bus access and/or talk to slave.\n");
//     //     /* ERROR HANDLING; you can check errno to see what went wrong */
//     //     ret = -1;
//     // }
//     //
//     // // Write to i2c the register we want to write to
//     // if (write(file, &reg, 1) != 1) {
//     //     /* ERROR HANDLING: i2c transaction failed */
//     //     printf("Failed to write register address to the i2c bus for reading.\n");
//     //     buffer = g_strerror(errno);
//     //     printf(buffer);
//     //     printf("\n\n");
//     //
//     //     // printf("%d", addr);
//     // }
//     //
//     // // When you have opened the device, you must specify with what device address you want to communicate:
//     // if (ioctl(file, I2C_SLAVE, addr) < 0) {
//     //     printf("Failed to acquire bus access and/or talk to slave.\n");
//     //     /* ERROR HANDLING; you can check errno to see what went wrong */
//     //     ret = -1;
//     // }
//     //
//     // if (read(file, p_buf, size) != size) {
//     //     /* ERROR HANDLING: i2c transaction failed */
//     //     printf("Failed to read register data from the i2c bus.\n");
//     //     buffer = g_strerror(errno);
//     //     printf(buffer);
//     //     printf("\n\n");
//     //
//     //     // printf("%d", addr);
//     // }
//     //
//     // close(file);
//     // return ret;
// }

/********************************************************************************/
/*                             FUNCTION DECLARATIONS                            */
/********************************************************************************/
/*!
* @brief        Initializes BHY smart sensor and its required connections
*
*/
int8_t bhy_initialize_support(void)
{
    uint8_t tmp_retry = RETRY_NUM;

    bhy.bus_write = &sensor_i2c_write;
    bhy.bus_read = &sensor_i2c_read;
    bhy.delay_msec  = &bhy_delay_msec;
    bhy.device_addr = BHY_I2C_SLAVE_ADDRESS;

    bhy_init(&bhy);

    bhy_set_reset_request(BHY_RESET_ENABLE);;

    while(tmp_retry--)
    {
        bhy_get_product_id(&bhy.product_id);

        if(PRODUCT_ID_7183 == bhy.product_id)
        {
            return BHY_SUCCESS;
        }

        bhy_delay_msec(BHY_PARAMETER_ACK_DELAY);
    }

    return BHY_PRODUCT_ID_ERROR;
}
/*!
* @brief        Initiates a delay of the length of the argument in milliseconds
*
* @param[in]    msec    Delay length in terms of milliseconds
*
*/
void bhy_delay_msec(uint32_t msec)
{
    // vTaskDelay(msec);
    // sleep(msec);

    linux_delay(msec);

}
/*!
 * @brief provides a print function to the bhy driver on DD2.0 platform
 */
void bhy_printf(const u8 * string)
{
    // trace_log("%s",string);

    linux_printf(string)
}
/*!
 * @brief provides the mcu reference code version
 */
uint8_t * bhy_get_version(void)
{
    return (version);
}
/** @}*/

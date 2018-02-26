/*!
  * Copyright (C) Robert Bosch. All Rights Reserved.
  *
  * <Disclaimer>
  * Common: Bosch Sensortec products are developed for the consumer goods
  * industry. They may only be used within the parameters of the respective valid
  * product data sheet.  Bosch Sensortec products are provided with the express
  * understanding that there is no warranty of fitness for a particular purpose.
  * They are not fit for use in life-sustaining, safety or security sensitive
  * systems or any system or device that may lead to bodily harm or property
  * damage if the system or device malfunctions. In addition, Bosch Sensortec
  * products are not fit for use in products which interact with motor vehicle
  * systems.  The resale and/or use of products are at the purchaser's own risk
  * and his own responsibility. The examination of fitness for the intended use
  * is the sole responsibility of the Purchaser.
  *
  * The purchaser shall indemnify Bosch Sensortec from all third party claims,
  * including any claims for incidental, or consequential damages, arising from
  * any product use not covered by the parameters of the respective valid product
  * data sheet or not approved by Bosch Sensortec and reimburse Bosch Sensortec
  * for all costs in connection with such claims.
  *
  * The purchaser must monitor the market for the purchased products,
  * particularly with regard to product safety and inform Bosch Sensortec without
  * delay of all security relevant incidents.
  *
  * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary
  * from the valid technical specifications of the product series. They are
  * therefore not intended or fit for resale to third parties or for use in end
  * products. Their sole purpose is internal client testing. The testing of an
  * engineering sample may in no way replace the testing of a product series.
  * Bosch Sensortec assumes no liability for the use of engineering samples. By
  * accepting the engineering samples, the Purchaser agrees to indemnify Bosch
  * Sensortec from all claims arising from the use of engineering samples.
  *
  * Special: This software module (hereinafter called "Software") and any
  * information on application-sheets (hereinafter called "Information") is
  * provided free of charge for the sole purpose to support your application
  * work. The Software and Information is subject to the following terms and
  * conditions:
  *
  * The Software is specifically designed for the exclusive use for Bosch
  * Sensortec products by personnel who have special experience and training. Do
  * not use this Software if you do not have the proper experience or training.
  *
  * This Software package is provided `` as is `` and without any expressed or
  * implied warranties, including without limitation, the implied warranties of
  * merchantability and fitness for a particular purpose.
  *
  * Bosch Sensortec and their representatives and agents deny any liability for
  * the functional impairment of this Software in terms of fitness, performance
  * and safety. Bosch Sensortec and their representatives and agents shall not be
  * liable for any direct or indirect damages or injury, except as otherwise
  * stipulated in mandatory applicable law.
  *
  * The Information provided is believed to be accurate and reliable. Bosch
  * Sensortec assumes no responsibility for the consequences of use of such
  * Information nor for any infringement of patents or other rights of third
  * parties which may result from its use. No license is granted by implication
  * or otherwise under any patent or patent rights of Bosch. Specifications
  * mentioned in the Information are subject to change without notice.
  *
  */


/********************************************************************************/
/*                                  HEADER FILES                                */
/********************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include "asf.h"
#include "task.h"
#include "arm_math.h"
#include "demo-tasks.h"

#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include ".\firmware\Bosch_PCB_7183_di03_BMI160_BMM150-7183_di03.2.1.11696_170103.h"



/********************************************************************************/
/*                                       MACROS                                 */
/********************************************************************************/
/* should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1 */
#define FIFO_SIZE                      300
#define ROTATION_VECTOR_SAMPLE_RATE    100
#define MAX_PACKET_LENGTH              18
#define OUT_BUFFER_SIZE                60


#define BHY_PARAM_OFFSET_ACC_GET    6
#define BHY_PARAM_OFFSET_MAG_GET    7
#define BHY_PARAM_OFFSET_GYRO_GET    8

#define BHY_PARAM_OFFSET_ACC_SET    15
#define BHY_PARAM_OFFSET_MAG_SET    16
#define BHY_PARAM_OFFSET_GYRO_SET    17

/*!
*   @brief Sensor configuration
*   This structure holds calib profile informations
*/
typedef struct
{
    s16  x_offset; /** 3-axis offset */
    s16  y_offset; /** 3-axis offset */
    s16  z_offset; /** 3-axis offset */
    s16  radius;   /** radius */
}ts_calibparam;

/*!
*   struct ts_calibparam
*   brief calibration parameters
*/
typedef struct
{
    ts_calibparam calibParam;
    uint8_t    accuracy;
}ts_calibprofile;



/********************************************************************************/
/*                                GLOBAL VARIABLES                              */
/********************************************************************************/



/********************************************************************************/
/*                                STATIC VARIABLES                              */
/********************************************************************************/
char out_buffer[OUT_BUFFER_SIZE] = " W: 0.999  X: 0.999  Y: 0.999  Z: 0.999   \r";

uint8_t fifo[FIFO_SIZE];

ts_calibprofile calib_profile[3];


/********************************************************************************/
/*                                 FUNCTIONS                                    */
/********************************************************************************/

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_calib_profile(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    float temp;
    uint8_t index;

    temp = sensor_data->data_quaternion.w / 16384.0f; /* change the data unit by dividing 16384 */
    out_buffer[3] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[4] = floor(temp) + '0';

    for (index = 6; index <= 8; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.x / 16384.0f;
    out_buffer[13] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[14] = floor(temp) + '0';

    for (index = 16; index <= 18; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.y / 16384.0f;
    out_buffer[23] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[24] = floor(temp) + '0';

    for (index = 26; index <= 28; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }

    temp = sensor_data->data_quaternion.z / 16384.0f;
    out_buffer[33] = temp < 0 ? '-' : ' ';
    temp = temp < 0 ? -temp : temp;
    out_buffer[34] = floor(temp) + '0';

    for (index = 36; index <= 38; index++)
    {
        temp = (temp - floor(temp)) * 10;
        out_buffer[index] = floor(temp) + '0';
    }


    DEBUG("x=%d, y=%d, z=%d, w=%d\n",
    sensor_data->data_quaternion.x,
    sensor_data->data_quaternion.y,
    sensor_data->data_quaternion.z,
    sensor_data->data_quaternion.w
    );
}
/*!
 * @brief This function is used to run bhy hub
 */
void demo_sensor(void)
{
    int8_t ret;

    /* BHY Variable*/
    uint8_t                    *fifoptr           = NULL;
    uint8_t                    bytes_left_in_fifo = 0;
    uint16_t                   bytes_remaining    = 0;
    uint16_t                   bytes_read         = 0;
    bhy_data_generic_t         fifo_packet;
    bhy_data_type_t            packet_type;
    BHY_RETURN_FUNCTION_TYPE   result;
    uint32_t                   count = 0;
    uint32_t                   i;
    /* the remapping matrix for BHA or BHI here should be configured according to its placement on customer's PCB. */
    /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
    int8_t                     bhy_mapping_matrix_config[3*3] = {0,1,0,-1,0,0,0,0,1};
    /* the remapping matrix for Magnetometer should be configured according to its placement on customer's PCB.  */
    /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
    int8_t                     mag_mapping_matrix_config[3*3] = {0,1,0,1,0,0,0,0,-1};
    struct cus_version_t      bhy_cus_version;


    /* init the bhy chip */
    if(bhy_driver_init(&bhy1_fw))
    {
        DEBUG("Fail to init bhy\n");
    }

    /* wait for the bhy trigger the interrupt pin go down and up again */
    while (ioport_get_pin_level(BHY_INT));

    while (!ioport_get_pin_level(BHY_INT));

    bhy_read_parameter_page(BHY_PAGE_2, PAGE2_CUS_FIRMWARE_VERSION, (uint8_t*)&bhy_cus_version, sizeof(struct cus_version_t));
    DEBUG("cus version base:%d major:%d minor:%d\n", bhy_cus_version.base, bhy_cus_version.major, bhy_cus_version.minor);

    /* the remapping matrix for BHI and Magmetometer should be configured here to make sure rotation vector is */
    /* calculated in a correct coordinates system. */
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_ACC, bhy_mapping_matrix_config);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_MAG, mag_mapping_matrix_config);
    bhy_mapping_matrix_set(PHYSICAL_SENSOR_INDEX_GYRO, bhy_mapping_matrix_config);

    /* install the callback function for parse fifo data */
    if(bhy_install_sensor_callback(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, sensors_callback_calib_profile))
    {
        DEBUG("Fail to install sensor callback\n");
    }


do_calibrate:
    
    /* first time run rotation vector, do calibrate actions for a while */
    DEBUG("Presss button1 and do calibrate with BSX running...\n");
    while(gpio_pin_is_high(GPIO_PUSH_BUTTON_1));


    /* enables the virtual sensor */
    if(bhy_enable_virtual_sensor(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, ROTATION_VECTOR_SAMPLE_RATE, 0, VS_FLUSH_NONE, 0, 0))
    {
        DEBUG("Fail to enable sensor id=%d\n", VS_TYPE_ROTATION_VECTOR);
    }

    while(count++ < 1500)
    {
        /* wait until the interrupt fires */
        /* unless we already know there are bytes remaining in the fifo */
        while (!ioport_get_pin_level(BHY_INT) && !bytes_remaining)
        {
        }

        bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read           += bytes_left_in_fifo;
        fifoptr              = fifo;
        packet_type          = BHY_DATA_TYPE_PADDING;

        do
        {
            /* this function will call callbacks that are registered */
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);

            /* prints all the debug packets */
            if (packet_type == BHY_DATA_TYPE_DEBUG)
            {
                bhy_print_debug_packet(&fifo_packet.data_debug, bhy_printf);
            }
            
            /* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
            /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
            /* packet */
        } while ((result == BHY_SUCCESS) && (bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)));

        bytes_left_in_fifo = 0;

        if (bytes_remaining)
        {
            /* shifts the remaining bytes to the beginning of the buffer */
            while (bytes_left_in_fifo < bytes_read)
            {
                fifo[bytes_left_in_fifo++] = *(fifoptr++);
            }
        }
    }
    count = 0;
    
    /* disable the rotation vector sensor */
    if(bhy_disable_virtual_sensor(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP))
    {
        DEBUG("Fail to disable sensor\n");
    }

    /* get the calibrate profile(acc mag gyro) from bhy and save to the host side. */
    for(i = 0; i < 3; i++)
    {
        if(bhy_read_parameter_page(BHY_PAGE_2, BHY_PARAM_OFFSET_ACC_GET+i, &calib_profile[i], sizeof(ts_calibprofile)))
        {
            DEBUG("calibrate profile %d loaded. 0x%d, x=0x%x, y=0x%x, z=0x%x,r=0x%d\n",
                i,
                calib_profile[i].accuracy,
                calib_profile[i].calibParam.x_offset,
                calib_profile[i].calibParam.y_offset,
                calib_profile[i].calibParam.z_offset,
                calib_profile[i].calibParam.radius);
        }
        else
        {
            DEBUG("calibrate read failed on %d, re-do calibrate..\n", i);
            goto do_calibrate;
        }

        /* set the valid profile(accuracy=3) to bhy */
        if(calib_profile[i].accuracy != 3)
        {
            DEBUG("calibrate accuracy failed on %d, re-do calibrate..\n", i);
            goto do_calibrate;
        }
    }
    DEBUG("profile save to host side success\n");


    DEBUG("enable the rotation vector sensor again with set calibrate profile(acc mag gyro) in advance\n");
    for(i = 0; i < 3; i++)
    {
        if(calib_profile[i].accuracy != 3
            || bhy_write_parameter_page(BHY_PAGE_2, BHY_PARAM_OFFSET_ACC_SET+i, &calib_profile[i].calibParam, sizeof(ts_calibparam)))
        {
            DEBUG("calibrate set failed on %d ...\n", i);
        }
    }
    
    if(bhy_enable_virtual_sensor(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, ROTATION_VECTOR_SAMPLE_RATE, 0, VS_FLUSH_NONE, 0, 0))
    {
        DEBUG("Fail to enable sensor\n");
    }
    
    while(1)
    {
        /* wait until the interrupt fires */
        /* unless we already know there are bytes remaining in the fifo */
        while (!ioport_get_pin_level(BHY_INT) && !bytes_remaining)
        {
        }

        bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read           += bytes_left_in_fifo;
        fifoptr              = fifo;
        packet_type          = BHY_DATA_TYPE_PADDING;

        do
        {
            /* this function will call callbacks that are registered */
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);

            /* prints all the debug packets */
            if (packet_type == BHY_DATA_TYPE_DEBUG)
            {
                bhy_print_debug_packet(&fifo_packet.data_debug, bhy_printf);
            }
            
            /* the logic here is that if doing a partial parsing of the fifo, then we should not parse  */
            /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
            /* packet */
        } while ((result == BHY_SUCCESS) && (bytes_read > (bytes_remaining ? MAX_PACKET_LENGTH : 0)));

        bytes_left_in_fifo = 0;

        if (bytes_remaining)
        {
            /* shifts the remaining bytes to the beginning of the buffer */
            while (bytes_left_in_fifo < bytes_read)
            {
                fifo[bytes_left_in_fifo++] = *(fifoptr++);
            }
        }
    }
}
/** @}*/

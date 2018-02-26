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
#include ".\firmware\Bosch_PCB_7183_di03_BMI160-7183_di03.2.1.11696_170103.h"



/********************************************************************************/
/*                                       MACROS                                 */
/********************************************************************************/
/* should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1 */
#define FIFO_SIZE                      300
#define MAX_PACKET_LENGTH              18
#define TICKS_IN_ONE_SECOND            32000.0F

/********************************************************************************/
/*                                GLOBAL VARIABLES                              */
/********************************************************************************/



/********************************************************************************/
/*                                STATIC VARIABLES                              */
/********************************************************************************/
/* system timestamp */
static uint32_t bhy_system_timestamp = 0;

uint8_t fifo[FIFO_SIZE];


/********************************************************************************/
/*                                 FUNCTIONS                                    */
/********************************************************************************/

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    /* Since a timestamp is always sent before every new data, and that the callbacks   */
    /* are called while the parsing is done, then the system timestamp is always equal  */
    /* to the sample timestamp. (in callback mode only)                                 */
    float time_stamp = (float)(bhy_system_timestamp) / TICKS_IN_ONE_SECOND;
    
    DEBUG("sensor_id %d\n", sensor_id);

    switch(sensor_id)
    {
        case VS_ID_ACTIVITY:
        case VS_ID_ACTIVITY_WAKEUP:
            DEBUG("activity recognized %f\n", time_stamp);
            break;

        default:
            DEBUG("unknown id = %d\n", sensor_id);
            break;
    }

    /* activity recognition is not time critical, so let's wait a little bit */
    delay_ms(200);
}

/*!
 * @brief This function is time stamp callback function that process data in fifo.
 *
 * @param[in]   new_timestamp
 */
static void timestamp_callback(bhy_data_scalar_u16_t *new_timestamp)
{
    /* updates the system timestamp */
    bhy_update_system_timestamp(new_timestamp, &bhy_system_timestamp);
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

    /* enables the activity recognition and assigns the callback */
    bhy_enable_virtual_sensor(VS_TYPE_ACTIVITY_RECOGNITION, VS_NON_WAKEUP, 1, 0, VS_FLUSH_NONE, 0, 0);
    
    bhy_install_sensor_callback(VS_TYPE_ACTIVITY_RECOGNITION, VS_NON_WAKEUP, sensors_callback);
    bhy_install_timestamp_callback(VS_NON_WAKEUP, timestamp_callback);

    DEBUG("%s", "System is now monitoring activity, press SW1 \n");

    /* wait for the push-button to be pressed */
    while(gpio_pin_is_high(GPIO_PUSH_BUTTON_1));

    DEBUG("%s", "Here is what has been recorded \n");

    /* continuously read and parse the fifo after the push of the button*/
    while (true)
    {
        bhy_read_fifo(fifo+bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        bytes_read         += bytes_left_in_fifo;
        fifoptr             = fifo;
        packet_type        = BHY_DATA_TYPE_PADDING;

        if (bytes_read)
        {
            do
            {
                /* this function will call callbacks that are registered */
                result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);
                /* the logic here is that if doing a partial parsing of the fifo, then we should not parse        */
                /* the last 18 bytes (max length of a packet) so that we don't try to parse an incomplete   */
                /* packet                                                                                                            */
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
        else
        {
            /* activity recognition is not time critical, so let's wait a little bit */
            delay_ms(100);
        }
    }

    return BHY_SUCCESS;
}
/** @}*/

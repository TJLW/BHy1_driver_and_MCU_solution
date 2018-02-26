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
#include "demo-tasks.h"

#include "bhy.h"
#include "bhy_support.h"
#include "bhy_uc_driver.h"
#include ".\firmware\Bosch_PCB_7183_di03_BMA2x2_Cus-7183_di03.2.1.11703.h"



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
uint8_t fifo[FIFO_SIZE];

static uint32_t bhy_system_timestamp = 0;
static uint32_t bhy_system_timestamp_previous = 0;

struct accel_physical_status_t phy_acc;
struct gyro_physical_status_t phy_gyro;
struct mag_physical_status_t phy_mag;
uint8_t physical_sensor_present_bitmap[8];

/********************************************************************************/
/*                                 FUNCTIONS                                    */
/********************************************************************************/
/*!
 * @brief This function is  callback function for get meta event
 *
 * @param[in]   event_data
 * @param[in]   event_type
 */
static void meta_event_callback(bhy_data_meta_event_t *event_data, bhy_meta_event_type_t event_type)
{
    switch(event_type)
    {
        case BHY_META_EVENT_TYPE_INITIALIZED:
            DEBUG("initialize success!\n");
            break;

        case BHY_META_EVENT_TYPE_SELF_TEST_RESULTS:
            if(event_data->event_specific == BHY_SUCCESS)
                DEBUG("self test result success!  sensor_type=%d \n", event_data->sensor_type);
            else
                DEBUG("self test result fail!  sensor_type=%d  \n", event_data->sensor_type);
            break;
        default:
            DEBUG("unknown meta event\n");
            break;
    }
}

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    uint16_t i = 0;
    
    switch(sensor_id)
    {
        case VS_ID_CUS1:
        case VS_ID_CUS1_WAKEUP:
            DEBUG("Cus1 id = %d      len = %d ", sensor_id, bhy_get_cus_evt_size(VS_ID_CUS1));
            for(i = 0; i < (bhy_get_cus_evt_size(VS_ID_CUS1) - 1); i++)
            {
                DEBUG("%2x ", sensor_data->data_custom.data[i]);
            }
            DEBUG("\n");
            break;
        default:
            DEBUG("unknown id = %d\n", sensor_id);
            break;
    }
}

/*!
 * @brief This function is used to run bhy hub
 */
void demo_sensor(void)
{
    /* BHY Variable*/
    uint8_t                    *fifoptr           = NULL;
    uint8_t                    bytes_left_in_fifo = 0;
    uint16_t                   bytes_remaining    = 0;
    uint16_t                   bytes_read         = 0;
    uint32_t                   i                  = 0;
    bhy_data_generic_t         fifo_packet;
    bhy_data_type_t            packet_type;
    BHY_RETURN_FUNCTION_TYPE   result;
    struct sensor_information_non_wakeup_t sensor_info_non_wakeup;
    struct sensor_information_wakeup_t sensor_info_wakeup;
    struct cus_version_t      bhy_cus_version;

    /* If custom sensor is related to IMU sensor, then the remapping matrix for BHA or BHI here should 
    /* be configured according to its placement on customer's PCB. */
    /* for details, please check 'Application Notes Axes remapping of BHA250(B)/BHI160(B)' document. */
    /* for how to configure remapping matrix, please check example of 'acceleromete_remapping_example.c'. */

    bhy_install_meta_event_callback(BHY_META_EVENT_TYPE_INITIALIZED, meta_event_callback);
    bhy_install_meta_event_callback(BHY_META_EVENT_TYPE_SELF_TEST_RESULTS, meta_event_callback);
    bhy_install_sensor_callback(VS_ID_CUS1, VS_WAKEUP, sensors_callback);

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


    /* get physical sensor present from sensor hub */
    bhy_read_parameter_page(BHY_PAGE_1, BHY_PARAM_SYSTEM_PHYSICAL_SENSOR_PRESENT, &physical_sensor_present_bitmap[0], 8);
    DEBUG("Physical Sensor Present:\n");
    for(i = 0; i < 8; i++)
        DEBUG("bitmap[%d] = 0x%x\n", i, physical_sensor_present_bitmap[i]);
    DEBUG("");


    /* get physical sensor status from sensor hub */
    bhy_get_physical_sensor_status(&phy_acc, &phy_gyro, &phy_mag);

	/*	Physical Sensor Status:
		Flags[bit 0]:  interrupt enable
		Flags[bits 5-7]: Sensor Power Mode values:
			0: Sensor Not Present
			1: Power Down
			2: Suspend
			3: Self-Test
			4: Interrupt Motion
			5: One Shot
			6: Low Power Active
			7: Active
	*/
    DEBUG("Physical Sensor Status:\n");
    DEBUG("Acc : sample rate %d, range %d, int %d pwr %d\n",\
        phy_acc.accel_sample_rate, phy_acc.accel_dynamic_range, phy_acc.accel_flag & 0x01, (phy_acc.accel_flag & 0xE0) >> 5);
    DEBUG("Gyro: sample rate %d, range %d, int %d pwr %d\n", \
        phy_gyro.gyro_sample_rate, phy_gyro.gyro_dynamic_range, phy_gyro.gyro_flag & 0x01, (phy_gyro.gyro_flag & 0xE0) >> 5);
    DEBUG("Mag : sample rate %d, range %d, int %d pwr %d\n", \
        phy_mag.mag_sample_rate, phy_mag.mag_dynamic_range, phy_mag.mag_flag & 0x01, (phy_mag.mag_flag & 0xE0) >> 5);
    DEBUG("");


    /* read custom sensor event size from hub for later fifo parse */
    bhy_sync_cus_evt_size();

    DEBUG("cus evt size = %d %d %d %d %d\n", bhy_get_cus_evt_size(VS_ID_CUS1), bhy_get_cus_evt_size(VS_ID_CUS2), \
                               bhy_get_cus_evt_size(VS_ID_CUS3), bhy_get_cus_evt_size(VS_ID_CUS4), \
                               bhy_get_cus_evt_size(VS_ID_CUS5));


    /* get virtual sensor information from sensor hub */
    DEBUG("Supported Virtual Sensor Information:\n");
    for(i = 1; i < 32; i++)
    {
        bhy_get_wakeup_sensor_information(i, &sensor_info_wakeup);

        if(sensor_info_wakeup.wakeup_sensor_type == i)
            DEBUG("id=%2d\n", i);
    }

    for(i = 33; i < 64; i++)
    {
        bhy_get_non_wakeup_sensor_information(i, &sensor_info_non_wakeup);

        if(sensor_info_non_wakeup.non_wakeup_sensor_type == i)
            DEBUG("id=%2d\n", i);
    }
    DEBUG("");


    /* enables the virtual sensor */
    bhy_enable_virtual_sensor(VS_ID_CUS1, VS_WAKEUP, 5, 0, VS_FLUSH_NONE, 0, 0);

    while(1)
    {
        /* wait until the interrupt fires */
        /* unless we already know there are bytes remaining in the fifo */
        while (!ioport_get_pin_level(BHY_INT) && !bytes_remaining);

        bhy_read_fifo(fifo + bytes_left_in_fifo, FIFO_SIZE - bytes_left_in_fifo, &bytes_read, &bytes_remaining);
        
        bytes_read           += bytes_left_in_fifo;
        fifoptr              = fifo;
        packet_type          = BHY_DATA_TYPE_PADDING;

        do
        {
            /* this function will call callbacks that are registered */
            result = bhy_parse_next_fifo_packet(&fifoptr, &bytes_read, &fifo_packet, &packet_type);

            /* prints all the debug packets */
            if (packet_type == BHY_DATA_TYPE_PADDING)
            {
                /* padding data only added at the end of each FIFO dump, discard it. */
                DEBUG(">Padding\n");
            }
            else if (packet_type == BHY_DATA_TYPE_DEBUG)
            {
                trace_log(">DebugString       :");
                bhy_print_debug_packet(&fifo_packet.data_debug, bhy_printf);
                trace_log("\n");
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

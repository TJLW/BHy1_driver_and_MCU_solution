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
#include ".\firmware\Bosch_PCB_7183_di01_BMI160_BMP280-7183_di01.2.1.10836.h"



/********************************************************************************/
/*                                       MACROS                                 */
/********************************************************************************/
/* ctrl_meas osrs_t[2:0] */
#define BMP280_OSRS_T_POS              (5)
#define BMP280_OSRS_T_SKIPPED          (0x00)
#define BMP280_OSRS_T_1X               (0x01)
#define BMP280_OSRS_T_2X               (0x02)
#define BMP280_OSRS_T_4X               (0x03)
#define BMP280_OSRS_T_8X               (0x04)
#define BMP280_OSRS_T_16X              (0x05)

/* ctrl_meas osrs_p[2:0] */
#define BMP280_OSRS_P_POS              (2)
#define BMP280_OSRS_P_SKIPPED          (0x00)
#define BMP280_OSRS_P_1X               (0x01)
#define BMP280_OSRS_P_2X               (0x02)
#define BMP280_OSRS_P_4X               (0x03)
#define BMP280_OSRS_P_8X               (0x04)
#define BMP280_OSRS_P_16X              (0x05)

/* ctrl_meas mode[1:0] Sensor Specific constants */
#define BMP280_MODE_POS                (0)
#define BMP280_SLEEP_MODE              (0x00)
#define BMP280_FORCED_MODE             (0x01)
#define BMP280_NORMAL_MODE             (0x03)

/* config t_sb[2:0] standby duration */
#define BMP280_STANDBYTIME_POS         (5)
#define BMP280_STANDBYTIME_0_5_MS      (0x00)
#define BMP280_STANDBYTIME_63_MS       (0x01)
#define BMP280_STANDBYTIME_125_MS      (0x02)
#define BMP280_STANDBYTIME_250_MS      (0x03)
#define BMP280_STANDBYTIME_500_MS      (0x04)
#define BMP280_STANDBYTIME_1000_MS     (0x05)
#define BMP280_STANDBYTIME_2000_MS     (0x06)
#define BMP280_STANDBYTIME_4000_MS     (0x07)

/* config filter[2:0] Filter coefficient */
#define BMP280_FILTER_COEFF_POS        (2)
#define BMP280_FILTER_COEFF_OFF        (0x00)
#define BMP280_FILTER_COEFF_2          (0x01)
#define BMP280_FILTER_COEFF_4          (0x02)
#define BMP280_FILTER_COEFF_8          (0x03)
#define BMP280_FILTER_COEFF_16         (0x04)


#define BMP280_REG_CTRL_MEAS_VAL(osrs_t, osrs_p, mode) ((osrs_t << BMP280_OSRS_T_POS) | (osrs_p << BMP280_OSRS_P_POS) | (mode << BMP280_MODE_POS))
#define BMP280_REG_CONFIG_VAL(standby, filter) ((standby << BMP280_STANDBYTIME_POS) | (filter << BMP280_FILTER_COEFF_POS))


/* recommended 6 usecase settings.for applications. Detail BMP280 see datasheet Table 15 
 *
 * !!! Please consult Bosch engineer for fine tuning these BMP parameters. !!!
 */
#define USECASE1_LOWPOWER_CTRL_MEAS           BMP280_REG_CTRL_MEAS_VAL(BMP280_OSRS_T_2X, BMP280_OSRS_P_16X, BMP280_NORMAL_MODE)  //0x57
#define USECASE1_LOWPOWER_CONFIG              BMP280_REG_CONFIG_VAL(BMP280_STANDBYTIME_63_MS, BMP280_FILTER_COEFF_4)             //0x28

#define USECASE2_DYNAMIC_CTRL_MEAS            BMP280_REG_CTRL_MEAS_VAL(BMP280_OSRS_T_1X, BMP280_OSRS_P_4X, BMP280_NORMAL_MODE)   //0x2f
#define USECASE2_DYNAMIC_CONFIG               BMP280_REG_CONFIG_VAL(BMP280_STANDBYTIME_0_5_MS, BMP280_FILTER_COEFF_16)           //0x10

#define USECASE3_LOWPOWER_FORCED_CTRL_MEAS    BMP280_REG_CTRL_MEAS_VAL(BMP280_OSRS_T_1X, BMP280_OSRS_P_1X, BMP280_FORCED_MODE)   //0x25
#define USECASE3_LOWPOWER_FORCED_CONFIG       BMP280_REG_CONFIG_VAL(BMP280_STANDBYTIME_1000_MS, BMP280_FILTER_COEFF_OFF)         //0xa0

#define USECASE4_CHANGE_DETECTION_CTRL_MEAS   BMP280_REG_CTRL_MEAS_VAL(BMP280_OSRS_T_1X, BMP280_OSRS_P_4X, BMP280_NORMAL_MODE)   //0x2f
#define USECASE4_CHANGE_DETECTION_CONFIG      BMP280_REG_CONFIG_VAL(BMP280_STANDBYTIME_125_MS, BMP280_FILTER_COEFF_4)            //0x48

#define USECASE5_DROP_DETECTION_CTRL_MEAS     BMP280_REG_CTRL_MEAS_VAL(BMP280_OSRS_T_1X, BMP280_OSRS_P_2X, BMP280_NORMAL_MODE)   //0x2b
#define USECASE5_DROP_DETECTION_CONFIG        BMP280_REG_CONFIG_VAL(BMP280_STANDBYTIME_0_5_MS, BMP280_FILTER_COEFF_OFF)          //0x00

#define USECASE6_INDOOR_NAVIGATION_CTRL_MEAS  BMP280_REG_CTRL_MEAS_VAL(BMP280_OSRS_T_2X, BMP280_OSRS_P_16X, BMP280_NORMAL_MODE)  //0x57
#define USECASE6_INDOOR_NAVIGATION_CONFIG     BMP280_REG_CONFIG_VAL(BMP280_STANDBYTIME_0_5_MS, BMP280_FILTER_COEFF_16)           //0x10



/* should be greater or equal to 69 bytes, page size (50) + maximum packet size(18) + 1 */
#define FIFO_SIZE                      300
#define MAX_PACKET_LENGTH              18

#define BMP280_PARAM_PAGE       2
#define BMP280_PARAM_ITEM       18

/********************************************************************************/
/*                                GLOBAL VARIABLES                              */
/********************************************************************************/



/********************************************************************************/
/*                                STATIC VARIABLES                              */
/********************************************************************************/
static uint8_t fifo[FIFO_SIZE];
static uint8_t bmp_settings[2];


/********************************************************************************/
/*                                 FUNCTIONS                                    */
/********************************************************************************/

/*!
 * @brief This function is  callback function for acquring sensor datas
 *
 * @param[in]   sensor_data
 * @param[in]   sensor_id
 */
static void sensors_callback_pressure(bhy_data_generic_t * sensor_data, bhy_virtual_sensor_t sensor_id)
{
    if(sensor_id == VS_ID_BAROMETER_WAKEUP)
    {
        /* 1/128Pa Detail see BHI datasheet */
        DEBUG("pressure[%d] data: 0x%x %dPa\n", sensor_data->data_scalar_u24.sensor_id,\
                            sensor_data->data_scalar_u24.data, sensor_data->data_scalar_u24.data/128);
    }
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

    /* recommended 6 usecase settings.for applications. Detail BMP280 see datasheet Table 15 
     *
     * !!! Please consult Bosch engineer for fine tuning these BMP parameters. !!!
     */
    bmp_settings[0] = USECASE1_LOWPOWER_CTRL_MEAS;
    bmp_settings[1] = USECASE1_LOWPOWER_CONFIG;
    bhy_write_parameter_page(BMP280_PARAM_PAGE, BMP280_PARAM_ITEM, &bmp_settings[0], 2);


    /* install the callback function for parse fifo data */
    if(bhy_install_sensor_callback(VS_TYPE_PRESSURE, VS_WAKEUP, sensors_callback_pressure))
    {
        DEBUG("Fail to install sensor callback\n");
    }

    /* enables the virtual sensor */
    if(bhy_enable_virtual_sensor(VS_TYPE_PRESSURE, VS_WAKEUP, 20, 0, VS_FLUSH_NONE, 0, 0))
    {
        DEBUG("Fail to enable sensor id=%d\n", VS_TYPE_PRESSURE);
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

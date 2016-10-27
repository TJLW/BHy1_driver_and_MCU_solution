/*
 * bhy_uc_driver.h
 *
 * Created: 8/12/2015 13:33:40
 *  Author: Marc-Andre Harvey
 */

 /*  Disclaimer
  *
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

#ifndef BHY_UC_DRIVER_H_
#define BHY_UC_DRIVER_H_

#include "BHy_support.h"
#include "bhy_uc_driver_types.h"

/****************************************************************************/
/*                          Driver Functions                                */
/****************************************************************************/


/* initializes the driver, the API and loads the ram patch into the sensor  */
BHY_RETURN_FUNCTION_TYPE bhy_driver_init
    (const u8 *bhy_fw_data/*, const u32 bhy_fw_len*/);

/* this function configures meta event */
BHY_RETURN_FUNCTION_TYPE bhy_meta_event_set_config( bhy_meta_event_type_t meta_event_id,
                                                bhy_meta_event_fifo_type_t fifo_sel,
                                                u8 enable_state, u8 int_enable_state);

/* this function gets configuration from specific meta event */
BHY_RETURN_FUNCTION_TYPE bhy_meta_event_get_config( bhy_meta_event_type_t meta_event_id,
                                                bhy_meta_event_fifo_type_t fifo_sel,
                                                u8* p_enable_state, u8* p_int_enable_state);

/*****************************************************************************
 * Function      : bhy_set_mapping_matrix
 * Description   : Set mapping matrix to a corresponding physical sensor.
 * Input         : index: physical sensor index.
 *                          PHYSICAL_SENSOR_INDEX_ACC = 0,
 *                          PHYSICAL_SENSOR_INDEX_MAG = 1,
 *                          PHYSICAL_SENSOR_INDEX_GYRO = 2,
 *                 p_mapping_matrix: pointer to a "s8 mapping_matrix[9]".
 * Output        : None
 * Return        :
*****************************************************************************/
BHY_RETURN_FUNCTION_TYPE bhy_set_mapping_matrix(bhy_physical_sensor_index_type_t index , s8* p_mapping_matrix);

/*****************************************************************************
 * Function      : bhy_get_mapping_matrix
 * Description   : Get mapping matrix from a corresponding physical sensor.
 * Input         : index: physical sensor index.
 *                          PHYSICAL_SENSOR_INDEX_ACC = 0,
 *                          PHYSICAL_SENSOR_INDEX_MAG = 1,
 *                          PHYSICAL_SENSOR_INDEX_GYRO = 2,
 * Output        : p_mapping_matrix: pointer to a "s8 mapping_matrix[9]".
 * Return        :
*****************************************************************************/
BHY_RETURN_FUNCTION_TYPE bhy_get_mapping_matrix(bhy_physical_sensor_index_type_t index , s8* p_mapping_matrix);

/* this functions enables the selected virtual sensor                       */
BHY_RETURN_FUNCTION_TYPE bhy_enable_virtual_sensor
    (bhy_virtual_sensor_t sensor_id, u8 wakeup_status, u16 sample_rate,
     u16 max_report_latency_ms, u8 flush_sensor, u16 change_sensitivity,
     u16 dynamic_range);

/* this functions disables the selected virtual sensor                      */
BHY_RETURN_FUNCTION_TYPE bhy_disable_virtual_sensor
    (bhy_virtual_sensor_t sensor_id, u8 wakeup_status);

/* retrieves the fifo data. it needs a buffer of at least 51 bytes to work  */
/* it outputs the data into the variable buffer. the number of bytes read   */
/* into bytes_read  and the bytes remaining in the fifo into bytes_left.    */
/* Setting BST_APPLICATION_BOARD to 1 will limit the size to 51 bytes all   */
/* the time. Arguments :                                                    */
/* buffer           Pointer to the buffer to use for the fifo readout.      */
/* buffer_size      Size of the buffer to work with. Needs to be 51 bytes+  */
/* bytes_read       Pointer to output the number of bytes actually read from*/
/*                  the fifo.                                               */
/* bytes_left       Pointer to output the number of bytes still in the fifo.*/
/*                  This function automatically keeps track of the current  */
/*                  fifo readout progress.
*/
BHY_RETURN_FUNCTION_TYPE bhy_read_fifo
    (u8 * buffer, u16 buffer_size, u16 * bytes_read, u16 * bytes_left);

/* This function parses the next fifo packet and return it into a generic   */
/* data structure while telling you what the data type is so you can        */
/* retrieve it. Here are the parameters:                                    */
/* fifo_buffer          pointer to the fifo byte that is the start of a     */
/*                      packet. This pointer will be automatically          */
/*                      incremented so you can call this function in a loop */
/* fifo_buffer_length   pointer to the amount of data left in the           */
/*                      fifo_buffer. This data will be automatically        */
/*                      decremented so you can call this function in a loop */
/* fifo_data_output     buffer in which to place the data                   */
/* fifo_data_type       data type output                                    */
BHY_RETURN_FUNCTION_TYPE bhy_parse_next_fifo_packet
    (u8 **fifo_buffer, u16 *fifo_buffer_length,
    bhy_data_generic_t * fifo_data_output, bhy_data_type_t * fifo_data_type);

/* This function will detect the timestamp packet accordingly and update    */
/* either the MSW or the LSW of the system timestamp. Arguments :           */
/* timestamp_packet     The timestamp packet processed by the parse_next_   */
/*                      fifo_packet, properly typecasted                    */
/* system_timestamp     Pointer to a 32bit variable holding the system      */
/*                      timestamp in 1/32000th seconds. it will wrap around */
/*                      every 36 hours.                                     */
BHY_RETURN_FUNCTION_TYPE bhy_update_system_timestamp(bhy_data_scalar_u16_t *timestamp_packet,
u32 * system_timestamp);

/* This function writes arbitrary data to an arbitrary parameter page. To be*/
/* used carefully since it can override system configurations. Refer to the */
/* datasheet for free to use parameter pages. Here are the arguments:       */
/* page                 Page number. Valid range 1 to 15.                   */
/* parameter            Parameter number. Valid range 0 to 127.             */
/* data                 Pointer to the data source to write to.             */
/* length               Number of bytes to write. Valid range 1 to 8.       */
BHY_RETURN_FUNCTION_TYPE bhy_write_parameter_page(u8 page, u8 parameter, u8 * data, u8 length);

/* This function reads arbitrary data to an arbitrary parameter page. To be*/
/* used carefully since it can override system configurations. Refer to the */
/* datasheet for free to use parameter pages. Here are the arguments:       */
/* page                 Page number. Valid range 1 to 15.                   */
/* parameter            Parameter number. Valid range 0 to 127.             */
/* data                 Pointer to the data source to write to.             */
/* length               Number of bytes to read. Valid range 1 to 16.       */
BHY_RETURN_FUNCTION_TYPE bhy_read_parameter_page(u8 page, u8 parameter, u8 * data, u8 length);

#if BHY_DEBUG

    /* This function outputs the debug data to function pointer. You need to    */
    /* provide a function that takes as argument a zero-terminated string and   */
    /* prints it                                                                */
    void bhy_print_debug_packet
    (bhy_data_debug_t *packet, void (*debug_print_ptr)(const u8 *));

#endif


#if BHY_CALLBACK_MODE

    /* These functions will install the callback and return an error code if    */
    /* there is already a callback installed                                    */
    BHY_RETURN_FUNCTION_TYPE bhy_install_sensor_callback ( bhy_virtual_sensor_t sensor_id, u8 wakeup_status, void (*sensor_callback)(bhy_data_generic_t *, bhy_virtual_sensor_t) );
    BHY_RETURN_FUNCTION_TYPE bhy_install_timestamp_callback ( u8 wakeup_status, void (*timestamp_callback)(bhy_data_scalar_u16_t *) );
    BHY_RETURN_FUNCTION_TYPE bhy_install_meta_event_callback ( bhy_meta_event_type_t meta_event_id, void (*meta_event_callback)(bhy_data_meta_event_t *, bhy_meta_event_type_t) );



    /* These functions will uninstall the callback and return an error code if  */
    /* there was no callback installed                                          */
    BHY_RETURN_FUNCTION_TYPE bhy_uninstall_sensor_callback ( bhy_virtual_sensor_t sensor_id, u8 wakeup_status );
    BHY_RETURN_FUNCTION_TYPE bhy_uninstall_timestamp_callback ( u8 wakeup_status );
    BHY_RETURN_FUNCTION_TYPE bhy_uninstall_meta_event_callback ( bhy_meta_event_type_t meta_event_id );

#endif


#endif /* BHY_UC_DRIVER_H_ */
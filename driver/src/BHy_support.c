/**
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:   BHy_support.c
*
* Date:   2015/07/17
*
* Revision: 1.0
*
* Usage:  BHy on SAM G55
*
**************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
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
*************************************************************************/
/*!
*
* @file   Bhy_support.c
* @author Marc-Andre Harvey
*
* @brief  BHY API Support Source File
*
*
*/

/************************************************************************/
/**\name Own Header File                                                */
/************************************************************************/
#include "bhy_support.h"

extern void mdelay(uint32_t ul_dly_ticks);

static s8 bhy_i2c_write_internal(u8 dev_addr, u8 reg_addr, u8 *reg_data, u16 length)
{
    u32 bhy_write_stat;

    bhy_i2c_packet.chip         =   dev_addr;   //i2c address
    *bhy_i2c_packet.addr        =   reg_addr;   //register address
    bhy_i2c_packet.addr_length  =   1;
    bhy_i2c_packet.buffer       =   reg_data;
    bhy_i2c_packet.length       =   length;

    bhy_write_stat = twi_master_write(TWI4, &bhy_i2c_packet);

    if (bhy_write_stat != TWI_SUCCESS)
    {
        //insert error handling code here
        //i2c_master_send_stop(&i2c_master_instance);
        //i2c_master_disable(&i2c_master_instance);
        //i2c_initialize();
        return BHY_ERROR;
    }

    return BHY_SUCCESS;

}

static s8 bhy_i2c_read_internal(u8 dev_addr, u8 reg_addr, u8 *rx_data, u16 length)
{
    u32 bhy_read_stat;

    bhy_i2c_packet.chip         =   dev_addr;   //i2c address
    *bhy_i2c_packet.addr        =   reg_addr;   //register address
    bhy_i2c_packet.addr_length  =   1;
    bhy_i2c_packet.buffer       =   rx_data;
    bhy_i2c_packet.length       =   length;

    bhy_read_stat = twi_master_read(TWI4, &bhy_i2c_packet);

    if (bhy_read_stat != TWI_SUCCESS)
    {
        //insert error handling code
        //i2c_master_disable(&i2c_master_instance);
        //i2c_initialize();
        return BHY_ERROR;
    }

    return BHY_SUCCESS;

}

s8 bhy_i2c_write(u8 dev_addr, u8 reg_addr, u8 *p_wr_buf, u16 wr_len)
{
    s8 ret = BHY_SUCCESS;

    /* split the write if write length is larger than limitation */
    while (wr_len > I2C_ONCE_WRITE_MAX_COUNT)
    {
        ret += bhy_i2c_write_internal(dev_addr, reg_addr, p_wr_buf, I2C_ONCE_WRITE_MAX_COUNT);
        wr_len -= I2C_ONCE_WRITE_MAX_COUNT;
        if (reg_addr != BHY_I2C_REG_UPLOAD_DATA_ADDR)
        {
            reg_addr += I2C_ONCE_WRITE_MAX_COUNT;
        }
        p_wr_buf += I2C_ONCE_WRITE_MAX_COUNT;
    }

    if (wr_len > 0)
    {
        ret += bhy_i2c_write_internal(dev_addr, reg_addr, p_wr_buf, wr_len);
    }

    return ret;
}

s8 bhy_i2c_read(u8 dev_addr, u8 reg_addr, u8 *p_rd_buf, u16 rd_len)
{
    s8 ret = BHY_SUCCESS;
    u16 rd_len_in_once = I2C_ONCE_READ_MAX_COUNT;
    /* split the read if read length is larger than limitation */
    while (rd_len > I2C_ONCE_READ_MAX_COUNT)
    {
        if(reg_addr <= BHY_I2C_REG_BUFFER_END_ADDR)/* check fifo buffer address */
        {
            if((reg_addr+I2C_ONCE_READ_MAX_COUNT)>=BHY_I2C_REG_BUFFER_END_ADDR)
            {
                /* if not in burst read mode, the max read lenght should not over BHY_I2C_REG_BUFFER_LENGTH(50)*/
                rd_len_in_once = (BHY_I2C_REG_BUFFER_LENGTH - reg_addr);
            }
            else
            {
                rd_len_in_once = I2C_ONCE_READ_MAX_COUNT;
            }
            ret += bhy_i2c_read_internal(dev_addr, reg_addr, p_rd_buf, rd_len_in_once);
            rd_len -= rd_len_in_once;
            p_rd_buf += rd_len_in_once;
            reg_addr += rd_len_in_once;
            if(reg_addr >= BHY_I2C_REG_BUFFER_END_ADDR)
            {
                reg_addr = BHY_I2C_REG_BUFFER_ZERO_ADDR;/* revert read address */
            }
        }
        else
        {
            ret += bhy_i2c_read_internal(dev_addr, reg_addr, p_rd_buf, I2C_ONCE_READ_MAX_COUNT);
            rd_len -= I2C_ONCE_READ_MAX_COUNT;
            p_rd_buf += I2C_ONCE_READ_MAX_COUNT;
            reg_addr += I2C_ONCE_READ_MAX_COUNT;
        }
    }

    if (rd_len > 0)
    {
        ret += bhy_i2c_read_internal(dev_addr, reg_addr, p_rd_buf, rd_len);
    }

    return ret;
}

s8 bhy_initialize_support(void)
{
    u8 tmp_retry = 3;

    bhy.bus_write = &bhy_i2c_write;
    bhy.bus_read = &bhy_i2c_read;
    bhy.delay_msec = &bhy_delay_msec;
    bhy.device_addr = BHY_I2C_SLAVE_ADDRESS;

    bhy_init(&bhy);

    bhy_reset();

    while(tmp_retry--)
    {
        bhy_get_product_id (&bhy.product_id);
        if(PRODUCT_ID_7183 == bhy.product_id)
        {
            return BHY_SUCCESS;
        }
        bhy_delay_msec(BHY_PARAMETER_ACK_DELAY);
    }

    return BHY_PRODUCT_ID_ERROR;
}

void bhy_delay_msec(u32 msec)
{
    mdelay(msec);
}



void bhy_reset(void)
{
    bhy_set_reset_request(BHY_RESET_ENABLE);
}

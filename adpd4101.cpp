/***************************************************************************//**
 *   @file   adpd410x.c
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "adpd4101.h"
#include <Wire.h>
#include <Arduino.h>

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * Find first set bit in word.
 */
uint32_t find_first_set_bit(uint32_t word)
{
	uint32_t first_set_bit = 0;
	while (word) {
		if (word & 0x1)
			return first_set_bit;
		word >>= 1;
		first_set_bit ++;
	}
	return 32;
}

/**
 * Get a field specified by a mask from a word.
 */
uint32_t field_get(uint32_t mask, uint32_t word)
{
	return (word & mask) >> find_first_set_bit(mask);
}





/**
 * @brief Read device register.
 * @param dev - Device handler.
 * @param address - Register address.
 * @param data - Pointer to the register value container.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_reg_read(uint16_t address, uint16_t *data)
{
	int32_t ret;
	uint8_t buff[] = {0, 0};

	ret = adpd4101_reg_read_bytes(address, (uint8_t *) buff, 2);
	if (ret != 0) {
		return ret;
	}

	*data = ((uint16_t)buff[0] << 8) & 0xff00;
	*data |= buff[1] & 0xff;

	return 0;
}

/**
 * @brief Read a specified number of bytes from device register.
 * @param dev - Device handler.
 * @param address - Register address.
 * @param data - Pointer to the register value container.
 * @param num_bytes - number of bytes to read. Max 255 for ADPD4101 (I2C).
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_reg_read_bytes(uint16_t address, uint8_t *data, uint8_t num_bytes)
{
	int32_t ret;
	uint8_t i;
	uint8_t *buff;
  
  // Number of bytes for an I2C read is an 8-bit number, or at most 255  (impossible 'cause num_byte is byte)
  if (num_bytes > 255)
    return -1;

  buff = (uint8_t *) calloc(2, sizeof(*buff));
  buff[0] = field_get(ADPD410X_UPPDER_BYTE_I2C_MASK, address);
  buff[0] |= 0x80;
  buff[1] = address & ADPD410X_LOWER_BYTE_I2C_MASK;

  Wire.beginTransmission(I2C_SLAVE_ADDRESS);            // Attention sensor @ deviceAddress!
 
  Wire.write(buff[0]);                             // move your memory pointer to registerAddress
  Wire.write(buff[1]);                      

  Wire.endTransmission();                               // end transmission  
  Wire.requestFrom(I2C_SLAVE_ADDRESS, num_bytes);       // Request num of bytes from slave device
  i=0;
  while(Wire.available()) {                             // Slave may send less than requested
      data[i++] = Wire.read();                          // Receive a byte as character
      }
  if(i==0) {
    free(buff);
    return -1;
  }

	free(buff);
	return 0;
}

/**
 * @brief Write device register.
 * @param dev - Device handler.
 * @param address - Register address.
 * @param data - New register value.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_reg_write(uint16_t address, uint16_t data)
{
	uint8_t buff[] = {0, 0, 0, 0};
  
  buff[0] = field_get(ADPD410X_UPPDER_BYTE_I2C_MASK, address);
  buff[0] |= 0x80;
  buff[1] = address & ADPD410X_LOWER_BYTE_I2C_MASK;
  buff[2] = field_get(0xff00, data);
  buff[3] = data & 0xff;

  Wire.beginTransmission(I2C_SLAVE_ADDRESS);    // Attention sensor @ deviceAddress!

  Wire.write(buff[0]);        // move your memory pointer to registerAddress
  Wire.write(buff[1]);
  
  Wire.write(buff[2]);        // new data to put into that memory register
  Wire.write(buff[3]);
  
  if(Wire.endTransmission()==0) { // end transmission
  return 0;                       
  }
  else {
    return -1;
  }

}



/**
 * @brief Do a read and write of a register to update only part of a register.
 * @param dev - Device handler.
 * @param address - Address of the register.
 * @param data - Value to be written to the device.
 * @param mask - Mask of the bit field to update.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_reg_write_mask(uint16_t address, uint16_t data, uint16_t mask)
{
	int32_t ret;
	uint16_t reg_val;
	uint32_t bit_pos;

	ret = adpd4101_reg_read(address, &reg_val);
	if (ret != 0)
		return -1;
	reg_val &= ~mask;
	bit_pos = find_first_set_bit((uint32_t)mask);
	reg_val |= (data << bit_pos) & mask;

	return adpd4101_reg_write(address, reg_val);
}

/**
 * @brief Update the clocking option of the device.
 * @param dev - Device handler.
 * @return 0 in case of success, -1 otherwise.
 */
static int32_t adpd4101_get_clk_opt(struct adpd4101_dev *dev)
{
	int32_t ret;
	uint16_t reg_temp;

	ret = adpd4101_reg_read(ADPD410X_REG_SYS_CTL, &reg_temp);
	if(ret != 0)
		return ret;
	dev->clk_opt = (reg_temp & BITM_SYS_CTL_ALT_CLOCKS) >>
		       BITP_SYS_CTL_ALT_CLOCKS;

	return 0;
}

/**
 * @brief Do a software reset.
 * @param dev - Device handler.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_reset(struct adpd4101_dev *dev)
{
	int32_t ret;

	ret = adpd4101_reg_write_mask(ADPD410X_REG_SYS_CTL, 1, BITM_SYS_CTL_SW_RESET);
	if(ret != 0)
		return ret;

	return adpd4101_get_clk_opt(dev);
}

/**
 * @brief Set operation mode.
 * @param dev - Device handler.
 * @param mode - New operation mode.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_set_opmode(enum adpd4101_opmode mode)
{
	return adpd4101_reg_write_mask(ADPD410X_REG_OPMODE, mode, BITM_OPMODE_OP_MODE);
}

/**
 * @brief Get operation mode.
 * @param dev - Device handler.
 * @param mode - Operation mode.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_get_opmode(enum adpd4101_opmode *mode)
{
	int32_t ret;
	uint16_t data;

	ret = adpd4101_reg_read(ADPD410X_REG_OPMODE, &data);
	if (ret != 0)
		return ret;

	*mode = data & BITM_OPMODE_OP_MODE;

	return ret;
}

/**
 * @brief Set number of active time slots.
 * @param dev - Device handler.
 * @param timeslot_no - Last time slot to be enabled.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_set_last_timeslot(uint8_t timeslot_no)      // "enum adpd4101_timeslots" was the original, but it was maybe a mistake
{
	return adpd4101_reg_write_mask(ADPD410X_REG_OPMODE, timeslot_no, BITM_OPMODE_TIMESLOT_EN);
}

/**
 * @brief Get number of active time slots.
 * @param dev - Device handler.
 * @param timeslot_no - Last time slot enabled.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_get_last_timeslot(enum adpd4101_timeslots *timeslot_no)
{
	int32_t ret;
	uint16_t data;

	ret = adpd4101_reg_read(ADPD410X_REG_OPMODE, &data);
	if (ret != 0)
		return ret;

	*timeslot_no = (data & BITM_OPMODE_TIMESLOT_EN) >> 8;

	return ret;
}

/**
 * @brief Set device sampling frequency.
 * @param dev - Device handler.
 * @param sampling_freq - New sampling frequency.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_set_sampling_freq(struct adpd4101_dev *dev, uint32_t sampling_freq)
{
	int32_t ret;
	uint32_t reg_load;
	uint16_t reg_temp;

	if((dev->clk_opt == ADPD410X_INTLFO_INTHFO) ||
	    (dev->clk_opt == ADPD410X_INTLFO_EXTHFO)) {
		ret = adpd4101_reg_read(ADPD410X_REG_SYS_CTL, &reg_temp);
		if(ret != 0)
			return ret;
		if(reg_temp & BITP_SYS_CTL_LFOSC_SEL)
			reg_load = ADPD410X_LOW_FREQ_OSCILLATOR_FREQ1;
		else
			reg_load = ADPD410X_LOW_FREQ_OSCILLATOR_FREQ2;
	} else {
		reg_load = dev->ext_lfo_freq;
	}

	reg_load /= sampling_freq;
	ret = adpd4101_reg_write(ADPD410X_REG_TS_FREQ, (reg_load & 0xFFFF));
	if(ret != 0)
		return ret;
	return adpd4101_reg_write(ADPD410X_REG_TS_FREQH, ((reg_load & 0x7F0000) >> 16));
}

/**
 * @brief Get device sampling frequency.
 * @param dev - Device handler.
 * @param sampling_freq - New sampling frequency.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_get_sampling_freq(struct adpd4101_dev *dev, uint32_t *sampling_freq)
{
	int32_t ret;
	uint32_t reg_load;
	uint16_t reg_temp;

	if ((dev->clk_opt == ADPD410X_INTLFO_INTHFO) ||
	    (dev->clk_opt == ADPD410X_INTLFO_EXTHFO)) {
		ret = adpd4101_reg_read(ADPD410X_REG_SYS_CTL, &reg_temp);
		if (ret != 0)
			return ret;

		if (reg_temp & BITP_SYS_CTL_LFOSC_SEL)
			reg_load = ADPD410X_LOW_FREQ_OSCILLATOR_FREQ1;
		else
			reg_load = ADPD410X_LOW_FREQ_OSCILLATOR_FREQ2;
	} else {
		reg_load = dev->ext_lfo_freq;
	}

	ret = adpd4101_reg_read(ADPD410X_REG_TS_FREQ, &reg_temp);
	if (ret != 0)
		return ret;

	*sampling_freq = reg_temp;

	ret = adpd4101_reg_read(ADPD410X_REG_TS_FREQH, &reg_temp);
	if (ret != 0)
		return ret;

	*sampling_freq = reg_load / (*sampling_freq | ((reg_temp & 0x7F) << 16));

	return ret;
}


/**
 * @brief Setup the device and the driver.
 * @param device - Pointer to the device handler.
 * @param init_param - Pointer to the initialization structure.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_setup(struct adpd4101_dev **device, struct adpd4101_init_param *init_param)
{
	int32_t ret;
	struct adpd4101_dev *dev;
	uint16_t reg_temp;

	dev = calloc(1, sizeof *dev);
	if(!dev)
		// return -ENOMEM;
    return -1;

	dev->ext_lfo_freq = init_param->ext_lfo_freq;

  Wire.begin();   // join the bus as controller

	ret = adpd4101_reset(dev);  // comes back with clk or mask_write_error
	if(ret == -1)
		goto error_3;

  ret = adpd4101_reg_write(0xB5, 0x04);   // 0xB5 ??????
	if(ret != 0)
		goto error_3;

	ret = adpd4101_reg_write(0xB5, 0x00);   // 0xB5 ??????
	if(ret != 0)
		goto error_3;

	ret = adpd4101_reg_read(ADPD410X_REG_CHIP_ID, &reg_temp);
	if(ret != 0)
		goto error_3;

	if((reg_temp & BITM_CHIP_ID) != ADPD410X_CHIP_ID)     // officially 0xC2
		goto error_3;

	dev->clk_opt = init_param->clk_opt;

	ret = adpd4101_reg_write_mask(ADPD410X_REG_SYS_CTL, dev->clk_opt, BITM_SYS_CTL_ALT_CLOCKS);
	if(ret != 0)
		goto error_3;

	/**
	 * Enable the 1MHz oscillator if the internal low frequency oscillator
	 * is used.
	 */
	if((dev->clk_opt == ADPD410X_INTLFO_INTHFO) ||
	    (dev->clk_opt == ADPD410X_INTLFO_EXTHFO)) {
		ret = adpd4101_reg_read(ADPD410X_REG_SYS_CTL, &reg_temp);
		if(ret != 0)
			goto error_3;
		reg_temp |= (BITM_SYS_CTL_OSC_1M_EN | BITM_SYS_CTL_LFOSC_SEL);
		ret = adpd4101_reg_write(ADPD410X_REG_SYS_CTL, reg_temp);
		if(ret != 0)
			goto error_3;
	}

	*device = dev;

	return 0;

error_3:
	ret = -3;   // or print out error message via serial?!?
  Serial.println(F("error_3"));
// error_gpio2:
// 	no_os_gpio_remove(dev->gpio2);
// error_gpio1:
// 	no_os_gpio_remove(dev->gpio1);
// error_gpio0:
// 	no_os_gpio_remove(dev->gpio0);
// error_phy:
// 	if(dev->dev_type == ADPD4100)
// 		no_os_spi_remove(dev->dev_ops.spi_phy_dev);
// 	else
// 		no_os_i2c_remove(dev->dev_ops.i2c_phy_dev);
// error_dev:
// 	free(dev);

	return ret;
}



/**
 * @brief Setup an active time slot.
 * @param dev - Device handler.
 * @param timeslot_no - Time slot ID to setup.
 * @param init - Pointer to the time slot initialization structure.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_timeslot_setup(struct adpd4101_dev *dev,
				enum adpd4101_timeslots timeslot_no,
				struct adpd4101_timeslot_init *init)
{
	int32_t ret;
	uint16_t data;

	ret = adpd4101_reg_read(ADPD410X_REG_OPMODE, &data);
	if(ret != 0)
		return ret;
	if(((data & BITM_OPMODE_TIMESLOT_EN) >> BITP_OPMODE_TIMESLOT_EN) <
	    timeslot_no)
		// return -EINVAL;
    return -1;

	/* Enable channel 2 */
	ret = adpd4101_reg_read(ADPD410X_REG_TS_CTRL(timeslot_no), &data);
	if(ret != 0)
		return ret;
	if(init->enable_ch2)
		data |= BITM_TS_CTRL_A_CH2_EN;
	else
		data &= ~BITM_TS_CTRL_A_CH2_EN;
	ret = adpd4101_reg_write(ADPD410X_REG_TS_CTRL(timeslot_no), data);
	if(ret != 0)
		return ret;

	/* Setup inputs */
	data = init->ts_inputs.option << (init->ts_inputs.pair * 4);
	ret = adpd4101_reg_write(ADPD410X_REG_INPUTS(timeslot_no), data);
	if(ret != 0)
		return ret;

	/* Set precondition PD */
	ret = adpd4101_reg_write_mask(ADPD410X_REG_CATHODE(timeslot_no), init->precon_option, BITM_CATHODE_A_PRECON);
	if(ret != 0)
		return ret;

	/**
	 *  Set TIA VREF and TRIM options. The 0xE000 is writing reserved bits
	 *  as specified in the datasheet.
	 */
	data = init->afe_trim_opt << BITP_AFE_TRIM_A_AFE_TRIM_VREF |
	       init->vref_pulse_opt << BITP_AFE_TRIM_A_VREF_PULSE_VAL |
	       init->chan2 << BITP_AFE_TRIM_A_TIA_GAIN_CH2 |
	       init->chan1 << BITP_AFE_TRIM_A_TIA_GAIN_CH1;
	ret = adpd4101_reg_write(ADPD410X_REG_AFE_TRIM(timeslot_no), data);
	if(ret != 0)
		return ret;

	/* Set LED pattern */
	data = init->pulse4_subtract << BITP_PATTERN_A_SUBTRACT |
	       init->pulse4_reverse << BITP_PATTERN_A_REVERSE_INTEG;
	ret = adpd4101_reg_write(ADPD410X_REG_PATTERN(timeslot_no), data);
	if(ret != 0)
		return ret;

	/* Set bytes number for time slot */
	data = (init->byte_no << BITP_DATA1_A_SIGNAL_SIZE) &
	       BITM_DATA1_A_SIGNAL_SIZE;
	ret = adpd4101_reg_write(ADPD410X_REG_DATA1(timeslot_no), data);
	if(ret != 0)
		return ret;

	/* Set decimate factor */
	data = (init->dec_factor << BITP_DECIMATE_A_DECIMATE_FACTOR) &
	       BITM_DECIMATE_A_DECIMATE_FACTOR;
	ret = adpd4101_reg_write(ADPD410X_REG_DECIMATE(timeslot_no), data);
	if(ret != 0)
		return ret;

	/* Set LED power */
	data = init->led1.value |
	       (init->led2.value << BITP_LED_POW12_A_LED_CURRENT2);
	ret = adpd4101_reg_write(ADPD410X_REG_LED_POW12(timeslot_no), data);
	if(ret != 0)
		return ret;
	data = init->led3.value |
	       (init->led4.value << BITP_LED_POW34_A_LED_CURRENT4);
	ret = adpd4101_reg_write(ADPD410X_REG_LED_POW34(timeslot_no), data);
	if(ret != 0)
		return ret;

	
  /** Integration offset */
  data = ((init->integ_off<<BITP_INTEG_OFFSET_A_INTEG_OFFSET_UPPER) & BITM_INTEG_OFFSET_A_INTEG_OFFSET) ;
  ret = adpd4101_reg_write(ADPD410X_REG_INTEG_OFFSET(timeslot_no), data);
	if(ret != 0)
  {
    return ret;
  }  
    
  /**Integration width */
  ret = adpd4101_reg_write_mask(ADPD410X_REG_INTEG_WIDTH(timeslot_no), init->integ_width, BITM_INTEG_WIDTH_A_INTEG_WIDTH);
	if(ret != 0)
		return ret;  
    
  /** LED offset and LED width*/
  data=(init->LED_off & BITM_LED_PULSE_A_LED_OFFSET) | ((init->LED_width<<BITP_LED_PULSE_A_LED_WIDTH) & BITM_LED_PULSE_A_LED_WIDTH);
  ret = adpd4101_reg_write(ADPD410X_REG_LED_PULSE(timeslot_no), data);
	if(ret != 0)
		return ret;  
  
  /* Enable zero adjust */
	ret = adpd4101_reg_read(ADPD410X_REG_ADC_OFF2(timeslot_no), &data);
	if(ret != 0)
		return ret;
	if(init->zero_adjust)
		data |= BITM_ADC_OFF2_A_ZERO_ADJUST;
	else
		data &= ~BITM_ADC_OFF2_A_ZERO_ADJUST;
	ret = adpd4101_reg_write(ADPD410X_REG_ADC_OFF2(timeslot_no), data);
	if(ret != 0)
		return ret;

/* Set ADC cycle and repeat */
	if(init->integ_num == 0)
		init->integ_num = 1;
	if(init->repeats_no == 0)
		init->repeats_no = 1;
	data = init->repeats_no | (init->integ_num << BITP_COUNTS_A_NUM_INT);  
  return adpd4101_reg_write(ADPD410X_REG_COUNTS(timeslot_no),
				  data);
}


/***********************************************************************************************************************/

/**
 * @brief Get number of bytes in the device FIFO.
 * @param dev - Device handler.
 * @param bytes - Pointer to the byte count container.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_get_fifo_bytecount(uint16_t *bytes)
{
	int32_t ret;

	ret = adpd4101_reg_read(ADPD410X_REG_FIFO_STATUS, bytes);
	if(ret != 0)
		return ret;

	*bytes &= BITM_INT_STATUS_FIFO_FIFO_BYTE_COUNT;

	return 0;
}


/**
 * @brief Clear the device FIFO.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_clear_fifo()
{
  return adpd4101_reg_write_mask(ADPD410X_REG_FIFO_STATUS, 1, BITM_INT_STATUS_FIFO_CLEAR_FIFO);
}



/**
 * @brief Get a data packet containing data from all active time slots and
 *        channels. adpd410x_get_data() helper function.
 * @param dev - Device handler.
 * @param data - Pointer to the data container.
 * @param no_slots - Number of active time slots.
 * @param dual_chan - Mask showing which of the active time slots are dual
 *                    channeled and which are single channeled.
 * @return 0 in case of success, -1 otherwise.
 */
static int32_t adpd4101_get_data_packet(uint32_t *data, uint8_t no_slots,	uint16_t dual_chan)
{
	int32_t ret;
	uint16_t temp_data = 0;
	uint16_t sample_index;
	int8_t i, got_one = 0;
	uint8_t *slot_bytecount_tab;

	slot_bytecount_tab = (uint8_t *)calloc(no_slots,
					       sizeof (*slot_bytecount_tab));
	if (!slot_bytecount_tab)
		// return -ENOMEM;
    return -1;
	for(i = 0; i < no_slots; i++) {
		ret = adpd4101_reg_read(ADPD410X_REG_DATA1(i), &temp_data);
		if(ret != 0)
			goto error_slot;
		slot_bytecount_tab[i] = (temp_data & BITM_DATA1_A_SIGNAL_SIZE);    
    }
  
	i = 0;
	sample_index = 0;
	do {
		ret = adpd4101_read_fifo(data + sample_index, 1, slot_bytecount_tab[i]);
		if (ret != 0)
			goto error_slot;

		sample_index++;
		if(((dual_chan & (1 << i)) != 0) && (got_one == 0)) {
			got_one = 1;
		} else {
			i++;
			got_one = 0;
		}
	} while(i < no_slots);

	free(slot_bytecount_tab);
	return 0;

error_slot:
	free(slot_bytecount_tab);

	return ret;
}




/**
 * @brief Get a full data packet from the device containing data from all active
 *        time slots.
 * @param dev - Device handler.
 * @param data - Pointer to the data container.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_get_data(uint32_t *data)
{
	int32_t ret;
	int8_t i;
	uint16_t temp_data, dual_chan = 0;
	uint8_t ts_no;

	ret = adpd4101_reg_read(ADPD410X_REG_OPMODE, &temp_data);
	if(ret != 0)
		return ret;
	ts_no = ((temp_data & BITM_OPMODE_TIMESLOT_EN) >> BITP_OPMODE_TIMESLOT_EN) + 1;

	for(i = 0; i < ts_no; i++) {
		ret = adpd4101_reg_read(ADPD410X_REG_TS_CTRL(i), &temp_data);
		if(ret != 0)
			return ret;
		if((temp_data & BITM_TS_CTRL_A_CH2_EN) != 0)
			dual_chan |= (1 << i);
	}

	return adpd4101_get_data_packet(data, ts_no, dual_chan);
}





/**
 * @brief Reads a certain number of bytes from the fifo and stores in data
 *        Used to read a large amount of data from the fifo efficiently (using
 *        as few register reads as possible.)
 * @param dev - Device handler.
 * @param data - Pointer to the data container.
 * @param num_samples - number of samples to read
 * @param datawidth - number of bytes per sample
 * @return 0 in case of success, -1 or an error code otherwise.
 */
int32_t adpd4101_read_fifo(uint32_t *data, uint16_t num_samples, uint8_t datawidth)
{
	int32_t ret = 0;
	uint8_t *data_byte_buff, i, j;
	uint16_t next_packet_size, bytes_read = 0, bytecount=0,
				   total_bytes = num_samples * datawidth;
	if (datawidth > 4 || total_bytes > ADPD410X_FIFO_DEPTH || data == NULL)
		return -1;

	data_byte_buff = (uint8_t *) calloc(total_bytes, sizeof (*data_byte_buff));
	if (!data_byte_buff)
		// return -ENOMEM;
    return -1;


	while (bytes_read < total_bytes) {
		next_packet_size = total_bytes - bytes_read;
		// Can read a maximum of 255 bytes at once for i2c

    // ret = adpd4101_get_fifo_bytecount(&bytecount);   // if it's needed...
    // if (ret != 0)
    //     return ret;

		if (next_packet_size > 255)
			next_packet_size = 255;
		ret = adpd4101_reg_read_bytes(ADPD410X_REG_FIFO_DATA, data_byte_buff + bytes_read, next_packet_size);
		if(ret != 0)
			goto fifo_free_return;

		bytes_read += next_packet_size;
	}

	i = 0;
	for (j = 0; j < num_samples; j++) {

		switch(datawidth) {
		case 0:
			break;
		case 1:
			data[j]  = (uint32_t)data_byte_buff[i++];
			break;
		case 2:
			data[j]  = (uint32_t)data_byte_buff[i++] << 8;
			data[j] |= (uint32_t)data_byte_buff[i++];
			break;
		case 3:
			data[j]  = (uint32_t)data_byte_buff[i++] << 8;
			data[j] |= (uint32_t)data_byte_buff[i++];
			data[j] |= (uint32_t)data_byte_buff[i++] << 16;
			break;
		case 4:
			data[j]  = (uint32_t)data_byte_buff[i++] << 8;
			data[j] |= (uint32_t)data_byte_buff[i++];
			data[j] |= (uint32_t)data_byte_buff[i++] << 24;
			data[j] |= (uint32_t)data_byte_buff[i++] << 16;
			break;
		} 
	}

fifo_free_return:
	free(data_byte_buff);
	return ret;
}





/**
 * @brief Free memory allocated by adpd410x_setup().
 * @param dev - Device handler.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t adpd4101_remove(struct adpd4101_dev *dev)
{
	int32_t ret;

	if(!dev)
    return -1;

  Wire.end();

	free(dev);

	return 0;
}

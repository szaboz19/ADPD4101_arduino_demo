/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "adpd4101_app.h"
#include "adpd4101_app_config.h"

#include <Arduino.h>

#define ADPD4101_NUM_CH 8
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/


/**
 * @brief Set device to get timestamp for low frequency oscillator calibration.
 * @param [in] dev - The device structure.
 * @return 0 in case of success, negative code otherwise.
 */
static int32_t adpd4101_app_calibrate_lfo_set_ts(struct adpd4101_app_dev *dev) {
  int32_t ret;
  uint16_t reg_data;

  /** Enable clock calibration circuitry */
  if (dev->chip_id == 0x02c2) {
    ret = adpd4101_reg_read(ADPD410X_REG_OSC1M, &reg_data);
    if (ret == -1)
      return ret;
    reg_data |= BITM_OSC1M_OSC_CLK_CAL_ENA;
    ret = adpd4101_reg_write(ADPD410X_REG_OSC1M, reg_data);
  }

  /** Enable GPIO0 input */
  ret = adpd4101_reg_read(ADPD410X_REG_GPIO_CFG, &reg_data);
  if (ret == -1)
    return ret;

  reg_data |= (1 & BITM_GPIO_CFG_GPIO_PIN_CFG0);

  ret = adpd4101_reg_write(ADPD410X_REG_GPIO_CFG, reg_data);
  if (ret == -1)
    return ret;

  /** Enable GPIO0 as time stamp input */
  ret = adpd4101_reg_read(ADPD410X_REG_GPIO_EXT, &reg_data);
  if (ret == -1)
    return ret;

  reg_data |= ((0 << BITP_GPIO_EXT_TIMESTAMP_GPIO) & BITM_GPIO_EXT_TIMESTAMP_GPIO);

  return adpd4101_reg_write(ADPD410X_REG_GPIO_EXT, reg_data);
}

/**
 * @brief Get time stamp for low frequency oscillator calibration.
 * @param [in] dev - The device structure.
 * @param [out] ts_val - Pointer to the timestamp value container.
 * @param [in] ts_gpio - Descriptor for the counter start/stop GPIO.
 * @return 0 in case of success, negative code otherwise.
 */
static int32_t adpd4101_app_calibrate_lfo_get_timestamp(
  struct adpd4101_app_dev *dev,
  uint32_t *ts_val)

{
  int32_t ret;
  uint16_t reg_data;


  /** Start time stamp calibration */
  ret = adpd4101_reg_read(ADPD410X_REG_OSC32K, &reg_data);
  if (ret == -1)
    return ret;
  reg_data |= BITM_OSC32K_CAPTURE_TIMESTAMP;
  ret = adpd4101_reg_write(ADPD410X_REG_OSC32K, reg_data);
  if (ret == -1)
    return ret;

  digitalWrite(21, HIGH);  // turn on D21
  delay(1);                // 1 ms
  digitalWrite(21, LOW);   // turn off D21

  /** Start time stamp calibration */
  ret = adpd4101_reg_read(ADPD410X_REG_OSC32K, &reg_data);
  if (ret == -1)
    return ret;
  reg_data |= BITM_OSC32K_CAPTURE_TIMESTAMP;
  ret = adpd4101_reg_write(ADPD410X_REG_OSC32K, reg_data);
  if (ret == -1)
    return ret;

  delay(10);
  digitalWrite(21, HIGH);  // turn on D21
  delay(1);
  digitalWrite(21, LOW);  // turn off D21

  ret = adpd4101_reg_read(ADPD410X_REG_OSC32K, &reg_data);
  if (ret == -1)
    return ret;

  if (reg_data & BITM_OSC32K_CAPTURE_TIMESTAMP)
    return ret;

  ret = adpd4101_reg_read(ADPD410X_REG_STAMP_H, &reg_data);
  if (ret == -1)
    return ret;

  *ts_val = (reg_data << 16) & 0xFFFF0000;

  ret = adpd4101_reg_read(ADPD410X_REG_STAMP_L, &reg_data);
  if (ret == -1)
    return ret;

  *ts_val |= reg_data;

  return 0;
}

/**
 * @brief Do low frequency oscillator calibration with respect to an external
 *        reference.
 * @param [in] dev - The device structure.
 * @return 0 in case of success, negative code otherwise.
 */
static int32_t adpd4101_app_calibrate_lfo(struct adpd4101_app_dev *dev) {
  int32_t ret;
  uint32_t ts_val_current, ts_val_last = 0, ts_val;
  uint16_t reg_data, cal_value;
  int8_t rdy = 0;

  ret = adpd4101_app_calibrate_lfo_set_ts(dev);
  if (ret == -1)
    return ret;

  pinMode(21, OUTPUT);    // set the digital pin 21 as output
  digitalWrite(21, LOW);  // turn on D21
  delay(1);               // wait 1 ms

  while (1) {
    ret = adpd4101_app_calibrate_lfo_get_timestamp(dev, &ts_val_current);
    if (ret == -1)
      return ret;

    if (ts_val_current < ts_val_last) {
      ts_val_last = 0;
      continue;
    }
    ts_val = ts_val_current - ts_val_last;
    ts_val_last = ts_val_current;

    ret = adpd4101_reg_read(ADPD410X_REG_OSC1M, &reg_data);
    if (ret == -1)
      return ret;

    cal_value = reg_data & BITM_OSC1M_OSC_1M_FREQ_ADJ;
    if (ts_val < (10000 - (10000 * 0.005)))
      cal_value++;
    else if (ts_val > (10000 + (10000 * 0.005)))
      cal_value--;
    else
      rdy = 1;
    if (rdy == 1)
      break;
    if ((cal_value == 0) || (cal_value == BITM_OSC1M_OSC_1M_FREQ_ADJ))
      break;

    reg_data &= ~BITM_OSC1M_OSC_1M_FREQ_ADJ;
    reg_data |= cal_value & BITM_OSC1M_OSC_1M_FREQ_ADJ;

    ret = adpd4101_reg_write(ADPD410X_REG_OSC1M, reg_data);
    if (ret == -1)
      return ret;
  };

  pinMode(21, INPUT);

  if (rdy == 1)
    return 0;
  else
    return ret;
}

/**
 * @brief Do high frequency oscillator calibration with respect to the low
 *        frequency oscillator.
 * @param [in] dev - The device structure.
 * @return 0 in case of success, negative code otherwise.
 */
int32_t adpd4101_app_calibrate_hfo(struct adpd4101_app_dev *dev) {
  int32_t ret;
  uint16_t reg_data;

  ret = adpd4101_reg_read(ADPD410X_REG_OSC32M_CAL, &reg_data);
  if (ret == -1)
    return ret;

  reg_data |= BITM_OSC32M_CAL_OSC_32M_CAL_START;

  do {
    ret = adpd4101_reg_read(ADPD410X_REG_OSC32M_CAL, &reg_data);
    if (ret == -1)
      return ret;
  } while (reg_data & BITM_OSC32M_CAL_OSC_32M_CAL_START);

  /** Disable clock calibration circuitry */
  ret = adpd4101_reg_read(ADPD410X_REG_OSC1M, &reg_data);
  if (ret == -1)
    return ret;

  reg_data &= ~BITM_OSC1M_OSC_CLK_CAL_ENA;

  return adpd4101_reg_write(ADPD410X_REG_OSC1M, reg_data);
}

/**
 * @brief Initial process of the application.
 * @param [out] device - Pointer to the application handler.
 * @param [in] init_param - Pointer to the application initialization structure.
 * @return 0 in case of success, negative code otherwise.
 */
int32_t adpd4101_app_init(struct adpd4101_app_dev **device) {
  int32_t ret;
  struct adpd4101_app_dev *dev;
  int8_t i;
  uint16_t data;

  dev = (struct adpd4101_app_dev *)calloc(1, sizeof *dev);
  if (!dev)
    return -1;

  ret = adpd4101_setup(&dev->adpd4101_handler, &adpd4101_param);
  if (ret < 0)
    goto error_cn;

  ret = adpd4101_reg_read(ADPD410X_REG_CHIP_ID, &dev->chip_id);
  if (ret < 0)
    goto error_cn;

  Serial.print(F("chip ID: "));
  Serial.println(dev->chip_id);

  ret = adpd4101_set_sampling_freq(dev->adpd4101_handler, ADPD410X_APP_CODE_ODR_DEFAULT);
  if (ret < 0)
    goto error_cn;
  Serial.print(F("sampling freq: "));
  Serial.print(ADPD410X_APP_CODE_ODR_DEFAULT);
  Serial.println(F(" Hz"));

  ret = adpd4101_set_last_timeslot(ADPD410X_ACTIVE_TIMESLOTS - 1);
  if (ret < 0)
    goto error_cn;
  ret = (ADPD410X_ACTIVE_TIMESLOTS);
  Serial.print(F("number of timeslots: "));
  Serial.println(ret);

  for (i = 0; i < ADPD410X_REG_DEFAULT_NR; i++) {
    ret = adpd4101_reg_write(reg_config_default[i][0], reg_config_default[i][1]);
    if (ret == -1)
      return ret;
  }
  Serial.println(F("default registers written"));

  for (i = 0; i < ADPD410X_ACTIVE_TIMESLOTS; i++) {
    ret = adpd4101_timeslot_setup(dev->adpd4101_handler, i, ts_init_tab + i);
    if (ret < 0)
      goto error_cn;

    /** Precondition VC1 and VC2 to TIA_VREF+215mV */
    ret = adpd4101_reg_read(ADPD410X_REG_CATHODE(i), &data);
    if (ret == -1)
      goto error_cn;
    data &= ~(BITM_CATHODE_A_VC2_SEL | BITM_CATHODE_A_VC1_SEL);
    data |= (2 << BITP_CATHODE_A_VC2_SEL) & BITM_CATHODE_A_VC2_SEL;
    data |= (2 << BITP_CATHODE_A_VC1_SEL) & BITM_CATHODE_A_VC1_SEL;
    ret = adpd4101_reg_write(ADPD410X_REG_CATHODE(i), data);
    if (ret == -1)
      goto error_cn;

  // ret=adpd4101_reg_read(ADPD410X_REG_INTEG_OFFSET(i),&data);
  // Serial.print(data, HEX);
  // Serial.print('\t');
  // ret=adpd4101_reg_read(ADPD410X_REG_INTEG_WIDTH(i),&data);
  // Serial.print(data,HEX);
  // Serial.print('\t');
  // ret=adpd4101_reg_read(ADPD410X_REG_LED_PULSE(i),&data);
  // Serial.print(data,HEX);
  // Serial.print('\t'); 
  // ret=adpd4101_reg_read(ADPD410X_REG_ADC_OFF2(i),&data);
  // Serial.println(data,HEX); 
  }
  Serial.println(F("Timeslots configured"));

  ret = adpd4101_app_calibrate_lfo(dev);
  if (ret == -1)
    goto error_cn;
  Serial.println(F("lfo calibrated"));

  ret = adpd4101_app_calibrate_hfo(dev);
  if (ret == -1)
    goto error_cn;
  Serial.println(F("hfo calibrated"));

  *device = dev;
  return ret;

error_cn:
  Serial.println(F("error_cn"));
  free(dev);

  return -1;
}

/**
 * @brief Free memory allocated by adpd4101_app_init().
 * @param [in] dev - The device structure.
 * @return 0 in case of success, negative code otherwise.
 */
int32_t adpd4101_app_remove(struct adpd4101_app_dev *dev) {
  int32_t ret;

  if (!dev)
    return -1;

  ret = adpd4101_remove(dev->adpd4101_handler);
  if (ret == -1)
    return ret;

  free(dev);

  return 0;
}







/**
 * @brief Read ADC Channel data. It's working well only with specific init conditions. (1 timeslot, 8 input channels, chan2 disabled, 32bit data)
 * @param device - Device driver descriptor.
 * @param buf - Input buffer.
 * @param len - Length of the input buffer.
 * @param channel - IIO channel information.
 * @return Number of bytes printed in the output buffer, or negative error code.
 */
int adpd4101_app_read_raw_chan(char *buf, uint32_t len, uint8_t ch_num) {
  int32_t ret;
  uint32_t data[ADPD410X_ACTIVE_TIMESLOTS];

  ret = adpd4101_set_opmode(ADPD410X_GOMODE);
  if (ret != 0)
    return ret;

  ret = adpd4101_get_data(data);
  if (ret != 0)
    return ret;

  ret = adpd4101_set_opmode(ADPD410X_STANDBY);
  if (ret != 0)
    return ret;

  return snprintf(buf, len, "%d", data[ch_num]);  // buffer gets the characters
}

/**
 * @brief Read ADC data samples .
 * @param device - Device driver descriptor.
 * @param buff - Input buffer.
 * @param nb_samples - Input number of samples.
 * @return Number of samples, or negative code.
 */
int32_t adpd4101_app_read_samples(uint32_t *buff, uint32_t nb_samples) {
  int32_t ret;
  uint32_t data[ADPD410X_ACTIVE_TIMESLOTS], i, offset = 0;  // no good!!! need to count all the bytes we need (buff-size/nb_samples!!!!)


  for (i = 0; i < ADPD410X_ACTIVE_TIMESLOTS; i++) {
    data[i] = 0;
  }

  ret = adpd4101_clear_fifo();
  if (ret != 0)
      return ret;

  for (i = 0; i < nb_samples; i++) {
    ret = adpd4101_set_opmode(ADPD410X_GOMODE);
    if (ret != 0)
      return ret;

    ret = adpd4101_get_data(data);
    if (ret != 0)
      return ret;

    ret = adpd4101_set_opmode(ADPD410X_STANDBY);
    if (ret != 0)
      return ret;

    memcpy(&buff[offset], data, ADPD410X_ACTIVE_TIMESLOTS * sizeof(uint32_t));
    offset += ADPD410X_ACTIVE_TIMESLOTS;
  }

  return nb_samples;
}



/**
 * @brief Start continous sampling - GOMODE.
 * @param void
 * @return 0 in case of success, negative code otherwise.
 */
int32_t adpd4101_app_start_continous() {
  return adpd4101_set_opmode(ADPD410X_GOMODE);
}


/**
 * @brief Stop continous sampling - STANDBY.
 * @param void
 * @return 0 in case of success, negative code otherwise.
 */
int32_t adpd4101_app_stop_continous() {
  return adpd4101_set_opmode(ADPD410X_STANDBY);
}


/**
 * @brief Check for new data in FIFO.
 * @param buff - input buffer
 * @param sample_rdy - flag indicating data read from FIFO
 * @return 0 in case of success, negative code otherwise.
 */
int32_t adpd4101_app_update_data(uint32_t *buff, bool *sample_rdy) {
  int32_t ret, nbr;
  uint16_t byte_count, i;
  uint32_t data[ADPD410X_ACTIVE_TIMESLOTS];

  for (i = 0; i < ADPD410X_ACTIVE_TIMESLOTS; i++) {
      data[i] = 0;
    }

  *sample_rdy = false;

  ret = adpd4101_get_fifo_bytecount(&byte_count);
  if (ret != 0)
        return ret;

  if (byte_count<ADPD410X_ACTIVE_TIMESLOTS*4)   // 4 byte data is set in the app_config (needs to be dynamic!!)
      return 0;
  ret = adpd4101_get_data(data);
      if (ret != 0)
         return ret;
  
  *sample_rdy = true;
  memcpy(&buff[0], data, ADPD410X_ACTIVE_TIMESLOTS * sizeof(uint32_t));
  return 0;
}


/**
 * @brief register read.
 * @param reg_name - reigister name
 * @return 0 in case of success, negative code otherwise.
 */
int32_t adpd4101_app_register_read(char* reg_name) {

  int32_t ret;
  uint16_t addr, reg_val;
  uint8_t buff[20];
  
  addr = strtol((char*)reg_name, NULL, 16);
  
  ret = adpd4101_reg_read(addr, &reg_val);
  if (ret == -1){
    Serial.print(F("ERROR! REGISTER READ NOT WORKING"));
    return ret;
  }

  Serial.print("0x");
  Serial.print(addr,HEX);
  Serial.print('\t');
  Serial.print("0x");
  Serial.print(reg_val,HEX);
  Serial.print('\n');

  return 0;
}


/**
 * @brief register write using masking.
 * @param reg_name - reigister name
 * @param reg_val - reigister value
 * @param mask_val - register mask
 * @return 0 in case of success, negative code otherwise.
 */
int32_t adpd4101_app_register_write(char* reg_name, char* reg_val, char* mask_val){
  int32_t ret;
  uint16_t addr, val, mask;

  addr = strtol((char*)reg_name, NULL, 16);
  val=strtol((char*)reg_val, NULL, 16);
  mask=strtol((char*)mask_val, NULL, 16);

  ret=adpd4101_reg_write_mask(addr, val, mask);
  if (ret == -1){
    Serial.print(F("ERROR! REGISTER WRITE NOT WORKING"));
    return ret;
  }

  Serial.print("0x");
  Serial.print(addr,HEX);
  Serial.print('\t');
  Serial.print("0x");
  Serial.print(val),HEX;
  Serial.print('\n');

  return 0;
}



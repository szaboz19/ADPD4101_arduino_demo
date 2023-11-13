#ifndef ADPD4101_H_
#define ADPD4101_H_


/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

// #include <stdbool.h>
// #include "no_os_spi.h"
// #include "no_os_i2c.h"
// #include "no_os_gpio.h"

#include "adpd4101_registers.h"
#include <Wire.h>
#include <Arduino.h>

/******************************************************************************/
/***************************** Defines ****************************************/
/******************************************************************************/


#define I2C_MAX_SPEED_HZ  400000
#define I2C_SLAVE_ADDRESS 0x24


/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/


/**
 * @enum adpd4101_opmode
 * @brief Operation modes of the device
 */
enum adpd4101_opmode {
	/** Standby mode, used for programming */
	ADPD410X_STANDBY,
	/** Active mode, used for sampling data */
	ADPD410X_GOMODE
};

/**
 * @enum adpd4101_ts_input_pair
 * @brief List of input pairs options for time slots
 */
enum adpd4101_ts_input_pair {
	/** Use input pair 1 and 2 */
	ADPD410X_INP12,
	/** Use input pair 3 and 4 */
	ADPD410X_INP34,
	/** Use input pair 5 and 6 */
	ADPD410X_INP56,
	/** Use input pair 7 and 8 */
	ADPD410X_INP78
};

/**
 * @enum adpd4101_ts_input_opt
 * @brief List of input configurations for time slot
 */
enum adpd4101_ts_input_opt {
	/** Both inputs disconnected */
	ADPD410X_INaDIS_INbDIS,
	/** First input connected to channel 1 */
	ADPD410X_INaCH1_INbDIS,
	/** First input connected to channel 2 */
	ADPD410X_INaCH2_INbDIS,
	/** Second input connected to channel 1 */
	ADPD410X_INaDIS_INbCH1,
	/** Second input connected to channel 2 */
	ADPD410X_INaDIS_INbCH2,
	/** First input -> channel 1, second input -> channel 2 */
	ADPD410X_INaCH1_INbCH2,
	/** First input -> channel 2, second input -> channel 1 */
	ADPD410X_INaCH2_INbCH1,
	/** First input + Second input -> channel 1 */
	ADPD410X_INaCH1_INbCH1,
	/** First input + Second input -> channel 2 */
	ADPD410X_INaCH2_INbCH2
};

/**
 * @struct adpd4101_ts_inputs
 * @brief Structure holding time slot input configuration
 */
struct adpd4101_ts_inputs {
	/** Input pair t channel connection option */
	enum adpd4101_ts_input_opt option;
  /** Input pair option */
	enum adpd4101_ts_input_pair pair;
};

/**
 * @enum adpd4101_precon_opt
 * @brief Time slot input precondition options
 */
enum adpd4101_precon_opt {
	/** Float inputs */
	ADPD410X_FLOAT_INS,
	/** Precondition inputs to VC1 */
	ADPD410X_VC1,
	/** Precondition inputs to VC2 */
	ADPD410X_VC2,
	/** Precondition inputs to VICM */
	ADPD410X_VICM,
	/** Precondition inputs to TIA input */
	ADPD410X_TIA_IN,
	/** Precondition inputs to TIA reference voltage */
	ADPD410X_TIA_VREF,
	/** Precondition inputs by shorting the differential pair */
	ADPD410X_SHORT_INS
};

/**
 * @enum adpd4101_tia_vref_volt
 * @brief TIA reference voltage options
 */
enum adpd4101_tia_vref_volt {
	/** 1,1385 V */
	ADPD410X_TIA_VREF_1V1385,
	/** 1,012 V */
	ADPD410X_TIA_VREF_1V012,
	/** 0,8855 V */
	ADPD410X_TIA_VREF_0V8855,
	/** 1,256 V */
	ADPD410X_TIA_VREF_1V265
};

/**
 * @enum adpd4101_tia_gain_res
 * @brief TIA resistor gain setting
 */
enum adpd4101_tia_gain_res {
	/** 200 kOhm */
	ADPD410X_TIA_GAIN_200K,
	/** 100 kOhm */
	ADPD410X_TIA_GAIN_100K,
	/** 50 kOhm */
	ADPD410X_TIA_GAIN_50K,
	/** 25 kOhm */
	ADPD410X_TIA_GAIN_25K,
	/** 12,5 kOhm */
	ADPD410X_TIA_GAIN_12K5
};

/**
 * @enum adpd4101_led_output_opt
 * @brief LED output option
 */
enum adpd4101_led_output_opt {
	/** Option A */
	ADPD410X_OUTPUT_A,
	/** Option B */
	ADPD410X_OUTPUT_B
};

/**
 * @struct _adpd4101_led_control
 * @brief Structure mapping LED output option and LED strength to one byte
 */
struct _adpd4101_led_control {
	/** LED option */
	enum adpd4101_led_output_opt led_output_select : 1;
  /** LED output strength */
	uint8_t led_current_select : 7;
	
};

/**
 * @union adpd4101_led_control
 * @brief Union of the LED mapping and value so they can be accessed both ways
 */
union adpd4101_led_control {
	/** LED control mapping */
	struct _adpd4101_led_control fields;
	/** LED control value */
	uint8_t value;
};

/**
 * @enum adpd4101_timeslots
 * @brief Available Time slots
 */
enum adpd4101_timeslots {
	/** Time slot A */
	ADPD410X_TS_A,
	/** Time slot B */
	ADPD410X_TS_B,
	/** Time slot C */
	ADPD410X_TS_C,
	/** Time slot D */
	ADPD410X_TS_D,
	/** Time slot E */
	ADPD410X_TS_E,
	/** Time slot F */
	ADPD410X_TS_F,
	/** Time slot G */
	ADPD410X_TS_G,
	/** Time slot H */
	ADPD410X_TS_H,
	/** Time slot I */
	ADPD410X_TS_I,
	/** Time slot J */
	ADPD410X_TS_J,
	/** Time slot K */
	ADPD410X_TS_K,
	/** Time slot L */
	ADPD410X_TS_L
};

/**
 * @struct adpd4101_timeslot_init
 * @brief Initialization structure for time slots
 */
struct adpd4101_timeslot_init {
	
	/** Time slot input configuration */
	struct adpd4101_ts_inputs ts_inputs;
	/** LED 1 output and current control */
	union adpd4101_led_control led1;
	/** LED 2 output and current control */
	union adpd4101_led_control led2;
	/** LED 3 output and current control */
	union adpd4101_led_control led3;
	/** LED 4 output and current control */
	union adpd4101_led_control led4;
	/** Enable ADC channel 2 for a time slot */
	bool enable_ch2;
  /** Time slot input precondition option */
	enum adpd4101_precon_opt precon_option;
	/** TIA reference voltage */
	enum adpd4101_tia_vref_volt afe_trim_opt;
	/** TIA alternative reference voltage for pulsing property */
	enum adpd4101_tia_vref_volt vref_pulse_opt;
	/** TIA resistor gain setting for channel 1 */
	enum adpd4101_tia_gain_res chan1;
	/** TIA resistor gain setting for channel 2 */
	enum adpd4101_tia_gain_res chan2;
	/** LED pulse subtracion pattern */
	uint8_t pulse4_subtract;
	/** LED pulse reverse pattern */
	uint8_t pulse4_reverse;
  /** Bytes number for time slot */
	uint8_t byte_no;
	/** Decimate factor - 1 */
	uint8_t dec_factor;
	/** ADC number of LED pulses per cycle */
	uint8_t repeats_no;
  /** Integration number */
  uint8_t integ_num;
  /** Integrafion offset */
  uint8_t integ_off;
  /**Integration width */
  uint8_t integ_width;
  /** LED offset */
  uint8_t LED_off;
  /** LED width */
  uint8_t LED_width;
  /** Zero adjust */
  bool zero_adjust;  
};

/**
 * @enum adpd4101_clk_opt
 * @brief External clock options
 */
enum adpd4101_clk_opt {
	/** Use internal low frequency and high frequency oscillators */
	ADPD410X_INTLFO_INTHFO,
	/** Use external low frequency and internal high frequency oscillator */
	ADPD410X_EXTLFO_INTHFO,
	/** Use internal low frequency and external high frequency oscillator */
	ADPD410X_INTLFO_EXTHFO,
	/** Use external high frequency oscillator and generate low frequency
	 *  using it */
	ADPD410X_GENLFO_EXTHFO
};

/**
 * @struct adpd4101_init_param
 * @brief Device driver initialization structure
 */
struct adpd4101_init_param {
	/** Device clock option */
	enum adpd4101_clk_opt clk_opt;
	/** External low frequency oscillator frequency, if applicable */
	uint32_t ext_lfo_freq;
};

/**
 * @struct adpd4101_dev
 * @brief Device driver handler
 */
struct adpd4101_dev {
	/** Device clock option */
	enum adpd4101_clk_opt clk_opt;
	/** External low frequency oscillator frequency, if applicable */
	uint32_t ext_lfo_freq;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/** Read device register. */
int32_t adpd4101_reg_read(uint16_t address, uint16_t *data);

/** Read a specified number of bytes from a register, from 1-255 */
int32_t adpd4101_reg_read_bytes(uint16_t address,	uint8_t *data, uint8_t num_bytes);

/** Write device register. */
int32_t adpd4101_reg_write(uint16_t address, uint16_t data);

/** Do a read and write of a register to update only part of a register. */
int32_t adpd4101_reg_write_mask(uint16_t address,	uint16_t data, uint16_t mask);

/** Do a software reset. */
int32_t adpd4101_reset(struct adpd4101_dev *dev);

/** Set operation mode. */
int32_t adpd4101_set_opmode(enum adpd4101_opmode mode);

/** Get operation mode. */
int32_t adpd4101_get_opmode(enum adpd4101_opmode *mode);

/** Set device sampling frequency. */
int32_t adpd4101_set_sampling_freq(struct adpd4101_dev *dev, uint32_t sampling_freq);

/** Set number of active time slots. */
int32_t adpd4101_set_last_timeslot(uint8_t timeslot_no);

/** Get number of active time slots. */
int32_t adpd4101_get_last_timeslot(enum adpd4101_timeslots *timeslot_no);

/** Get device sampling frequency. */
int32_t adpd4101_get_sampling_freq(struct adpd4101_dev *dev,
				   uint32_t *sampling_freq);

/** Setup an active time slot. */
int32_t adpd4101_timeslot_setup(struct adpd4101_dev *dev,
				enum adpd4101_timeslots timeslot_no,
				struct adpd4101_timeslot_init *init);

/** Get number of bytes in the device FIFO. */
int32_t adpd4101_get_fifo_bytecount(uint16_t *bytes);

/** Clear the Device FIFO */
int32_t adpd4101_clear_fifo();

/** Read a packet with a certain number of bytes from the FIFO. */
int32_t adpd4101_read_fifo(uint32_t *data, uint16_t num_samples, uint8_t datawidth);

/** Get a full data packet from the device containing data from all active time
 *  slots. */
int32_t adpd4101_get_data(uint32_t *data);

/** Setup the device and the driver. */
int32_t adpd4101_setup(struct adpd4101_dev **device, struct adpd4101_init_param *init_param);

/** Free memory allocated by adpd400x_setup(). */
int32_t adpd4101_remove(struct adpd4101_dev *dev);




#endif /* ADPD4101_H_ */

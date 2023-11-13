#ifndef ADPD4101_APP_H_
#define ADPD4101_APP_H_


/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

#include "adpd4101.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @struct adpd4101_app_dev
 * @brief Application handler structure
 */
struct adpd4101_app_dev {
	/** ADPD device handler */
	struct adpd4101_dev *adpd4101_handler;
	/** Chip id */
	uint16_t chip_id;
};



/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/** Initial process of the application. */
int32_t adpd4101_app_init(struct adpd4101_app_dev **device);

/** Read ADC Channel data. */
int adpd4101_app_read_raw_chan(char *buf, uint32_t len, uint8_t ch_num);

/** Read ADC data samples */
int32_t adpd4101_app_read_samples(uint32_t *buff, uint32_t nb_samples);

/** Start measurement -> GOMODE!! */
int32_t adpd4101_app_start_continous();

/** Stop measurement -> STANDBY!! */
int32_t adpd4101_app_stop_continous();

/** read FIFO during continous measuremnt, if there is something to read */
int32_t adpd4101_app_update_data(uint32_t *buff, bool *sample_rdy);

/** read register */
int32_t adpd4101_app_register_read(char* reg_name);

/** write register mask */
int32_t adpd4101_app_register_write(char* reg_name,char* reg_valu, char* mask_val);


/** Free memory allocated by adpd410x_app_init(). */
int32_t adpd4101_app_remove(struct adpd4101_app_dev *dev);


#endif /* ADPD4101_APP_H_ */


